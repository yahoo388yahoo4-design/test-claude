#!/usr/bin/env python3
"""Round-trip test for the Android path: ARKitScenes episode -> synthetic Android session -> converter.

1. Read an ARKitScenes episode (raw layout, or the materialised "sample" layout with rgb/ depth_mm/
   confidence/ intrinsics/ frames.json, which is turned into raw layout first).
2. Write it exactly the way the Android app (capture/android) writes a session in ARCore mode:
   r2s-capture v1 with platform "android", HEVC video.mp4 (yuv420p, like MediaCodec) + video.mp4.pts.csv,
   raw-deflate depth.zlib.bin / conf.zlib.bin (confidence via ARCore's 0..255 scale, quantised like the
   app), depth_smooth.zlib.bin, extras/conf255.zlib.bin, extras/arcore_frames.jsonl, Android-unit
   sensor logs converted to FORMAT.md units, planes.json, clock.csv, status.csv. ARCore drops depth on
   some frames, so every 7th frame gets none.
3. Convert with tools/convert.py --from android --no-depth-filter to ARKitScenes + LiteReality (traj every
   frame and 10 Hz). The filter is off so the comparison tests the app's depth encoding (raw-deflate uint16
   LE, dw/dh, byte ranges) bit for bit; the filter itself is covered by tools/depth_filter_test.py.
4. Compare with the original (poses, intrinsics, RGB, depth, confidence) and run tools/validate.py on
   the session and on both outputs, and on the multicam smoke session (poses must WARN, not FAIL).

  python android_roundtrip.py EPISODE_DIR --tools ../../tools --work /tmp/android_rt
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

SKIP_DEPTH_EVERY = 7


def sample_to_raw(sample: Path, out: Path) -> Path:
    """Materialised sample layout (frames.json) -> ARKitScenes raw layout."""
    meta = json.loads((sample / "frames.json").read_text())
    vid = str(meta["video_id"])
    raw = out / vid
    for d in ("lowres_wide", "lowres_wide_intrinsics", "lowres_depth", "confidence"):
        (raw / d).mkdir(parents=True, exist_ok=True)
    for fr in meta["frames"]:
        ts = f"{float(fr['rgb_timestamp']):.3f}"
        stem = f"{vid}_{ts}"
        shutil.copy2(sample / fr["rgb"], raw / "lowres_wide" / f"{stem}.png")
        shutil.copy2(sample / fr["depth_mm"], raw / "lowres_depth" / f"{stem}.png")
        shutil.copy2(sample / fr["confidence"], raw / "confidence" / f"{stem}.png")
        K = fr["intrinsics"]
        (raw / "lowres_wide_intrinsics" / f"{stem}.pincam").write_text(
            f"{K['width']} {K['height']} {K['fx']} {K['fy']} {K['cx']} {K['cy']}")
    shutil.copy2(sample / "lowres_wide.traj", raw / "lowres_wide.traj")
    for suf in ("_3dod_mesh.ply", "_3dod_annotation.json"):
        if (sample / f"{vid}{suf}").exists():
            shutil.copy2(sample / f"{vid}{suf}", raw / f"{vid}{suf}")
    return raw


def write_android_session(ep, out: Path, deflate, log=print) -> dict:
    """Mimic the app's ARCore mode output (ArRecorder.kt / SensorRecorder.kt / SessionWriter.kt)."""
    from common import imwrite_rgb
    out.mkdir(parents=True, exist_ok=True)
    (out / "extras").mkdir(exist_ok=True)
    tmp = out / "_png"
    tmp.mkdir(exist_ok=True)
    f0 = ep.frames[0]
    dfh, cfh = open(out / "depth.zlib.bin", "wb"), open(out / "conf.zlib.bin", "wb")
    sfh, c255 = open(out / "depth_smooth.zlib.bin", "wb"), open(out / "extras" / "conf255.zlib.bin", "wb")
    lines, rich, n_depth, dims = [], [], 0, None
    for k, img in ep.iter_images(range(len(ep.frames))):
        f = ep.frames[k]
        imwrite_rgb(tmp / f"{k:06d}.png", img)
        T = f.T.reshape(-1).tolist() if f.T is not None else None
        K = [f.K[0, 0], f.K[1, 1], f.K[0, 2], f.K[1, 2]]
        track = "normal" if f.T is not None else "not_available"
        line = {"i": k, "t": f.t, "w": f.w, "h": f.h, "K": K, "T": T, "track": track,
                "exp": 1 / 120, "iso": 400, "amb": 0.8, "d": None, "c": None}
        r = {"n": k, "t": f.t, "t_arcore_ns": int(f.t * 1e9), "track": track, "T": T, "T_display": T, "K": K,
             "w": f.w, "h": f.h, "exposure_ns": int(1e9 / 120), "iso": 400, "i": k}
        dep = ep.depth(k) if k % SKIP_DEPTH_EVERY != SKIP_DEPTH_EVERY - 1 else None
        if dep is not None:
            def put(fh, arr):
                b = deflate(np.ascontiguousarray(arr).tobytes())
                rng = [fh.tell(), len(b)]
                fh.write(b)
                return rng
            line["d"] = put(dfh, dep.astype("<u2"))
            conf = ep.conf(k)
            if conf is not None:
                v255 = np.array([40, 128, 230], np.uint8)[np.clip(conf, 0, 2)]           # ARCore 0..255 scale
                q = np.where(v255 < 85, 0, np.where(v255 < 170, 1, 2)).astype(np.uint8)    # app quantisation
                line["c"] = put(cfh, q)
                r["raw_depth"] = {"t": f.t, "d": line["d"], "c": line["c"], "c255": put(c255, v255),
                                  "dw": dep.shape[1], "dh": dep.shape[0]}
            line["sd"] = put(sfh, dep.astype("<u2"))
            line["dh"], line["dw"] = dep.shape
            dims = dep.shape
            n_depth += 1
        lines.append(line)
        rich.append(r)
    for fh in (dfh, cfh, sfh, c255):
        fh.close()
    fps = round((len(ep.frames) - 1) / max(ep.frames[-1].t - f0.t, 1e-9), 2)
    # MediaCodec-like HEVC 4:2:0 (lossless luma path keeps the comparison about the pipeline, not x265)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", str(tmp / "%06d.png"),
                    "-c:v", "libx265", "-x265-params", "lossless=1:log-level=error", "-pix_fmt", "yuv420p",
                    "-tag:v", "hvc1", str(out / "video.mp4")], check=True)
    shutil.rmtree(tmp)
    with open(out / "video.mp4.pts.csv", "w") as fh:
        fh.write("frame,pts_us,t_ns\n")
        for k, f in enumerate(ep.frames):
            fh.write(f"{k},{int(f.t * 1e6)},{int(f.t * 1e6) * 1000}\n")
    with open(out / "frames.jsonl", "w") as fh:
        for line in lines:
            fh.write(json.dumps(line) + "\n")
    with open(out / "extras" / "arcore_frames.jsonl", "w") as fh:
        for r in rich:
            fh.write(json.dumps(r) + "\n")
    # Sensors, in FORMAT.md units, from the trajectory (gyro by finite differences, 200 Hz).
    posed = [f for f in ep.frames if f.T is not None]
    t0, t1 = posed[0].t, posed[-1].t
    with open(out / "imu.csv", "w") as imu, open(out / "accel.csv", "w") as acc, open(out / "gyro.csv", "w") as gyr, \
            open(out / "extras" / "sensors_raw.csv", "w") as raw:
        imu.write("t,ax,ay,az,gx,gy,gz,grx,gry,grz,qx,qy,qz,qw,mx,my,mz,mag_acc,heading\n")
        acc.write("t,x,y,z\n")
        gyr.write("t,x,y,z\n")
        raw.write("t,sensor,v0,v1,v2,v3,v4,v5,accuracy\n")
        for a, b in zip(posed[:-1], posed[1:]):
            dt = b.t - a.t
            R = a.T[:3, :3].T @ b.T[:3, :3]
            ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
            w = np.zeros(3) if ang < 1e-9 else ang / dt * np.array(
                [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
            g_cam = a.T[:3, :3].T @ np.array([0, -1.0, 0])     # gravity direction in camera axes (g units)
            for s in np.arange(a.t, b.t, 0.005):
                imu.write(f"{s:.9f},0,0,0,{w[0]},{w[1]},{w[2]},{g_cam[0]},{g_cam[1]},{g_cam[2]},0,0,0,1,0,0,0,-1,-1\n")
                acc.write(f"{s:.9f},{g_cam[0]},{g_cam[1]},{g_cam[2]}\n")
                gyr.write(f"{s:.9f},{w[0]},{w[1]},{w[2]}\n")
                raw.write(f"{s:.9f},gyro,{w[0]},{w[1]},{w[2]},,,,3\n")
    (out / "clock.csv").write_text("uptime,unix\n" + "".join(
        f"{t:.9f},{1790000000 + t:.3f}\n" for t in np.arange(t0, t1, 1.0)))
    (out / "status.csv").write_text("t,unix,thermal,battery,battery_state,low_power\n" + "".join(
        f"{t:.9f},{1790000000 + t:.3f},0,0.8,unplugged,false\n" for t in np.arange(t0, t1, 1.0)))
    (out / "planes.json").write_text("[]")
    session = {
        "format": "r2s-capture", "version": 1, "platform": "android", "mode": "arcore_rgbd",
        "device": "Xiaomi 2304FPN6DC (synthetic from ARKitScenes)", "os": "Android 15 (API 35)", "app_version": "0.1.0 (1)",
        "start_unix": 1790000000 + t0, "start_uptime": t0, "end_unix": 1790000000 + t1, "end_uptime": t1,
        "clock": "t = SystemClock.elapsedRealtimeNanos()/1e9 (CLOCK_BOOTTIME), the 'uptime' clock of FORMAT.md",
        "video": {"file": "video.mp4", "width": f0.w, "height": f0.h, "fps": fps, "codec": "hevc", "bitrate": 0,
                  "source": "synthetic", "pts_file": "video.mp4.pts.csv"},
        "depth": {"width": dims[1], "height": dims[0], "unit": "mm", "dtype": "uint16", "compression": "raw-deflate"} if dims else None,
        "counts": {"frames": len(lines), "arcore_frames": len(lines), "depth": n_depth, "depth_smooth": n_depth},
        "settings": {"hires_stills": False, "arcore_mp4": False, "lock": False, "geospatial": False},
        "android": {"arcore": {"image_size": [f0.w, f0.h], "depth_mode": "AUTOMATIC", "depth_sensor_usage": "DO_NOT_USE",
                               "timestamp_source": "REALTIME"},
                    "synthetic_from": str(ep.path)},
    }
    (out / "session.json").write_text(json.dumps(session, indent=1))
    (out / "DONE").write_text("ok\n")
    info = {"session": str(out), "frames": len(lines), "depth": n_depth}
    log(json.dumps(info))
    return info


def multicam_smoke(android_session: Path, out: Path) -> dict:
    """Camera2 mode layout (cams/<name>.mp4 + .jsonl, empty frames.jsonl): reader must expose unposed frames."""
    from readers import detect, open_episode
    out.mkdir(parents=True, exist_ok=True)
    (out / "cams").mkdir(exist_ok=True)
    lines = [json.loads(x) for x in (android_session / "frames.jsonl").read_text().splitlines()]
    shutil.copy2(android_session / "video.mp4", out / "cams" / "wide_2.mp4")
    with open(out / "cams" / "wide_2.jsonl", "w") as fh:
        for d in lines:
            fh.write(json.dumps({"i": d["i"], "t": d["t"], "w": d["w"], "h": d["h"], "K": d["K"], "exp": d["exp"], "iso": d["iso"],
                                 "lens_intrinsics": [d["K"][0] * 16, d["K"][1] * 16, d["K"][2] * 16, d["K"][3] * 16, 0]}) + "\n")
    (out / "frames.jsonl").write_text("")
    (out / "cams" / "calibration.json").write_text("{}")
    meta = json.loads((android_session / "session.json").read_text())
    meta["mode"] = "multicam"
    meta.pop("video", None), meta.pop("depth", None)   # the app writes neither key in Camera2 mode
    meta["android"]["multicam"] = {"streams": [{"name": "wide_2", "physical_id": "2", "w": lines[0]["w"], "h": lines[0]["h"]}]}
    (out / "session.json").write_text(json.dumps(meta))
    ep = open_episode(out, "auto")
    got = list(ep.iter_images([0, len(ep.frames) - 1]))
    return {"detected": detect(out), "frames": len(ep.frames), "posed": sum(f.T is not None for f in ep.frames),
            "decoded": [g[0] for g in got], "image_shape": list(got[0][1].shape), "has_depth": ep.has_depth,
            "extras": sorted(ep.extras())}


def compare(orig: Path, conv: Path, read_traj, rot_angle_deg, imread_any, imread_rgb) -> dict:
    to, tc = read_traj(orig / "lowres_wide.traj"), read_traj(conv / "lowres_wide.traj")
    names_o = {p.stem.split("_")[-1] for p in (orig / "lowres_wide").glob("*.png")}
    keys = sorted(set(to) & set(tc))
    rerr = [rot_angle_deg(to[k][1][:3, :3], tc[k][1][:3, :3]) for k in keys]
    terr = [float(np.linalg.norm(to[k][1][:3, 3] - tc[k][1][:3, 3])) for k in keys]
    res = {"traj": {"orig_lines": len(to), "orig_lines_at_frames": len(set(to) & names_o), "converted_lines": len(tc),
                    "matched": len(keys), "rot_err_deg_max": max(rerr) if rerr else None,
                    "trans_err_mm_max": max(terr) * 1000 if terr else None}}
    rgb, d_eq, d_n, c_eq, k_err, missing = [], 0, 0, 0, [], 0
    for p in sorted((orig / "lowres_wide").glob("*.png")):
        q = conv / "lowres_wide" / p.name
        if not q.exists():
            missing += 1
            continue
        rgb.append(float(np.abs(imread_rgb(p).astype(int) - imread_rgb(q).astype(int)).mean()))
        a, b = orig / "lowres_depth" / p.name, conv / "lowres_depth" / p.name
        if b.exists():
            d_n += 1
            d_eq += bool(np.array_equal(imread_any(a), imread_any(b)))
            ca, cb = orig / "confidence" / p.name, conv / "confidence" / p.name
            c_eq += bool(cb.exists() and np.array_equal(imread_any(ca), imread_any(cb)))
        ko = np.array((orig / "lowres_wide_intrinsics" / f"{p.stem}.pincam").read_text().split(), float)
        kc = np.array((conv / "lowres_wide_intrinsics" / f"{p.stem}.pincam").read_text().split(), float)
        k_err.append(float(np.abs(ko - kc).max()))
    res["frames"] = {"orig": len(names_o), "missing": missing, "rgb_mean_abs_err": float(np.mean(rgb)) if rgb else None,
                     "rgb_max_frame_err": float(np.max(rgb)) if rgb else None,
                     "depth_written": d_n, "depth_identical": d_eq, "conf_identical": c_eq,
                     "intrinsics_max_abs_err": max(k_err) if k_err else None}
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode", type=Path)
    ap.add_argument("--tools", type=Path, default=Path(__file__).resolve().parents[2] / "tools")
    ap.add_argument("--work", type=Path, required=True)
    args = ap.parse_args()
    sys.path.insert(0, str(args.tools.resolve()))
    import convert  # noqa
    from common import deflate, imread_any, imread_rgb, rot_angle_deg  # noqa
    from readers import READERS  # noqa
    from readers.arkitscenes import ARKitScenesEpisode, read_traj  # noqa
    assert "android" in READERS, "readers/android_reader.py not registered"

    if args.work.exists():
        shutil.rmtree(args.work)
    args.work.mkdir(parents=True)
    ep_dir = args.episode
    if (ep_dir / "frames.json").exists() and not (ep_dir / "lowres_wide").is_dir():
        ep_dir = sample_to_raw(ep_dir, args.work / "raw")
    vid = ep_dir.name
    ep = ARKitScenesEpisode(ep_dir)
    print(f"episode {vid}: {len(ep.frames)} frames")
    sess = args.work / "android_session"
    report = {"episode": vid, "frames": len(ep.frames), "session": write_android_session(ep, sess, deflate)}
    report["detected_reader"] = __import__("readers").detect(sess)
    for label, hz in (("every_frame", 0), ("10hz", 10)):
        out = args.work / f"out_{label}"
        convert.main([str(sess), "--from", "android", "--to", "arkitscenes", "litereality", "--out", str(out),
                      "--video-id", vid, "--traj-hz", str(hz), "--no-depth-filter"])
        conv = out / "arkitscenes" / vid
        r = compare(ep_dir, conv, read_traj, rot_angle_deg, imread_any, imread_rgb)
        lr = out / "litereality" / sess.name
        r["litereality"] = {"frames": len(list(lr.glob("frame_*.jpg"))), "json": len(list(lr.glob("frame_*.json"))),
                            "depth": len(list(lr.glob("depth_*.png"))), "conf": len(list(lr.glob("conf_*.png"))),
                            "obj": (lr / "textured_output.obj").exists(),
                            "extras_android": (lr / "extras" / "android" / "arcore_frames.jsonl").exists()}
        val = {}
        for name, path in (("session", sess), ("arkitscenes", conv), ("litereality", lr)):
            p = subprocess.run([sys.executable, str(args.tools / "validate.py"), str(path)], capture_output=True, text=True)
            try:
                v = json.loads(p.stdout)
                val[name] = {"result": v["result"], "fail_or_warn": [c for c in v["checks"] if c["status"] != "PASS"]}
            except Exception:
                val[name] = {"result": "ERROR", "stderr": p.stderr[-2000:], "stdout": p.stdout[-2000:]}
        r["validate"] = val
        report[label] = r
    report["multicam_reader_smoke"] = multicam_smoke(sess, args.work / "multicam_session")
    p = subprocess.run([sys.executable, str(args.tools / "validate.py"), str(args.work / "multicam_session")],
                       capture_output=True, text=True)
    try:
        v = json.loads(p.stdout)
        report["multicam_reader_smoke"]["validate"] = {"result": v["result"], "exit": p.returncode,
                                                       "fail_or_warn": [c for c in v["checks"] if c["status"] != "PASS"]}
    except Exception:
        report["multicam_reader_smoke"]["validate"] = {"result": "ERROR", "exit": p.returncode, "stderr": p.stderr[-2000:]}
    print(json.dumps(report, indent=1))
    (args.work / "android_roundtrip_report.json").write_text(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
