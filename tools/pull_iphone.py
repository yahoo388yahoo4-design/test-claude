#!/usr/bin/env python3
"""Copy R2S Capture sessions off an iPhone over USB, without touching the app.

    python pull_iphone.py                          # copy every new session to ~/captures
    python pull_iphone.py --convert ~/converted    # ...and convert each new one (ARKitScenes + LiteReality)
    python pull_iphone.py --list                   # only list what is on the phone
    python pull_iphone.py --session 20261004_221530_arkit_rgbd

Needs `pip install pymobiledevice3` and usbmuxd (Ubuntu: `sudo apt install usbmuxd`). Unlock the phone
and tap Trust the first time. It reads the app's Documents folder through Apple's house_arrest service
(the same one Finder / iTunes file sharing uses), so the app must be installed with file sharing on,
which R2S Capture is. The bundle id is found automatically, even if the sideloading tool changed it.

Re-running is safe: files that already exist locally with the same size are skipped, so an interrupted
copy resumes. Sessions still being recorded (session.json "complete": false) are skipped unless
--include-incomplete.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

try:
    from pymobiledevice3.exceptions import AfcFileNotFoundError
    from pymobiledevice3.lockdown import create_using_usbmux
    from pymobiledevice3.services.house_arrest import HouseArrestService
    from pymobiledevice3.services.installation_proxy import InstallationProxyService
except ImportError:
    sys.exit("pymobiledevice3 is missing: run  pip install pymobiledevice3  (in your venv) and try again.")


def is_r2s_app(bundle_id: str, info: dict) -> bool:
    names = " ".join(str(info.get(k, "")) for k in ("CFBundleExecutable", "CFBundleName", "CFBundleDisplayName"))
    return "r2scapture" in bundle_id.lower() or "R2SCapture" in names or "R2S Capture" in names


async def find_bundle(lockdown, wanted: str | None) -> str:
    apps = await InstallationProxyService(lockdown=lockdown).get_apps(application_type="User")
    if wanted:
        if wanted not in apps:
            sys.exit(f"No app with bundle id {wanted} on the phone.")
        return wanted
    hits = [b for b, info in apps.items() if is_r2s_app(b, info)]
    if not hits:
        sys.exit("R2S Capture is not installed on this phone (no matching app found). Use --bundle to name it.")
    if len(hits) > 1:
        print(f"Several matching apps: {', '.join(hits)}; using {hits[0]} (pick one with --bundle).")
    return hits[0]


async def sessions_root(afc) -> str:
    # VendDocuments exposes the Documents folder at /Documents; some iOS versions root it at /.
    for root in ("/Documents/sessions", "/sessions"):
        try:
            if await afc.isdir(root):
                return root
        except AfcFileNotFoundError:
            continue
    sys.exit("The app has no sessions folder yet: record something first.")


async def remote_files(afc, path: str) -> list[tuple[str, int]]:
    """(path relative to `path`, size) for every file below `path`."""
    out = []
    async for dirpath, _dirs, files in afc.walk(path):
        for f in files:
            full = f"{dirpath.rstrip('/')}/{f}"
            st = await afc.stat(full)
            out.append((full[len(path):].lstrip("/"), int(st.get("st_size", 0))))
    return out


async def read_json(afc, path: str) -> dict:
    try:
        return json.loads(await afc.get_file_contents(path))
    except Exception:
        return {}


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{int(n)} B"
        n /= 1024
    return f"{n:.1f} GB"


async def run(a) -> int:
    lockdown = await create_using_usbmux(serial=a.udid)
    name = lockdown.all_values.get("DeviceName", "iPhone")
    print(f"Connected: {name} ({lockdown.all_values.get('ProductType', '?')}, iOS {lockdown.product_version})")
    bundle = await find_bundle(lockdown, a.bundle)
    print(f"App: {bundle}")

    dest = Path(a.dest).expanduser()
    new_sessions: list[Path] = []
    async with await HouseArrestService.create(lockdown=lockdown, bundle_id=bundle, documents_only=True) as afc:
        root = await sessions_root(afc)
        names = sorted(n for n in await afc.listdir(root) if n not in (".", ".."))
        if a.session:
            missing = set(a.session) - set(names)
            if missing:
                print(f"Not on the phone: {', '.join(sorted(missing))}")
            names = [n for n in names if n in a.session]
        if not names:
            print("No sessions on the phone.")
            return 0

        for n in names:
            meta = await read_json(afc, f"{root}/{n}/session.json")
            files = await remote_files(afc, f"{root}/{n}")
            total = sum(s for _, s in files)
            complete = bool(meta.get("complete"))
            local = dest / n
            todo = [(p, s) for p, s in files
                    if not (local / p).is_file() or (local / p).stat().st_size != s]
            state = "complete" if complete else "INCOMPLETE"
            if a.list:
                have = "copied" if not todo else ("partly copied" if local.exists() else "new")
                print(f"  {n:40s} {meta.get('mode', '?'):15s} {human(total):>10s}  {state:10s} {have}")
                continue
            if not complete and not a.include_incomplete:
                print(f"  skip {n}: still recording or not finished (use --include-incomplete)")
                continue
            if not todo:
                print(f"  {n}: already copied")
                continue
            print(f"  {n}: {len(todo)} files, {human(sum(s for _, s in todo))}")
            t0 = time.time()
            done = 0
            for p, s in todo:
                target = local / p
                target.parent.mkdir(parents=True, exist_ok=True)
                tmp = target.with_name(target.name + ".part")
                await afc.pull(f"{root}/{n}/{p}", str(tmp), progress_bar=False)
                if tmp.stat().st_size != s:
                    tmp.unlink(missing_ok=True)
                    print(f"    size mismatch on {p}; run again to retry")
                    continue
                tmp.replace(target)
                done += s
                rate = done / max(time.time() - t0, 1e-3)
                print(f"    {p}  ({human(done)} at {human(rate)}/s)", flush=True)
            new_sessions.append(local)

    if a.list:
        return 0
    print(f"Copied {len(new_sessions)} session(s) to {dest}")
    if a.convert and new_sessions:
        out = Path(a.convert).expanduser()
        for s in new_sessions:
            print(f"Converting {s.name} -> {out / s.name}")
            r = subprocess.run([sys.executable, str(HERE / "convert.py"), str(s), "--out", str(out / s.name)])
            if r.returncode != 0:
                print(f"  convert.py failed for {s.name} (exit {r.returncode})")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", default="~/captures", help="where sessions are copied (default ~/captures)")
    ap.add_argument("--convert", metavar="OUT", help="run convert.py on each newly copied session into OUT/<session>")
    ap.add_argument("--list", action="store_true", help="only list the sessions on the phone")
    ap.add_argument("--session", nargs="+", help="copy only these session folder names")
    ap.add_argument("--include-incomplete", action="store_true", help="also copy sessions without complete=true")
    ap.add_argument("--bundle", help="app bundle id (default: found automatically)")
    ap.add_argument("--udid", help="device serial/UDID when several phones are plugged in")
    a = ap.parse_args()
    try:
        return asyncio.run(run(a))
    except KeyboardInterrupt:
        print("\nInterrupted; run the same command again to resume.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
