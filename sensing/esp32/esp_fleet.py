#!/usr/bin/env python3
"""Set up 6 ESP32-S3 boards as RuView CSI sensing nodes (firmware: ruvnet/RuView firmware/esp32-csi-node).

One config file (fleet.json) describes the fleet; this script downloads and verifies the firmware
bundle, flashes each board, writes its WiFi / node id / TDM slot with RuView's own provision.py, and
checks that every node is streaming.

  pip install 'esptool>=5.0' esp-idf-nvs-partition-gen pyserial
  python esp_fleet.py init --ssid MyWiFi --password-file ~/.wifi-pass      # writes fleet.json
  python esp_fleet.py fetch                                                 # firmware + provision.py, SHA-256 checked
  python esp_fleet.py setup                                                 # plug boards in one at a time
  python esp_fleet.py monitor                                               # are all 6 streaming?

Single steps: `flash --node 3 --port /dev/ttyACM0`, `provision --node 3 --port ...`, `ports`.
`--dry-run` (before the subcommand) prints every command instead of running it.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tarfile
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_FLEET = HERE / "fleet.json"
RELEASE = "v0.8.14-esp32"
REPO_URL = "https://github.com/ruvnet/RuView"
# Release SHA256SUMS.txt for v0.8.14-esp32 (checked 2026-10-15); a downloaded manifest must agree.
PINNED_SHA = {
    "v0.8.14-esp32": {
        "esp32-csi-node-firmware-4mb.tar.gz": "369f559937a5cd02454f3897e88e0c0a8f0e2383e7848fc48c08dbbc8a5e152c",
        "esp32-csi-node-firmware-8mb.tar.gz": "31b25068aad34a119047aaf9b6c2580693fe3017bf4ad4c2f682b731fef01474",
    }
}
# Flash layout per bundle (README "Quick Start" / partitions_*.csv): same offsets, different file names.
VARIANTS = {
    # Headless build: right for bare boards (DevKitC-1, XIAO, SuperMini). Fits 4 MB and 8 MB flash.
    "4mb": {"archive": "esp32-csi-node-firmware-4mb.tar.gz", "flash_size": "4MB",
            "files": [("0x0", "bootloader.bin"), ("0x8000", "partition-table-4mb.bin"),
                      ("0xf000", "ota_data_initial.bin"), ("0x20000", "esp32-csi-node-4mb.bin")]},
    # Display build: only for 8 MB boards that have an LCD. On bare boards its panel probe can
    # false-positive and drop CSI to 0 pps (firmware README, sdkconfig.defaults.devkitc).
    "8mb": {"archive": "esp32-csi-node-firmware-8mb.tar.gz", "flash_size": "8MB",
            "files": [("0x0", "bootloader.bin"), ("0x8000", "partition-table.bin"),
                      ("0xf000", "ota_data_initial.bin"), ("0x20000", "esp32-csi-node.bin")]},
}
DRY = False


# ------------------------------------------------------------------------------------ helpers
def run(cmd: list[str], check: bool = True, capture: bool = False):
    print("  $ " + " ".join(f'"{c}"' if " " in c else c for c in cmd))
    if DRY:
        return subprocess.CompletedProcess(cmd, 0, "", "")
    return subprocess.run(cmd, check=check, text=True, capture_output=capture)


def cache_dir(release: str) -> Path:
    base = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "r2s-esp32" / release
    base.mkdir(parents=True, exist_ok=True)
    return base


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_sums(text: str) -> dict[str, str]:
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2 and len(parts[0]) == 64:
            out[parts[-1].lstrip("*").lstrip("./")] = parts[0].lower()
    return out


def download(url: str, dst: Path):
    print(f"  downloading {url}")
    tmp = dst.with_suffix(dst.suffix + ".part")
    with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
        shutil.copyfileobj(r, f)
    tmp.replace(dst)


def lan_ip() -> str:
    """This machine's LAN address (no packet is sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def load_fleet(path: Path) -> dict:
    if not path.exists():
        sys.exit(f"{path} not found: run `esp_fleet.py init` first")
    return json.loads(path.read_text())


def save_fleet(path: Path, fleet: dict):
    path.write_text(json.dumps(fleet, indent=1) + "\n")


def serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial missing: pip install pyserial")
    return sorted(p.device for p in list_ports.comports()
                  if p.vid is not None or "USB" in (p.description or "") or "ACM" in p.device)


# ------------------------------------------------------------------------------------ commands
def cmd_init(a):
    if a.fleet.exists() and not a.force:
        sys.exit(f"{a.fleet} exists (use --force to overwrite)")
    pw = Path(a.password_file).expanduser() if a.password_file else None
    if pw and pw.exists() and os.name == "posix" and pw.stat().st_mode & 0o077:
        print(f"note: {pw} is readable by others; provision.py requires `chmod 600 {pw}`")
    target = lan_ip() if a.target_ip == "auto" else a.target_ip
    nodes = []
    for i in range(1, a.nodes + 1):
        nodes.append({"node_id": i, "label": f"esp-{i}", "tdm_slot": i - 1 if a.tdm else None,
                      "zone": a.zone, "mac": None, "port": None, "flashed": None, "provisioned": None})
    fleet = {"version": 1, "release": a.release, "variant": a.variant, "chip": "esp32s3",
             "wifi": {"ssid": a.ssid, "password_file": str(pw) if pw else None},
             "target": {"ip": target, "port": a.target_port},
             "tdm_total": a.nodes if a.tdm else None, "edge_tier": a.edge_tier,
             "ota_psk_file": a.ota_psk_file, "nodes": nodes}
    save_fleet(a.fleet, fleet)
    print(f"wrote {a.fleet}: {a.nodes} nodes -> {target}:{a.target_port}, firmware {a.release} ({a.variant}), "
          f"TDM {'on' if a.tdm else 'off'}")
    if target.startswith("127."):
        print("warning: could not find a LAN address; pass --target-ip <this computer's LAN IP>")


def fetch(release: str, variant: str) -> tuple[Path, Path]:
    """Download + verify the firmware bundle and provision.py. Returns (bundle dir, provision.py)."""
    v = VARIANTS[variant]
    cdir = cache_dir(release)
    arc = cdir / v["archive"]
    sums_p = cdir / "SHA256SUMS.txt"
    base = f"{REPO_URL}/releases/download/{release}"
    if not DRY:
        if not sums_p.exists():
            download(f"{base}/SHA256SUMS.txt", sums_p)
        if not arc.exists():
            download(f"{base}/{v['archive']}", arc)
        sums = parse_sums(sums_p.read_text())
        want = sums.get(v["archive"])
        pinned = PINNED_SHA.get(release, {}).get(v["archive"])
        if pinned and want and pinned != want:
            sys.exit(f"release SHA256SUMS disagrees with the pinned checksum for {v['archive']}: refusing")
        got = sha256(arc)
        if got != (pinned or want):
            arc.unlink()
            sys.exit(f"{v['archive']}: SHA-256 mismatch (got {got}); deleted, run fetch again")
    out = cdir / variant
    if not DRY:
        if not (out / "SHA256SUMS.txt").exists():
            out.mkdir(exist_ok=True)
            with tarfile.open(arc) as t:
                for m in t.getmembers():  # flat archive of regular files; refuse anything else
                    name = Path(m.name).name
                    if not m.isfile() or not name or name != m.name.lstrip("./"):
                        continue
                    f = t.extractfile(m)
                    (out / name).write_bytes(f.read())
        inner = parse_sums((out / "SHA256SUMS.txt").read_text())
        for _, fname in v["files"]:
            if inner.get(fname) != sha256(out / fname):
                sys.exit(f"{fname}: does not match the bundle's SHA256SUMS.txt")
    prov = cdir / "provision.py"
    if not DRY and not prov.exists():
        download(f"https://raw.githubusercontent.com/ruvnet/RuView/{release}/firmware/esp32-csi-node/provision.py", prov)
    print(f"firmware {release} {variant}: {out} (verified)\nprovision.py: {prov}")
    return out, prov


def cmd_fetch(a):
    f = load_fleet(a.fleet) if a.fleet.exists() else {"release": RELEASE, "variant": "4mb"}
    fetch(a.release or f["release"], a.variant or f["variant"])


def read_mac(port: str, chip: str) -> str | None:
    if DRY:
        return None
    r = run([sys.executable, "-m", "esptool", "--chip", chip, "--port", port, "read-mac"], check=False, capture=True)
    for line in (r.stdout or "").splitlines():
        if line.strip().startswith("MAC:"):
            return line.split("MAC:")[1].strip().lower()
    return None


def flash(fleet: dict, node: dict, port: str, erase: bool):
    bdir, _ = fetch(fleet["release"], fleet["variant"])
    v = VARIANTS[fleet["variant"]]
    esptool = [sys.executable, "-m", "esptool", "--chip", fleet["chip"], "--port", port, "--baud", "460800"]
    if erase:
        run(esptool + ["erase-flash"])
    args = []
    for off, fname in v["files"]:
        args += [off, str(bdir / fname)]
    run(esptool + ["write-flash", "--flash-mode", "dio", "--flash-size", v["flash_size"]] + args)
    node["flashed"] = time.strftime("%Y-%m-%dT%H:%M:%S")


def provision(fleet: dict, node: dict, port: str, first: bool):
    _, prov = fetch(fleet["release"], fleet["variant"])
    w = fleet["wifi"]
    cmd = [sys.executable, str(prov), "--port", port, "--chip", fleet["chip"], "--ssid", w["ssid"],
           "--target-ip", fleet["target"]["ip"], "--target-port", str(fleet["target"]["port"]),
           "--node-id", str(node["node_id"]), "--edge-tier", str(fleet.get("edge_tier", 2))]
    if w.get("password_file"):
        cmd += ["--password-file", w["password_file"]]
    if fleet.get("tdm_total") and node.get("tdm_slot") is not None:
        cmd += ["--tdm-slot", str(node["tdm_slot"]), "--tdm-total", str(fleet["tdm_total"])]
    if node.get("zone"):
        cmd += ["--zone", node["zone"]]
    if fleet.get("ota_psk_file"):
        cmd += ["--ota-psk-file", fleet["ota_psk_file"]]
    if first:
        cmd += ["--reset"]
    run(cmd)
    node["provisioned"] = time.strftime("%Y-%m-%dT%H:%M:%S")


def get_node(fleet: dict, nid: int) -> dict:
    for n in fleet["nodes"]:
        if n["node_id"] == nid:
            return n
    sys.exit(f"node {nid} is not in the fleet file")


def cmd_flash(a):
    fleet = load_fleet(a.fleet)
    n = get_node(fleet, a.node)
    flash(fleet, n, a.port, a.erase)
    n["port"] = a.port
    save_fleet(a.fleet, fleet)


def cmd_provision(a):
    fleet = load_fleet(a.fleet)
    n = get_node(fleet, a.node)
    provision(fleet, n, a.port, a.reset)
    save_fleet(a.fleet, fleet)


def wait_for_new_port(before: set[str], timeout: float = 120) -> str:
    t0 = time.time()
    while time.time() - t0 < timeout:
        new = sorted(set(serial_ports()) - before)
        if new:
            time.sleep(1.0)  # let the USB-CDC port settle
            return new[0]
        time.sleep(0.5)
    sys.exit("no new serial port appeared (try another USB cable: charge-only cables have no data lines)")


def cmd_setup(a):
    fleet = load_fleet(a.fleet)
    if not fleet["wifi"].get("ssid"):
        sys.exit("fleet.json has no WiFi SSID")
    fetch(fleet["release"], fleet["variant"])
    todo = [n for n in fleet["nodes"] if n["node_id"] >= a.start and (a.only is None or n["node_id"] == a.only)]
    for n in todo:
        print(f"\n=== node {n['node_id']} of {len(fleet['nodes'])} ===")
        if a.port:
            port = a.port
            if not DRY:
                input(f"Connect the board for node {n['node_id']} to {port}, then press Enter… ")
        elif DRY:
            port = "/dev/ttyACM0"
        else:
            before = set(serial_ports())
            print(f"Unplug other ESP boards. Plug in the board for node {n['node_id']} (USB port marked USB/OTG on "
                  "S3 DevKits); if it doesn't show up, hold BOOT while plugging in.")
            port = wait_for_new_port(before)
        print(f"  board on {port}")
        mac = read_mac(port, fleet["chip"])
        if mac:
            dup = [m for m in fleet["nodes"] if m.get("mac") == mac and m["node_id"] != n["node_id"]]
            if dup:
                sys.exit(f"this board ({mac}) is already node {dup[0]['node_id']}; plug in a different one")
            n["mac"] = mac
        flash(fleet, n, port, erase=not a.no_erase)
        provision(fleet, n, port, first=True)
        n["port"] = port
        save_fleet(a.fleet, fleet)
        print(f"  done: label this board '{n['label']}' and stick tag {n['node_id']} on it (tags/esp_tags_A4.pdf).")
        if not DRY and n is not todo[-1] and not a.port:
            input("  Unplug it (or leave it powered elsewhere) and press Enter for the next board… ")
    print("\nAll boards set up. Power them where they will live, then run: esp_fleet.py monitor")


# ------------------------------------------------------------------------------------ monitor
CSI_MAGIC, VITALS_MAGIC, WASM_MAGIC = 0xC5110001, 0xC5110002, 0xC5110004


def parse_packet(b: bytes) -> dict | None:
    """ADR-018 CSI frame / 32-byte vitals packet (firmware README "Wire Protocols"), little endian."""
    if len(b) < 5:
        return None
    magic = struct.unpack_from("<I", b, 0)[0]
    if magic == CSI_MAGIC and len(b) >= 20:
        node, n_ant, n_sub, freq, seq, rssi, noise = struct.unpack_from("<BBHIIbb", b, 4)
        return {"type": "csi", "node": node, "n_ant": n_ant, "n_sub": n_sub, "freq_mhz": freq, "seq": seq,
                "rssi": rssi, "noise": noise, "iq_bytes": len(b) - 20}
    if magic == VITALS_MAGIC and len(b) >= 32:
        node, flags, br, hr, rssi, npers = struct.unpack_from("<BBHIbB", b, 4)
        motion, pscore, ts = struct.unpack_from("<ffI", b, 16)
        return {"type": "vitals", "node": node, "presence": bool(flags & 1), "fall": bool(flags & 2),
                "motion": bool(flags & 4), "breathing_bpm": br / 100, "heart_bpm": hr / 10000, "rssi": rssi,
                "persons": npers, "motion_energy": motion, "presence_score": pscore, "uptime_ms": ts}
    if magic == WASM_MAGIC:
        return {"type": "wasm", "node": b[4]}
    return None


def _bpm(x) -> str:
    """0 means the node has no estimate yet (needs a still person in view)."""
    return f"{x:.1f}" if x else "-"


def cmd_monitor(a):
    expected = set()
    if a.fleet.exists():
        expected = {n["node_id"] for n in load_fleet(a.fleet)["nodes"]}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((a.bind, a.port))
    except OSError as e:
        sys.exit(f"cannot listen on UDP {a.port}: {e} (stop esp_view.py or the RuView sensing server first)")
    s.settimeout(0.5)
    stats: dict[int, dict] = {}
    t0 = last = time.time()
    print(f"listening on UDP {a.bind}:{a.port} … Ctrl-C to stop")
    try:
        while a.seconds <= 0 or time.time() - t0 < a.seconds:
            try:
                b, addr = s.recvfrom(4096)
                p = parse_packet(b)
                if p:
                    st = stats.setdefault(p["node"], {"csi": 0, "vitals": 0, "ip": addr[0], "last": {}})
                    st[p["type"]] = st.get(p["type"], 0) + 1
                    st["ip"] = addr[0]
                    st["last"][p["type"]] = p
            except socket.timeout:
                pass
            if time.time() - last >= a.every:
                dt = time.time() - last
                last = time.time()
                print(time.strftime("\n%H:%M:%S"))
                print(" node  ip               csi/s  rssi  ch-MHz  sub  presence  motion  persons  breath/min  heart/min")
                for nid in sorted(set(stats) | expected):
                    st = stats.get(nid)
                    if not st:
                        print(f" {nid:>4}  -- no packets --")
                        continue
                    c = st["last"].get("csi", {})
                    v = st["last"].get("vitals", {})
                    pps = st["csi"] / dt
                    flag = "  <- below 20/s floor" if pps < 20 else ""
                    print(f" {nid:>4}  {st['ip']:<15} {pps:6.1f}  {c.get('rssi', ''):>4}  {c.get('freq_mhz', ''):>6}  "
                          f"{c.get('n_sub', ''):>3}  {str(v.get('presence', '-')):>8}  {str(v.get('motion', '-')):>6}  "
                          f"{v.get('persons', '-'):>7}  {_bpm(v.get('breathing_bpm')):>10}  {_bpm(v.get('heart_bpm')):>9}{flag}")
                    st["csi"] = st["vitals"] = 0
    except KeyboardInterrupt:
        pass


def cmd_ports(a):
    for p in serial_ports():
        print(p)


def main(argv=None):
    global DRY
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", type=Path, default=DEFAULT_FLEET, help="fleet config (default: fleet.json here)")
    ap.add_argument("--dry-run", action="store_true", help="print commands, change nothing")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("init", help="write fleet.json")
    p.add_argument("--ssid", required=True, help="2.4 GHz WiFi network the nodes join (ESP32-S3 has no 5 GHz)")
    p.add_argument("--password-file", help="file holding the WiFi password (chmod 600); never stored here")
    p.add_argument("--target-ip", default="auto", help="computer running the RuView sensing server (default: this one)")
    p.add_argument("--target-port", type=int, default=5005)
    p.add_argument("--nodes", type=int, default=6)
    p.add_argument("--variant", choices=sorted(VARIANTS), default="4mb",
                   help="4mb = headless build for bare boards (default); 8mb = boards with a display")
    p.add_argument("--release", default=RELEASE)
    p.add_argument("--no-tdm", dest="tdm", action="store_false", help="don't assign TDM slots (default: slot = id-1)")
    p.add_argument("--edge-tier", type=int, choices=[0, 1, 2], default=2)
    p.add_argument("--zone", help="room / zone name written to every node")
    p.add_argument("--ota-psk-file", help="64-hex OTA key file (see firmware README 'OTA key provisioning')")
    p.add_argument("--force", action="store_true")
    p.set_defaults(fn=cmd_init)

    p = sub.add_parser("fetch", help="download + verify firmware bundle and provision.py")
    p.add_argument("--release")
    p.add_argument("--variant", choices=sorted(VARIANTS))
    p.set_defaults(fn=cmd_fetch)

    p = sub.add_parser("setup", help="flash + provision every board, one at a time")
    p.add_argument("--start", type=int, default=1, help="first node id (resume)")
    p.add_argument("--only", type=int, help="just this node id")
    p.add_argument("--port", help="use this serial port instead of detecting the newly plugged board")
    p.add_argument("--no-erase", action="store_true", help="keep existing flash contents (default: erase first)")
    p.set_defaults(fn=cmd_setup)

    p = sub.add_parser("flash", help="flash one board")
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--port", required=True)
    p.add_argument("--erase", action="store_true")
    p.set_defaults(fn=cmd_flash)

    p = sub.add_parser("provision", help="write WiFi / node id / TDM to one board (no reflash)")
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--port", required=True)
    p.add_argument("--reset", action="store_true", help="first-time provisioning of a recycled board")
    p.set_defaults(fn=cmd_provision)

    p = sub.add_parser("monitor", help="listen for the nodes' UDP stream and show rates per node")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--every", type=float, default=2.0)
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl-C)")
    p.set_defaults(fn=cmd_monitor)

    p = sub.add_parser("ports", help="list USB serial ports")
    p.set_defaults(fn=cmd_ports)

    a = ap.parse_args(argv)
    DRY = a.dry_run
    a.fn(a)


if __name__ == "__main__":
    main()
