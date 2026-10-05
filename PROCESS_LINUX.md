# From iPhone to ARKitScenes / LiteReality on Linux

Works for Mode A (RGB-D) and Mode B (RGB-D + RoomPlan) sessions. Tested formats: see `tools/roundtrip_test.py`.

## 1. One-time setup on the Linux computer

```bash
cd ~
git clone https://github.com/yahoo388yahoo4-design/test-claude r2s-capture
python3 -m venv ~/r2s-venv
~/r2s-venv/bin/pip install numpy opencv-python scipy pymobiledevice3
sudo apt install usbmuxd
```

## 2. Copy sessions off the phone

**Quickest: one command over USB (nothing to tap in the app).** Plug in the iPhone, unlock it, then:

```bash
cd ~/r2s-capture && git pull                    # get the latest tools
~/r2s-venv/bin/python ~/r2s-capture/tools/pull_iphone.py --convert ~/converted
```

It finds the app on the phone, copies every finished session that isn't already in `~/captures`, and
converts each new one into `~/converted/<session>/` (ARKitScenes + LiteReality). Leave out `--convert` to
only copy. Other options: `--list` (show what's on the phone), `--session NAME`, `--dest DIR`. Re-running
is safe and resumes an interrupted copy. First time only: tap **Trust** on the phone when asked.

**Option A: Wi-Fi (phone and computer on the same network).** On the computer:

```bash
hostname -I                                   # first address = this computer's IP
~/r2s-venv/bin/python ~/r2s-capture/tools/receiver.py --root ~/captures --port 8765 --token mysecret \
    --convert ~/converted
```

If Ubuntu's firewall is on: `sudo ufw allow 8765/tcp`.
In the app: **Settings → Upload receiver** = `http://<computer-ip>:8765#mysecret`. Then in the sessions
list, swipe left on a session → **Upload**. Sessions land in `~/captures/<session>/`, and with
`--convert` both formats are written to `~/converted/<session>/` automatically (log in
`~/captures/<session>/convert.log`).

**Option B: USB cable.**

```bash
sudo apt install ifuse
ifuse --list-apps | grep -i r2s               # note the bundle id (Plume may have changed it)
mkdir -p ~/iphone && ifuse --documents <bundle-id> ~/iphone
cp -r ~/iphone/sessions/* ~/captures/
fusermount -u ~/iphone
```

## 3. Convert (if you didn't use `--convert`)

```bash
cd ~/r2s-capture/tools
~/r2s-venv/bin/python convert.py ~/captures/<session> --to arkitscenes litereality --out ~/converted/<session>
~/r2s-venv/bin/python validate.py ~/converted/<session>/arkitscenes/<video_id>     # optional check
```

Useful flags: `--video-id 48000001` (numeric ARKitScenes id), `--wide` (full-res frames),
`--vga` (640x480 at 30 Hz), `--traj-hz 10`.

Output:

* `~/converted/<session>/arkitscenes/<video_id>/` with `lowres_wide/`, `lowres_depth/`, `confidence/`,
  `lowres_wide_intrinsics/`, `lowres_wide.traj`, the mesh, `metadata.csv`, and for Mode B the
  RoomPlan boxes as `<video_id>_3dod_annotation.json`.
* `~/converted/<session>/litereality/<session>/` in the 3D Scanner App "All Data" layout
  (`frame_*.jpg/json`, `depth_*.png`, `conf_*.png`, `roomplan/room.usdz`), which LiteReality reads directly.
* `extras/` next to each: IMU, GPS, barometer, everything the formats have no slot for.

## 4. Crash logs (for app bugs)

```bash
sudo apt install libimobiledevice-utils
idevicecrashreport -e ~/crashes && ls -t ~/crashes | grep -i r2s
```

Or on the phone: Settings → Privacy & Security → Analytics & Improvements → Analytics Data, entries
starting with `R2SCapture`. Builds from 4 Oct 2026 onward also write `crash_logs/` into the app's
Documents folder (Files app → On My iPhone → R2S Capture).
