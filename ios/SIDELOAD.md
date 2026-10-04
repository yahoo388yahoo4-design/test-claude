# Installing R2S Capture without a Mac or a paid developer account

The GitHub Actions workflow builds an **unsigned** `R2SCapture-unsigned.ipa`. A sideloading tool
re-signs it with your own (free) Apple ID and installs it on the iPhone.

> **Untested by us.** We couldn't run any of these routes (no iPhone or Windows/Mac here). The steps
> follow the tools' own documentation as of 2026-10.

Limits with a free Apple ID: the app **expires after 7 days** and has to be refreshed; a free ID can
have **at most 3 sideloaded apps at once** (AltStore itself takes one, leaving 2); the iPhone needs
**Developer Mode** turned on. A paid Apple Developer Program account ($99/yr) avoids all of this through
TestFlight (see README §5).

## 0. Get the .ipa

GitHub → the repo → **Actions** → the latest green **ios-capture** run → **Artifacts** →
`R2SCapture-unsigned-ipa-<n>`. The download is a `.zip` containing `R2SCapture-unsigned.ipa`.

* For AltStore (installs from the phone): download the artifact in **Safari on the iPhone** while signed
  in to GitHub, open it in **Files** (tap the zip to unzip), and the `.ipa` sits in Downloads. Or move it
  over with AirDrop / iCloud Drive.
* For Sideloadly (installs from the computer): download and unzip it on the computer.

## 1. Main route: AltStore Classic + AltServer

1. **Computer:** install **AltServer** from <https://altstore.io>.
   * Windows: first install **iTunes and iCloud from Apple's website** (the direct-download versions,
     *not* the Microsoft Store versions; AltServer needs their device and authentication libraries).
   * macOS: install AltServer and enable its Mail plug-in when it asks.
2. Connect the iPhone by **USB cable**, unlock it and tap **Trust This Computer**.
   On Windows, open iTunes, select the phone and tick **"Sync with this iPhone over Wi-Fi"** (this is
   what lets AltStore refresh apps in the background later).
3. AltServer icon (Windows tray / macOS menu bar) → **Install AltStore** → pick your iPhone → sign in with
   your Apple ID (a secondary Apple ID is fine).
4. **iPhone:**
   * Settings → General → **VPN & Device Management** → tap your Apple ID under Developer App → **Trust**.
   * Settings → Privacy & Security → **Developer Mode** → On → restart, then confirm "Turn On".
5. Open **AltStore** → **My Apps** → **+** (top left) → pick `R2SCapture-unsigned.ipa` from Files.
   AltStore signs and installs it ("R2S Capture" appears on the home screen).
6. **Refresh within 7 days:** this happens automatically while AltServer runs on a computer on the same
   Wi-Fi; or open AltStore → My Apps → **Refresh All**. A new build installs the same way (step 5) and
   replaces the old one.

On Linux, run **AltServer-Linux** (github.com/NyaMisty/AltServer-Linux) with `usbmuxd` and an anisette
server instead of the Windows/macOS AltServer; the iPhone-side steps are the same. You can also install
**SideStore** (sidestore.io): after a one-time pairing from a computer it refreshes apps on the phone
itself through a VPN loopback, with no computer needed afterwards.

## 2. Alternative: Sideloadly (Windows / macOS)

1. Install **Sideloadly** from <https://sideloadly.io> (on Windows, again iTunes + iCloud from Apple's
   website, not the Store).
2. USB-connect and trust the computer, then drag `R2SCapture-unsigned.ipa` into Sideloadly, enter your
   Apple ID, and press **Start**.
3. On the iPhone: trust the developer profile (Settings → General → VPN & Device Management) and turn on
   Developer Mode (Settings → Privacy & Security), as in steps 4 above.
4. Expiry is again 7 days. Sideloadly can auto-refresh over Wi-Fi if you enable that in its advanced
   options and leave it running.

## 3. First launch

Allow Camera, Motion & Fitness, Location (optional, for GPS) and Local Network (only needed for Wi-Fi
upload). Captures appear in **Files → On My iPhone → R2S Capture → sessions**, or upload them to
`tools/receiver.py` (Settings → Upload receiver, e.g. `http://192.168.1.20:8765#mytoken`).

If the CI build fails, nothing installs. Check the `xcodebuild-log` artifact of the run; this app hasn't
been compiled before its first CI run, so a round of compile fixes is expected.
