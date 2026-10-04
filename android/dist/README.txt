Debug APK for Real2Sim Capture (Android), split into <=8 MB parts.
  cat real2sim-capture-debug.apk.part* > real2sim-capture-debug.apk
  sha256sum -c real2sim-capture-debug.apk.sha256
  adb install -r real2sim-capture-debug.apk
