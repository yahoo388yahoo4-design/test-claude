#!/usr/bin/env bash
# Compiles NavCore.swift (the platform-independent navigation core) with the tests and runs them.
# Needs swiftc on PATH (Linux toolchain from swift.org works) or SWIFTC=/path/to/swiftc.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="${TMPDIR:-/tmp}/r2s_navtests"
"${SWIFTC:-swiftc}" -O "$here/../R2SCapture/Nav/NavCore.swift" "$here/../R2SCapture/Nav/NavMapPrior.swift" "$here/../R2SCapture/Nav/NavPerception.swift" "$here/../R2SCapture/Nav/NavSettings.swift" "$here/main.swift" -o "$out"
"$out"
