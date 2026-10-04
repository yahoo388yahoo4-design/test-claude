"""Input format registry. Add a reader with one line: READERS["name"] = module.EpisodeClass."""
from __future__ import annotations

from pathlib import Path

from .r2s import R2SEpisode
from .scanner3d import Scanner3DEpisode
from .strayscanner import StrayEpisode
from .arkitscenes import ARKitScenesEpisode

READERS = {
    "r2s": R2SEpisode,               # this repo's iOS app (FORMAT.md); Android writers use it too
    "strayscanner": StrayEpisode,    # Stray Scanner export folder
    "3dscannerapp": Scanner3DEpisode,  # 3D Scanner App "All Data" export (LiteReality's input)
    "arkitscenes": ARKitScenesEpisode,  # an ARKitScenes raw episode (round-trip tests, re-export)
}
# Optional readers living in other threads' folders register themselves here if present.
try:  # pragma: no cover
    from .android_reader import AndroidEpisode  # noqa: F401
    READERS = {"android": AndroidEpisode, **READERS}  # first: auto-detect must try it before "r2s"
except ImportError:
    pass


def detect(path) -> str:
    path = Path(path)
    for name, cls in READERS.items():
        if cls.detect(path):
            return name
    raise ValueError(f"{path}: no reader recognises this folder (tried {', '.join(READERS)})")


def open_episode(path, fmt: str = "auto"):
    fmt = detect(path) if fmt == "auto" else fmt
    return READERS[fmt](Path(path))
