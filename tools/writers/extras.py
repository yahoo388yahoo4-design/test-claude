"""Copy what the target formats have no slot for (IMU, GPS, barometer, planes, world map, extra cameras,
per-frame exposure, all-frame poses) into <out>/extras/."""
from __future__ import annotations

import json
import shutil
from pathlib import Path

from common import arkit_c2w_to_zup_cv


def write_extras(ep, out: Path, log=print) -> dict:
    ex = Path(out) / "extras"
    ex.mkdir(parents=True, exist_ok=True)
    copied = []
    for name, src in ep.extras().items():
        dst = ex / name
        if src.is_dir():
            shutil.copytree(src, dst, dirs_exist_ok=True)
        else:
            shutil.copy2(src, dst)
        copied.append(name)
    with open(ex / "poses_all_frames.csv", "w") as fh:
        fh.write("t,tracking," + ",".join(f"arkit_c2w_{r}{c}" for r in range(3) for c in range(4)) + "," +
                 ",".join(f"zup_cv_c2w_{r}{c}" for r in range(3) for c in range(4)) + ",fx,fy,cx,cy,w,h,exp,iso\n")
        for f in ep.frames:
            if f.T is None:
                continue
            Tz = arkit_c2w_to_zup_cv(f.T)
            vals = list(f.T[:3].reshape(-1)) + list(Tz[:3].reshape(-1))
            fh.write(f"{f.t:.6f},{f.tracking}," + ",".join(f"{v:.7f}" for v in vals) +
                     f",{f.K[0, 0]:.4f},{f.K[1, 1]:.4f},{f.K[0, 2]:.4f},{f.K[1, 2]:.4f},{f.w},{f.h},"
                     f"{f.meta.get('exp') if f.meta.get('exp') is not None else ''},"
                     f"{f.meta.get('iso') if f.meta.get('iso') is not None else ''}\n")
    summ = ep.summary()
    (ex / "source_summary.json").write_text(json.dumps(summ, indent=2))
    info = {"extras": str(ex), "copied": copied}
    log(json.dumps(info))
    return info
