from __future__ import annotations

import json
import math
import os
from pathlib import Path


def load_j3_gain_profile(
    path: Path,
    *,
    command_id: int,
    feedback_id: int,
) -> tuple[float, float]:
    profile = json.loads(Path(path).read_text())
    if profile.get("schema_version") != 1 or profile.get("profile_type") != "dm4340p_mit_gain":
        raise ValueError(f"unsupported J3 gain profile: {path}")
    if profile.get("motor_name") != "motor_2":
        raise ValueError("J3 gain profile motor_name must be motor_2")
    if int(profile.get("command_id", -1)) != command_id:
        raise ValueError("J3 gain profile command_id does not match calibration")
    if int(profile.get("feedback_id", -1)) != feedback_id:
        raise ValueError("J3 gain profile feedback_id does not match calibration")
    kp = float(profile["kp"])
    kd = float(profile["kd"])
    if not math.isfinite(kp) or not 0.0 <= kp <= 500.0:
        raise ValueError("J3 gain profile kp must be finite and in [0, 500]")
    if not math.isfinite(kd) or not 0.0 <= kd <= 5.0:
        raise ValueError("J3 gain profile kd must be finite and in [0, 5]")
    return kp, kd


def save_j3_gain_profile(profile: dict, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(profile, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)
