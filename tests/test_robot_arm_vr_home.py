from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party" / "robot_arm_vr" / "src"))

from rpo_teleop.jetson_link import home_reached  # noqa: E402


def test_home_reached_requires_every_joint_inside_tolerance() -> None:
    home = np.array([0.1, -0.8])
    tol = np.radians(2.0)

    assert home_reached(home + np.array([tol * 0.5, -tol * 0.99]), home)
    assert not home_reached(home + np.array([0.0, tol * 1.01]), home)


def test_home_reached_rejects_bad_feedback() -> None:
    home = np.array([0.1, -0.8])

    assert not home_reached(np.array([0.1]), home)
    assert not home_reached(np.array([0.1, np.nan]), home)


def test_superarm_home_is_lerobot_calibration_zero() -> None:
    config_path = (
        ROOT / "third_party" / "robot_arm_vr" / "config" /
        "superarm_j1_j2_jetson.json"
    )
    config = json.loads(config_path.read_text())

    assert config["home_q"] == [0.0, 0.0]
