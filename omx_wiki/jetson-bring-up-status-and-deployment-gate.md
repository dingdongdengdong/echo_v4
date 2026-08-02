---
title: "Jetson bring-up status and deployment gate"
tags: ["jetson", "deployment", "quest", "amazinghand", "can", "safety"]
created: 2026-08-02T05:15:06.848Z
updated: 2026-08-02T05:15:06.848Z
sources: []
links: ["jetson-safe-bring-up-runbook.md"]
category: environment
confidence: medium
schemaVersion: 1
---

# Jetson bring-up status and deployment gate

# Jetson bring-up status (2026-08-02)

## Target
Run the current standalone Jetson configuration: Quest 2 WebXR + two DM4340 motors through CANable/slcan + AmazingHand (8 servos) + two RGB cameras + LeRobot recording. This is the active path; the 5-axis ROS 2 bridge is future/optional.

## Implemented and versioned
- HEAD `f34da5a` makes the clone self-contained with pinned `third_party/lerobot` and `third_party/AmazingHandControl` submodules.
- The previous commits implemented Quest data collection (`64f1779`) and Jetson-only 2-motor + AmazingHand integration (`3ea95bd`).
- Current submodules: LeRobot `2aba372b` and AmazingHandControl `2a59fd8`.
- Calibration is present in `config/right_arm_two_motor.json`: CAN `slcan`, `/dev/ttyACM0`, 1 Mbps; motor command IDs 1/2 and feedback IDs 17/18.
- Safety/control tools exist: CAN feedback probe, Quest-only check, two-motor Quest check, manual range calibration, preflight, teleoperation, and local dataset recording.

## Host evidence
- Jetson is aarch64, JetPack/L4T R36.4.4, with 236 GB free on `/`.
- `/dev/ttyACM0` is present and belongs to group `dialout`. No `/dev/ttyUSB*` or `/dev/video*` nodes were present at inspection time.
- `uv.lock` is consistent. Its dry-run plans 104 package installs, including `torch==2.11.0+cu128`, CUDA 12.8 packages, LeRobot 0.6.1, Vuer, motorbridge, python-can, and test tools.
- `.venv` currently contains none of the application dependencies (including `torch`, `lerobot`, `pytest`, `python-can`, `pyserial`, and `vuer`). Therefore no CLI/test/runtime validation has been executed on this Jetson.

## Deployment gate
Do not enable torque or start teleoperation yet. First create the managed environment successfully and prove the no-motion checks. Treat any ARM/PyTorch install incompatibility as an environment blocker, not a reason to bypass the lock or substitute unpinned packages.

See [[jetson-safe-bring-up-runbook]] for the ordered next steps.
