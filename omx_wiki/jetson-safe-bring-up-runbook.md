---
title: "Jetson safe bring-up runbook"
tags: ["jetson", "runbook", "safety", "can", "quest", "deployment"]
created: 2026-08-02T05:14:44.518Z
updated: 2026-08-02T05:14:44.518Z
sources: []
links: ["jetson-bring-up-status-and-deployment-gate.md"]
category: convention
confidence: medium
schemaVersion: 1
---

# Jetson safe bring-up runbook

# Jetson safe bring-up runbook

Prerequisite context: [[jetson-bring-up-status-and-deployment-gate]]. Keep the real arm torque-disabled until the final explicit motion gate.

## 1. Make the software environment reproducible
From the repo root, run:
```bash
source "$HOME/.local/bin/env"
uv sync --locked --extra hardware --extra test
uv run pytest -q
uv run ruff check . --exclude third_party --exclude .venv
```
Success is a completed install plus passing tests/lint. If the install fails at ARM/CUDA/PyTorch, capture the exact error and select a JetPack-compatible PyTorch path before proceeding; do not mix it ad hoc with the lockfile.

## 2. Attach and identify all required hardware (no commands sent)
```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
v4l2-ctl --list-devices
id -nG
```
Record stable `/dev/serial/by-id/...` paths where available. Expected logical roles: CANable `/dev/ttyACM0`, AmazingHand serial port (currently absent), and two camera nodes (currently absent). Ensure the login user is in `dialout`; re-login after group changes.

## 3. Quest-only HTTPS and tracking test (no robot connection)
Generate `cert.pem`/`key.pem` for the Jetson LAN IP exactly as documented in `README.md`, then run:
```bash
uv run python scripts/quest_check.py --lan-ip <JETSON_LAN_IP>
```
Open the printed URL on Quest, accept the certificate warning, enter VR, and verify right-controller tracking. This is safe because it does not open the robot bridge or send motor commands.

## 4. Passive motor and calibration confirmation
Use the feedback-only CAN probe with the actual CANable port and IDs 1,2. Verify command-to-feedback mapping `1->17` and `2->18`. Then inspect the existing calibration file before changing it. If a recalibration is required, use `roboparty-calibrate-two-motors --lerobot`; it disables torque and writes a new local calibration result. Do not issue hardware-zero/flash changes.

## 5. Integrated pre-motion check
Run the two-motor Quest check and validate both camera frames and AmazingHand serial communication using the identified devices. A passing result must include Quest tracking, CAN feedback, expected two-axis calibration, hand communication, and both camera images. Any missing device is a stop condition.

## 6. Explicit motion gate, then local recording
Only after the prior checks pass and an operator has chosen to permit movement: begin with a physically safe, supervised low-speed clutch test. Verify grip release/tracking loss releases arm torque and process exit releases arm/hand torque. Then record two short local episodes with `--dataset.push_to_hub=false`. Validate action/state order is `motor_0.pos`, `motor_1.pos`, `right_hand_grasp.pos` before any Hub upload or model training.

## Current next action
Complete step 1. It is the sole blocking prerequisite detectable without moving hardware.
