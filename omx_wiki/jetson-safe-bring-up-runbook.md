---
title: "Jetson safe bring-up runbook"
tags: ["jetson", "runbook", "safety", "can", "quest2", "ik", "dataset", "korean"]
created: 2026-08-08T08:10:51.173Z
updated: 2026-08-08T18:25:51+09:00
sources: ["https://github.com/huggingface/lerobot/blob/v0.6.1/src/lerobot/scripts/lerobot_record.py"]
links: ["roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko.md", "jetson-bring-up-status-and-deployment-gate.md", "roboparty-xr-ik-dataset-debugging-ko.md", "lerobot-keyboard-rerun-recording-ko.md"]
category: convention
confidence: medium
schemaVersion: 1
---

# Jetson safe bring-up runbook

> 현재 상태: 2026-08-08, `/home/dong/echo_v4`. 목표는 실제 팔을 불필요하게 움직이지 않고 Quest→IK→Robot→dataset 경로를 단계별로 검증하는 것이다.

상세 구조는 [[roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko]], 현재 상태는 [[jetson-bring-up-status-and-deployment-gate]], 오류 분류는 [[roboparty-xr-ik-dataset-debugging-ko]]를 본다.

## 1. 소프트웨어 gate

```bash
cd /home/dong/echo_v4
uv lock --check
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q
.venv/bin/ruff check lerobot_robot_roboparty tests
```

현재 확인 결과는 lock 통과, temp 3축 모델 전환 후 pytest `46 passed`, targeted ruff 통과다. legacy `teleop/`, `televuer/`까지 포함한 전체 ruff 74건은 별도 cleanup 항목이며 핵심 LeRobot IK test의 실패를 의미하지 않는다.

## 2. Git/재현성 gate

현재는 detached HEAD `5131505`와 미커밋 IK 작업이 공존한다. 실제 dataset을 수집하기 전 다음을 남겨야 한다.

```bash
git status --short --branch
git log --oneline --decorate -5
git submodule status
```

수집 dataset 메타데이터에는 반드시 parent commit, LeRobot submodule commit, calibration file checksum, URDF archive checksum을 함께 기록한다. 현재 미커밋 상태를 장시간 수집의 기준으로 삼지 않는다.

## 3. 장치 재열거 — 명령 전송 없음

```bash
ls -l /dev/serial/by-id /dev/ttyACM* /dev/video*
v4l2-ctl --list-devices
id -nG
```

현재 기대 경로:

```text
CANable2    /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2070388B3136-if00
AmazingHand /dev/serial/by-id/usb-1a86_USB_Single_Serial_5C63050237-if00
C920         /dev/video0, /dev/video1
D435i        /dev/video2 ... /dev/video7
```

USB를 다시 꽂으면 `/dev/video*` 번호는 변할 수 있다. 가능하면 `/dev/v4l/by-id`를 쓰고, 없으면 record 직전에 실제 RGB node를 다시 확인한다.

## 4. Quest-only gate — 로봇 연결 없음

인증서 SAN이 현재 Jetson LAN IP와 일치하는지 확인한 뒤 다음만 실행한다.

```bash
.venv/bin/python scripts/quest_check.py --lan-ip <JETSON_LAN_IP>
```

Quest 브라우저에서 출력된 HTTPS URL을 열고 오른쪽 controller를 움직인다. 성공 기준:

```text
PASS Vuer is listening; no robot connection or motor command is active
PASS Quest right-controller tracking received
```

로컬 페이지가 `Websocket session is missing`으로 실패하면 네트워크/UFW를 먼저 확인하고 hosted Vuer client 경로도 비교한다.

## 5. 팔 passive gate

- `config/right_arm_two_motor.json`의 adapter, 1 Mbps, command/feedback ID 1/2→17/18을 확인한다.
- feedback-only CAN probe로 두 motor state가 읽히는지 확인한다.
- torque enable, MIT position target, hardware zero/flash write는 이 단계에서 보내지 않는다.
- calibration과 URDF limit 교집합이 비어 있지 않은지 확인한다.

중요: full `RobopartyTwoMotorAmazingHand.connect()`는 시작하면서 arm disable을 보내지만 AmazingHand `connect()`에서 8서보 torque를 enable한다. 따라서 “완전 no-motion” 점검에는 full integrated connect를 사용하지 말고 Quest-only, passive CAN, hand read, camera read를 분리한다.

## 6. 손과 카메라 독립 gate

- AmazingHand는 stable by-id port로 현재 위치만 읽는다.
- C920와 D435i의 실제 RGB node, 해상도, FPS, FourCC를 확인한다.
- 두 카메라가 같은 dataset FPS를 안정적으로 제공하는지 5초 이상 확인한다.
- 과거 장비 기준 공통 보수 FPS는 15였지만 현재 장치 모드로 다시 측정한다.

## 7. IK software gate

실제 motor 없이 다음을 확인한다.

1. reduced joint order가 `joint1`, `joint2`인지
2. `joint3`가 handoff `arm_temp.json` 홈값 `-0.042588 rad`에 잠기는지
3. end-effector가 `hand_mount`인지
4. controller rotation이 URDF J1=-Z, J2=-X 축으로 정확히 투영되는지
5. calibration 논리 J1/J2=0이 실제 temp URDF J1/J2=0 자세와 일치하는지
6. controller xyz 이동만으로 임시 2축 target이 변하지 않는지
7. Quest engage 첫 frame에서 joint target이 measured joint와 같아 점프하지 않는지
8. tracking loss/IK failure/B가 arm torque request를 0으로 만드는지
9. grip/squeeze 변화만으로는 arm torque request나 IK 상대 기준이 바뀌지 않는지

현재 관련 targeted test는 `20 passed`다.

## 8. 감독된 저속 motion gate

이 단계부터 실제 움직임이 가능하다. 작업 공간을 비우고 비상 정지/전원 차단이 가능한 상태에서 다음만 작게 확인한다.

1. A로 arm, B로 disarm이 되는지
2. grip을 잡고 놓아도 arm torque와 IK 기준이 유지되는지
3. tracking을 가리면 arm torque가 해제되는지
4. 아주 작은 Quest 회전의 축/부호가 기대와 맞는지
5. controller를 약 45° 숙였을 때 J2 목표도 약 45°인지
6. 프로세스 종료 시 arm/hand torque가 모두 해제되는지

처음에는 gain을 키우지 않는다. target, IK solution, FK reached, residual, measured joint, sent action을 먼저 로그한다.

## 9. 짧은 local dataset gate

IK dataset을 원하면 record 명령에 다음 플래그가 반드시 있어야 한다.

```bash
--robot.arm_control_mode=ik
--dataset.push_to_hub=false
```

현재 AmazingHand port는 stable by-id를 사용한다. `episode_time_s=inf`, `reset_time_s=inf`로 시작한 뒤
5~10초 정도만 감독 조작하고 `→`로 episode를 끝낸다. 별도 패널은 없으며 Mac SSH 터미널의
`→/←/ESC`(또는 `n/r/q`)가 LeRobot record event를 직접 제어한다. 자세한 Mac/Rerun 흐름은
[[lerobot-keyboard-rerun-recording-ko]]를 따른다. partial 저장을 피하려면 reset 단계에서
`ESC/q`로 세션을 종료한다.

```text
action: right_arm_joint_1.pos, right_arm_joint_2.pos, right_hand_grasp.pos
observation.state: 같은 3축
observation.images: front, wrist
```

추가 확인:

- frame 수가 실제 FPS×수동 recording 경과시간과 근접한가
- 두 영상 timestamp/내용이 정상인가
- action/state에 NaN이 없는가
- action이 clipping boundary에 계속 붙지 않는가
- recorded action과 robot `send_action()`의 실제 반환값이 일치하는가
- A/B 및 tracking loss 구간이 dataset에서 어떻게 기록되는가

## 10. 장시간 수집 전 stop 조건

다음 중 하나라도 있으면 장시간 dataset 수집을 중지한다.

- motor feedback loss
- IK residual이 정한 임계값보다 큼
- controller tracking age가 timeout을 반복 초과
- 카메라 frame age/FPS가 불안정
- direct/IK axis 의미가 서로 다름
- URDF zero와 실제 팔 zero가 다름
- 실제 sent action과 기록 action이 의미 있게 다름
- 2축 dataset을 미래 5축 dataset과 같은 schema로 간주하려는 상태

현재 다음 작업은 [[roboparty-xr-ik-dataset-debugging-ko]]의 진단 로그를 구현하고, 1개의 짧은 local IK episode로 end-to-end schema를 증명하는 것이다.
