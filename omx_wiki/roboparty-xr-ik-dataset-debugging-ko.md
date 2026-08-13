---
title: "Roboparty XR IK dataset debugging KO"
tags: ["roboparty", "quest2", "ik", "urdf", "lerobot", "dataset", "korean", "debugging"]
created: 2026-08-08T08:10:51.973Z
updated: 2026-08-08T20:05:00+09:00
sources: ["handoff_orientation.tar.gz", "https://github.com/huggingface/lerobot/blob/v0.6.1/src/lerobot/utils/keyboard_input.py"]
links: ["roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko.md", "jetson-safe-bring-up-runbook.md", "jetson-bring-up-status-and-deployment-gate.md", "lerobot-keyboard-rerun-recording-ko.md", "8012-vr-arm-path-confusion-retrospective-ko.md"]
category: architecture
confidence: high
schemaVersion: 1
---

# RoboParty J1·J2 XR–IK–Dataset 디버깅 순서

> 전체 구조는 [[roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko]]를 먼저 읽는다. 이 문서는 증상에서 원인을 좁히는 순서다.
> 8012 실행 경로와 최신 URDF를 잘못 선택했던 원인 및 재발 방지는 [[8012-vr-arm-path-confusion-retrospective-ko]]에 기록했다.

## 1. 기준 구성을 먼저 확인한다

```text
URDF archive: handoff_orientation.tar.gz
URDF member:  orientation/urdf/robot_arm_temp.urdf
active:       joint1, joint2
locked:       joint3=-0.042588 rad
EE:           hand_mount
hand:         right_hand_grasp 단일 축
dataset:      right_arm_joint_1/2 + right_hand_grasp
FPS:          15
```

`urdf_temp4.zip`, handoff 5축 `robot_arm.urdf`, Atom01 URDF가 로그에 나타나면 현재 실물 기준으로는 잘못된 실행 경로다.

## 2. 증상별 첫 확인점

| 증상 | 가능성이 높은 원인 | 먼저 확인할 값 |
|---|---|---|
| 머리를 움직이면 팔도 움직임 | 가공된 `tv_wrapper` pose 사용 | `Quest2Vuer`가 raw `tvuer.right_arm_pose`를 읽는지 |
| 앞으로 밀었는데 옆으로 감 | RUB→FLU 또는 `base_yaw_deg` | raw delta와 mapped delta |
| 좌우만 반대 | motor sign 또는 mirror | mirror보다 motor sign과 yaw를 먼저 확인 |
| 최초 tracking/rearm 순간 점프 | relative 기준이 실제 FK로 engage되지 않음 | engage 전 measured J1/J2, FK, target |
| 숙인 각도와 팔 각도가 다름 | controller orientation 대신 xyz position을 IK에 사용 | orientation preview와 IK J1/J2 각도 |
| 멀리 갈수록 곡선으로 어긋남 | J1/J2 reachable surface | target XYZ, reached XYZ, residual |
| 전체가 일정 거리/각도만큼 틀림 | temp 모델의 J3 고정 방향 또는 TCP 불일치 | `arm_temp.json` 홈값, `hand_mount` 실측 |
| 빠르게 움직일 때 뒤처짐 | measured joint 기준 rate limit | 0.07 rad/frame, 실제 FPS |
| 끝 범위에서만 오차 증가 | calibration∩URDF clipping | requested/bounded joint radians |
| dataset replay와 실물 명령 차이 | processor action과 sent action 차이 | normalized processor/sent action 동시 로그 |
| 손만 멈춤 | AmazingHand serial/servo 경로 | trigger, grasp action, servo feedback |
| grip을 몇 번 누르면 팔이 풀리거나 뚝뚝 끊김 | squeeze가 arm clutch에 잘못 결합됨 | `arm_enabled`가 squeeze 변화에 따라 1↔0인지 |

### 2.1 이번에 확인된 "grip을 누르면 팔이 풀림"의 직접 원인과 수정

통합 processor가 기존에는 다음 조건을 사용했다.

```text
arm_enabled = armed AND tracking AND squeezing
```

Quest의 grip/squeeze 값이 짧게 0으로 바뀔 때마다 torque를 해제하고 IK 상대 기준도 지웠다. 다시
1이 되면 현재 pose에서 재-engage했기 때문에 손은 즉시 반응하지만 팔은 반복적으로 멈추고 다시
시작하여 뚝뚝 끊겨 보였다. 이는 dataset frame backlog가 아니라 입력 의미를 잘못 결합한 문제다.

현재 통합 경로는 다음과 같다.

```text
initial/reset: armed; valid tracking에서 자동 engage
A:             B 이후 rearm
B:             disarmed
arm_enabled =  armed AND tracking
trigger:       AmazingHand grasp only
squeeze:       arm enable에 사용하지 않음
```

따라서 episode 시작 후 tracking이 잡히면 자동 engage하며, 멈출 때 B를 누르고 다시 시작할 때 A를 누른다. grip/squeeze를 반복해도 팔 torque와
IK 원점은 유지된다. tracking loss와 IK 실패는 여전히 팔 torque를 해제한다.

### 2.2 이번에 확인된 "끝 위치를 계속 추종함"의 원인

2026-08-08 direct 진단에서 J2가 아래 방향 목표를 받은 뒤 컨트롤러가 멈춰도 끝 범위까지 계속
이동했고, torque가 켜진 상태에서 팔을 수동으로 옮겨도 다시 같은 위치로 돌아갔다. 이것은 dataset
replay나 입력 queue가 아니라 **position servo가 마지막 상대 위치 target을 계속 추종한 것**이다.

진단 직후 프로세스를 종료하고 passive CAN으로 확인한 상태는 다음과 같다.

```text
J1 logical = -0.1675 rad
J2 logical = -1.6480 rad
J2 URDF lower limit까지 남은 거리 = 0.0974 rad
status = disabled (J1/J2 모두)
```

당시 direct/position-only IK mapping은 controller가 기준점에서 멀어진 만큼의 **위치 목표**를 만들었다. 따라서
controller를 단순히 멈추는 것은 stop 명령이 아니며, target과 실제 관절이 다르면 rate limit만큼 계속
이동한다. 수동으로 관절을 옮겨도 target이 바뀌지 않으므로 다시 돌아간다. 이는 edge computer 부하
때문이 아니다. 현재 2축 IK는 controller 상대 orientation에서 매 frame 절대 관절 목표를 다시 계산하고,
최신 measured J1/J2 기준으로만 0.07 rad/frame 제한을 적용한다. 이전 EE target을 별도로 전진시키는
backlog는 제거했다.

### 2.3 이번에 확인된 "거의 안 움직임"의 직접 원인

마지막 passive feedback과 현재 calibration을 조합한 논리 관절각은 대략 J1 `-0.122 rad`, J2
`-0.024 rad`였다. temp URDF에서는 이것이 팔이 거의 곧게 선 `q≈0` 자세다. 이 자세의 위치
Jacobian은 다음처럼 rank 1이 된다.

```text
q = [0, 0]
J_position = [[0, 0],
              [0, 0.238503],
              [0, 0]]
```

- J1은 손목이 회전축 위에 있어 EE **위치**를 전혀 바꾸지 못한다.
- J2는 이 순간 한 방향만 1차적으로 움직일 수 있다.
- 다른 방향 target은 IK가 거의 무시하거나, 특이점 근처에서 큰 관절 변화를 요구한 뒤
  `0.07 rad/frame` 제한에 잘려 작고 끊겨 보일 수 있다.

따라서 이번 증상의 주원인은 관절 limit lock이 아니라 **2DOF temp 팔의 곧은 자세 특이점과
도달 불가능한 Cartesian target 투영**이었다. 현재 임시 2축 record 경로는 position IK 대신
orientation IK를 사용하므로 이 특이점을 피한다.

또한 LeRobot calibration의 논리 0이 실제 temp URDF의 관절 0과 같은 자세인지 확인해야 한다.
캘리브레이션 때 잡은 "중간 자세"가 물리적으로 굽은 자세였다면, 논리각과 URDF각 사이의 별도
zero offset이 필요하다. `arm_temp.json home_q[:2]`를 검증 없이 offset으로 넣으면 안 된다.

### 2.4 orientation sync 실측 결과

2026-08-08 read-only Quest sync에서 controller 회전을 URDF 축에 직접 투영한 뒤 다음을 확인했다.

```text
왼쪽 약 90도 회전:
orient_preview_j1j2 = (-94.2, +9.1) deg
orientation_ik_j1j2 = (-94.2, +9.1) deg
```

Robot FLU에서 J1 축이 `-Z`이므로 왼쪽 회전은 J1 음수 방향이다. 약 `+9°`의 J2 성분은 controller를
돌리는 동안 함께 들어온 pitch 성분이며, preview와 실제 processor 목표가 동일하므로 solver의 교차축
오차는 아니다. 모터는 이 검사 내내 disabled였다.

## 3. 좌표계 검사는 IK보다 먼저 한다

Quest 원시 좌표에서 작은 단일축 이동을 만든다.

```text
Quest -Z(앞) → Robot +X(앞)
Quest +X(오른쪽) → Robot -Y(오른쪽)
Quest +Y(위) → Robot +Z(위)
```

이 매핑이 틀리면 IK gain이나 URDF를 조정하지 않는다. 먼저 `base_yaw_deg`와 모터 sign을 분리해 검사한다. mirror는 좌우 반사이므로 마지막 수단이다.

권장 순서:

1. direct 모드에서 J1과 J2를 각각 작은 범위로 확인한다.
2. raw Quest delta와 변환 후 FLU delta를 기록한다.
3. 방향이 맞은 후에만 IK 모드로 전환한다.
4. IK에서는 target과 FK reached의 차이를 본다.

## 4. J3 고정과 생략된 링크가 중요한 이유

현재 실물은 J4/J5 쪽 링크 하나가 빠지고 손목/손이 앞쪽에 바로 달린 임시 팔이다. 따라서 5축
`robot_arm.urdf`의 J3–J5 링크를 고정해서 남기는 대신, handoff가 현재 실물용으로 제공한 3축
`robot_arm_temp.urdf`를 사용한다. Pinocchio reduced model은 남은 J3만 다음 홈값으로 고정한다.

```text
J3 = -0.0425876757 rad
```

실제 손목/TCP 위치가 이 temp URDF와 다르면 J1·J2 IK를 아무리 조정해도 손끝에 일정한 오차가
남는다. 이 경우 속도나 Quest scale을 바꾸기 전에 `joint2 → joint3 → link5 → hand_mount`의 실제
길이와 설치 방향을 실측한다. J3는 dataset action에는 포함하지 않는다.

## 5. 한 프레임에 기록할 디버그 값

문제가 재현되면 아래를 JSONL 한 줄로 남긴다.

```text
timestamp
quest_pose_raw
quest_pose_robot_flu
tracking / squeeze / trigger / armed / arm_enabled
measured_j1_j2_rad
fk_hand_mount_rotation
clutch_target_rotation
ik_requested_j1_j2_rad
bounded_j1_j2_rad
fk_reached_rotation
ik_orientation_residual_rad
normalized_processor_action
robot_sent_action
right_hand_grasp
```

판정:

- raw부터 틀림: Quest/Vuer 이벤트 문제
- raw는 맞고 FLU만 틀림: 좌표 매핑 문제
- target은 맞고 reached가 다름: 2DOF/URDF/IK 문제
- reached는 맞고 실물만 다름: calibration, motor sign, 고정각 문제
- processor와 sent가 다름: Robot 안전 clip 또는 stale observation 문제

## 6. LeRobot dataset 검사

별도 panel 없이 LeRobot의 수동 키 제어로 local episode를 먼저 만든다. `inf`는 timer를 없애고
`→/←/ESC` 또는 `n/r/q`가 phase를 제어하도록 한다.

```bash
roboparty-record \
  --robot.type=roboparty_two_motor_amazing_hand \
  --robot.id=two_motor_amazing_hand \
  --robot.hand_port=<AMAZING_HAND_PORT> \
  --robot.can_port=<CANABLE_PORT> \
  --robot.two_motor_calibration_path=config/right_arm_two_motor.json \
  --robot.arm_control_mode=ik \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --dataset.repo_id=local/roboparty-j1-j2-smoke \
  --dataset.num_episodes=1 \
  --dataset.single_task='Test J1 J2 and grasp' \
  --dataset.episode_time_s=inf \
  --dataset.reset_time_s=inf \
  --dataset.fps=15 \
  --dataset.push_to_hub=false
```

확인 항목:

```text
action names exactly:
  right_arm_joint_1.pos
  right_arm_joint_2.pos
  right_hand_grasp.pos

must not exist:
  motor_0.pos / motor_1.pos
  joint3 / joint4 / joint5
  control.arm_torque_enabled
  raw controller pose as policy action
```

추가로 frame 수가 약 `실제 recording 경과시간 × 15`, 두 카메라 영상이 유효하며 action에 NaN이 없는지
확인한다. Rerun은 시각화 전용이므로 키 입력은 focus된 SSH 터미널에서 한다. 전체 수동 운용은
[[lerobot-keyboard-rerun-recording-ko]]를 따른다. recording 중 `ESC/q`는
partial episode도 저장하므로, 정상 episode를 `→/n`으로 끝낸 뒤 reset 단계에서 종료한다.

## 7. 안전한 소프트웨어 검증

다음 명령은 모터 동작 없이 실행할 수 있다.

```bash
cd /home/dong/echo_v4
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest -q
.venv/bin/ruff check lerobot_robot_roboparty tests
uv lock --check
omx wiki wiki_lint --json
```

장치가 연결된 뒤에도 먼저 열거만 한다.

```bash
ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* /dev/video* 2>/dev/null
v4l2-ctl --list-devices
.venv/bin/python scripts/quest_check.py --lan-ip <JETSON_LAN_IP>
```

## 8. 실제 동작 검증 순서

1. 모터 torque-disabled 상태에서 J1/J2 feedback 확인
2. Quest tracking과 A/B/trigger 입력 확인; squeeze 변화가 arm_enabled를 바꾸지 않는지 확인
3. calibration 논리 0과 temp URDF J1/J2=0의 실제 자세가 같은지 확인
4. J2가 약간 굽은 비특이 자세에서 direct 모드로 한 축씩 매우 작은 이동
5. IK engage 순간 무점프와 J1/J2 동시 반응 확인
6. 수동으로 5~10초 정도 조작한 뒤 `→`로 local dataset 1 episode 종료
7. dataset schema·FPS·영상·action 범위 확인
8. 정상일 때만 장시간 수집

실제 motion gate는 [[jetson-safe-bring-up-runbook]]을 따른다.
