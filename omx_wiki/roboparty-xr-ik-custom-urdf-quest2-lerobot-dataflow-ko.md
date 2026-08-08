---
title: "Roboparty XR IK custom URDF Quest2 LeRobot dataflow KO"
tags: ["roboparty", "quest2", "webxr", "ik", "urdf", "lerobot", "dataset", "korean", "architecture"]
created: 2026-08-08T08:09:45.410Z
updated: 2026-08-08T23:10:00+09:00
sources:
  - "handoff_orientation.tar.gz"
  - "https://huggingface.co/docs/lerobot/main/isaac_teleop"
  - "https://github.com/huggingface/lerobot/blob/v0.6.1/docs/source/il_robots.mdx"
  - "https://rerun.io/docs/reference/sdk-operating-modes"
links: ["jetson-bring-up-status-and-deployment-gate.md", "roboparty-xr-ik-dataset-debugging-ko.md", "jetson-safe-bring-up-runbook.md", "lerobot-keyboard-rerun-recording-ko.md"]
category: architecture
confidence: high
schemaVersion: 1
---

# RoboParty J1·J2 + AmazingHand의 Quest2 → LeRobot 전체 흐름

> 검증 기준: 2026-08-08, `/home/dong/echo_v4`. 자동 테스트 `62 passed`,
> 실물 J1/J2 유지·±2°·+5°·연속 궤적 응답을 검증했고 종료 후 토크를 소자했다.

## 0. 최신 운용 상태: SuperArm 형상 + J1/J2 전용

2026-08-08 마지막 실기 검증 경로는 `handoff_orientation` 3축 프리뷰가 아니라
`beautifulmelon/robot_arm_vr` 기반의 다음 설정이다.

```text
Quest 2 WebXR
  → robot_arm_vr 2-DOF IK (20 Hz)
  → UDP 5005/5006/5007
  → robot_arm_vr_bridge
  → DM4340 motor ID 1, 2 only
```

| 구분 | 현재 값 |
|---|---|
| 형상/IK URDF | `third_party/robot_arm_vr/assets/superarm_j1_j2/superarm_j1_j2.urdf` |
| IK 설정 | `third_party/robot_arm_vr/config/superarm_j1_j2_jetson.json` |
| 가동 관절 | `joint_rev_1`, `joint_rev_2` |
| 고정 관절 | `joint_rev_3`, `joint_rev_4`, 이후 손목 체인 |
| EE | `hand_mount` |
| 형상 자산 | SuperArm STL 18개, URDF mesh reference 36개 |
| 현재 LeRobot calibration | `config/right_arm_two_motor.json` |
| 소프트웨어 정지 | Quest 오른손 B, 토크 유지 HOLD, 프로세스 재시작 전까지 해제 불가 |

웹 뷰에서 더 이상 다른 축이 돌아가면 잘못된 config를 실행한 것이다.
정상 config의 URDF movable joint는 정확히 두 개뿐이다. 다만 현재 실물은
뒤쪽 링크를 생략한 개조형이므로, SuperArm 전체 형상의 EE 리치와 실물 링크 길이가
정확히 같지 않을 수 있다. 이 길이 보정은 5축 완성 후 해도 되지만, 현재
2축 IK가 낼 수 있는 위치는 3차원 부피가 아니라 얇은 곡면(position rank 2/3)이다.

### LeRobot 캘리브레이션과 모터 KP/KD는 다른 값이다

| 설정 | 역할 | 현재 정책 |
|---|---|---|
| LeRobot calibration | raw motor angle ↔ J1/J2 논리각, 영점, 방향, 관절 범위 | 2026-08-08 현재 파일을 그대로 사용 |
| `kp`, `kd` | DM4340 MIT/impedance 제어의 위치/속도 게인 | J1 `120/2.5`, J2 `180/4.0` |
| step/rate limit | 안전계층 목표 변화와 모터 명령 주기 | `velocity_scale=0.16`, safety 20 Hz, motor interpolation 50 Hz |

2026-08-08 현재 개조형 짧은 팔에서 실측해 기본값으로 적용한 모터 프로필:

```text
J1 kp = 120.0, kd = 2.5
J2 kp = 180.0, kd = 4.0
max_relative_target_rad = 0.03 rad/frame
safety/control rate = 20 Hz
motor interpolation rate = 50 Hz
```

`robot_arm_vr_bridge` 경로에서는 `0.03 rad/frame` 제한을 URDF 속도
`3.77 rad/s` 대비 `velocity_scale=0.16`, `control_dt=0.05 s`로 동일하게 구현한다.
즉 `3.77 × 0.16 × 0.05 ≈ 0.0302 rad/frame`이다. 이 값을 바꿔도 캘리브레이션
파일의 영점·방향·범위는 바뀌지 않는다.

### 2026-08-08 모터 지연 원인과 수정

이전의 약 3~4초 지연과 “값이 밀려서 들어오는” 증상은 Quest나 IK 데이터 폭주가
아니라 CAN 피드백 큐 누적이었다.

```text
한 제어 주기:
  MIT 위치 명령 → J1/J2 피드백 2개 생성
  refresh 요청    → J1/J2 피드백 2개 추가 생성
  기존 코드      → 2개만 소비
  결과           → 주기당 과거 프레임 2개씩 누적
```

`RawRelativeDMBackend._read_raw()`가 각 refresh 전 과거 MIT 응답을 비우고 최신
ID 일치 응답만 사용하도록 수정했다. 수정 후 `kp=40` 시험부터 J2가
`0.18 s` 안에 반응해, 이전의 3~4초 지연이 피드백 표시 지연임을 확인했다.

최종 프로필 검증 결과:

| 시험 | 결과 |
|---|---|
| J1 `+2°` | 최종 `+2.12°`, 최대 오버슈트 `0.23°`, 반응 약 `0.13 s` |
| J2 `-2°` | 최종 `-2.142°`, 최대 오버슈트 `0.317°`, 반응 약 `0.13 s` |
| J2 `+5°`, 50 Hz 보간 | 최종 `+4.524°`, 오버슈트 없음, 반응 약 `0.08 s` |
| 12초 연속 궤적 | RMS 오차 J1 `0.43°`, J2 `0.60°`; 최대 오차 `0.98°/1.09°` |
| 최종 2초 유지 | J1 `0°`, J2 `0.044°` 변화; DM4340 MOS 온도 양축 약 `31°C`; 오류 `0` |

`kp=240/kd=5`도 비교했지만 J2 `+5°` 정적 오차가 `180/4`보다 약 `0.09°`만
줄어든 반면 강성은 더 커져 기본값으로 선택하지 않았다. 중력에 따른 J2의
약 `0.4~0.5°` 정적 처짐은 현재 데이터 수집 단계에서는 수용하고, 5축 완성 후
URDF 질량·중심을 확정한 다음 중력 피드포워드로 보정한다.

> 아래 1~10장은 LeRobot `roboparty-record` 통합 경로의 구조를 설명한다.
> 현재 SuperArm 실기 동작 비교는 위 0장의 `robot_arm_vr` 경로로 먼저 검증한다.

## 1. LeRobot 데이터셋 통합 경로의 구성

현재 목표는 **5축 팔 전체를 움직이는 것**이 아니다. J4/J5 쪽 링크 하나를 생략하고 손목/손을
앞쪽에 바로 단 현재 실물용 3축 URDF를 사용하되, 실제 action은 다음 세 개만 사용한다.

```text
right_arm_joint_1.pos
right_arm_joint_2.pos
right_hand_grasp.pos
```

- J1·J2: 실제 DM4340 두 모터
- J3: 모터를 제어하거나 dataset에 기록하지 않고 handoff temp 홈 자세에 고정
- J4·J5: 현재 실물 및 temp URDF 제어 체인에 없음
- AmazingHand: 8개 서보를 하나의 full-grasp 축으로 제어
- Quest A/B: 팔 arm/disarm
- Quest 오른쪽 grip/squeeze: 팔 활성 상태와 무관
- Quest 오른쪽 trigger: AmazingHand grasp
- 수집 속도: 15 FPS
- 관절 변화 제한: 0.03 rad/frame (2026-08-02 실기 프로필)

## 2. 어떤 URDF를 사용하는가

런타임 기준 파일은 다음 하나다.

```text
/home/dong/echo_v4/handoff_orientation.tar.gz
└─ orientation/
   ├─ urdf/robot_arm_temp.urdf
   ├─ config/arm_temp.json
   └─ code/transforms.py       # 좌표 규약의 참고 원본
```

선택한 `robot_arm_temp.urdf`는 아카이브가 "3축 (임시 팔, 지금 실물)"로 명시한 모델이다.

```text
base_link
  └─ joint1 (-Z) → link1
      └─ joint2 (-X) → link2
          └─ joint3 (-Z) → link5
              └─ fixed → hand_mount
```

코드는 TAR.GZ를 디스크에 임시로 풀지 않고 Python `tarfile`로 URDF와 JSON을 직접 읽는다. 잘못된 handoff 파일을 조용히 사용하는 것을 막기 위해 멤버 SHA-256도 검증한다.

| 멤버 | SHA-256 |
|---|---|
| `orientation/urdf/robot_arm_temp.urdf` | `6e96a11420373a405cac13b56fb40391ecd1fdd67b2b8d34df4abd8945c01d27` |
| `orientation/config/arm_temp.json` | `9b6a7204cf2f81a559f145e69702806b805dacead2f0838abbb89c29167216a8` |

이전에 시험했던 `urdf_temp4.zip`과 handoff의 5축 `robot_arm.urdf`는 현재 런타임 IK에 사용하지 않는다.

## 3. 현재 실물용 3축 URDF를 2축으로 쓰는 방법

`RobopartyTwoMotorKinematics`는 Pinocchio 전체 모델을 읽은 뒤 reduced model을 만든다.

```text
active: joint1, joint2
locked: joint3 = -0.0425876757 rad
EE:     hand_mount
```

고정각은 임의의 0 rad가 아니라 handoff `arm_temp.json`의 `home_q[2]`다. J4/J5 쪽 생략된 링크를
5축 모델에 남겨두지 않으며, solver의 자유도는 J1·J2 두 개뿐이다. TCP는 `link5`에 고정된
`hand_mount`다.

현재 임시 2축 IK가 계산하는 것은 **EE orientation**이다. 2026-08-08 sync에서 controller를 약 45°
숙였을 때 orientation preview는 J2 `+45.4°`였지만 기존 position-only IK는 J2 `-69.7°`를 요구했다.
따라서 J1/J2만 있는 동안에는 Quest quaternion의 상대 회전을 `hand_mount` 상대 회전으로 만들고
URDF의 J1=-Z, J2=-X 축으로 직접 투영해 관절을 계산한다. controller `xyz`는 action으로 수집되지만 임시 2축
팔 목표에서는 제외한다. 두 관절로 임의의 3D 위치와 3D 방향을 동시에 맞출 수 없기 때문이다.

## 4. Quest2 pose는 어디서 오는가

```text
Quest 2 Browser
  └─ WebXR MotionControllers
       └─ Vuer HTTPS/WSS :8012
            └─ televuer.TeleVuer.right_arm_pose
                 └─ Quest2Vuer.get_action()
```

`Quest2Vuer`는 `tv_wrapper.get_motion_state_data()`의 가공 pose를 사용하지 않는다. 그 함수에는 기존 humanoid용 head translation 제거와 Atom waist offset이 들어 있기 때문이다. 현재 경로는 `TeleVuer.right_arm_pose`에서 **원시 오른쪽 컨트롤러 pose**를 직접 읽는다.

LeRobot teleoperator action은 다음 raw 입력을 낸다.

```text
controller.x/y/z
controller.qx/qy/qz/qw       # xyzw
controller.tracking
controller.squeeze
controller.trigger
controller.a/b
```

이 raw XR 값은 아직 로봇 action이 아니다. processor가 좌표계·클러치·IK·안전 제한을 적용한 뒤에만 Robot으로 보낸다.

## 5. WebXR 좌표에서 로봇 좌표로

handoff 규약은 다음과 같다.

```text
WebXR local-floor RUB: x=오른쪽, y=위, z=뒤
Robot base FLU:        x=앞,     y=왼쪽, z=위
```

기본 변환 행렬:

```text
R_WEBXR_TO_ROBOT =
  [ 0  0 -1 ]
  [-1  0  0 ]
  [ 0  1  0 ]
```

따라서 다음이 성립한다.

```text
Quest 앞으로(-z)  → Robot 앞으로(+x)
Quest 오른쪽(+x)  → Robot 오른쪽(-y)
Quest 위(+y)      → Robot 위(+z)
```

`base_yaw_deg`로 로봇이 놓인 방향을 보정할 수 있다. `mirror`는 반사이므로 기본값은 `false`이며, 위치뿐 아니라 회전에도 켤레변환을 적용한다.

## 6. LeRobot XR 방식의 상대 기준과 IK

현재 online 처리 순서는 다음과 같다.

```text
raw Quest right-controller pose
  → RUB→FLU basis mapping
  → tracking / A·B safety gate
  → 최초 tracking 또는 A rearm의 첫 유효 frame에서 engage-relative 기준 설정
  → engage 시점의 실제 J1·J2 FK를 EE 기준으로 저장
  → Quest 상대 회전을 hand_mount 목표 회전에 적용
  → handoff 현재 실물용 3축 URDF의 J1=-Z, J2=-X 축 투영
  → calibration ∩ URDF joint limits
  → 최신 measured J1·J2 기준 step ≤ 0.07 rad/frame
  → LeRobot normalized action
```

최초 tracking 또는 A rearm 뒤 첫 유효 frame에는 다음 두 pose를 동시에 저장한다.

```text
Quest 기준 pose = 현재 raw controller pose를 FLU로 변환한 값
Robot 기준 pose = 현재 실제 J1·J2 상태로 계산한 hand_mount FK
```

따라서 arm engage 순간 Quest 절대 위치로 팔이 점프하지 않는다. processor는 시작과 episode reset 때
armed 상태이며, 유효한 tracking이 들어오면 현재 실제 J1·J2 FK에서 자동으로 기준을 만든다. B를
누르면 팔 torque와 상대 기준을 해제하며, 다시 A를 누르면 현재 자세에서 새 기준을 만든다. tracking이 끊겨도 팔 torque와 상대 기준을
해제하지만 손 grasp는 마지막 값을 유지한다.

`controller.squeeze`는 raw XR feature로 계속 읽지만 **현재 J1·J2 + AmazingHand 통합 processor의
팔 enable 조건에는 사용하지 않는다**. grip을 잡거나 놓아도 팔 torque가 풀리거나 IK 원점이
재설정되지 않는다. 단독 진단용 `roboparty-quest-two-motor-teleop` CLI만 기존 hold-to-run grip
clutch를 유지한다.

## 7. AmazingHand는 IK와 분리되어 있다

AmazingHand는 URDF 손가락 IK를 사용하지 않는다.

```text
Quest trigger 0.0~1.0
  → right_hand_grasp.pos 0~100
  → FullGraspMapper
  → AmazingHand servo ID 1~8
```

- grip/squeeze 상태와 무관하게 trigger grasp 축은 독립적으로 처리된다.
- tracking이 사라지면 마지막 유효 grasp를 유지한다.
- A/B는 팔 arm/disarm이며 손 grasp 값은 바꾸지 않는다.
- 향후 개별 손가락 dataset을 만들려면 별도의 다축 hand action schema가 필요하다.

## 8. Robot 내부 이름과 dataset 이름

캘리브레이션 파일은 기존 하드웨어 이름을 유지한다.

```text
internal calibration: motor_0, motor_1
```

그러나 LeRobot 공개 observation/action은 의미가 드러나는 이름으로 변환한다.

```text
motor_0 ↔ right_arm_joint_1
motor_1 ↔ right_arm_joint_2
hand    ↔ right_hand_grasp
```

Robot observation/action feature 순서:

```text
right_arm_joint_1.pos
right_arm_joint_2.pos
right_hand_grasp.pos
```

J1·J2는 캘리브레이션 radian 범위를 LeRobot `[-100, 100]`으로 정규화한다. hand grasp는 `[0, 100]`이다. `control.arm_torque_enabled`는 processor와 Robot 사이의 내부 gate이며 dataset feature에서 제거된다.

## 9. LeRobot dataset 연결

`roboparty-record`는 다음 흐름으로 frame을 만든다.

```text
Robot.get_observation()
  ├─ 실제 J1·J2 feedback
  ├─ AmazingHand observation
  └─ front/wrist images

Quest2Vuer.get_action()
  └─ raw XR controller values

QuestTwoMotorAmazingHandProcessor
  └─ A/B gate + relative mapping + IK + 모든 action limit

Robot.send_action()
  └─ 같은 limit을 방어적으로 재확인하고 CAN/serial 전송

LeRobotDataset.add_frame()
  └─ observation + processor의 최종 normalized action + task
```

현재 LeRobot record loop는 `send_action()` 반환값이 아니라 processor action을 저장한다. 그래서 RoboParty processor가 Robot과 동일한 관절 step·관절범위 제한을 먼저 적용하도록 구성했다. 정상 frame에서는 기록 action과 실제 전송 action이 같아야 한다.

raw Quest pose와 고정 J3는 dataset action/state에 포함되지 않는다. J4·J5는 현재 제어 체인 자체에
없다. 필요하면 raw pose와 IK 내부 값은 별도의 디버그 JSONL에만 기록한다.

### 9.1 별도 패널 없이 Mac에서 episode를 제어하는 흐름

LeLab이나 커스텀 panel은 사용하지 않는다. 시간은 `episode_time_s=inf`, `reset_time_s=inf`로 두고
Mac SSH 터미널의 `→/←/ESC`가 기존 LeRobot event를 직접 제어한다. Rerun은 카메라·state·action
시각화만 담당한다. reverse SSH, 정확한 키 의미, partial episode 주의사항, 실행 명령은
[[lerobot-keyboard-rerun-recording-ko]]에 분리했다.

## 10. direct와 IK의 용도

| 모드 | 용도 | record 허용 |
|---|---|---|
| `direct` | J1/J2 축·부호·모터 연결 진단 | 현재 2축 `roboparty-record`에서 거부 |
| `ik` | Quest 상대 회전을 J1/J2 orientation IK로 변환해 dataset 수집 | 필수 |

`arm_control_mode` 기본값은 진단 안전성을 위해 `direct`로 남아 있다. 따라서 record 명령에는 반드시 다음을 넣어야 하며, CLI도 이를 검사한다.

```bash
--robot.arm_control_mode=ik
```

## 11. 왜 Quest 손과 팔이 아직 다를 수 있는가

1. 자유도는 J1·J2 두 개뿐이므로 임의의 6D target을 동시에 맞출 수 없다.
2. 현재 임시 모드는 손목 quaternion을 우선하며 controller `xyz` 이동은 팔 목표에서 제외한다.
3. IK TCP는 손가락 끝이 아니라 `hand_mount`다.
4. 실제 `joint2 → link5 → hand_mount` 길이/설치 방향이 temp URDF와 다르면 FK 전체가 일정하게 틀어진다.
5. `base_yaw_deg`, 모터 sign, homing offset이 실물과 다를 수 있다.
6. 빠른 손 회전은 measured joint 기준 0.07 rad/frame 제한 때문에 지연되어 보일 수 있다.
7. orientation 축 투영을 사용하므로 이전 position-only IK의 곧은 자세 rank-1 문제는 현재 2축
   record 경로에 적용되지 않는다.
8. LeRobot calibration에서 잡은 논리 0 자세와 URDF의 기구학적 0 자세가 다르면 별도의
   calibration-to-URDF offset이 필요하다. handoff `home_q`를 검증 없이 offset으로 사용하지 않는다.

따라서 지금 단계에서는 controller angle과 J1·J2의 축·부호·크기를 먼저 맞춰
**J1·J2 + grasp dataset을 안정적으로 쌓는 것**이 우선이다. full Cartesian position+orientation
보정은 더 많은 팔 관절을 실제로 연결한 뒤 진행한다.

## 12. Mac SSH + Rerun 수동 record 예시

실행 계약의 핵심은 다음 네 플래그다. 장치별 전체 명령과 Mac tunnel은
[[lerobot-keyboard-rerun-recording-ko]]를 그대로 사용한다.

```text
--dataset.episode_time_s=inf --dataset.reset_time_s=inf
--display_mode=rerun --display_ip=127.0.0.1 --display_port=9876
```

실제 장비에서는 [[jetson-safe-bring-up-runbook]]의 no-motion gate를 먼저 통과한다. 증상별 분리는 [[roboparty-xr-ik-dataset-debugging-ko]]를 따른다.
