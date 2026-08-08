---
title: "Jetson bring-up status and deployment gate"
tags: ["jetson", "deployment", "quest2", "ik", "urdf", "lerobot", "dataset", "safety", "korean"]
created: 2026-08-08T08:10:50.358Z
updated: 2026-08-08T20:05:00+09:00
sources: []
links: ["roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko.md", "jetson-safe-bring-up-runbook.md"]
category: environment
confidence: medium
schemaVersion: 1
---

# Jetson bring-up status and deployment gate

> 최종 확인: 2026-08-08, `/home/dong/echo_v4`. 이 페이지는 2026-08-02의 초기 bring-up 상태를 현재 checkout 기준으로 갱신한다.

## 현재 목표

Quest 2 오른쪽 컨트롤러로 RoboParty J1·J2와 AmazingHand를 조작하고 C920/D435i 영상을 포함한 LeRobot dataset을 우선 수집한다.

상세 연결 구조와 팔–VR 오차 원인은 [[roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko]]를 기준 문서로 사용한다. 안전한 실행 순서는 [[jetson-safe-bring-up-runbook]]을 따른다.

## 현재 소프트웨어 상태

- Git은 detached HEAD `5131505 arm_fixed`이며 `origin/main` 계보보다 3커밋 앞이다.
- current working tree에는 2축 reduced IK 작업이 미커밋 상태다.
  - `lerobot_robot_roboparty/two_motor_kinematics.py`
  - `RobopartyTwoMotorAmazingHandConfig.arm_control_mode`
  - Quest relative clutch → Pinocchio reduced IK
  - Linux Pinocchio/urdfdom ABI pin
  - IK/Quest 회귀 테스트
- 기준 모델은 `handoff_orientation.tar.gz` 안에서 "3축 (임시 팔, 지금 실물)"로 표시된 `orientation/urdf/robot_arm_temp.urdf`다.
- active joint는 `joint1`, `joint2`; `joint3`만 `arm_temp.json` 홈값 `-0.042588 rad`에 고정하고 end effector는 `hand_mount`다. J4/J5 쪽 링크 하나가 빠진 현재 조립을 5축 모델보다 직접적으로 표현한다.
- temp 3축 모델 전환 후 전체 pytest는 `46 passed`, 핵심 IK/Quest/2모터 test는 `20 passed`다.
- `lerobot_robot_roboparty tests` ruff는 통과한다.
- legacy `teleop/`, `televuer/`까지 포함한 전체 ruff는 74건 실패하므로 아직 전체 저장소 clean 상태는 아니다.
- `uv lock --check`는 통과한다.

## 현재 장치 상태

이번 확인에서는 모터 명령을 보내지 않고 장치만 열거했다.

```text
CANable2:
/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2070388B3136-if00

AmazingHand serial:
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C63050237-if00

Logitech C920:
/dev/video0, /dev/video1

Intel RealSense D435i:
/dev/video2 ... /dev/video7
```

`config/right_arm_two_motor.json`은 CAN `slcan`, 1 Mbps, command ID 1/2, feedback ID 17/18을 사용한다.

## 현재 호환 판정

| 연결 | 판정 | 설명 |
|---|---|---|
| Quest 2 ↔ Vuer | 가능 | HTTPS/WSS WebXR, 오른쪽 controller-only 경로가 테스트됨 |
| Vuer ↔ handoff URDF | 코드 호환 | raw WebXR pose를 RUB→FLU로 변환하며 Atom/head offset을 우회함; 실물 base yaw 검증은 필요 |
| custom URDF ↔ 2축 IK | 코드 호환 | J1=-Z, J2=-X orientation projection test 통과, 실제 6D hand pose의 완전 재현은 불가능 |
| 2축 IK ↔ DM4340 | 인터페이스 연결 | IK radian → calibration normalization → CAN action 경로 존재 |
| Quest trigger ↔ AmazingHand | 연결 | 8서보를 하나의 grasp scalar로 제어 |
| Robot/카메라 ↔ LeRobot dataset | 코드 연결 | joint state/action + front/wrist image schema가 생성됨 |
| 실제 end-to-end local dataset | 아직 없음 | 생성된 최근 local dataset들은 모두 `total_episodes=0`, `total_frames=0`; 교정 후 짧은 episode 검증 필요 |
| 현재 2축 dataset ↔ 미래 5축 policy | 직접 호환 안 됨 | action 이름과 차원이 달라 새 schema/재수집 또는 명시적 변환 필요 |

## 현재 가장 중요한 알려진 차이

1. 2축 IK는 Quest의 임의 3D 위치와 손목 방향을 모두 맞출 수 없다.
2. 현재 임시 2축 solver는 controller orientation을 우선하고 translation은 팔 목표에서 제외한다.
3. Atom/Unitree head/waist offset은 제거됐지만 실물 설치 방향의 `base_yaw_deg`는 아직 검증해야 한다.
4. TCP는 손가락 끝이 아니라 `hand_mount`다.
5. joint step 0.07 rad/frame 제한 때문에 빠른 Quest 회전을 로봇이 늦게 따라갈 수 있다.
6. `roboparty-record`는 2축 구성에서 `--robot.arm_control_mode=ik`를 필수로 검사한다.
7. calibration 내부 이름은 `motor_0/1`, LeRobot dataset 공개 이름은 `right_arm_joint_1/2`로 의도적으로 분리된다.
8. 이전 position-only IK는 곧은 자세에서 위치 Jacobian rank-1 문제를 보였다. 현재 orientation IK는
   이 위치 특이점 대신 controller angle을 J1/J2 회전으로 맞춘다.
9. 이전 통합 processor는 `squeeze`를 arm clutch로 사용해 grip 값이 흔들릴 때마다 torque와 IK
   기준을 반복 해제했다. 현재는 시작/reset 시 armed, valid tracking에서 자동 engage, B=disarm, A=rearm이며
   squeeze와 trigger는 팔 활성 조건에서 분리했다. trigger는 AmazingHand grasp만 제어한다.

## 다음 배포 gate

다음 milestone은 “더 크게 움직여 보기”가 아니라 아래 evidence를 남기는 것이다.

1. IK WIP를 브랜치/커밋으로 정리한다.
2. Quest target, IK solution, FK reached, residual, measured joints, 실제 sent action을 같은 timestamp로 로그한다.
3. temp URDF joint axis/zero, calibration 논리 0, J3의 기계적 고정 방향, 생략된 J4/J5 구간,
   link length/TCP를 실제 팔과 비교한다.
4. `--robot.arm_control_mode=ik`를 포함한 5~10초 local episode 1개를 `push_to_hub=false`로 기록한다.
5. action/state 이름, frame 수, 실제 FPS, 두 카메라 영상, action 범위를 확인한다.
6. orientation residual이 큰 target은 hold 또는 torque-off하는 quality gate를 추가한다.
7. J1·J2 + grasp schema가 안정적인지 확인한 뒤 장시간 수집을 시작한다.

실제 motion 전에는 [[jetson-safe-bring-up-runbook]]의 passive CAN, Quest tracking, camera, hand communication, torque release gate를 다시 확인한다.
