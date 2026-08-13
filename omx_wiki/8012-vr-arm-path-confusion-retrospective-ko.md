---
title: "8012 VR arm 경로 혼동 회고와 재발 방지"
tags: ["quest2", "8012", "robot-arm-vr", "urdf", "j3", "debugging", "retrospective", "korean"]
created: 2026-08-13T13:33:23.800Z
updated: 2026-08-13T13:37:38.257Z
sources: []
links: ["roboparty-xr-ik-dataset-debugging-ko.md", "jetson-safe-bring-up-runbook.md"]
category: debugging
confidence: medium
schemaVersion: 1
---

# 8012 VR arm 경로 혼동 회고와 재발 방지

## 먼저 사과와 책임

이번 작업에서 이미 검증했던 8월 9일 계열의 `8012` VR 시뮬레이터와 수정 URDF를 먼저 확인하지 않고 다른 실행 경로와 기본 파일을 따라갔다. 그 결과 사용자가 이미 고친 모델과 축 설정을 보지 못했고, `4443`과 오래된 J1/J2 설정을 반복해서 제시했다. 불필요하게 시간을 쓰게 하고 실제 팔 동작까지 혼란스럽게 만든 점을 미안하게 생각한다. 원인은 사용자의 설명 부족이 아니라 내가 실행 중인 정확한 repo, submodule commit, port, config, URDF, motor sign을 한 체인으로 검증하지 않은 데 있다.

## 무엇을 잘못 확인했는가

1. 부모 repo의 LeRobot/Vuer 경로부터 보면서, 실제 8012 VR 화면을 제공하는 pinned `third_party/robot_arm_vr` submodule을 늦게 확인했다.
2. submodule README의 기본값 `4443`을 현재 실행 사실보다 우선했다. 사용자가 이미 `8012`라고 여러 번 말했는데도 listening socket과 프로세스 인자를 먼저 확인하지 않았다.
3. 기본 `superarm_j1_j2_jetson.json`을 사용했고, commit `b2fd9495896c7cfe60a969bc52af93db96a7544a`에 추가된 최신 `robot_arm_temp_j1_j2_updated.urdf`와 config를 즉시 선택하지 않았다.
4. `b2fd949`가 새 URDF/config 파일은 추가했지만 `run.sh` 기본 선택까지 바꾸지는 않았다는 점을 놓쳤다. `run.sh --temp`도 최신 파일이 아니라 `config/arm_temp.json`을 고른다.
5. 서로 독립인 세 층을 섞었다: 웹 서버 port(8012/4443), 시뮬레이터가 읽는 URDF/config, 실제 모터의 axis sign.
6. J1 좌우 mirror 문제는 최신 URDF의 `joint1 axis="0 0 -1"`과 실제 bridge sign을 함께 봐야 했는데, 처음에는 화면 mirror/좌표계 문제처럼 넓게 추측했다.

## 확인된 기준 경로

```text
parent repo:   /home/dong/echo_v4
VR submodule:  third_party/robot_arm_vr
submodule SHA: b2fd9495896c7cfe60a969bc52af93db96a7544a
web port:      8012
URDF:          assets/robot_arm_temp_j1_j2_updated/robot_arm_temp_j1_j2_updated.urdf
config:        config/robot_arm_temp_j1_j2_updated.json
J1 physical:   sign -1
J2 physical:   sign +1
J3 physical:   command 3, feedback 19, sign -1
safe startup:  --no-home-on-xr-start --disable-home-button
```

J3는 기존 visual zero를 보존하기 위해 기존 fixed joint의 origin `rpy="0 0 0.042587675661"`을 그대로 두고 revolute `axis="0 0 -1"`만 활성화한다. 실물 J3는 `.local/calibration/right_arm_three_motor.json`의 측정 range/offset과 `.local/calibration/right_arm_j3_mit_gain.json`의 검증된 `kp=50`, `kd=1`을 사용한다. J3 gain은 코드에 임의로 박지 않고 외부 profile의 schema, motor name, command/feedback ID, 값 범위를 검증한 뒤 적용한다.

2026-08-13 실물 조종에서 J3가 VR 손목 롤의 반대로 움직이는 것이 확인되어 J3 physical sign을
`+1`에서 `-1`로 교정했다. 최종 physical sign은 J1/J2/J3 순서로 `[-1, +1, -1]`이다. 이는 화면
mirror 옵션 문제가 아니라 DM4340의 raw-positive 방향과 URDF joint-positive 방향의 차이다.

## 앞으로의 재발 방지 체크 순서

실행하거나 파일을 바꾸기 전에 아래를 순서대로 모두 확인한다.

```bash
# 1. 정확한 부모와 submodule commit
git -C /home/dong/echo_v4 show -s --oneline
git -C /home/dong/echo_v4/third_party/robot_arm_vr show -s --oneline

# 2. 실제 listening port와 프로세스 인자
ss -ltnp | grep ':8012'
pgrep -af '05_teleop_sim|robot_arm_vr_bridge'

# 3. runtime --config가 가리키는 JSON과 JSON의 urdf_path
# 4. URDF joint1/joint2/joint3 axis와 limits
# 5. bridge의 calibration path, CAN IDs, feedback IDs, motor signs, kp/kd profile
# 6. no-motion WAIT/HOLD 상태를 확인한 뒤에만 Quest grip으로 RUN
```

`server up`은 `Quest connected`나 `motor moving safely`와 같은 뜻이 아니다. 포트, config, URDF, 물리 sign을 각각 증거로 확인하고, 자동 home은 계속 비활성화한다.

## 현재 검증된 실행 명령

터미널 1에서 3축 bridge를 먼저 띄운다. 기본값이 위의 3-motor calibration, J3 gain profile,
CANable2, AmazingHand 경로를 선택한다.

```bash
cd /home/dong/echo_v4
.venv/bin/roboparty-robot-arm-vr-bridge
```

터미널 2에서 최신 URDF를 쓰는 8012 Quest 화면을 띄운다.

```bash
cd /home/dong/echo_v4/third_party/robot_arm_vr
.venv/bin/python -u scripts/05_teleop_sim.py \
  --config config/robot_arm_temp_j1_j2_updated.json \
  --profile jetson --port 8012 --ip 10.175.216.203 \
  --motors jetson --jetson-host 127.0.0.1 \
  --no-home-on-xr-start --disable-home-button
```

Quest 주소는 `https://10.175.216.203:8012`이고, 브라우저 확인용 대시보드는
`https://10.175.216.203:8012/dashboard`이다. 시작 직후에는 현재 실물 관절각을 읽어 화면만
동기화하고 `HOLD`한다. 오른손 Grip을 누르기 전에는 `RUN`하지 않는다. J3 wrist roll은 오른손
thumbstick X로 제어한다.

관련 문서: [[roboparty-xr-ik-dataset-debugging-ko]], [[jetson-safe-bring-up-runbook]]
