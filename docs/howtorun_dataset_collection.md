# HOWTORUN 상세 설명 — 8012 J1·J2·J3 dataset 수집

이 문서는 README 상단의 `HOWTORUN` 명령을 **왜 그 순서로 실행하는지**, 각 프로세스와 옵션이
무슨 역할을 하는지, 어떤 조건에서 기록을 중단해야 하는지를 설명합니다. 실제 실행 명령은
[README의 HOWTORUN](../README.md#howtorun--8012-j1j2j3-dataset-수집)을 기준으로 합니다.

## 1. 수집 대상

현재 수집 경로는 다음 데이터를 15 Hz LeRobot dataset으로 저장합니다.

| Dataset field | 값 | 출처 | 사용하는 이유 |
|---|---|---|---|
| `observation.state` | J1/J2/J3 실제 관절각 `q_act` | DM4340 CAN feedback | 정책이 실제 로봇 상태를 관측하도록 하기 위해 |
| `action` | J1/J2/J3 최종 명령각 `q_cmd` + AmazingHand grasp 0–100% | Quest → IK → safety + Trigger | 사람이 시연한 팔과 손 명령을 학습 target으로 사용하기 위해 |
| `observation.images.front` | 640×480 RGB | Generic USB2.0 PC CAMERA | 작업 공간과 물체의 전역 상태를 보기 위해 |
| `observation.images.wrist` | 640×480 RGB | D435i RGB node | 손과 물체의 근접 상태를 보기 위해 |
| `task` | 자연어 작업 설명 | `TASK` 환경변수 | episode가 어떤 작업의 시연인지 구분하기 위해 |

관절 순서는 항상 아래와 같습니다.

```text
right_arm_joint_1.pos
right_arm_joint_2.pos
right_arm_joint_3.pos
```

`action`은 위 관절 순서 뒤에 `right_hand_grasp.pos`를 추가합니다. 실제 손 위치 feedback은 아직
`observation.state`에 포함하지 않습니다.

AmazingHand는 이 dataset의 state/action 축에 넣지 않습니다. 현재 손은 명령을 보낼 수 있지만 실제
관절 위치 feedback이 없으므로, 명령값을 실제 observation처럼 저장하면 학습 데이터의 의미가
깨집니다.

## 2. 전체 데이터 흐름

```text
Quest 2 controller
    │ HTTPS/WebXR :8012
    ▼
05_teleop_sim.py
    │ controller pose → clutch → J1/J2/J3 IK → q_cmd
    │ UDP command/state/beacon :5005/:5006/:5007
    ▼
roboparty-robot-arm-vr-bridge
    │ safety limits → DM4340 command IDs 1/2/3
    │ CAN feedback IDs 0x11/0x12/0x13 → q_act
    ▼
8012 /state + C920 + D435i RGB
    │ read-only sidecar
    ▼
record_lerobot_8012_dataset.sh
    └── LeRobotDataset: state/action/front/wrist/task
```

세 프로세스를 나눈 이유는 하드웨어 소유권을 하나로 유지하기 위해서입니다.

- bridge만 CANable과 AmazingHand serial을 엽니다.
- WebXR 서버만 Quest session과 IK 상태를 소유합니다.
- recorder는 기존 `/state`와 카메라만 읽으며 CAN, hand serial, WebXR server를 다시 열지 않습니다.

이 분리를 지키면 두 프로그램이 같은 CAN adapter를 동시에 열거나, 두 WebXR site가 서로 다른
controller state를 받는 문제를 피할 수 있습니다.

## 3. 사전 CAN probe

```bash
.venv/bin/roboparty-can-probe \
  --port /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2095336A5845-if00 \
  --motor-ids 1,2,3 --timeout 0.2
```

### 기능

- motor command IDs `1,2,3`에 상태 refresh 요청만 보냅니다.
- 예상 feedback IDs는 각각 `0x11,0x12,0x13`입니다.
- position, velocity, torque, temperature, motor state를 출력합니다.
- enable, zero, position command는 보내지 않습니다.

### 왜 사용하는가

USB 장치가 존재한다는 사실만으로는 세 모터가 CAN에서 응답하는지 알 수 없습니다. 세 ID가 모두
`HIT`이고 `state=disabled`인지 확인해야 CAN-H/CAN-L, 전원, 1 Mbps bitrate, motor ID 경로가 실제로
정상임을 확인할 수 있습니다.

### 주의

bridge가 실행 중일 때는 같은 CANable serial port를 probe로 다시 열지 않습니다. 종료 점검에서는
WebXR 서버를 먼저 종료하고 bridge를 종료한 다음 probe를 실행합니다.

## 4. 터미널 1 — hardware bridge

```bash
cd /home/dong/echo_v4
.venv/bin/roboparty-robot-arm-vr-bridge \
  --velocity-scale 0.25
```

### 기능

- CANable과 J1/J2/J3 DM4340을 한 프로세스에서 소유합니다.
- AmazingHand serial을 엽니다.
- motor IDs `1/2/3`, feedback IDs `0x11/0x12/0x13`을 사용합니다.
- physical signs `J1=-1`, `J2=+1`, `J3=-1`을 적용합니다.
- calibration, URDF limits, velocity limit, soft start와 watchdog을 적용합니다.
- UDP `5005/5006/5007`로 WebXR 서버와 command/state/beacon을 교환합니다.
- 종료 시 J1/J2/J3와 AmazingHand torque를 disable합니다.

### `--velocity-scale 0.25`를 쓰는 이유

URDF/config의 최대 관절속도를 그대로 쓰지 않고 safety layer가 허용하는 속도를 25%로 줄입니다.
첫 dataset 수집에서는 최고 속도보다 사람이 반복 가능한 느린 시연과 정지 여유가 중요합니다.
이 값은 Quest mapping scale이 아니라 **모터 safety velocity limit의 배율**입니다.

## 5. 터미널 2 — Quest/WebXR와 IK

```bash
cd /home/dong/echo_v4/third_party/robot_arm_vr
.venv/bin/python -u scripts/05_teleop_sim.py \
  --config config/robot_arm_temp_j1_j2_updated.json \
  --profile jetson \
  --port 8012 \
  --ip 10.175.216.203 \
  --motors jetson \
  --jetson-host 127.0.0.1 \
  --no-home-on-xr-start
```

### 프로세스 기능

- `https://10.175.216.203:8012`에서 Quest WebXR page를 제공합니다.
- 오른쪽 controller pose를 받아 clutch와 IK를 거쳐 J1/J2/J3 목표각을 계산합니다.
- dashboard와 `/state`에 실제 관절각, 명령각, motor/Quest 상태를 게시합니다.
- 오른쪽 Grip을 잡은 동안만 새로운 IK target을 추종합니다.

### 옵션별 이유

| 옵션 | 기능 | 이 설정을 쓰는 이유 |
|---|---|---|
| `--config robot_arm_temp_j1_j2_updated.json` | 현재 3-DOF URDF와 home/limit/scale 선택 | 예전 4443/default 팔과 섞이지 않게 함 |
| `--profile jetson` | 실물용 UDP 포트 block 선택 | Isaac/test 포트와 명령이 섞이는 것을 방지 |
| `--port 8012` | Quest HTTPS와 `/state` port 고정 | recorder가 정확한 runtime만 읽게 함 |
| `--ip 10.175.216.203` | Quest가 접근할 LAN interface 선택 | `127.0.0.1`이나 Tailscale 주소는 Quest LAN 접속용이 아님 |
| `--motors jetson` | IK 결과를 UDP bridge로 전달 | `none`은 visualization-only라 실물 팔이 움직이지 않음 |
| `--jetson-host 127.0.0.1` | 같은 Jetson의 bridge로 직접 연결 | beacon 탐색 대신 현재 단일 장비 구성을 명시 |
| `--no-home-on-xr-start` | Start XR 직후 자동 home 금지 | headset 연결만으로 팔이 자동 이동하는 것을 방지 |

`third_party/robot_arm_vr/run.sh --temp` 또는 port `4443`은 standalone 기본 설정입니다. 현재 실물
3축 dataset 수집에서는 8012 명령과 섞지 않습니다.

## 6. Quest 조작과 상태

| 입력/상태 | 의미 |
|---|---|
| Start XR | WebXR controller tracking 시작 |
| 오른쪽 Grip | 현재 controller/robot pose를 기준으로 clutch engage |
| controller 이동·회전 | unified IK가 J1/J2/J3 목표를 계산 |
| Grip 해제 | 새로운 추종을 멈추고 HOLD |
| 오른쪽 A | config의 `home_q`로 이동 요청 |
| 오른쪽 B | software HOLD/재개 토글. 1회는 HOLD, 다시 1회는 현재 실물 자세에서 재개 준비 |
| 오른쪽 Trigger | AmazingHand grasp; `right_hand_grasp.pos` action에 0–100%로 기록 |

WebXR의 mirror는 물리 motor 방향 수정 기능이 아닙니다. J3 방향 문제가 있으면 화면 mirror가 아니라
bridge의 physical sign과 배선을 확인합니다.

## 7. 터미널 3 — `--validate-only`

```bash
cd /home/dong/echo_v4
./scripts/record_lerobot_8012_dataset.sh --validate-only
```

### 기능

- 정확한 port `8012`의 `/state`만 읽습니다.
- robot name이 `robot_arm_temp_j1_j2_updated.urdf`이고 DOF가 3인지 확인합니다.
- Quest tracking이 `ok`이고 waiting 상태가 아닌지 확인합니다.
- motor link, `TRIP`, motor error와 J1/J2/J3 `q_act`/`q_cmd`를 확인합니다.
- C920와 D435i RGB에서 각각 실제 640×480 frame 한 장을 읽습니다.
- dataset을 만들지 않고 motor command도 보내지 않습니다.

### 왜 사용하는가

웹페이지가 열린 것, UDP link가 살아 있는 것, 마지막 관절값이 화면에 보이는 것은 실제 recording
준비 완료와 다릅니다. control loop가 `TRIP`된 뒤에도 마지막 cached 관절값이 잠시 보일 수 있습니다.
따라서 아래 두 PASS가 모두 있어야 recorder로 넘어갑니다.

```text
PASS correct 8012 site
PASS Quest tracking and motor health
```

## 8. Dataset recorder

```bash
DATASET_ROOT="outputs/lerobot_datasets/cube-pick-v1-$(date +%Y%m%d-%H%M%S)" \
DATASET_REPO_ID="local/roboparty-cube-pick-v1" \
NUM_EPISODES=20 \
TASK='Pick up the cube and place it in the tray' \
FPS=15 \
MIN_MOTION_RAD=0.02 \
PUSH_TO_HUB=false \
./scripts/record_lerobot_8012_dataset.sh
```

### 환경변수별 기능과 이유

| 변수 | 기능 | 이유 |
|---|---|---|
| `DATASET_ROOT` | 실제 로컬 저장 directory | timestamp를 넣어 기존 수집 결과 덮어쓰기를 방지 |
| `DATASET_REPO_ID` | LeRobot dataset 논리 ID | 학습/시각화 단계에서 dataset을 식별 |
| `NUM_EPISODES` | 저장할 성공 episode 수 | 폐기 episode를 제외하고 목표 수량까지 반복 |
| `TASK` | 모든 frame에 연결할 작업 문장 | language-conditioned policy와 dataset 구분에 사용 |
| `FPS=15` | state/action/camera sampling rate | 두 카메라와 Jetson 부하를 함께 만족하는 보수적 공통 속도 |
| `MIN_MOTION_RAD=0.02` | 실제 관절 이동 acceptance threshold | Quest만 연결된 stationary episode가 저장되는 것을 방지 |
| `PUSH_TO_HUB=false` | 로컬 저장만 수행 | 첫 검증 전 불완전 dataset 업로드를 방지 |

### Motion gate

각 episode가 끝나면 recorder가 CAN feedback 기반 `q_act`의 관절별 범위를 계산합니다.

- `ENGAGED` frame이 0개면 폐기합니다.
- 세 관절 중 어느 것도 `0.02 rad` 이상 움직이지 않았으면 폐기합니다.
- 통과하면 관절별 motion과 engaged frame 수를 출력하고 episode를 저장합니다.

이 gate는 큰 동작을 요구하는 장치가 아니라 **완전히 정지한 잘못된 episode를 걸러내는 최소 조건**입니다.
작업 품질, 물체 성공 여부, 카메라 가림 여부는 사람이 별도로 판단해야 합니다.

## 9. Episode lifecycle

| 키 | recording 단계 | reset 단계 |
|---|---|---|
| `n` / `→` | 현재 episode 종료 후 저장 검사 | 다음 episode 기록 시작 |
| `r` / `←` | 현재 buffer 폐기 후 같은 episode 재시작 | 사용하지 않음 |
| `q` / `ESC` | 조기 종료하고 저장된 episode finalize | 조기 종료하고 finalize |

한 episode의 권장 순서:

1. 물체와 팔을 동일한 시작 조건에 둡니다.
2. `RECORDING episode ...`를 확인합니다.
3. XR tracking이 유효해지면 `XR CONNECTED` 시각과 episode 경과시간이 표시됩니다.
4. 수집 중에는 5초마다 `COLLECTING elapsed=HH:MM:SS frames=... arm=ENGAGED|HOLD`로 진행 상태를 확인합니다.
5. Grip을 잡고 작업을 천천히 수행합니다.
6. 작업 성공 후 Grip을 놓습니다.
7. `n`을 눌러 episode를 저장합니다.
8. reset 단계에서 물체를 원위치하고 `n`을 눌러 다음 episode를 시작합니다.
9. 실패, 충돌, 카메라 가림이 있으면 `r`로 해당 episode를 버립니다.

## 10. 안전 상태의 의미

| 상태 | 의미 | Dataset 수집 |
|---|---|---|
| `IDLE` | torque disabled | 금지 |
| `HOLD` | torque는 있지만 새로운 target을 추종하지 않음 | 준비 상태 |
| `RUN` / `ENGAGED` | Grip 기반 target 추종 중 | 기록 가능 |
| `TRIP` | watchdog, feedback loss 또는 제어 예외로 안전 정지 | 금지; 원인 확인 후 사람이 해제 |

`TRIP`은 recorder가 만드는 상태가 아닙니다. hardware bridge의 기존 safety state이며, recorder는
그 상태에서 잘못된 dataset이 저장되지 않도록 거부합니다. CAN feedback timeout이 한 번 발생하면
bridge는 세 모터를 disable하고 control loop를 정지합니다. 이후 보이는 관절값은 마지막 cached 값일
수 있으므로, runtime을 종료한 뒤 feedback-only probe로 J1/J2/J3를 다시 확인합니다.

## 11. 정상 종료

종료 순서는 항상 다음과 같습니다.

1. recorder에서 `q`를 눌러 dataset을 finalize합니다.
2. 터미널 2 WebXR/teleop을 `Ctrl+C`로 종료합니다.
3. 터미널 1 bridge를 `Ctrl+C`로 종료합니다.
4. passive CAN probe에서 J1/J2/J3가 모두 `state=disabled`인지 확인합니다.

bridge부터 먼저 종료하면 WebXR 서버가 계속 command를 보내는 상태와 실제 hardware owner가 사라지는
상태가 겹칩니다. recorder를 강제 종료하면 마지막 episode/video encoding이 finalize되지 않을 수
있으므로 먼저 `q`를 사용합니다.

## 12. 자주 발생하는 실패

### `validation failed: waiting for Quest Start XR tracking`

Quest에서 `https://10.175.216.203:8012`를 열고 인증서 경고를 승인한 뒤 **Start XR**을 누릅니다.
headset sleep이나 WebXR 종료 후에는 다시 연결해야 합니다.

### `validation failed: 8012 motor trip is active`

기록하지 않습니다. WebXR 서버를 먼저, bridge를 다음으로 종료한 뒤 feedback-only probe로 J1/J2/J3를
확인합니다. 응답 누락 원인을 해결하지 않고 자동으로 trip을 반복 해제하지 않습니다.

### `DISCARD episode: the arm was never engaged`

Quest tracking만 연결되고 오른쪽 Grip을 잡지 않은 상태입니다. Grip을 잡고 실제 작업을 수행합니다.

### `DISCARD episode: measured joint motion ... below ...`

팔의 실제 움직임이 `MIN_MOTION_RAD`보다 작았습니다. threshold를 무조건 낮추기보다 Quest mapping과
실물 추종 여부를 확인한 뒤 episode를 다시 기록합니다.

### `Missing required camera`

카메라 USB 연결과 `/dev/v4l/by-id/` 이름을 다시 확인합니다. 장치 번호 `/dev/videoN`은 재연결 후
바뀔 수 있으므로 가능한 한 stable by-id path를 사용합니다.
