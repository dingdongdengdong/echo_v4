# arm_v2 — 신규 5축 팔 + 평행 그리퍼 텔레오퍼레이션

기존 [README.md](README.md)는 **J1/J2/J3 3축 임시 팔 + AmazingHand** 구성입니다.
이 문서는 새로 들어온 **5축 팔(Damiao DM-J4340P ×5) + Feetech STS3215 평행 그리퍼**
구성만 다룹니다. 두 구성은 config 파일로 갈리며 서로를 지우지 않습니다.

작성 기준: 2026-09-01. 시뮬레이션 검증 완료, 실물 구동은 캘리브레이션 대기 중입니다.

---

## 0. 30초 요약

```bash
# 노트북에서 시뮬만 (모터 없음) — 지금 바로 됩니다
cd third_party/robot_arm_vr
.venv/bin/python -u scripts/05_teleop_sim.py \
  --config config/arm_v2_yaw180.json --no-hand \
  --profile test --motors none

# 젯슨에서 실물 — 캘리브레이션과 부호 실측이 먼저입니다 (§4)
.venv/bin/roboparty-robot-arm-vr-bridge \
  --config third_party/robot_arm_vr/config/arm_v2_yaw180_jetson.json \
  --calibration .local/calibration/right_arm_five_motor.json \
  --no-hand --motor-signs <5개> --kp <5개> --kd <5개>
```

**`--no-hand`가 필수입니다.** 신규 팔에는 AmazingHand가 없어서 빼면
`Frame with name hand_mount not found`로 죽습니다.

---

## 1. 팔 구성

|  | 기존 (README.md) | 신규 (이 문서) |
|---|---|---|
| 관절 | J1/J2/J3 (실구동 3) | **joint1~5** |
| 모터 | Damiao DM-J4340P | Damiao DM-J4340P ×5 |
| 손 | AmazingHand (서보 8) | **STS3215 평행 그리퍼 (서보 1)** |
| 위치 자유도 | 2/3 (랭크 부족) | **3/3** |
| 도달 범위 | 0.24 ~ 0.25 m | **0.08 ~ 0.56 m** |
| 질량 | 1.413 kg | 2.641 kg |
| config | `robot_arm_temp_j1_j2_updated.json` | `arm_v2_yaw180.json` |

기존 3축은 자코비안 랭크가 2/3라 "안팎으로 밀고 당기기"가 안 됐습니다.
신규 팔은 랭크 3/3, 조건수 1.10으로 그 제약이 없습니다.

### URDF 3종 — 왜 나뉘어 있나

```
assets/arm_v2/
  arm_v2.urdf              원본 (그리퍼 조가 mimic 으로 연동)
  arm_v2_nomimic.urdf      좌우 로커 독립
  arm_v2_ik.urdf           ★ 텔레옵이 실제로 쓰는 것
```

**placo가 `<mimic>`을 무시해서** 종속 관절까지 구동 관절로 셉니다. 원본을 IK에
물리면 5축이 9축으로 인식되어 IK가 이상하게 풀립니다. 반드시 `_ik.urdf`를 쓰세요.

화면에서 조가 움직이는 것은 `gripper_link.attach()`가 원본 URDF로 표시용 로봇을
따로 만들어 처리합니다. 그래서 **`<이름>_ik.urdf` 옆에 `<이름>.urdf`가 있어야 합니다.**

---

## 2. ★ joint1 기준을 180° 돌린 이유

config가 두 벌입니다. **`yaw180` 쪽을 쓰세요.**

```
config/arm_v2.json            원본 (기구 담당 인계본 그대로)
config/arm_v2_yaw180.json     ★ joint1 기준 180° 회전
config/arm_v2_jetson.json          위 둘의 젯슨 절대경로 사본
config/arm_v2_yaw180_jetson.json
```

joint1 리밋이 **±178°**라 한 바퀴에서 4°가 모자랍니다. 원본 기준에서는
그 도달 불가 쐐기가 **정면**에 있습니다.

```
joint1 값  →  툴이 향하는 방위각
  -178°  →    2°   ← 리밋
  -172°  →    8°   ← 원본 홈
     0°  →  180°
  +178°  →  358°   ← 리밋

도달 불가 쐐기 : 방위각 358° ~ 2°   ← 정면
```

**±178°는 선이 꼬이지 않게 두는 소프트 리밋**입니다. 그러면 "선이 안 감긴
지점"이 곧 joint1 영점이므로, 기준을 옮기면 쐐기도 같이 옮겨집니다.

|  | 홈에서 joint1 | 홈에서 선 상태 | 리밋까지 여유 |
|---|---|---|---|
| `arm_v2` | −172° | **거의 다 감긴 상태** | 6° |
| `arm_v2_yaw180` | **0°** | **중립** | 178° |

원본 홈은 선이 178° 가까이 감긴 자세로 작업 내내 머무릅니다. 회전본은 홈이
중립이고 좌우로 178°씩 남습니다.

**바뀐 것은 URDF 한 줄뿐이고, 물리적 자세는 완전히 동일합니다.**

```xml
<origin xyz="0 0 0.05505" rpy="0 0 0" />                   <!-- 원본 -->
<origin xyz="0 0 0.05505" rpy="0 0 3.14159265358979" />    <!-- 회전 -->
```

```
                  joint1=-172°(원본)   joint1=0°(회전)
  툴 z 높이            290.8 mm          290.8 mm   동일
  수평거리             161.2 mm          161.2 mm   동일
  툴 아래로             20.7°             20.7°     동일
  카메라 수직성분        +35.1 mm          +35.1 mm  동일
  자코비안 조건수          1.1               1.1     동일
  툴 방위               +8.0°              0.0°    ← 이것만
```

> **조립·캘리브레이션 때:** 팔을 홈 자세에 놓고 **선이 안 감긴 상태**로 만든 뒤
> 그 지점을 모터 영점으로 잡으세요. `yaw180` config가 그것을 전제합니다.

### 홈 자세

```
joint1   joint2   joint3   joint4   joint5
  0.00   +54.51   -84.45   -80.78   +4.90   (deg)
```

팔이 앞으로 뻗어 살짝 아래(20.7°)를 보고, 카메라가 그리퍼 위에 있습니다.
바닥의 물건을 내려다보며 접근하기 좋은 자세입니다.

**첫 통전 때 팔이 홈으로 스스로 크게 움직입니다. 주변을 비우고 비상 정지를
손에 쥐고 시작하세요.**

---

## 3. 노트북에서 시뮬레이션 (모터 없음)

### 3.1 환경 구축 (최초 1회)

```bash
cd third_party/robot_arm_vr
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python numpy placo "uvicorn[standard]" fastapi
uv pip install --python .venv/bin/python "pin>=3.7,<3.8" "cmeel-urdfdom>=4,<5" "cmeel-tinyxml2>=10,<11"
```

> **`uvicorn`이 아니라 `uvicorn[standard]`입니다.** 맨몸 uvicorn에는 WebSocket
> 구현체가 없어서 페이지는 뜨는데 Quest가 절대 연결되지 않습니다. 증상은
> `No supported WebSocket library detected` + `Unsupported upgrade request`가
> 무한 반복되고 "Quest 접속 대기 중"만 나오는 것입니다.
>
> `placo`는 `pin` 없이 설치하면 `libpinocchio_parsers.3.7.0.dylib` 로드 실패로
> import 자체가 안 됩니다. 버전은 [pyproject.toml](pyproject.toml)의
> `kinematics` extra와 맞춘 값입니다.

### 3.2 실행

```bash
cd third_party/robot_arm_vr
.venv/bin/python -u scripts/05_teleop_sim.py \
  --config config/arm_v2_yaw180.json --no-hand \
  --profile test --motors none
```

기동 배너가 이렇게 나와야 합니다.

```
  Quest 접속  : https://<IP>:4523
  대시보드    : https://<IP>:4523/dashboard
  로봇        : arm_v2_yaw180_ik.urdf  5-DOF  EE=tool_frame
  제어 주기   : 30 Hz   스케일 0.861   리치 0.08~0.56 m
```

**IP는 `--ip` 없이 자동 감지됩니다.** 웹서버는 항상 `0.0.0.0`에 바인딩하고,
`--ip`는 화면에 찍을 URL과 인증서 SAN에만 쓰입니다. 인증서는 이 머신의 모든
사설 IP를 SAN에 넣어 자동 발급되므로 망이 바뀌어도 재발급이 필요 없습니다.

터미널의 `Quest 접속:` 줄이 **항상 정답 주소**입니다.

### 3.3 프로파일 = 포트 블록

| 프로파일 | 웹 | UDP 지령/상태/비컨 | 용도 |
|---|---|---|---|
| `jetson` | 4443 | 5005/5006/5007 | 실물 |
| `isaac` | 4453 | 5015/5016/5017 | Isaac Sim |
| `test` | 4523 | 5085/5086/5087 | 손으로 돌려볼 때 |

같은 프로파일을 두 번 띄우면 **"이미 쓰이고 있습니다"로 안전하게 종료**합니다.
`run.sh`는 같은 포트의 이전 서버를 자동 정리하지만, `05_teleop_sim.py`를 직접
실행하면 그 정리가 없습니다.

```bash
lsof -nP -iTCP:4523 -sTCP:LISTEN -t | xargs kill     # 끄기
```

### 3.4 서버가 검증하는 것

```
[ ] 배너에 arm_v2_yaw180_ik.urdf  5-DOF  EE=tool_frame
[ ] /state 의 링크가 10개 (팔 6 + rocker_l/r, jaw_l/r 4)
[ ] dof_mismatch 가 None
[ ] /mesh/arm/link5.stl 이 200
[ ] 대시보드 팔 선택기에 "신규 5축 v2 (joint1 기준 180° 회전)"
[ ] VR HUD 첫 줄에 "메시 10/10"
[ ] 오른손 트리거를 당기면 화면에서 조가 닫힌다
```

링크가 6개로 나오면 `gripper_link.attach()`가 표시용 원본 URDF를 못 찾아서 IK
로봇(그리퍼 관절이 fixed)만 보이는 것입니다. `arm_v2_yaw180_ik.urdf` 옆에
`arm_v2_yaw180.urdf`가 있는지 확인하세요.

---

## 4. VR 조종

| 버튼 | 동작 |
|---|---|
| 오른손 **Grip** | 꾹 누른 채 조종. 떼면 그 자리 정지 |
| 오른손 **트리거** | 그리퍼 여닫기 (0~1이 그대로 `Command.grasp`) |
| 오른손 **A** | 홈 자세 복귀 (`--a-home-speed-scale`, 기본 1/3 속도) |
| 오른손 **B** | 소프트웨어 HOLD **토글** |
| 왼손 X / Y | 미러 토글 / yaw +90° |

> **B는 토글입니다.** 예전에는 한 번 누르면 프로세스 재시작 전까지 안 풀리는
> 래치였는데, 조종 중에 발이 묶여서 토글로 바뀌었습니다. 재개할 때 기존 무장
> 로직이 실제 관절 자세로 IK를 다시 맞추므로 HOLD 전 지령으로 튀지 않습니다.
>
> 잠깐 멈추고 싶을 때는 **Grip을 놓으세요.** 그게 원래 그 용도입니다.

조종 중 연결이 끊기면 `⚠️ Quest 연결 끊김`이 뜹니다. 헤드셋에서 **[Start XR]**을
다시 누르세요. (헤드셋을 벗거나 슬립에 들면 WebXR 세션이 끊깁니다.)

### 그리퍼 값 대응

| 트리거 | 개구 | 조 간격 |
|---|---|---|
| 0.00 (뗌) | 57.9 mm | 84.1 mm |
| 0.50 | 29.2 mm | 54.5 mm |
| 1.00 (당김) | 0.5 mm | 25.5 mm |

**최대 개구가 57 mm입니다.** 캔(∅66) · 페트병 몸통(∅65) · 종이컵 입구(∅80)는
못 잡습니다. 태스크 품목을 정할 때 1차 필터가 됩니다.

---

## 5. 젯슨 실물 구동

### 5.1 순서 — 건너뛰지 마세요

```
1) CAN 프로브        모터가 다 응답하는지, ID가 무엇인지
2) 선 중립 맞추기     joint1 영점이 될 자세
3) 5축 캘리브레이션
4) 부호 실측          ★ 이걸 안 하면 관절이 반대로 갑니다
5) 가짜 젯슨으로 확인   통전 전 프로토콜만
6) 브릿지 기동
7) 그리퍼 연결        팔이 안정된 뒤에
```

### 5.2 CAN 프로브

```bash
cd /home/dong/echo_v4
.venv/bin/roboparty-can-probe --port /dev/roboparty-can --motor-ids 1,2,3,4,5 --timeout 0.2
```

5개가 다 안 잡히면 CAN ID가 1~5가 아닙니다. 여기서 나온 ID를 이후 `--motor-ids`에
그대로 쓰세요.

### 5.3 5축 캘리브레이션

```bash
.venv/bin/roboparty-calibrate-two-motors \
  --port /dev/roboparty-can \
  --motor-ids 0x01,0x02,0x03,0x04,0x05 \
  --joint-names joint1,joint2,joint3,joint4,joint5 \
  --lerobot \
  --output .local/calibration/right_arm_five_motor.json
```

명령 이름이 `two-motors`지만 **모터 개수에 대해 범용**입니다.

절차는 (1) 관절을 중간/홈 자세에 놓고 ENTER, (2) 각 관절을 전체 가동범위로 움직인
뒤 ENTER 입니다.

> **시작하면서 토크를 끕니다. 팔을 손으로 받치고 시작하세요.**
> joint2·joint3는 중력을 받아서 훅 떨어집니다.

캘리브레이션 파일이 없으면 브릿지가 아예 안 뜹니다.

### 5.4 부호 실측

캘리브레이션 파일에 **부호는 들어가지 않습니다.** 관절을 하나씩 움직여서 URDF
방향과 실물 방향이 같은지 확인하고, 그 결과가 `--motor-signs` 5개 값이 됩니다.

**`joint5` 축이 `(0,0,-1)`이라 여기가 제일 뒤집히기 쉽습니다.**

### 5.5 브릿지 기동

```bash
.venv/bin/roboparty-robot-arm-vr-bridge \
  --config third_party/robot_arm_vr/config/arm_v2_yaw180_jetson.json \
  --calibration .local/calibration/right_arm_five_motor.json \
  --no-hand \
  --motor-signs  <j1> <j2> <j3> <j4> <j5> \
  --kp <5개> --kd <5개> \
  --velocity-scale 0.10
```

**4축 이상에서는 부호와 게인의 기본값을 쓰지 않고 거부합니다.** 저장된
`DEFAULT_MOTOR_SIGNS = (-1, +1, -1)`과 `DEFAULT_KP = (120, 180)`은 짧은 J1/J2/J3
팔에서 실측한 값이라, 다른 팔에 조용히 적용되면 관절이 반대로 갑니다.

```
--motor-signs is required for 5 axes; the stored defaults were
measured on the short J1/J2/J3 arm
```

2축·3축 호출은 기존 기본값과 플래그(`--motor1-sign` 등)가 그대로 동작합니다.

### 5.6 그리퍼

팔이 안정된 뒤 따로 붙이세요.

```bash
  --gripper --gripper-port /dev/<시리얼> \
  --gripper-id <ID> \
  --gripper-zero-deg <실측> --gripper-sign <+1 또는 -1>
```

포트는 CAN이 아니라 시리얼입니다. `ls -l /dev/serial/by-id/`로 찾으세요.

`--gripper-zero-deg`와 `--gripper-sign`은 `gripper_map`의 스윕 좌표(10~120°)를
실물 서보 영점에 대응시키는 값입니다. **안전한 기본값이 없어서 명시하지 않으면
거부합니다.**

> **팔에는 안전계층이 있지만 그리퍼에는 없습니다.**
> [parallel_gripper.py](lerobot_robot_roboparty/parallel_gripper.py)가 최소한을
> 넣어뒀습니다 — 서보각을 피팅 구간(10~120°) 밖으로 절대 내보내지 않고,
> 165°/s로 슬루 제한을 겁니다. 트리거를 한 번에 확 당겨도 완전히 닫히는 데
> 약 0.6초가 걸리므로 링크가 끊기거나 값이 튀어도 조가 순간적으로 닫히지 않습니다.
>
> **첫 통전은 조 사이에 아무것도 없이, 손 넣지 말고** 확인하세요.
> 파지력은 아직 측정하지 않았습니다.

### 5.7 통전 전 가짜 젯슨으로

```bash
# 터미널 1
.venv/bin/python scripts/07_fake_jetson.py --profile test
# 터미널 2
.venv/bin/python -u scripts/05_teleop_sim.py --config config/arm_v2_yaw180.json \
  --no-hand --profile test --motors jetson --jetson-host 127.0.0.1
```

---

## 6. 코드에서 바뀐 것

### `lerobot_robot_roboparty/robot_arm_vr_bridge.py`

```python
SUPPORTED_DOF = (2, 3, 4, 5)   # 이전: cfg.dof not in (2, 3) 이면 거부
TUNED_DOF     = (2, 3)         # 저장된 기본값이 유효한 범위
```

`RawRelativeDMBackend`는 원래부터 모터 개수에 범용이었고, `main()`만 2/3축
전제였습니다. 부호·게인·CAN ID를 축별 인자로 바꾸고, 4축 이상에서는 기본값을
거부하도록 했습니다.

새 인자: `--motor-signs`, `--motor-ids`, `--kp`/`--kd`(개수 가변), `--gripper*`

### `lerobot_robot_roboparty/parallel_gripper.py` (신규)

`Command.grasp`(0~1) → 개구(mm) → STS3215 서보각 변환. 변환식은 기구 담당의
실측 56점 피팅이고, 그 `gripper_map.py`는 URDF 옆에 있습니다 —
피팅이 개정되면 팔과 함께 오도록.

### `third_party/robot_arm_vr`

- `assets/arm_v2/` — URDF 3종, 메시 10, 렌더 13, `gripper_map.py`
- `config/arm_v2*.json` — 4벌 (원본/회전 × 맥/젯슨)
- `src/rpo_teleop/gripper_link.py`, `scripts/10~14_*.py`
- 젯슨 갈래 병합 — `speed_scale` 프로토콜, B 토글, A 홈 감속

---

## 7. 알려진 문제

### 절대경로 config

**이 저장소의 모든 config가 `urdf_path`에 절대경로를 박습니다.** 그래서 젯슨에서
쓴 config는 다른 머신에서 못 엽니다. 그 때문에 config가 머신별로 나뉩니다
(`arm_v2.json` / `arm_v2_jetson.json`).

같은 이유로 이 테스트들이 **젯슨이 아닌 곳에서 실패합니다.** 정상입니다.

```
robot_arm_vr : tests/test_wrist_roll.py::test_updated_arm_grip_pose_roll_drives_j3_...
echo_v4      : tests/test_robot_arm_vr_bridge.py::test_updated_vr_model_enables_j3_...
```

### 서브모듈 핀이 깨져 있음

`third_party/robot_arm_vr`와 `third_party/lerobot` 둘 다 **원격에 없는 커밋**을
가리킵니다.

```
fatal: remote error: upload-pack: not our ref f33eaf7474...
```

젯슨 로컬에서 커밋만 하고 push하지 않아 생긴 문제입니다. 클론 직후 서브모듈
디렉터리가 비어 보이면 이것입니다. `git checkout -f HEAD`로 워크트리를 복구한 뒤
가장 가까운 브랜치를 쓰세요.

### 미해결

```
[ ] 그리퍼 서보 포트 · ID          (문서 어디에도 없음)
[ ] 그리퍼 영점 · 부호              (실물 실측)
[ ] 5축 부호 · kp/kd               (실물 실측)
[ ] 그리퍼 파지력                   (미측정 — 가벼운 쓰레기가 대상이라 과하면 찌그러짐)
[ ] 카메라를 link5 −x 로 이설할지    (기구 담당 판단)
```

---

## 8. 참고

- [README.md](README.md) — 기존 J1/J2/J3 + AmazingHand 구성
- [docs/howtorun_dataset_collection.md](docs/howtorun_dataset_collection.md) — 데이터셋 수집 상세
- `third_party/robot_arm_vr/README.md` — VR 서버 자체 문서
- 인계 문서 `67_기구담당_원본_v2.txt` §7 함정 목록, `68_젯슨_인수인계.txt`
