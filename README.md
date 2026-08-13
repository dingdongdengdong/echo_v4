# Roboparty Quest 2 → LeRobot 오른팔 텔레오퍼레이션

이 저장소는 handoff의 현재 실물용 3축 임시 팔 URDF를 기준으로 J1·J2·J3와 AmazingHand를 Meta Quest 2로 조작하고, 그 결과를 Hugging Face LeRobot 데이터셋/ACT 학습 형식으로 기록하기 위한 통합을 포함합니다. J4/J5 쪽 생략된 링크는 IK 체인에 넣지 않습니다.

## 현재 검증된 8012 J1·J2·J3 실행 경로

> **이 실물 구성에서는 `4443`이나 `third_party/robot_arm_vr/run.sh --temp`를 사용하지 않습니다.**
> 그 경로는 standalone 기본 config를 선택합니다. 현재 기준은 port `8012`,
> `robot_arm_temp_j1_j2_updated.json`, physical signs `[-1, +1, -1]`입니다.

먼저 장치가 모두 보이는지 확인합니다.

```bash
ls -l /dev/serial/by-id/
.venv/bin/roboparty-can-probe \
  --port /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2095336A5845-if00 \
  --motor-ids 1,2,3 --timeout 0.2
```

두 터미널에서 아래 순서로 실행합니다.

```bash
# 터미널 1: 실물 J1/J2/J3 + AmazingHand bridge
cd /home/dong/echo_v4
.venv/bin/roboparty-robot-arm-vr-bridge
```

```bash
# 터미널 2: 최신 3-DOF URDF를 쓰는 Quest/WebXR 서버
cd /home/dong/echo_v4/third_party/robot_arm_vr
.venv/bin/python -u scripts/05_teleop_sim.py \
  --config config/robot_arm_temp_j1_j2_updated.json \
  --profile jetson --port 8012 --ip 10.175.216.203 \
  --motors jetson --jetson-host 127.0.0.1 \
  --no-home-on-xr-start --disable-home-button
```

- Quest: `https://10.175.216.203:8012` → **Start XR**
- Dashboard: `https://10.175.216.203:8012/dashboard`
- 시작 후 현재 실물 자세로 동기화하지만 오른손 **Grip** 전에는 `HOLD`입니다.
- J3 wrist roll은 오른손 thumbstick X입니다.
- J3가 반대로 보일 때 WebXR mirror를 켜는 것이 아니라 bridge의 physical sign을 확인합니다.
- 종료는 **텔레옵(터미널 2) 먼저**, bridge(터미널 1) 다음으로 `Ctrl+C`를 누르고, passive probe에서
  J1/J2/J3가 모두 `state=disabled`인지 확인합니다.

## 실행 구조

```text
# 현재 3축 + AmazingHand 구성
Meta Quest 2 ──HTTPS/WebXR──> Jetson (Vuer + LeRobot + 카메라)
                                  ├── USB/CAN ──> DM4340 3축
                                  └── USB serial ──> AmazingHand 8서보

# 향후 5축 구성
Meta Quest 2 ──HTTPS/WebXR──> Jetson/Mac ──Tailscale──> Ubuntu 22.04 ROS 2 bridge
```

- **Jetson**: 현재 3축 팔, AmazingHand, 두 RGB 카메라, Quest WebXR와 LeRobot 기록을 모두 실행합니다.
- **Mac**: 더 이상 현재 3축 구성의 필수 장치가 아니며 개발/점검용으로만 사용할 수 있습니다.
- **Ubuntu 22.04 서버**: 향후 5축 ROS 2 팔을 사용할 때의 선택 구성입니다.
- LeRobot 0.6은 Python 3.12+, ROS 2 Humble은 기본 Python 3.10이므로 두 프로세스를 분리합니다.
- 서버 포트는 공인 인터넷에 노출하지 않고 `tailscale0`에서만 허용합니다.

> 로봇이 USB로 Mac에만 연결되어 있고 Ubuntu 서버에서 `/joint_states`를 볼 수 없다면, 먼저 로봇 제조사 드라이버를 Ubuntu에서 실행하거나 해당 ROS 토픽을 Ubuntu가 접근할 수 있게 해야 합니다. 이 통합의 로봇 경계는 ROS 2 토픽입니다.

## Jetson 단독 실행 준비

LeRobot과 AmazingHandControl은 이 저장소의 Git submodule로 고정되어 있습니다. `uv`가 Python 3.12
환경을 생성하므로 Ubuntu 기본 Python 버전과 분리됩니다.

```bash
sudo apt update
sudo apt install -y git git-lfs curl openssl v4l-utils can-utils
git lfs install
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"

mkdir -p ~/Robotics && cd ~/Robotics
git clone --recurse-submodules https://github.com/dingdongdengdong/roboparty_xr_teleop.git
cd roboparty_xr_teleop
git submodule update --init --recursive
uv sync --extra hardware
```

USB 장치와 카메라를 확인합니다.

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
v4l2-ctl --list-devices
sudo usermod -aG dialout "$USER"   # 적용하려면 한 번 로그아웃/로그인
```

Quest가 접근할 수 있는 Jetson의 같은 Wi-Fi/LAN 주소로 인증서를 생성합니다. PEM 파일은 Git에
포함되지 않습니다.

```bash
JETSON_LAN_IP=192.168.0.30
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout key.pem -out cert.pem -subj '/CN=roboparty-jetson' \
  -addext "subjectAltName=IP:${JETSON_LAN_IP}"
```

Quest 브라우저 주소는 `https://<JETSON_LAN_IP>:8012/?ws=wss://<JETSON_LAN_IP>:8012`입니다.

## 안전 동작

- 현재 **LeRobot 2모터 + AmazingHand 통합 경로는 유효한 Quest tracking이 들어오면 자동 engage**합니다.
- **B**: J1·J2 즉시 disarm, **A**: B 이후 rearm. grip/squeeze를 반복해도 팔 torque나 IK 기준점은 바뀌지 않습니다.
- 오른쪽 **trigger**는 AmazingHand grasp만 제어하며 팔 arm/disarm과 독립입니다.
- 컨트롤러 추적이 250 ms 이상 끊기면 현재 관절 위치를 유지합니다.
- 최초 tracking 또는 A rearm 순간의 상대 좌표를 사용하므로 Quest 좌표 원점으로 팔이 점프하지 않습니다.
- 프레임당 EE 이동 3 cm, workspace ±20 cm, 명령당 관절 변화 기본 0.07 rad로 제한합니다.
- ROS 상태가 0.5초보다 오래되거나 6개 관절이 완전하지 않으면 서버가 명령을 거부합니다.
- 최초 진단은 반드시 `--robot.command_enabled=false`로 수행하세요.

## 1. Ubuntu 22.04 서버 설정

서버 Tailscale IP는 `100.96.41.100`을 기본값으로 사용합니다.

```bash
git clone https://github.com/Roboparty/roboparty_xr_teleop.git
cd roboparty_xr_teleop
source /opt/ros/humble/setup.bash

# Mac과 서버에 똑같이 설정할 긴 임의 토큰
export ROBOPARTY_BRIDGE_TOKEN="$(openssl rand -hex 32)"
printf '%s\n' "$ROBOPARTY_BRIDGE_TOKEN"   # 안전한 경로로 Mac에 한 번 복사

python3 -c "import rclpy; from sensor_msgs.msg import JointState; print('ROS 2 OK')"
python3 teleop/ros_bridge.py --bind-host 100.96.41.100 --port 8765
```

UFW 사용 시 Tailscale 인터페이스에만 허용합니다.

```bash
sudo ufw allow in on tailscale0 to any port 8765 proto tcp
```

별도 터미널에서 상태를 확인합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states --once
ros2 topic info /joint_ref_states
```

`/joint_states`에는 다음 이름과 radian 위치가 모두 있어야 합니다.

```text
right_motor0 right_motor1 right_motor2 right_motor3 right_motor4 right_gripper
```

로봇 드라이버의 이름이 다르면 드라이버 측 매핑 또는 `teleop/ros_bridge.py`의 이름 매핑을 실제 하드웨어에 맞춰야 합니다.

## 2. Mac 설정

```bash
cd /path/to/Robotics/roboparty_xr_teleop
git submodule update --init --recursive

python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip

# 저장소에 고정된 LeRobot submodule을 사용합니다.
pip install -e "third_party/lerobot[dataset]"
pip install -e ".[hardware,kinematics]"

export ROBOPARTY_BRIDGE_TOKEN='<Ubuntu에서 생성한 동일한 토큰>'
ping 100.96.41.100
```

이 저장소에서 자동 생성한 로컬 환경을 사용할 때는 `source .local/mac.env` 한 줄로 토큰과 현재 IP 설정을 불러올 수 있습니다. `.local/`은 Git에서 제외됩니다.

현재 구성처럼 서버에 ROS 2가 직접 설치되어 있지 않고 Docker를 사용할 경우 다음 명령으로 동일한 브리지를 재배포할 수 있습니다.

```bash
scripts/deploy_ros_bridge_docker.sh 100.96.41.100 .local/secrets/bridge_token
```

### Quest용 HTTPS 인증서

Quest와 Mac이 같은 Wi-Fi에 있어야 합니다. `<MAC_LAN_IP>`는 Tailscale IP가 아니라 Quest가 접근할 수 있는 Mac의 Wi-Fi IP입니다.

```bash
MAC_LAN_IP=192.168.0.20   # 실제 값으로 변경
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout key.pem -out cert.pem -subj '/CN=roboparty-mac' \
  -addext "subjectAltName=IP:${MAC_LAN_IP}"
```

Quest 브라우저에서 다음 URL을 열고 인증서 경고를 승인한 뒤 `Enter VR`/`Pass-Through`를 선택합니다.

```text
https://<MAC_LAN_IP>:8012/?ws=wss://<MAC_LAN_IP>:8012
```

## 3. 명령을 보내지 않는 연결 시험

로봇 상태와 관계없이 Quest 연결만 먼저 검사할 수 있습니다. 이 명령은 로봇 bridge에 연결하지 않고 모터 명령도 보내지 않습니다.

```bash
.venv/bin/python scripts/quest_check.py --lan-ip 10.175.216.169
```

출력된 `OPEN ON QUEST` URL을 Quest 브라우저에서 열고 인증서 경고를 승인한 다음 `Enter VR`을 선택합니다. 오른쪽 컨트롤러를 움직였을 때 `PASS Quest right-controller tracking received`가 나와야 합니다.

Mac에 Atom01용 CANable이 `/dev/cu.usbmodem...`으로 연결되면 다음 feedback-only 검사로 오른팔 CAN ID 19–23을 확인할 수 있습니다. 이 검사는 enable, zero, disable, 위치 명령을 보내지 않습니다.

```bash
.venv/bin/roboparty-can-probe \
  --port /dev/cu.usbmodem2070388B31361 \
  --motor-ids 0x01,0x02
```

Damiao의 `0x11`, `0x12`가 feedback/master ID라면 위 명령의 command/ESC ID는 각각 `0x01`, `0x02`입니다. 정상 응답은 `command=0x01 feedback=0x11`, `command=0x02 feedback=0x12`로 표시됩니다.

`WAIT CANable opened, but no motor feedback arrived`이면 USB 인식은 성공했지만 모터 전원, CAN-H/CAN-L, 1 Mbps 설정 또는 종단저항 경로가 아직 준비되지 않은 상태입니다. feedback가 확인되기 전에는 모터 enable이나 teleoperation을 실행하지 마세요.

### 두 모터 LeRobot 방식 캘리브레이션

먼저 파일을 변경하지 않는 검사만 실행합니다.

```bash
.venv/bin/roboparty-calibrate-two-motors \
  --port /dev/cu.usbmodem2070388B31361 \
  --motor-ids 0x01,0x02 \
  --joint-names motor_0,motor_1
```

실제 캘리브레이션은 LeRobot의 follower-arm 절차처럼 진행됩니다. 명령이 두 모터의 torque를 disable한 뒤 (1) 두 관절을 가동범위 중간/home 자세에 놓고 ENTER, (2) 두 관절을 각각 전체 가동범위로 움직인 뒤 ENTER를 받습니다. 소프트웨어 homing offset, raw/logical 최소·최대값 및 LeRobot `range_m100_100` 정규화 정보가 `.local/calibration/right_arm_two_motor.json`에 저장됩니다.

```bash
.venv/bin/roboparty-calibrate-two-motors \
  --port /dev/cu.usbmodem2070388B31361 \
  --motor-ids 0x01,0x02 \
  --joint-names motor_0,motor_1 \
  --lerobot
```

이 절차는 motor hardware zero/flash를 변경하지 않으며 완료 후에도 torque-disabled 상태를 유지합니다. 기존 zero-offset 전용 파일이 필요한 경우에만 `--capture --zero-pose-confirmed`를 사용할 수 있습니다.

저장 후 Quest와 모터를 동시에 read-only로 확인합니다.

```bash
.venv/bin/roboparty-quest-two-motor-check \
  --lan-ip 10.175.216.169
```

이 단계에서도 모터 enable/zero/위치 명령은 전송하지 않습니다.

하드웨어를 연결하기 전에는 다음 명령이 `WAIT robot hardware state not ready`와 함께 `PRECHECK PASSED`로 끝나는 것이 정상입니다.

```bash
export ROBOPARTY_BRIDGE_TOKEN="$(cat .local/secrets/bridge_token)"
.venv/bin/python scripts/preflight.py \
  --server 100.96.41.100 \
  --lan-ip 10.175.216.169
```

카메라와 로봇을 연결한 다음에는 하드웨어 상태와 두 카메라까지 필수로 검사합니다.

```bash
.venv/bin/python scripts/preflight.py \
  --server 100.96.41.100 \
  --lan-ip 10.175.216.169 \
  --camera 0 --camera 1 \
  --require-hardware
```

카메라 번호는 Mac의 실제 장치에 맞게 바꿉니다. 그리퍼 open/closed 값은 **radian**이며 실제 하드웨어 값으로 반드시 교체합니다.

```bash
roboparty-teleoperate \
  --robot.type=roboparty_right_arm \
  --robot.id=right_arm \
  --robot.bridge_host=100.96.41.100 \
  --robot.gripper_open_rad=0.0 \
  --robot.gripper_closed_rad=1.0 \
  --robot.command_enabled=false \
  --robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}' \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --fps=30 \
  --display_data=true
```

상태/카메라/Quest 추적이 모두 정상일 때만 `--robot.command_enabled=true`로 변경합니다. 처음에는 로봇을 작업대에서 띄우거나 비상정지 가능한 상태로 낮은 속도에서 시험하세요.

## 4. LeRobot 데이터 기록

### 현재 2모터 Quest 동작 시험

```bash
roboparty-quest-two-motor-teleop --lan-ip <MAC_LAN_IP>
```

오른쪽 그립을 누르는 순간 현재 모터/컨트롤러 위치를 기준점으로 잡습니다. Quest 원시 WebXR 좌표는
handoff 규약에 따라 RUB(`x=오른쪽, y=위, z=뒤`)에서 로봇 FLU(`x=앞, y=왼쪽, z=위`)로 변환됩니다.
그 후 robot `y` 이동이 joint1, robot `z` 이동이 joint2를 상대 제어합니다. 그립을 놓거나 추적이
끊기면 두 모터를 disable합니다. 축과
방향은 `--motor0-axis`, `--motor1-axis`, `--motor0-sign`, `--motor1-sign`으로 바꿀 수 있습니다.
명령 위치는 캘리브레이션 범위뿐 아니라 handoff 현재 실물용 3축 URDF의 joint1 ±3.1067 rad, joint2 ±1.7453 rad
범위로도 제한되며 두 범위의 교집합만 사용합니다.

### 2모터 + AmazingHand full-grasp 통합

AmazingHand USB 직렬 어댑터를 연결한 뒤 장치 이름을 확인합니다. 포트는 자동 선택하지 않습니다.

```bash
ls /dev/ttyACM* /dev/ttyUSB*
uv sync --extra hardware
```

현재 2축 팔을 Quest controller angle-following으로 사용할 때는 `kinematics` extra도 설치하고
`--robot.arm_control_mode=ik`를 지정합니다. 이 모드는 `handoff_orientation.tar.gz` 안에서
현재 실물용으로 표시된 `orientation/urdf/robot_arm_temp.urdf`를 읽고 joint1/joint2만 활성화합니다.
J4/J5 쪽 링크가 빠진 이 3축 모델에서 joint3만 `orientation/config/arm_temp.json`의 홈값
`-0.042588 rad`에 고정합니다. end effector는 손목에 붙은 `hand_mount`입니다. 기본
`direct` 모드는 기존 축별 검증용으로 유지되지만 `roboparty-record`의 현재 2축 구성은 IK 모드를
필수로 검사합니다.

임시 2축 IK는 임의의 6D pose를 풀 수 없으므로 **controller 상대 회전 → `hand_mount` 상대 회전**을
우선하고 controller `xyz` 이동은 팔 목표에서 제외합니다. URDF의 J1=-Z, J2=-X 회전축으로
controller 상대 회전을 직접 투영하므로 controller를 약 45° 숙이면 J2도 약 45° 목표를 갖습니다. 매 frame의 안전 제한은 이전
명령 target이 아니라 최신 measured J1/J2에서 계산하므로 별도의 target backlog가 누적되지 않습니다.
5축이 완성되면 같은 LeRobot processor 경계 안에서 full position+orientation IK로 교체합니다.

```bash
uv sync --extra hardware --extra kinematics --extra viz
```

`viz`는 별도 패널을 추가하는 것이 아니라 LeRobot이 기본 지원하는 Rerun backend를 설치합니다.

`--all-extras`는 배포 장비에서 사용하지 않습니다. Linux x86_64에서는 LeRobot 자체의 기본
`torch`/`torchvision` 의존성이 CUDA 12.8 패키지를 설치하므로, RTX 학습/추론 서버에서는 큰
`nvidia-*` 다운로드가 정상입니다. `test`와 `kinematics` extra는 해당 기능을 개발하거나 검사할 때만
추가합니다.

통합 Robot은 팔 두 축을 `[-100, 100]`, 손 전체 grasp를 `[0, 100]`으로 노출합니다. 손 내부에서는
SCS0009 서보 ID 1–8을 모두 사용하며, 짝수 ID의 방향 반전은 AmazingHandControl과 동일하게 적용됩니다.

```bash
roboparty-teleoperate \
  --robot.type=roboparty_two_motor_amazing_hand \
  --robot.id=two_motor_amazing_hand \
  --robot.hand_port=/dev/ttyUSB1 \
  --robot.can_port=/dev/ttyACM0 \
  --robot.can_interface=slcan \
  --robot.two_motor_calibration_path=config/right_arm_two_motor.json \
  --robot.arm_control_mode=ik \
  --robot.cameras='{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: /dev/video1, width: 640, height: 480, fps: 30}}' \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --fps=15 \
  --display_data=true
```

- 시작 및 episode reset 직후: 유효한 Quest tracking이 들어오면 J1·J2가 현재 자세에서 자동 engage합니다.
- 오른쪽 grip/squeeze: 팔 활성 상태와 무관합니다. 잡거나 놓아도 torque와 IK 상대 기준점을 재설정하지 않습니다.
- 오른쪽 trigger: 손을 연속 제어합니다. `0%=open`, `100%=full grasp`입니다.
- Quest 추적이 끊기면 손은 마지막 grasp를 유지합니다.
- B/A: 팔 disarm/rearm이며 손 grasp에는 영향을 주지 않습니다. B 뒤에는 A를 눌러 새 현재 자세에서 다시 engage합니다.
- Quest tracking이 끊기면 팔 torque를 해제하며, tracking 복귀 시 현재 자세에서 상대 IK 기준을 다시 잡습니다.
- controller 회전은 시작 자세에 대한 상대 `hand_mount` 회전으로 J1/J2에 반영됩니다. controller 위치 이동만으로는 임시 2축 팔을 움직이지 않습니다.
- 종료 시 팔과 손 torque를 모두 해제합니다.

> `roboparty-quest-two-motor-teleop` 단독 진단 CLI는 기존 hold-to-run grip clutch를 유지합니다.
> 위 A/B 동작은 `roboparty-teleoperate`와 `roboparty-record`의 2모터 + AmazingHand 통합 processor에 적용됩니다.

### Mac 키보드 + Rerun으로 수동 episode 기록

LeLab이나 별도 control panel은 사용하지 않습니다. 실시간 Quest/IK/Robot/dataset loop는 Jetson에서
실행하고, Mac은 SSH 터미널의 LeRobot 키 입력과 Rerun viewer만 담당합니다.

Mac의 첫 번째 터미널에서 Rerun viewer를 loopback에만 엽니다.

```bash
uv tool install 'rerun-sdk==0.33.1'
rerun --bind 127.0.0.1 --port 9876
```

Mac의 두 번째 터미널에서 Rerun 포트를 Jetson으로 reverse-forward하며 SSH에 접속합니다. 현재 Jetson
LAN 주소가 바뀌었으면 `10.175.216.203`만 실제 주소로 교체합니다.

```bash
ssh -t -o ExitOnForwardFailure=yes \
  -R 9876:127.0.0.1:9876 \
  dong@10.175.216.203
```

그 SSH 터미널 안에서 Hub 업로드 없이 수동 episode를 기록합니다. LeRobot의 숫자 parser와 기존
record loop가 `inf`를 그대로 처리하므로 별도 timing 구현 없이 키를 누를 때까지 각 phase가 계속됩니다.
`0`은 기존 LeRobot 의미대로 해당 phase를 생략합니다.

```bash
cd /home/dong/echo_v4
source .venv/bin/activate

roboparty-record \
  --robot.type=roboparty_two_motor_amazing_hand \
  --robot.id=two_motor_amazing_hand \
  --robot.hand_port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C63050237-if00 \
  --robot.can_port=/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_2070388B3136-if00 \
  --robot.can_interface=slcan \
  --robot.two_motor_calibration_path=config/right_arm_two_motor.json \
  --robot.arm_control_mode=ik \
  --robot.cameras='{front: {type: opencv, index_or_path: /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: /dev/v4l/by-path/platform-3610000.usb-usb-0:2:1.3-video-index0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --dataset.repo_id=local/roboparty-two-motor-amazing-hand \
  --dataset.num_episodes=2 \
  --dataset.single_task='Grasp the object with the right hand' \
  --dataset.episode_time_s=inf \
  --dataset.reset_time_s=inf \
  --dataset.fps=15 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --display_mode=rerun \
  --display_ip=127.0.0.1 \
  --display_port=9876
```

키 입력은 **Rerun 창이 아니라 Mac의 SSH 터미널에 focus를 둔 상태**에서 합니다.

| 키 | LeRobot 기록 동작 |
|---|---|
| `→` | 현재 recording을 끝내고 reset으로 이동, reset 중에는 다음 episode 시작 |
| `←` | 현재 episode buffer를 폐기하고 다시 기록 |
| `ESC` | 기록 세션을 종료하고 저장된 dataset을 finalize |

SSH 환경에서 방향키 escape sequence가 가로채지는 경우에는 LeRobot에 이미 포함된 동등 키
`n`(다음), `r`(재기록), `q`(종료)를 사용합니다. Rerun은 카메라·J1/J2 state·최종 action을 보여줄
뿐이고 기록 단계 전환이나 로봇 명령을 직접 처리하지 않습니다.

이 구성의 LeRobot action/state 순서는 다음 3축으로 고정됩니다.

```text
right_arm_joint_1.pos right_arm_joint_2.pos right_hand_grasp.pos
```

실제 dataset을 Hub에 올릴 때는 `--dataset.push_to_hub=true --dataset.private=true`와 본인의
`<HF_USER>/<DATASET_NAME>` repo ID를 사용합니다.

현재 수집 계약은 J1·J2와 단일 full-grasp 세 축으로 고정합니다. J3–J5 또는 개별 손가락 축을 추가할
경우 기존 dataset과 섞지 않고 별도의 action schema와 dataset 버전을 만들어야 합니다.

저장되는 action은 Quest pose가 아니라 위의 J1·J2·grasp 관절 action이며, observation에는 같은 관절 상태와
설정한 RGB 카메라 영상이 들어갑니다. 따라서 이 3축 schema를 유지한 채 LeRobot의 표준 ACT 학습 및
정책 rollout 흐름을 사용할 수 있습니다.

## 5. ACT 학습 예시

LeRobot 버전에 맞는 `lerobot-train`을 사용합니다.

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/roboparty-j1-j2-amazing-hand \
  --policy.type=act \
  --output_dir=outputs/train/roboparty_j1_j2_hand_act \
  --job_name=roboparty_j1_j2_hand_act \
  --policy.device=mps
```

Mac GPU/LeRobot 버전에서 MPS 연산이 지원되지 않으면 `--policy.device=cpu`를 사용하거나 학습만 Ubuntu GPU 서버에서 수행하고, 데이터/체크포인트를 Hugging Face Hub 또는 Tailscale로 전달합니다.

## 네트워크 점검

```bash
# Mac
nc -vz 100.96.41.100 8765

# Ubuntu: bridge가 Tailscale 주소에만 listen하는지 확인
ss -ltnp | grep 8765
```

브리지는 TLS 대신 Tailscale 암호화 + 애플리케이션 토큰 인증을 사용합니다. 토큰을 Git에 커밋하지 마세요.
