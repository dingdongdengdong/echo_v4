# Roboparty Quest 2 → LeRobot 오른팔 텔레오퍼레이션

이 저장소는 Roboparty의 오른팔(5축 + 그리퍼)을 Meta Quest 2로 조작하고, 그 결과를 Hugging Face LeRobot 데이터셋/ACT 학습 형식으로 기록하기 위한 통합을 포함합니다.

## 실행 구조

```text
# 현재 2축 + AmazingHand 구성
Meta Quest 2 ──HTTPS/WebXR──> Jetson (Vuer + LeRobot + 카메라)
                                  ├── USB/CAN ──> DM4340 2축
                                  └── USB serial ──> AmazingHand 8서보

# 향후 5축 구성
Meta Quest 2 ──HTTPS/WebXR──> Jetson/Mac ──Tailscale──> Ubuntu 22.04 ROS 2 bridge
```

- **Jetson**: 현재 2축 팔, AmazingHand, 두 RGB 카메라, Quest WebXR와 LeRobot 기록을 모두 실행합니다.
- **Mac**: 더 이상 현재 2축 구성의 필수 장치가 아니며 개발/점검용으로만 사용할 수 있습니다.
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

- Quest 오른쪽 **grip/squeeze**를 누르는 동안만 움직입니다.
- **B**: 즉시 pause, **A**: 다시 arm. A를 눌러도 grip을 다시 잡기 전에는 움직이지 않습니다.
- 컨트롤러 추적이 250 ms 이상 끊기면 현재 관절 위치를 유지합니다.
- engage 순간의 상대 좌표를 사용하므로 Quest 좌표 원점으로 팔이 점프하지 않습니다.
- 프레임당 EE 이동 3 cm, workspace ±20 cm, 명령당 관절 변화 기본 0.05 rad로 제한합니다.
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

오른쪽 그립을 누르는 순간 현재 모터/컨트롤러 위치를 기준점으로 잡습니다. `urdf_temp4`의 현재 2축
구성에서는 motor 0이 joint1(베이스 yaw), motor 1이 joint2(숄더 pitch)입니다. 그립을 누른 동안
컨트롤러 `y` 이동이 joint1/motor 0, 사용자가 느끼는 상하 방향인 컨트롤러 `z` 이동이
joint2/motor 1을 상대 제어합니다. 그립을 놓거나 추적이 끊기면 두 모터를 disable합니다. 축과
방향은 `--motor0-axis`, `--motor1-axis`, `--motor0-sign`, `--motor1-sign`으로 바꿀 수 있습니다.
명령 위치는 캘리브레이션 범위뿐 아니라 `urdf_temp4`의 joint1 ±3.1067 rad, joint2 ±1.7453 rad
범위로도 제한되며 두 범위의 교집합만 사용합니다.

### 2모터 + AmazingHand full-grasp 통합

AmazingHand USB 직렬 어댑터를 연결한 뒤 장치 이름을 확인합니다. 포트는 자동 선택하지 않습니다.

```bash
ls /dev/ttyACM* /dev/ttyUSB*
uv sync --extra hardware
```

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
  --robot.cameras='{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: /dev/video1, width: 640, height: 480, fps: 30}}' \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --fps=20 \
  --display_data=true
```

- 오른쪽 grip: 팔 두 축 clutch. 놓거나 추적이 끊기면 팔 torque를 해제합니다.
- 오른쪽 trigger: grip과 독립적으로 손을 연속 제어합니다. `0%=open`, `100%=full grasp`입니다.
- Quest 추적이 끊기면 손은 마지막 grasp를 유지합니다.
- B/A: 팔 disarm/rearm이며 손 grasp에는 영향을 주지 않습니다.
- 종료 시 팔과 손 torque를 모두 해제합니다.

먼저 Hub 업로드 없이 짧은 episode를 기록합니다.

```bash
roboparty-record \
  --robot.type=roboparty_two_motor_amazing_hand \
  --robot.id=two_motor_amazing_hand \
  --robot.hand_port=/dev/ttyUSB1 \
  --robot.can_port=/dev/ttyACM0 \
  --robot.can_interface=slcan \
  --robot.two_motor_calibration_path=config/right_arm_two_motor.json \
  --robot.cameras='{front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: /dev/video1, width: 640, height: 480, fps: 30}}' \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --dataset.repo_id=local/roboparty-two-motor-amazing-hand \
  --dataset.num_episodes=2 \
  --dataset.single_task='Grasp the object with the right hand' \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=10 \
  --dataset.fps=20 \
  --dataset.push_to_hub=false \
  --display_data=true
```

이 구성의 LeRobot action/state 순서는 다음 3축으로 고정됩니다.

```text
motor_0.pos motor_1.pos right_hand_grasp.pos
```

실제 dataset을 Hub에 올릴 때는 `--dataset.push_to_hub=true --dataset.private=true`와 본인의
`<HF_USER>/<DATASET_NAME>` repo ID를 사용합니다.

손 구현은 8서보 통신 계층과 현재의 `FullGraspMapper`가 분리되어 있습니다. 추후 자유 손 추적은 새
8축 mapper를 추가하고, 5축 팔은 기존 `roboparty_right_arm` 계층과 같은 방식으로 결합하여 transport를
다시 구현하지 않고 확장합니다.

```bash
export HF_USER=<huggingface-user>
roboparty-record \
  --robot.type=roboparty_right_arm \
  --robot.id=right_arm \
  --robot.bridge_host=100.96.41.100 \
  --robot.gripper_open_rad=0.0 \
  --robot.gripper_closed_rad=1.0 \
  --robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}' \
  --teleop.type=quest2_vuer \
  --teleop.id=quest2 \
  --teleop.cert_file=cert.pem \
  --teleop.key_file=key.pem \
  --dataset.repo_id=${HF_USER}/roboparty-right-arm-xr \
  --dataset.num_episodes=50 \
  --dataset.single_task='Pick up the object with the right arm' \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.fps=30 \
  --display_data=true
```

저장되는 action은 Quest pose가 아니라 LeRobot 표준 관절 action입니다.

```text
right_motor0.pos ... right_motor4.pos right_gripper.pos
```

observation에는 같은 관절 상태와 `front`, `wrist` RGB 영상이 들어갑니다. 따라서 표준 ACT 학습과 `lerobot-record` 대신 정책 rollout 도구를 그대로 사용할 수 있습니다.

## 5. ACT 학습 예시

LeRobot 버전에 맞는 `lerobot-train`을 사용합니다.

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/roboparty-right-arm-xr \
  --policy.type=act \
  --output_dir=outputs/train/roboparty_right_act \
  --job_name=roboparty_right_act \
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
