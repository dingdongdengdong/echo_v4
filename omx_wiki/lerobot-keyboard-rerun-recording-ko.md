---
title: "LeRobot keyboard and Rerun recording KO"
tags: ["lerobot", "dataset", "keyboard", "rerun", "mac", "jetson", "quest2", "korean"]
created: 2026-08-08T18:25:51+09:00
updated: 2026-08-08T18:25:51+09:00
sources:
  - "https://github.com/huggingface/lerobot/blob/v0.6.1/docs/source/il_robots.mdx"
  - "https://github.com/huggingface/lerobot/blob/v0.6.1/src/lerobot/utils/keyboard_input.py"
  - "https://github.com/huggingface/lerobot/blob/v0.6.1/src/lerobot/scripts/lerobot_record.py"
  - "https://rerun.io/docs/reference/sdk-operating-modes"
links: ["roboparty-xr-ik-custom-urdf-quest2-lerobot-dataflow-ko.md", "roboparty-xr-ik-dataset-debugging-ko.md", "jetson-safe-bring-up-runbook.md"]
category: convention
confidence: high
schemaVersion: 1
---

# LeRobot 키보드 + Rerun으로 RoboParty dataset 기록

> 검증 기준: 2026-08-08, `/home/dong/echo_v4`. LeLab이나 별도 control panel을 만들지 않고 vendored LeRobot의 기존 기능만 사용한다.

## 1. 역할 분리

```text
Quest 2 --WebXR/Vuer--> Jetson Quest teleoperator
                            |
                            v
                    J1/J2 IK + grasp processor
                            |
                            v
                    Robot + LeRobot record loop
                            |
             +--------------+---------------+
             |                              |
             v                              v
      LeRobotDataset                 Rerun SDK stream
                                            |
                                     reverse SSH :9876
                                            |
                                            v
Mac SSH terminal ------------------> Mac Rerun viewer
  Right/Left/ESC                      camera/state/action 표시만 담당
```

- Jetson: Quest 수신, IK, 실제 robot loop, 카메라, dataset 저장을 모두 실행한다.
- Mac SSH 터미널: LeRobot episode lifecycle 키만 전달한다.
- Mac Rerun: processed J1/J2/grasp action, observation state, 카메라를 표시한다.
- Rerun은 기록 버튼이나 robot command를 제공하는 control panel이 아니다.

## 2. 시간제한 없는 LeRobot phase

LeRobot의 숫자 parser는 `inf`를 Python `float('inf')`로 읽는다. 현재 record loop는 경과시간이 이 값보다
작은 동안 계속되고, 기존 키 event가 loop를 끝낸다. 따라서 새 timer 구현이나 LeRobot core patch가 필요 없다.

```bash
--dataset.episode_time_s=inf
--dataset.reset_time_s=inf
```

`0`은 기존 LeRobot 의미인 phase 생략이다.

## 3. 키 의미

키 입력은 **Rerun 창이 아니라 `ssh -t`로 접속한 Mac 터미널에 focus**를 둔 상태에서 한다. headless
Jetson에서는 LeRobot terminal listener가 CSI/SS3 방향키와 bare ESC를 해석한다.

| 키 | 기존 LeRobot event | 결과 |
|---|---|---|
| `→` 또는 `n` | `exit_early` | recording 종료 후 reset, reset 중에는 다음 episode 시작 |
| `←` 또는 `r` | `rerecord_episode` | 현재 episode buffer 폐기 후 다시 기록 |
| `ESC` 또는 `q` | `stop_recording` | 세션 종료, 영상 인코딩 및 dataset finalize |

방향키 escape sequence가 SSH/tmux에서 가로채지면 동등 키 `n/r/q`를 사용한다.

### partial episode 주의

recording 도중 `ESC/q`를 누르면 현재 partial episode도 저장된다. partial을 남기지 않는 권장 종료 순서는
다음과 같다.

1. 정상 동작을 마친다.
2. `→/n`으로 recording을 끝내고 reset 단계에 들어간다.
3. reset 단계에서 `ESC/q`로 세션을 종료한다.

잘못 기록한 episode는 `←/r`로 폐기하고 다시 기록한다.

## 4. Jetson 환경

LeRobot이 지원하는 Rerun dependency를 그대로 설치한다.

```bash
cd /home/dong/echo_v4
uv sync --extra hardware --extra kinematics --extra viz
```

현재 lockfile의 Rerun 버전은 `0.33.1`이다.

## 5. Mac Rerun + reverse SSH

Mac 첫 번째 터미널:

```bash
uv tool install 'rerun-sdk==0.33.1'
rerun --bind 127.0.0.1 --port 9876
```

Mac 두 번째 터미널:

```bash
ssh -t -o ExitOnForwardFailure=yes \
  -R 9876:127.0.0.1:9876 \
  dong@10.175.216.203
```

reverse tunnel 때문에 Jetson의 `127.0.0.1:9876`이 Mac viewer의 loopback port로 전달된다. Rerun port를
LAN 전체에 직접 노출할 필요가 없다.

## 6. Jetson record 명령

다음 명령은 위 SSH 터미널 안에서 실행한다.

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
  --dataset.repo_id=local/roboparty-j1-j2-amazing-hand \
  --dataset.num_episodes=50 \
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

Rerun에 IP와 port를 모두 주면 LeRobot은 네트워크 전송용으로 image compression을 활성화한다. 실제 motion
전에는 [[jetson-safe-bring-up-runbook]]의 gate를 통과하고, dataset 내용은
[[roboparty-xr-ik-dataset-debugging-ko]] 순서로 검사한다.

> D435i의 `/dev/v4l/by-id/...-video-index0`는 이 Jetson에서 depth 노드(`/dev/video2`)와 RGB
> 노드(`/dev/video6`)가 같은 이름으로 충돌한다. OpenCV RGB 기록은 실제로 640x480@30fps를
> 열어 검증한 `...1.3-video-index0` by-path를 사용한다.
