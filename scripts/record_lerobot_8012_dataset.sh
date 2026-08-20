#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

DATASET_REPO_ID="${DATASET_REPO_ID:-local/roboparty-8012-j1-j2-j3}"
TASK="${TASK:-Pick up the cube}"
NUM_EPISODES="${NUM_EPISODES:-2}"
FPS="${FPS:-15}"
EPISODE_TIME_S="${EPISODE_TIME_S:-inf}"
RESET_TIME_S="${RESET_TIME_S:-inf}"
MIN_MOTION_RAD="${MIN_MOTION_RAD:-0.02}"
STATE_URL="${STATE_URL:-https://127.0.0.1:8012/state}"
FRONT_CAMERA="${FRONT_CAMERA:-/dev/v4l/by-id/usb-Generic_USB2.0_PC_CAMERA-video-index0}"
WRIST_CAMERA="${WRIST_CAMERA:-/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4:1.3-video-index0}"
DATASET_ROOT="${DATASET_ROOT:-}"

if [[ "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Record the verified robot_arm_vr 8012 J1/J2/J3 + AmazingHand runtime.

This is a passive LeRobot sidecar: it reads the existing 8012 /state endpoint
and two cameras. It does not start another Vuer site and never owns CAN or hand
serial hardware. The correct 8012 bridge/WebXR runtime must already be running.

Optional environment variables:
  DATASET_REPO_ID, TASK, NUM_EPISODES, FPS
  EPISODE_TIME_S, RESET_TIME_S, MIN_MOTION_RAD, STATE_URL, DATASET_ROOT
  FRONT_CAMERA, WRIST_CAMERA, PUSH_TO_HUB, PRIVATE, DISPLAY_DATA

MIN_MOTION_RAD defaults to 0.02 rad. An episode is discarded unless the arm
was engaged and at least one measured joint moved by that amount. Set it to 0
only when stationary episodes are intentional.

Examples:
  NUM_EPISODES=20 TASK='Pick up the cube' \
    scripts/record_lerobot_8012_dataset.sh
  DATASET_ROOT=outputs/lerobot_datasets/motion-test \
    NUM_EPISODES=1 TASK='Small safe J1 J2 J3 movement test' \
    scripts/record_lerobot_8012_dataset.sh
  scripts/record_lerobot_8012_dataset.sh --validate-only
EOF
  exit 0
fi

for path in "$FRONT_CAMERA" "$WRIST_CAMERA"; do
  if [[ ! -e "$path" ]]; then
    printf 'Missing required camera: %s\n' "$path" >&2
    exit 1
  fi
done

cmd=(
  .venv/bin/python -m lerobot_robot_roboparty.robot_arm_vr_recorder
  "--state-url=$STATE_URL"
  "--front-camera=$FRONT_CAMERA"
  "--wrist-camera=$WRIST_CAMERA"
  "--repo-id=$DATASET_REPO_ID"
  "--task=$TASK"
  "--num-episodes=$NUM_EPISODES"
  "--fps=$FPS"
  "--episode-time-s=$EPISODE_TIME_S"
  "--reset-time-s=$RESET_TIME_S"
  "--min-motion-rad=$MIN_MOTION_RAD"
)

if [[ -n "$DATASET_ROOT" ]]; then
  cmd+=("--root=$DATASET_ROOT")
fi

if [[ "${PUSH_TO_HUB:-false}" == "true" ]]; then
  cmd+=(--push-to-hub)
fi
if [[ "${PRIVATE:-true}" == "true" ]]; then
  cmd+=(--private)
fi
if [[ "${DISPLAY_DATA:-true}" != "true" ]]; then
  cmd+=(--no-display)
fi
if [[ "${VIDEO:-true}" != "true" ]]; then
  cmd+=(--no-video)
fi

case "${1:-}" in
  --validate-only) cmd+=(--validate-only) ;;
  "") ;;
  *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
esac

printf 'Running:'
printf ' %q' "${cmd[@]}"
printf '\n'
exec "${cmd[@]}"
