#!/usr/bin/env bash
set -euo pipefail

SERVER="${1:-100.96.41.100}"
TOKEN_FILE="${2:-.local/secrets/bridge_token}"
CONTAINER_NAME="roboparty-ros-bridge"

if [[ ! -s "$TOKEN_FILE" ]]; then
  echo "Missing bridge token: $TOKEN_FILE" >&2
  exit 1
fi

mkdir -p .local/secrets
TOKEN="$(tr -d '\n' < "$TOKEN_FILE")"
if (( ${#TOKEN} < 24 )); then
  echo "Bridge token must contain at least 24 characters" >&2
  exit 1
fi
printf 'ROBOPARTY_BRIDGE_TOKEN=%s\n' "$TOKEN" > .local/secrets/bridge.env
chmod 600 .local/secrets/bridge.env

ssh "$SERVER" 'mkdir -p ~/.local/share/roboparty_xr_teleop ~/.config/roboparty && chmod 700 ~/.config/roboparty'
scp -q teleop/ros_bridge.py "$SERVER":~/.local/share/roboparty_xr_teleop/ros_bridge.py
scp -q .local/secrets/bridge.env "$SERVER":~/.config/roboparty/bridge.env

ssh "$SERVER" bash -s -- "$SERVER" "$CONTAINER_NAME" <<'REMOTE'
set -euo pipefail
bind_host="$1"
container_name="$2"
chmod 600 "$HOME/.config/roboparty/bridge.env"
chmod 644 "$HOME/.local/share/roboparty_xr_teleop/ros_bridge.py"
docker pull ros:humble-ros-base
if docker container inspect "$container_name" >/dev/null 2>&1; then
  docker rm -f "$container_name" >/dev/null
fi
docker run -d \
  --name "$container_name" \
  --restart unless-stopped \
  --network host \
  --env-file "$HOME/.config/roboparty/bridge.env" \
  -v "$HOME/.local/share/roboparty_xr_teleop/ros_bridge.py:/opt/roboparty/ros_bridge.py:ro" \
  ros:humble-ros-base \
  python3 /opt/roboparty/ros_bridge.py --bind-host "$bind_host" --port 8765
sleep 2
docker ps --filter "name=$container_name" --format '{{.Names}} {{.Status}}'
docker logs --tail 20 "$container_name"
REMOTE
