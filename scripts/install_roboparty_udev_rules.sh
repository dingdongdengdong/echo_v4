#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_RULE="$REPO_ROOT/scripts/udev/99-roboparty-serial.rules"
TARGET_RULE="/etc/udev/rules.d/99-roboparty-serial.rules"

if [[ ! -f "$SOURCE_RULE" ]]; then
  printf 'Missing udev rule: %s\n' "$SOURCE_RULE" >&2
  exit 1
fi

sudo install -o root -g root -m 0644 "$SOURCE_RULE" "$TARGET_RULE"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
udevadm settle

printf 'Installed %s\n' "$TARGET_RULE"
printf 'Current Roboparty aliases:\n'
ls -l /dev/roboparty-can /dev/roboparty-hand 2>/dev/null || true

if [[ ! -e /dev/roboparty-hand ]]; then
  printf 'WAIT /dev/roboparty-hand: the AmazingHand USB adapter has not enumerated yet.\n'
fi
