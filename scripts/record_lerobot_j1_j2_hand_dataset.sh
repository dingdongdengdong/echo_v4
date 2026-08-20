#!/usr/bin/env bash
set -euo pipefail

# Compatibility name retained for the command already shared with operators.
# The implementation now records the verified 8012 J1/J2/J3 runtime.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/record_lerobot_8012_dataset.sh" "$@"
