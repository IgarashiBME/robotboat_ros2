#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SCRIPT_DIR/env.sh"

exec ros2 launch mavlink_ros2_bridge mavlink_bridge.launch.py
