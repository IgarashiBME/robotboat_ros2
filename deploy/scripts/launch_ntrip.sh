#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$SCRIPT_DIR/env.sh"

exec ros2 launch ntrip_client ntrip_client.launch.py
