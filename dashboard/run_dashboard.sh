#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/safe_landing_dashboard_ros_logs}"
mkdir -p "$ROS_LOG_DIR"
exec python3 "$ROOT/dashboard/dashboard_server.py" "$@"
