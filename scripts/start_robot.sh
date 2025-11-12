#!/usr/bin/env bash
# ==============================================================================
# Autonomous Delivery Robot - Launch Runner
# Usage:
#   ./scripts/start_robot.sh [hardware|mock]
# ==============================================================================

MODE="${1:-hardware}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/../ros2_ws"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
else
    echo "Error: Workspace has not been built. Run ./scripts/build.sh first."
    exit 1
fi

if [ "${MODE}" == "mock" ] || [ "${MODE}" == "sim" ]; then
    echo "Starting Autonomous Delivery Robot in MOCK / SIMULATION mode..."
    ros2 launch robot_bringup bringup.launch.py use_mock_hardware:=true use_rviz:=true
else
    echo "Starting Autonomous Delivery Robot with REAL HARDWARE..."
    ros2 launch robot_bringup bringup.launch.py use_mock_hardware:=false use_rviz:=false
fi
