#!/usr/bin/env bash
# ==============================================================================
# Autonomous Delivery Robot - Colcon Build Script
# Optimized for Raspberry Pi 4B CPU & Memory Constraints
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/../ros2_ws"

echo "Navigating to ROS 2 Workspace: ${WORKSPACE_DIR}"
cd "${WORKSPACE_DIR}"

# Source system ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: /opt/ros/humble/setup.bash not found. Ensure ROS 2 Humble is installed."
fi

# Determine available RAM to prevent Raspberry Pi 4B freezing
TOTAL_MEM_KB=$(grep MemTotal /proc/meminfo | awk '{print $2}' || echo "8000000")
if [ "${TOTAL_MEM_KB}" -lt 4500000 ]; then
    echo "Notice: Detected <= 4GB RAM. Restricting colcon parallel workers to 2 to prevent OOM."
    PARALLEL_FLAGS="--parallel-workers 2"
else
    PARALLEL_FLAGS=""
fi

echo "Building ROS 2 packages..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    ${PARALLEL_FLAGS}

echo "=============================================================================="
echo "Build complete! To use the workspace, run:"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo "=============================================================================="
