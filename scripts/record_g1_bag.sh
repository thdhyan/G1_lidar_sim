#!/usr/bin/env bash
# Record a full-sensor bag from the real G1.
# Usage:
#   ./record_g1_bag.sh                  # auto-named bag in ./bags/
#   ./record_g1_bag.sh my_bag_name      # custom name
#   ./record_g1_bag.sh -d /path/to/dir  # custom output directory

set -euo pipefail

# --- parse args ---
OUTPUT_DIR="$(dirname "$0")/../bags"
BAG_NAME=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--dir) OUTPUT_DIR="$2"; shift 2 ;;
        *)        BAG_NAME="$1"; shift ;;
    esac
done

if [[ -z "$BAG_NAME" ]]; then
    BAG_NAME="g1_$(date +%Y%m%d_%H%M%S)"
fi

BAG_PATH="${OUTPUT_DIR}/${BAG_NAME}"
mkdir -p "$OUTPUT_DIR"

# --- topics ---
# Real-robot LiDAR (Unitree driver publishes here)
LIDAR_RAW="/utlidar/cloud"
# Re-framed canonical topic (lidar_bridge, if running)
LIDAR_CANON="/livox/mid360/points"

TOPICS=(
    "$LIDAR_RAW"
    "$LIDAR_CANON"
    "/g1/camera/rgb"
    "/g1/camera/depth"
    "/g1/camera/camera_info"
    "/g1/joint_states"
    "/tf"
    "/tf_static"
    "/clock"
    "/g1/odom"
    "/g1/cmd_vel"
)

# Build --topics arg list (ros2 bag record takes space-separated topics after -e or explicit list)
TOPIC_ARGS=()
for t in "${TOPICS[@]}"; do
    TOPIC_ARGS+=("$t")
done

echo "Bag  : ${BAG_PATH}"
echo "Topics:"
for t in "${TOPICS[@]}"; do
    echo "  $t"
done
echo ""
echo "Press Ctrl-C to stop recording."
echo ""

# Source ROS2 if not already sourced
if [[ -z "${ROS_DISTRO:-}" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

ros2 bag record \
    --output "$BAG_PATH" \
    --storage mcap \
    --compression-mode file \
    --compression-format zstd \
    "${TOPIC_ARGS[@]}"
