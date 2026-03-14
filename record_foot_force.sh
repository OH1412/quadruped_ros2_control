#!/bin/bash

# Record `/guide/foot_force` ROS2 topic to a timestamped file under ./logs
# Usage: ./record_foot_force.sh [duration_seconds]

LOG_DIR="./logs"
mkdir -p "$LOG_DIR"

NEW_LOG_DIR="${LOG_DIR}/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$NEW_LOG_DIR"

FOOT_FORCE_LOG="${NEW_LOG_DIR}/guide_foot_force.log"
EST_FOOT_FORCE_LOG="${NEW_LOG_DIR}/guide_estimated_foot_force.log"

DURATION=10
if [ "$1" != "" ]; then
    DURATION=$1
fi

if ! command -v ros2 &> /dev/null; then
    echo "Error: ros2 not found. Source ROS2 environment first."
    exit 1
fi

echo "Recording /guide/foot_force and /guide/estimated_foot_force for ${DURATION}s -> ${NEW_LOG_DIR}"

if ! ros2 topic info /guide/foot_force &> /dev/null; then
    echo "Warning: topic /guide/foot_force not currently available, will attempt to listen anyway."
fi
if ! ros2 topic info /guide/estimated_foot_force &> /dev/null; then
    echo "Warning: topic /guide/estimated_foot_force not currently available, will attempt to listen anyway."
fi

# start both recorders in background and wait for them
timeout ${DURATION} ros2 topic echo /guide/foot_force > "$FOOT_FORCE_LOG" 2>&1 &
PID1=$!
timeout ${DURATION} ros2 topic echo /guide/estimated_foot_force > "$EST_FOOT_FORCE_LOG" 2>&1 &
PID2=$!

wait $PID1
wait $PID2

echo "Recording finished. Files:"
ls -lh "$FOOT_FORCE_LOG" "$EST_FOOT_FORCE_LOG"
