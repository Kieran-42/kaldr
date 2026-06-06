#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslogs}"
CAMERA_MODEL="${CAMERA_MODEL:-zedm}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
LOG_LEVEL="${LOG_LEVEL:-info}"
USE_RVIZ="${USE_RVIZ:-true}"
RTABMAP_DATABASE_PATH="${RTABMAP_DATABASE_PATH:-~/.ros/rtabmap.db}"
RTABMAP_ARGS="${RTABMAP_ARGS:-}"
NEW_MAP_ON_START="${NEW_MAP_ON_START:-true}"
TMUX_SESSION="${TMUX_SESSION:-nav_stack}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is required but was not found." >&2
    exit 1
fi

if [[ "$NEW_MAP_ON_START" == "true" ]]; then
    if [[ -n "$RTABMAP_ARGS" ]]; then
        RTABMAP_ARGS="$RTABMAP_ARGS --delete_db_on_start"
    else
        RTABMAP_ARGS="--delete_db_on_start"
    fi
fi

RTABMAP_ARGS_LAUNCH=""
if [[ -n "$RTABMAP_ARGS" ]]; then
    RTABMAP_ARGS_LAUNCH="rtabmap_args:=$RTABMAP_ARGS"
fi

STACK_CMD="export ROS_LOG_DIR=\"$ROS_LOG_DIR\"; \
source /opt/ros/humble/setup.bash; \
source \"$ROOT_DIR/install/setup.bash\"; \
ros2 launch manual_control robot_nav_stack.launch.py \
launch_bridge:=true \
use_rviz:=$USE_RVIZ \
camera_model:=$CAMERA_MODEL \
serial_port:=$SERIAL_PORT \
rtabmap_database_path:=$RTABMAP_DATABASE_PATH \
$RTABMAP_ARGS_LAUNCH \
log_level:=$LOG_LEVEL"

TELEOP_CMD="export ROS_LOG_DIR=\"$ROS_LOG_DIR\"; \
source /opt/ros/humble/setup.bash; \
source \"$ROOT_DIR/install/setup.bash\"; \
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r cmd_vel:=/cmd_vel"

RELATIVE_MOVE_CMD="export ROS_LOG_DIR=\"$ROS_LOG_DIR\"; \
source /opt/ros/humble/setup.bash; \
source \"$ROOT_DIR/install/setup.bash\"; \
cd \"$ROOT_DIR\"; \
python3 nav2_relative_move.py -h"

mkdir -p "$ROS_LOG_DIR"

echo "Root:        $ROOT_DIR"
echo "ROS logs:    $ROS_LOG_DIR"
echo "Camera:      $CAMERA_MODEL"
echo "Serial port: $SERIAL_PORT"
echo "RTAB-Map DB: $RTABMAP_DATABASE_PATH"
echo "New map:     $NEW_MAP_ON_START"
echo "tmux:        $TMUX_SESSION"
echo

if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "tmux session '$TMUX_SESSION' already exists."
    echo "Attach with:"
    echo "  tmux attach -t $TMUX_SESSION"
    exit 1
fi

tmux new-session -d -s "$TMUX_SESSION" -n "Robot Stack" "bash -lc '$STACK_CMD; exec bash'"
sleep 3
tmux new-window -t "$TMUX_SESSION" -n "Teleop Keyboard" "bash -lc '$TELEOP_CMD; exec bash'"
tmux new-window -t "$TMUX_SESSION" -n "Relative Move" "bash -lc '$RELATIVE_MOVE_CMD; exec bash'"
tmux select-window -t "$TMUX_SESSION:Teleop Keyboard"

echo "Opened robot stack and teleop in tmux session '$TMUX_SESSION'."
echo
echo "Attach with:"
echo "  tmux attach -t $TMUX_SESSION"
echo
echo "Windows:"
echo "  Robot Stack"
echo "  Teleop Keyboard"
echo "  Relative Move"
echo
echo "Stop everything with:"
echo "  tmux kill-session -t $TMUX_SESSION"
