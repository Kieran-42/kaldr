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

open_terminal() {
    local title="$1"
    local cmd="$2"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash"
        return 0
    fi

    if command -v x-terminal-emulator >/dev/null 2>&1; then
        x-terminal-emulator -T "$title" -e bash -lc "$cmd; exec bash" &
        return 0
    fi

    if command -v konsole >/dev/null 2>&1; then
        konsole --new-tab -p tabtitle="$title" -e bash -lc "$cmd; exec bash" &
        return 0
    fi

    if command -v xterm >/dev/null 2>&1; then
        xterm -T "$title" -e bash -lc "$cmd; exec bash" &
        return 0
    fi

    return 1
}

mkdir -p "$ROS_LOG_DIR"

echo "Root:        $ROOT_DIR"
echo "ROS logs:    $ROS_LOG_DIR"
echo "Camera:      $CAMERA_MODEL"
echo "Serial port: $SERIAL_PORT"
echo "RTAB-Map DB: $RTABMAP_DATABASE_PATH"
echo "New map:     $NEW_MAP_ON_START"
echo

if open_terminal "Robot Stack" "$STACK_CMD"; then
    echo "Opened robot stack in a new terminal."
else
    echo "No terminal emulator found. Starting robot stack in the background here."
    bash -lc "$STACK_CMD" &
    STACK_PID=$!
    trap 'kill "$STACK_PID" 2>/dev/null || true' EXIT INT TERM
fi

sleep 3

if open_terminal "Teleop Keyboard" "$TELEOP_CMD"; then
    echo "Opened teleop_twist_keyboard in a new terminal."
    echo
    echo "Use the teleop terminal for driving. Close the stack terminal to stop the system."
else
    echo "No terminal emulator found for teleop."
    echo "Run this manually in another shell:"
    echo "  $TELEOP_CMD"
fi

if [[ -n "${STACK_PID:-}" ]]; then
    wait "$STACK_PID"
fi
