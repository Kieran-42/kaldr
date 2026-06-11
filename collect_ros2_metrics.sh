#!/usr/bin/env bash

set -euo pipefail

# ==============================
# MECHA-KALDR ROS 2 Metrics Tool
# Collects topic Hz, bandwidth, graph data, and system metadata.
# ==============================

DURATION="${1:-15}"   # seconds per measurement; default = 15
OUT_ROOT="${2:-$HOME/kaldr_ros2_metrics}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$OUT_ROOT/run_$STAMP"

mkdir -p "$OUT_DIR/raw/hz" "$OUT_DIR/raw/bw" "$OUT_DIR/summary"

echo "=========================================="
echo "MECHA-KALDR ROS 2 Metrics Collection"
echo "Duration per topic: ${DURATION}s"
echo "Output directory: $OUT_DIR"
echo "=========================================="

# Make sure ROS 2 is available
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command not found. Did you source ROS 2 and your workspace?"
    echo "Example:"
    echo "  source /opt/ros/\$ROS_DISTRO/setup.bash"
    echo "  source install/setup.bash"
    exit 1
fi

# ------------------------------
# Topics to measure
# ------------------------------

HZ_TOPICS=(
    "/zed/zed_node/rgb/color/rect/image"
    "/zed/zed_node/depth/depth_registered"
    "/zed/zed_node/point_cloud/cloud_registered"
    "/imu/data"
    "/zed/zed_node/odom"
    "/odom"
    "/rtabmap/map"
    "/global_costmap/costmap"
    "/local_costmap/costmap"
    "/cmd_vel_nav"
    "/cmd_vel"
    "/tf"
)

BW_TOPICS=(
    "/zed/zed_node/rgb/color/rect/image"
    "/zed/zed_node/depth/depth_registered"
    "/zed/zed_node/point_cloud/cloud_registered"
    "/rtabmap/map"
    "/global_costmap/costmap"
    "/local_costmap/costmap"
)

# ------------------------------
# Save system-level information
# ------------------------------

echo "[1/6] Saving ROS 2 system information..."

ros2 node list > "$OUT_DIR/node_list.txt" || true
ros2 topic list > "$OUT_DIR/topic_list.txt" || true
ros2 topic list -t > "$OUT_DIR/topic_list_with_types.txt" || true
ros2 service list > "$OUT_DIR/service_list.txt" || true
ros2 action list > "$OUT_DIR/action_list.txt" || true
ros2 param list > "$OUT_DIR/param_list.txt" || true

# ------------------------------
# Helper functions
# ------------------------------

safe_name() {
    echo "$1" | sed 's#^/##' | sed 's#/#__#g'
}

topic_exists() {
    local topic="$1"
    ros2 topic list | grep -qx "$topic"
}

collect_hz() {
    local topic="$1"
    local name
    name="$(safe_name "$topic")"

    if topic_exists "$topic"; then
        echo "  Measuring Hz: $topic"
        timeout "$DURATION" ros2 topic hz "$topic" > "$OUT_DIR/raw/hz/${name}_hz.txt" 2>&1 || true
    else
        echo "  Skipping Hz, topic not active: $topic"
        echo "Topic not active: $topic" > "$OUT_DIR/raw/hz/${name}_hz.txt"
    fi
}

collect_bw() {
    local topic="$1"
    local name
    name="$(safe_name "$topic")"

    if topic_exists "$topic"; then
        echo "  Measuring bandwidth: $topic"
        timeout "$DURATION" ros2 topic bw "$topic" > "$OUT_DIR/raw/bw/${name}_bw.txt" 2>&1 || true
    else
        echo "  Skipping bandwidth, topic not active: $topic"
        echo "Topic not active: $topic" > "$OUT_DIR/raw/bw/${name}_bw.txt"
    fi
}

# ------------------------------
# Collect Hz
# ------------------------------

echo "[2/6] Measuring topic frequencies..."

for topic in "${HZ_TOPICS[@]}"; do
    collect_hz "$topic"
done

# ------------------------------
# Collect bandwidth
# ------------------------------

echo "[3/6] Measuring topic bandwidth..."

for topic in "${BW_TOPICS[@]}"; do
    collect_bw "$topic"
done

# ------------------------------
# Collect topic connection info
# ------------------------------

echo "[4/6] Saving verbose topic connection information..."

mkdir -p "$OUT_DIR/raw/topic_info"

for topic in "${HZ_TOPICS[@]}"; do
    name="$(safe_name "$topic")"
    if topic_exists "$topic"; then
        ros2 topic info "$topic" --verbose > "$OUT_DIR/raw/topic_info/${name}_info.txt" 2>&1 || true
    else
        echo "Topic not active: $topic" > "$OUT_DIR/raw/topic_info/${name}_info.txt"
    fi
done

# ------------------------------
# Generate TF tree if available
# ------------------------------

echo "[5/6] Attempting to generate TF tree..."

if ros2 pkg executables tf2_tools 2>/dev/null | grep -q view_frames; then
    (
        cd "$OUT_DIR"
        timeout 10 ros2 run tf2_tools view_frames > tf_view_frames_log.txt 2>&1 || true
    )

    if [ -f "$OUT_DIR/frames.pdf" ]; then
        mv "$OUT_DIR/frames.pdf" "$OUT_DIR/tf_tree.pdf"
    fi
else
    echo "tf2_tools view_frames not available. Install with:" > "$OUT_DIR/tf_view_frames_log.txt"
    echo "sudo apt install ros-\${ROS_DISTRO}-tf2-tools" >> "$OUT_DIR/tf_view_frames_log.txt"
fi

# ------------------------------
# Build CSV summaries
# ------------------------------

echo "[6/6] Creating CSV summaries..."

HZ_CSV="$OUT_DIR/summary/topic_hz_summary.csv"
BW_CSV="$OUT_DIR/summary/topic_bw_summary.csv"

echo "topic,average_rate_hz,min_hz,max_hz,std_dev_hz,window_samples,status" > "$HZ_CSV"

for topic in "${HZ_TOPICS[@]}"; do
    name="$(safe_name "$topic")"
    file="$OUT_DIR/raw/hz/${name}_hz.txt"

    if grep -q "average rate:" "$file"; then
        avg="$(grep "average rate:" "$file" | tail -1 | awk '{print $3}')"
        min="$(grep "min:" "$file" | tail -1 | sed -E 's/.*min: ([^ ]+)s.*/\1/' | awk '{if ($1 > 0) print 1/$1; else print ""}')"
        max="$(grep "max:" "$file" | tail -1 | sed -E 's/.*max: ([^ ]+)s.*/\1/' | awk '{if ($1 > 0) print 1/$1; else print ""}')"
        std="$(grep "std dev:" "$file" | tail -1 | sed -E 's/.*std dev: ([^ ]+)s.*/\1/' | awk '{if ($1 > 0) print $1; else print ""}')"
        window="$(grep "window:" "$file" | tail -1 | sed -E 's/.*window: ([0-9]+).*/\1/')"
        echo "$topic,$avg,$min,$max,$std,$window,measured" >> "$HZ_CSV"
    else
        echo "$topic,,,,,,not_measured" >> "$HZ_CSV"
    fi
done

echo "topic,average_bandwidth,status" > "$BW_CSV"

for topic in "${BW_TOPICS[@]}"; do
    name="$(safe_name "$topic")"
    file="$OUT_DIR/raw/bw/${name}_bw.txt"

    if grep -q "average:" "$file"; then
        avg_bw="$(grep "average:" "$file" | tail -1 | sed 's/^[[:space:]]*average:[[:space:]]*//')"
        echo "$topic,\"$avg_bw\",measured" >> "$BW_CSV"
    else
        echo "$topic,,not_measured" >> "$BW_CSV"
    fi
done

# ------------------------------
# Create thesis-ready markdown table
# ------------------------------

MD="$OUT_DIR/summary/thesis_metrics_table.md"

cat > "$MD" <<EOF
# ROS 2 Communication Metrics

Measurement duration per topic: ${DURATION} seconds  
Collection time: ${STAMP}

## Topic Frequency Summary

| Topic | Average Rate (Hz) | Status |
|---|---:|---|
EOF

tail -n +2 "$HZ_CSV" | while IFS=',' read -r topic avg min max std window status; do
    echo "| \`$topic\` | ${avg:-N/A} | $status |" >> "$MD"
done

cat >> "$MD" <<EOF

## Topic Bandwidth Summary

| Topic | Average Bandwidth | Status |
|---|---:|---|
EOF

tail -n +2 "$BW_CSV" | while IFS=',' read -r topic avg status; do
    echo "| \`$topic\` | ${avg:-N/A} | $status |" >> "$MD"
done

echo ""
echo "=========================================="
echo "Collection complete."
echo "Raw data:      $OUT_DIR/raw"
echo "CSV summaries: $OUT_DIR/summary"
echo "Thesis table:  $OUT_DIR/summary/thesis_metrics_table.md"
echo "=========================================="
