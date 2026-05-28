# manual_control

## cmd_vel serial bridge

This package now includes a ROS 2 node that subscribes to `cmd_vel` and forwards
the commanded velocity to an Arduino over serial.

### Run

```bash
ros2 run manual_control cmd_vel_serial_bridge --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baud_rate:=115200 \
  -p topic_name:=/cmd_vel
```

### Serial format

Each `Twist` message is sent as one newline-delimited ASCII record:

```text
CMD_VEL,<linear.x>,<linear.y>,<linear.z>,<angular.x>,<angular.y>,<angular.z>
```

Example:

```text
CMD_VEL,0.250000,0.000000,0.000000,0.000000,0.000000,0.800000
```

### Arduino-side parsing example

```cpp
String line = Serial.readStringUntil('\n');

char frame[16];
float lx, ly, lz, ax, ay, az;

if (sscanf(
      line.c_str(),
      "%15[^,],%f,%f,%f,%f,%f,%f",
      frame, &lx, &ly, &lz, &ax, &ay, &az) == 7) {
  // lx and az are the usual differential-drive cmd_vel fields.
}
```

## Move Forward Using Odometry

This package also includes a ROS 2 node that uses odometry feedback to move the
robot forward a fixed distance and then stop.

### Run

```bash
ros2 run manual_control move_forward_odom --ros-args \
  -p odom_topic:=/zed/zed_node/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p target_distance_m:=0.05 \
  -p linear_speed_mps:=0.03
```

The node waits for the first odometry sample, records the starting pose, drives
forward, and stops when the XY displacement reaches the configured target.
