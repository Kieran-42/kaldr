import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(rotation) -> float:
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TurnAngleMapOdom(Node):
    def __init__(self) -> None:
        super().__init__('turn_angle_map_odom')

        self.declare_parameter('feedback_source', 'odom_topic')
        self.declare_parameter('odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_yaw_deg', 90.0)
        self.declare_parameter('angular_speed_radps', 0.2)
        self.declare_parameter('min_angular_speed_radps', 0.05)
        self.declare_parameter('yaw_tolerance_deg', 1.0)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_timeout_sec', 0.75)
        self.declare_parameter('transform_timeout_sec', 0.2)
        self.declare_parameter('max_tf_age_sec', 0.75)
        self.declare_parameter('max_yaw_deg', 180.0)
        self.declare_parameter('stop_burst_count', 5)

        self._feedback_source = str(self.get_parameter('feedback_source').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._target_yaw_rad = math.radians(
            float(self.get_parameter('target_yaw_deg').value)
        )
        self._angular_speed_radps = float(
            self.get_parameter('angular_speed_radps').value
        )
        self._min_angular_speed_radps = float(
            self.get_parameter('min_angular_speed_radps').value
        )
        self._yaw_tolerance_rad = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value)
        )
        self._control_rate_hz = max(
            float(self.get_parameter('control_rate_hz').value),
            1.0,
        )
        self._odom_timeout_sec = max(
            float(self.get_parameter('odom_timeout_sec').value),
            0.0,
        )
        self._transform_timeout_sec = max(
            float(self.get_parameter('transform_timeout_sec').value),
            0.0,
        )
        self._max_tf_age_sec = max(
            float(self.get_parameter('max_tf_age_sec').value),
            0.0,
        )
        self._max_yaw_rad = math.radians(
            float(self.get_parameter('max_yaw_deg').value)
        )
        self._stop_burst_count = max(
            int(self.get_parameter('stop_burst_count').value),
            1,
        )

        if abs(self._target_yaw_rad) <= 0.0:
            raise ValueError('target_yaw_deg must be non-zero')

        if abs(self._target_yaw_rad) > self._max_yaw_rad:
            raise ValueError('abs(target_yaw_deg) must be <= max_yaw_deg')

        if self._angular_speed_radps <= 0.0:
            raise ValueError('angular_speed_radps must be > 0')

        if self._min_angular_speed_radps <= 0.0:
            raise ValueError('min_angular_speed_radps must be > 0')

        if self._feedback_source not in ('odom_topic', 'tf'):
            raise ValueError('feedback_source must be "odom_topic" or "tf"')

        self._min_angular_speed_radps = min(
            self._min_angular_speed_radps,
            self._angular_speed_radps,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            20,
        )

        self._latest_odom_yaw = None
        self._last_odom_time = None
        self._start_yaw = None
        self._previous_yaw = None
        self._accumulated_turn = 0.0
        self._goal_reached = False
        self._stop_messages_remaining = self._stop_burst_count

        self._control_timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_loop,
        )

        if self._feedback_source == 'odom_topic':
            self.get_logger().info(
                f'Waiting for odometry on {self._odom_topic} to turn '
                f'{math.degrees(self._target_yaw_rad):.1f} deg'
            )
        else:
            self.get_logger().info(
                'Waiting for TF '
                f'{self._map_frame} -> {self._base_frame} to turn '
                f'{math.degrees(self._target_yaw_rad):.1f} deg'
            )

    def _odom_callback(self, msg: Odometry) -> None:
        self._latest_odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self._last_odom_time = self.get_clock().now()

    def _current_tf_yaw(self) -> float:
        transform = self._tf_buffer.lookup_transform(
            self._map_frame,
            self._base_frame,
            Time(),
            timeout=Duration(seconds=self._transform_timeout_sec),
        )

        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds > 0 and self._max_tf_age_sec > 0.0:
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age > self._max_tf_age_sec:
                raise RuntimeError(
                    f'TF is stale: latest transform is {age:.2f} seconds old'
                )

        return quaternion_to_yaw(transform.transform.rotation)

    def _current_odom_yaw(self) -> float:
        if self._latest_odom_yaw is None or self._last_odom_time is None:
            raise RuntimeError(f'No odometry received on {self._odom_topic}')

        age = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        if age > self._odom_timeout_sec:
            raise RuntimeError(
                f'Odometry on {self._odom_topic} is stale after {age:.2f} seconds'
            )

        return self._latest_odom_yaw

    def _current_yaw(self) -> float:
        if self._feedback_source == 'odom_topic':
            return self._current_odom_yaw()

        return self._current_tf_yaw()

    def _publish_stop_and_exit(self, reason: str) -> None:
        if self._stop_messages_remaining == self._stop_burst_count:
            self.get_logger().info(reason)

        self._cmd_pub.publish(Twist())
        self._stop_messages_remaining -= 1

        if self._stop_messages_remaining <= 0:
            raise SystemExit

    def _update_accumulated_turn(self, current_yaw: float) -> None:
        if self._start_yaw is None:
            self._start_yaw = current_yaw
            self._previous_yaw = current_yaw
            self.get_logger().info(
                'Recorded starting yaw '
                f'{math.degrees(current_yaw):.1f} deg'
            )
            return

        delta = normalize_angle(current_yaw - self._previous_yaw)
        self._accumulated_turn += delta
        self._previous_yaw = current_yaw

    def _control_loop(self) -> None:
        if self._goal_reached:
            self._publish_stop_and_exit(
                'Target reached. Stopped after '
                f'{math.degrees(self._accumulated_turn):.1f} deg'
            )
            return

        try:
            current_yaw = self._current_yaw()
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF '
                f'{self._map_frame} -> {self._base_frame}: {exc}'
            )
            return
        except RuntimeError as exc:
            if self._start_yaw is None:
                self.get_logger().warning(str(exc))
                return

            self.get_logger().warning(f'{exc}. Publishing stop command.')
            self._goal_reached = True
            self._publish_stop_and_exit('Stopping because feedback timed out')
            return

        self._update_accumulated_turn(current_yaw)
        if self._start_yaw is None:
            return

        remaining = self._target_yaw_rad - self._accumulated_turn
        if abs(remaining) <= self._yaw_tolerance_rad:
            self._goal_reached = True
            self._publish_stop_and_exit(
                'Target reached. Stopped after '
                f'{math.degrees(self._accumulated_turn):.1f} deg'
            )
            return

        direction = 1.0 if remaining > 0.0 else -1.0
        speed = min(self._angular_speed_radps, max(abs(remaining), 0.0))
        speed = max(speed, self._min_angular_speed_radps)

        cmd = Twist()
        cmd.angular.z = direction * speed
        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurnAngleMapOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted. Publishing stop command.')
        node._cmd_pub.publish(Twist())
    except SystemExit:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
