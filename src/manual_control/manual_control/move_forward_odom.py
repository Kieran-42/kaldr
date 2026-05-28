import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class MoveForwardOdom(Node):
    def __init__(self) -> None:
        super().__init__('move_forward_odom')

        self.declare_parameter('odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_distance_m', 0.05)
        self.declare_parameter('linear_speed_mps', 0.03)
        self.declare_parameter('control_rate_hz', 15.0)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('stop_burst_count', 5)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._target_distance_m = float(
            self.get_parameter('target_distance_m').value
        )
        self._linear_speed_mps = float(
            self.get_parameter('linear_speed_mps').value
        )
        self._control_rate_hz = max(
            float(self.get_parameter('control_rate_hz').value),
            1.0,
        )
        self._odom_timeout_sec = max(
            float(self.get_parameter('odom_timeout_sec').value),
            0.0,
        )
        self._stop_burst_count = max(
            int(self.get_parameter('stop_burst_count').value),
            1,
        )

        if self._target_distance_m <= 0.0:
            raise ValueError('target_distance_m must be > 0')

        if self._linear_speed_mps <= 0.0:
            raise ValueError('linear_speed_mps must be > 0')

        self._start_xy = None
        self._latest_xy = None
        self._last_odom_time = None
        self._goal_reached = False
        self._stop_messages_remaining = self._stop_burst_count

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            10,
        )
        self._control_timer = self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_loop,
        )

        self.get_logger().info(
            'Waiting for odometry on '
            f'{self._odom_topic} to move forward {self._target_distance_m:.3f} m'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose.position
        xy = (float(pose.x), float(pose.y))
        self._latest_xy = xy
        self._last_odom_time = self.get_clock().now()

        if self._start_xy is None:
            self._start_xy = xy
            self.get_logger().info(
                f'Recorded starting odom pose x={xy[0]:.3f}, y={xy[1]:.3f}'
            )

    def _distance_traveled(self) -> float:
        if self._start_xy is None or self._latest_xy is None:
            return 0.0

        dx = self._latest_xy[0] - self._start_xy[0]
        dy = self._latest_xy[1] - self._start_xy[1]
        return math.hypot(dx, dy)

    def _odom_is_stale(self) -> bool:
        if self._last_odom_time is None:
            return True

        age = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        return age > self._odom_timeout_sec

    def _publish_stop_and_exit(self, reason: str) -> None:
        if self._stop_messages_remaining == self._stop_burst_count:
            self.get_logger().info(reason)

        self._cmd_pub.publish(Twist())
        self._stop_messages_remaining -= 1

        if self._stop_messages_remaining <= 0:
            raise SystemExit

    def _control_loop(self) -> None:
        if self._goal_reached:
            self._publish_stop_and_exit(
                f'Target reached. Stopped after {self._distance_traveled():.3f} m'
            )
            return

        if self._start_xy is None:
            return

        if self._odom_is_stale():
            self.get_logger().warning('Odometry is stale. Publishing stop command.')
            self._goal_reached = True
            self._publish_stop_and_exit('Stopping because odometry timed out')
            return

        distance_traveled = self._distance_traveled()
        remaining = self._target_distance_m - distance_traveled

        if remaining <= 0.0:
            self._goal_reached = True
            self._publish_stop_and_exit(
                f'Target reached. Stopped after {distance_traveled:.3f} m'
            )
            return

        cmd = Twist()
        cmd.linear.x = min(self._linear_speed_mps, max(remaining, 0.01))
        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveForwardOdom()

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
