#!/usr/bin/env python3

import argparse
import math
import sys
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_ACTION_NAME = "navigate_to_pose"
DEFAULT_GLOBAL_FRAME = "map"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_ODOM_TOPIC = "/zed/zed_node/odom"
DEFAULT_CMD_VEL_TOPIC = "/cmd_vel"


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def yaw_to_quaternion(yaw_rad):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw_rad / 2.0),
        "w": math.cos(yaw_rad / 2.0),
    }


def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_motion(forward, left, yaw_deg, max_translation, max_yaw_deg):
    translation = math.hypot(forward, left)
    if translation > max_translation:
        raise ValueError(
            f"requested translation is {translation:.3f} m, "
            f"above --max-translation {max_translation:.3f} m"
        )

    if abs(yaw_deg) > max_yaw_deg:
        raise ValueError(
            f"requested yaw is {yaw_deg:.1f} deg, "
            f"above --max-yaw {max_yaw_deg:.1f} deg"
        )


class Nav2RelativeMove(Node):
    def __init__(
        self,
        action_name,
        global_frame,
        base_frame,
        odom_topic,
        cmd_vel_topic,
        tf_timeout,
        server_timeout,
    ):
        super().__init__("nav2_relative_move")
        self.global_frame = global_frame
        self.base_frame = base_frame
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.tf_timeout = tf_timeout
        self.server_timeout = server_timeout
        self.latest_odom = None
        self.last_odom_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )

    def _odom_callback(self, msg):
        self.latest_odom = msg
        self.last_odom_time = self.get_clock().now()

    def make_relative_pose(self, forward, left, yaw_deg):
        relative_pose = PoseStamped()
        relative_pose.header.frame_id = self.base_frame
        relative_pose.header.stamp = self.get_clock().now().to_msg()
        relative_pose.pose.position.x = forward
        relative_pose.pose.position.y = left
        relative_pose.pose.position.z = 0.0

        q = yaw_to_quaternion(math.radians(yaw_deg))
        relative_pose.pose.orientation.x = q["x"]
        relative_pose.pose.orientation.y = q["y"]
        relative_pose.pose.orientation.z = q["z"]
        relative_pose.pose.orientation.w = q["w"]
        return relative_pose

    def transform_to_global(self, relative_pose):
        self.get_logger().info(
            f"Looking up transform {self.global_frame} <- {self.base_frame}"
        )
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_timeout),
        )

        global_pose = do_transform_pose(relative_pose.pose, transform)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = global_pose
        return goal_pose

    def wait_for_nav2(self):
        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        if not self.nav_client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(
                "Nav2 action server was not available. Start the stack with "
                "./start_robot_stack.sh and wait for Nav2 lifecycle nodes to activate."
            )

    def send_goal(self, goal_pose, result_timeout: Optional[float]):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None:
            raise RuntimeError("failed to get a response from Nav2")

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        timeout_sec = None if result_timeout <= 0.0 else result_timeout
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)

        if not result_future.done():
            self.get_logger().error("Timed out waiting for Nav2 result.")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False

        result = result_future.result()
        status_name = STATUS_NAMES.get(result.status, str(result.status))
        self.get_logger().info(f"Nav2 result status: {status_name}")
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def wait_for_odom(self, timeout_sec):
        self.get_logger().info(f"Waiting for odometry on {self.odom_topic}...")
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self.latest_odom is None:
            if self.get_clock().now() > deadline:
                raise RuntimeError(
                    f"Timed out waiting for odometry on {self.odom_topic}"
                )
            rclpy.spin_once(self, timeout_sec=0.05)

    def assert_odom_fresh(self, timeout_sec):
        if self.last_odom_time is None:
            raise RuntimeError(f"No odometry received on {self.odom_topic}")

        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if age > timeout_sec:
            self.publish_stop()
            raise RuntimeError(
                f"Odometry on {self.odom_topic} is stale after {age:.2f} seconds"
            )

    def publish_stop(self, count=5):
        stop = Twist()
        for _ in range(count):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive_with_odom(
        self,
        forward,
        left,
        yaw_deg,
        linear_speed,
        angular_speed,
        odom_timeout,
    ):
        if abs(left) > 1e-6:
            raise RuntimeError(
                "Odom fallback cannot execute --left because this robot is "
                "controlled with linear.x/angular.z. Start Nav2 with a valid "
                "map frame for lateral relative goals."
            )

        self.wait_for_odom(odom_timeout)
        start_pose = self.latest_odom.pose.pose
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        start_yaw = quaternion_to_yaw(start_pose.orientation)

        if abs(forward) > 1e-6:
            direction = 1.0 if forward > 0.0 else -1.0
            target_distance = abs(forward)
            self.get_logger().info(
                f"Odom fallback driving {forward:.3f} m on {self.cmd_vel_topic}"
            )

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                self.assert_odom_fresh(odom_timeout)
                pose = self.latest_odom.pose.pose
                dx = pose.position.x - start_x
                dy = pose.position.y - start_y
                traveled = direction * (
                    dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
                )
                remaining = target_distance - traveled

                if remaining <= 0.0:
                    break

                cmd = Twist()
                cmd.linear.x = direction * min(linear_speed, max(remaining, 0.0))
                self.cmd_pub.publish(cmd)

            self.publish_stop()

        if abs(yaw_deg) > 1e-6:
            self.wait_for_odom(odom_timeout)
            start_yaw = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
            target_yaw = math.radians(yaw_deg)
            direction = 1.0 if target_yaw > 0.0 else -1.0
            self.get_logger().info(
                f"Odom fallback rotating {yaw_deg:.1f} deg on {self.cmd_vel_topic}"
            )

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                self.assert_odom_fresh(odom_timeout)
                current_yaw = quaternion_to_yaw(
                    self.latest_odom.pose.pose.orientation
                )
                turned = direction * normalize_angle(current_yaw - start_yaw)
                remaining = abs(target_yaw) - turned

                if remaining <= math.radians(1.0):
                    break

                cmd = Twist()
                cmd.angular.z = direction * min(angular_speed, max(remaining, 0.0))
                self.cmd_pub.publish(cmd)

            self.publish_stop()

        self.get_logger().info("Odom fallback move complete.")
        return True

    def dry_run_log(self, relative_pose, goal_pose):
        rel = relative_pose.pose
        goal = goal_pose.pose
        self.get_logger().info(
            "Dry run only. No goal sent to Nav2. "
            f"relative=({rel.position.x:.3f}, {rel.position.y:.3f}) "
            f"{self.base_frame}, "
            f"global=({goal.position.x:.3f}, {goal.position.y:.3f}) "
            f"{self.global_frame}"
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Send a robot-relative movement to the Nav2 stack launched by "
            "start_robot_stack.sh."
        )
    )
    parser.add_argument("--forward", type=float, default=0.10, help="Meters forward.")
    parser.add_argument("--left", type=float, default=0.0, help="Meters left.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw in degrees.")
    parser.add_argument(
        "--mode",
        choices=["auto", "nav2", "odom"],
        default="auto",
        help=(
            "auto tries Nav2 first and falls back to odom/cmd_vel when Nav2 "
            "cannot run; nav2 uses only NavigateToPose; odom uses direct "
            "odometry/cmd_vel control."
        ),
    )
    parser.add_argument("--global-frame", default=DEFAULT_GLOBAL_FRAME)
    parser.add_argument("--base-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--cmd-vel-topic", default=DEFAULT_CMD_VEL_TOPIC)
    parser.add_argument("--action-name", default=DEFAULT_ACTION_NAME)
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--odom-timeout", type=float, default=5.0)
    parser.add_argument("--linear-speed", type=float, default=0.03)
    parser.add_argument("--angular-speed", type=float, default=0.20)
    parser.add_argument(
        "--result-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for Nav2 result. Use 0 to wait forever.",
    )
    parser.add_argument(
        "--max-translation",
        type=float,
        default=1.0,
        help="Safety limit for combined forward/left distance in meters.",
    )
    parser.add_argument(
        "--max-yaw",
        type=float,
        default=90.0,
        help="Safety limit for absolute yaw in degrees.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and log the goal pose without sending it to Nav2.",
    )
    return parser


def main():
    args = build_parser().parse_args()

    try:
        validate_motion(
            args.forward,
            args.left,
            args.yaw,
            args.max_translation,
            args.max_yaw,
        )
    except ValueError as exc:
        print(f"Refusing command: {exc}", file=sys.stderr)
        return 2

    if args.linear_speed <= 0.0:
        print("Refusing command: --linear-speed must be > 0", file=sys.stderr)
        return 2

    if args.angular_speed <= 0.0:
        print("Refusing command: --angular-speed must be > 0", file=sys.stderr)
        return 2

    rclpy.init()
    node = Nav2RelativeMove(
        action_name=args.action_name,
        global_frame=args.global_frame,
        base_frame=args.base_frame,
        odom_topic=args.odom_topic,
        cmd_vel_topic=args.cmd_vel_topic,
        tf_timeout=args.tf_timeout,
        server_timeout=args.server_timeout,
    )

    try:
        if args.dry_run and args.mode == "odom":
            node.get_logger().info(
                "Dry run only. No cmd_vel sent. "
                f"odom move forward={args.forward:.3f} m, "
                f"left={args.left:.3f} m, yaw={args.yaw:.1f} deg"
            )
            return 0

        if args.mode == "odom":
            success = node.drive_with_odom(
                args.forward,
                args.left,
                args.yaw,
                args.linear_speed,
                args.angular_speed,
                args.odom_timeout,
            )
            return 0 if success else 1

        relative_pose = node.make_relative_pose(args.forward, args.left, args.yaw)
        goal_pose = node.transform_to_global(relative_pose)

        node.get_logger().info(
            f"Requested relative move: forward={args.forward:.3f} m, "
            f"left={args.left:.3f} m, yaw={args.yaw:.1f} deg"
        )
        node.get_logger().info(
            f"Goal in {args.global_frame}: "
            f"x={goal_pose.pose.position.x:.3f}, "
            f"y={goal_pose.pose.position.y:.3f}, "
            f"z={goal_pose.pose.position.z:.3f}"
        )

        if args.dry_run:
            node.dry_run_log(relative_pose, goal_pose)
            return 0

        node.wait_for_nav2()
        success = node.send_goal(goal_pose, args.result_timeout)
        if success or args.mode == "nav2":
            return 0 if success else 1

        node.get_logger().warn("Nav2 goal did not succeed; falling back to odom.")
        success = node.drive_with_odom(
            args.forward,
            args.left,
            args.yaw,
            args.linear_speed,
            args.angular_speed,
            args.odom_timeout,
        )
        return 0 if success else 1
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted before completion.")
        return 130
    except (RuntimeError, TransformException) as exc:
        if args.dry_run:
            node.get_logger().error(str(exc))
            return 1

        if args.mode == "auto":
            node.get_logger().warn(f"Nav2 path unavailable: {exc}")
            try:
                success = node.drive_with_odom(
                    args.forward,
                    args.left,
                    args.yaw,
                    args.linear_speed,
                    args.angular_speed,
                    args.odom_timeout,
                )
                return 0 if success else 1
            except Exception as fallback_exc:
                node.get_logger().error(f"Odom fallback failed: {fallback_exc}")
                return 1

        node.get_logger().error(str(exc))
        return 1
    except Exception as exc:
        node.get_logger().error(f"Failed to send relative Nav2 goal: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
