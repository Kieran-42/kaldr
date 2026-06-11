import argparse
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


def yaw_to_quaternion(yaw_rad):
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw_rad / 2.0),
        "w": math.cos(yaw_rad / 2.0),
    }


class RelativeNavGoal(Node):
    def __init__(self, forward, left, yaw_deg, global_frame, base_frame):
        super().__init__("relative_nav_goal")

        self.forward = forward
        self.left = left
        self.yaw_rad = math.radians(yaw_deg)

        self.global_frame = global_frame
        self.base_frame = base_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self):
        self.get_logger().info("Waiting for Nav2 NavigateToPose action server...")
        self.nav_client.wait_for_server()

        self.get_logger().info(
            f"Looking up transform {self.global_frame} <- {self.base_frame}"
        )

        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=2.0),
        )

        relative_pose = PoseStamped()
        relative_pose.header.frame_id = self.base_frame
        relative_pose.header.stamp = self.get_clock().now().to_msg()

        relative_pose.pose.position.x = self.forward
        relative_pose.pose.position.y = self.left
        relative_pose.pose.position.z = 0.0

        q = yaw_to_quaternion(self.yaw_rad)
        relative_pose.pose.orientation.x = q["x"]
        relative_pose.pose.orientation.y = q["y"]
        relative_pose.pose.orientation.z = q["z"]
        relative_pose.pose.orientation.w = q["w"]

        goal_pose = do_transform_pose(relative_pose.pose, transform)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = goal_pose

        self.get_logger().info(
            f"Sending relative goal: forward={self.forward:.3f} m, "
            f"left={self.left:.3f} m, yaw={math.degrees(self.yaw_rad):.1f} deg"
        )

        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        self.get_logger().info(f"Nav2 result status: {result.status}")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--forward", type=float, default=0.10)
    parser.add_argument("--left", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw in degrees")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")

    args = parser.parse_args()

    rclpy.init()
    node = RelativeNavGoal(
        forward=args.forward,
        left=args.left,
        yaw_deg=args.yaw,
        global_frame=args.global_frame,
        base_frame=args.base_frame,
    )

    try:
        node.send_goal()
    except Exception as exc:
        node.get_logger().error(f"Failed to send relative Nav2 goal: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
