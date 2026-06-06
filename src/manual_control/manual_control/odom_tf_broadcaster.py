from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('parent_frame', '')
        self.declare_parameter('child_frame', '')

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._parent_frame = str(self.get_parameter('parent_frame').value)
        self._child_frame = str(self.get_parameter('child_frame').value)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_sub = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._odom_callback,
            20,
        )

        self.get_logger().info(
            f'Publishing odometry TF from {self._odom_topic}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        parent_frame = self._parent_frame or msg.header.frame_id
        child_frame = self._child_frame or msg.child_frame_id

        if not parent_frame or not child_frame:
            self.get_logger().warning(
                'Skipping odometry TF because frame ids are empty'
            )
            return

        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
