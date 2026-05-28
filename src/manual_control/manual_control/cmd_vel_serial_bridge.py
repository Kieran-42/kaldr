import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial
    from serial import SerialException
except ModuleNotFoundError as exc:
    serial = None
    SerialException = Exception
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


class CmdVelSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_serial_bridge')

        if serial is None:
            raise RuntimeError(
                'Missing Python serial support. Install `python3-serial` and '
                'rebuild/source the workspace before running `cmd_vel_serial_bridge`.'
            ) from _SERIAL_IMPORT_ERROR

        self.declare_parameter('topic_name', '/cmd_vel')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'CMD_VEL')
        self.declare_parameter('reconnect_period_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('command_timeout_sec', 0.35)
        self.declare_parameter('connect_settle_sec', 2.0)
        self.declare_parameter('assert_dtr', False)
        self.declare_parameter('assert_rts', False)

        self._topic_name = self.get_parameter('topic_name').value
        self._serial_port = self.get_parameter('serial_port').value
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._reconnect_period_sec = float(
            self.get_parameter('reconnect_period_sec').value
        )
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        self._command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self._connect_settle_sec = float(
            self.get_parameter('connect_settle_sec').value
        )
        self._assert_dtr = bool(self.get_parameter('assert_dtr').value)
        self._assert_rts = bool(self.get_parameter('assert_rts').value)

        self._serial = None
        self._last_connect_attempt = 0.0
        self._last_cmd_time = 0.0
        self._last_zero_sent = False
        self._last_payload = None
        self._last_logged_payload = None
        self._last_log_time = 0.0
        self._latest_cmd = Twist()

        self._subscription = self.create_subscription(
            Twist,
            self._topic_name,
            self._cmd_vel_callback,
            10,
        )
        self._reconnect_timer = self.create_timer(0.5, self._ensure_serial_connection)
        self._publish_timer = self.create_timer(
            1.0 / max(self._publish_rate_hz, 1.0),
            self._publish_latest_command,
        )

        self._ensure_serial_connection()

    def _ensure_serial_connection(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return

        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_period_sec:
            return

        self._last_connect_attempt = now

        try:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=0.1,
                write_timeout=0.1,
                dsrdtr=False,
                rtscts=False,
            )
            self._serial.dtr = self._assert_dtr
            self._serial.rts = self._assert_rts
            if self._connect_settle_sec > 0.0:
                time.sleep(self._connect_settle_sec)
            self.get_logger().info(
                f'Connected to Arduino on {self._serial_port} at {self._baud_rate} baud '
                f'(DTR={self._assert_dtr}, RTS={self._assert_rts}, settle={self._connect_settle_sec:.1f}s)'
            )
        except SerialException as exc:
            self._serial = None
            self.get_logger().warning(
                f'Unable to open serial port {self._serial_port}: {exc}'
            )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._last_cmd_time = time.monotonic()
        self._last_zero_sent = False

    def _format_payload(self, msg: Twist) -> str:
        return (
            f'{self._frame_id},'
            f'{msg.linear.x:.6f},{msg.linear.y:.6f},{msg.linear.z:.6f},'
            f'{msg.angular.x:.6f},{msg.angular.y:.6f},{msg.angular.z:.6f}\n'
        )

    def _publish_latest_command(self) -> None:
        self._ensure_serial_connection()
        if self._serial is None or not self._serial.is_open:
            return

        now = time.monotonic()
        cmd = self._latest_cmd

        if self._last_cmd_time == 0.0 or (
            now - self._last_cmd_time > self._command_timeout_sec
        ):
            cmd = Twist()
            if self._last_zero_sent:
                return
            self._last_zero_sent = True

        payload = self._format_payload(cmd)

        try:
            self._serial.write(payload.encode('ascii'))
            self._serial.flush()
            self._last_payload = payload
            self._log_command(payload, cmd)
        except SerialException as exc:
            self.get_logger().warning(f'Serial write failed: {exc}')
            self._close_serial()

    def _log_command(self, payload: str, msg: Twist) -> None:
        now = time.monotonic()
        is_zero = (
            msg.linear.x == 0.0
            and msg.linear.y == 0.0
            and msg.linear.z == 0.0
            and msg.angular.x == 0.0
            and msg.angular.y == 0.0
            and msg.angular.z == 0.0
        )

        if is_zero:
            if payload != self._last_logged_payload:
                self.get_logger().info('Streaming stop command to motor controller')
                self._last_logged_payload = payload
                self._last_log_time = now
            return

        if payload != self._last_logged_payload or now - self._last_log_time >= 1.0:
            self.get_logger().info(
                'Streaming cmd_vel '
                f'lin=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.linear.z:.3f}) '
                f'ang=({msg.angular.x:.3f},{msg.angular.y:.3f},{msg.angular.z:.3f})'
            )
            self._last_logged_payload = payload
            self._last_log_time = now

    def _close_serial(self) -> None:
        if self._serial is None:
            return

        try:
            if self._serial.is_open:
                self._serial.close()
        except SerialException:
            pass
        finally:
            self._serial = None

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
