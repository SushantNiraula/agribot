import time
import socketio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

class ImuToFlaskBridge(Node):
    def __init__(self):
        super().__init__("imu_to_flask_bridge")

        self.declare_parameter("flask_url", "http://10.32.25.117:5000")  # change this
        self.declare_parameter("topic", "/imu/data_raw")
        self.declare_parameter("emit_rate_hz", 10.0)
        self.declare_parameter("robot_id", "agribot-01")

        self.flask_url = str(self.get_parameter("flask_url").value)
        self.topic = str(self.get_parameter("topic").value)
        self.emit_rate = float(self.get_parameter("emit_rate_hz").value)
        self.robot_id = str(self.get_parameter("robot_id").value)

        self.sio = socketio.Client(
            reconnection=True,
            reconnection_attempts=0,
            reconnection_delay=1,
            reconnection_delay_max=5,
            logger=False,
            engineio_logger=False,
        )

        self.sio.on("connect", self._on_connect)
        self.sio.on("disconnect", self._on_disconnect)

        self._connect_socket()

        self.sub = self.create_subscription(Imu, self.topic, self._on_imu, qos_profile_sensor_data)
        self._last_emit = 0.0

        self.get_logger().info(
            f"Subscribed to {self.topic}, sending to {self.flask_url} at {self.emit_rate} Hz"
        )

    def _connect_socket(self):
        try:
            if not self.sio.connected:
                self.sio.connect(self.flask_url, transports=["websocket", "polling"])
                self.get_logger().info("Connected to Flask SocketIO server.")
        except Exception as e:
            self.get_logger().warn(f"Socket connect failed: {e}")

    def _on_connect(self):
        self.get_logger().info("SocketIO connected.")

    def _on_disconnect(self):
        self.get_logger().warn("SocketIO disconnected.")

    def _on_imu(self, msg: Imu):
        now = time.time()
        if now - self._last_emit < (1.0 / max(self.emit_rate, 1.0)):
            return
        self._last_emit = now

        telemetry = {
            "ts": float(now),
            "robot_id": str(self.robot_id),
            "imu": {
            "frame_id": str(msg.header.frame_id),

            "ang_vel": {
            "x": float(msg.angular_velocity.x),
            "y": float(msg.angular_velocity.y),
            "z": float(msg.angular_velocity.z),
            },
            "lin_acc": {
            "x": float(msg.linear_acceleration.x),
            "y": float(msg.linear_acceleration.y),
            "z": float(msg.linear_acceleration.z),
            },

            # force plain Python bool
            "orientation_valid": bool(msg.orientation_covariance[0] != -1.0),
            }
        }

        try:
            if self.sio.connected:
                self.get_logger().info(f"Sending telemetry ts={now:.2f} gz={float(msg.angular_velocity.z):.3f}")
                self.sio.emit("telemetry", telemetry)
            else:
                self._connect_socket()
        except Exception as e:
            self.get_logger().warn(f"Emit failed: {e}")

def main():
    rclpy.init()
    node = ImuToFlaskBridge()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.sio.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
