#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:
    # fallback if you installed python3-smbus instead of smbus2
    from smbus import SMBus

MPU_ADDR = 0x68

# Registers
WHO_AM_I     = 0x75
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
SIGNAL_PATH_RESET = 0x68

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# Scale factors (for FS settings below)
ACCEL_LSB_PER_G   = 16384.0  # ±2g
GYRO_LSB_PER_DPS  = 131.0    # ±250 dps
G_TO_M_S2         = 9.80665
DEG_TO_RAD        = math.pi / 180.0


def to_int16(high, low):
    v = (high << 8) | low
    return v - 65536 if v & 0x8000 else v


class MPU6050Node(Node):
    def __init__(self):
        super().__init__("mpu6050_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "/imu/data_raw")
        self.declare_parameter("rate_hz", 50.0)

        self.bus_id = int(self.get_parameter("i2c_bus").value)
        self.addr   = int(self.get_parameter("address").value)
        self.frame  = str(self.get_parameter("frame_id").value)
        self.topic  = str(self.get_parameter("topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(Imu, self.topic, qos_profile_sensor_data)

        self.bus = None
        self.connected = False
        self._connect()

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"Publishing {self.topic} at {self.rate_hz:.1f} Hz (addr 0x{self.addr:02X})")

    # ---------- I2C bring-up ----------
    def _connect(self):
        try:
            self.bus = SMBus(self.bus_id)

            # Verify device identity
            who = self.bus.read_byte_data(self.addr, WHO_AM_I)
            if who != 0x68:
                raise RuntimeError(f"WHO_AM_I expected 0x68, got 0x{who:02X}")

            self._init_mpu()
            self.connected = True
            self.get_logger().info("MPU6050 connected and initialized.")
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"MPU6050 connect/init failed: {e}")

    def _init_mpu(self):
        # Hard reset
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Wake + select PLL gyro clock (stable)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # Reset signal paths
        self.bus.write_byte_data(self.addr, SIGNAL_PATH_RESET, 0x07)
        time.sleep(0.1)

        # Sample rate divider and DLPF
        # Sample rate = 1kHz / (1 + SMPLRT_DIV) when DLPF enabled
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x09)  # ~100Hz internal
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)      # DLPF ~44Hz (good starting point)

        # Full-scale ranges:
        # Gyro: 0x00 => ±250 dps
        # Accel: 0x00 => ±2g
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00)

    # ---------- Robust read helpers ----------
    def _read_block(self, reg, length, retries=3):
        for attempt in range(retries):
            try:
                return self.bus.read_i2c_block_data(self.addr, reg, length)
            except OSError:
                time.sleep(0.01 * (attempt + 1))
        # One last raise so we can reconnect higher up
        return self.bus.read_i2c_block_data(self.addr, reg, length)

    # ---------- Publish loop ----------
    def _tick(self):
        if not self.connected:
            self._connect()
            return

        try:
            a = self._read_block(ACCEL_XOUT_H, 6)
            g = self._read_block(GYRO_XOUT_H, 6)

            ax_raw = to_int16(a[0], a[1])
            ay_raw = to_int16(a[2], a[3])
            az_raw = to_int16(a[4], a[5])

            gx_raw = to_int16(g[0], g[1])
            gy_raw = to_int16(g[2], g[3])
            gz_raw = to_int16(g[4], g[5])

            # Convert to SI units
            ax = (ax_raw / ACCEL_LSB_PER_G) * G_TO_M_S2
            ay = (ay_raw / ACCEL_LSB_PER_G) * G_TO_M_S2
            az = (az_raw / ACCEL_LSB_PER_G) * G_TO_M_S2

            gx = (gx_raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD
            gy = (gy_raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD
            gz = (gz_raw / GYRO_LSB_PER_DPS) * DEG_TO_RAD

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame

            # We are not computing orientation here
            msg.orientation_covariance[0] = -1.0

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            self.pub.publish(msg)

        except OSError as e:
            # Bus glitch: mark disconnected and retry next tick
            self.get_logger().warn(f"I2C read failed (will reconnect): {e}")
            self.connected = False


def main():
    rclpy.init()
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
