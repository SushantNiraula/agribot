import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        self.sub= self.create_subscription(Twist, '/cmd_vel', 10, self.cb)
        self.pub= self.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
    def cb(self, msg:Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist = msg
        self.pub.publish(out)
def main():
    rclpy.init()
    node = CmdVelStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()