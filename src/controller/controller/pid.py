import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64

class Pid(Node):

    def __init__(self):
        super().__init__('pid')
        self.ctrl_pub = self.create_publisher(Float64, '/controller/command', 10)

        self.pose_sub = self.create_subscription(Float64, '/dynamics/position', self.pose_cb, 10)
        self.vel_sub = self.create_subscription(Float64, '/dynamics/velocity', self.vel_cb, 10)

        self.dt = 0.25  # seconds
        # define parameters

        self.declare_parameter('kp')
        self.declare_parameter('ki')
        self.declare_parameter('kd')
        self.declare_parameter('setpoint')

        self.ctrl_msg = Float64()

        self.pose = 0.0
        self.vel = 0.0

        time.sleep(1)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.setpoint = self.get_parameter('setpoint').get_parameter_value().double_value

        time.sleep(1)

        self.timer = self.create_timer(self.dt, self.step)

    def pose_cb(self, msg):
        self.pose = msg.data

    def vel_cb(self, msg):
        self.vel = msg.data

    def step(self):
        p_element = self.kp * (self.setpoint - self.pose)
        i_element = self.ki * ()
        d_element = self.kd * ()

        
def main(args=None):
    rclpy.init(args=args)
    controller = Pid()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()