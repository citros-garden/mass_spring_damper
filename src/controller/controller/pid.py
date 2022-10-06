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

        self._last_error = 0.0
        self._last_vel_feedback = 0.0

        time.sleep(1)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.setpoint = self.get_parameter('setpoint').get_parameter_value().double_value

        time.sleep(1)
        self.get_logger().info(f"Starting PID controller with: Kp = {self.kp}, Ki = {self.ki}, Kd = {self.kd}")

        self.timer = self.create_timer(self.dt, self.step)

    def pose_cb(self, msg):
        self.pose = msg.data

    def vel_cb(self, msg):
        self.vel = msg.data

    def step(self):
        error = self.setpoint - self.pose
        i_error = self._last_error + self.ki*error*self.dt
        d_error = (self.vel - self._last_vel_feedback)/self.dt

        p_element = self.kp * (error)
        i_element = self.ki * (i_error)
        d_element = self.kd * (d_error)
        
        self.ctrl_msg.data = p_element + i_element + d_element

        self.ctrl_pub.publish(self.ctrl_msg)

        self._last_error = i_error
        self._last_vel_feedback = self.vel

def main(args=None):
    rclpy.init(args=args)
    controller = Pid()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()