import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Float64

class Dynamics(Node):

    def __init__(self):
        super().__init__('dynamics')
        self.pose_pub = self.create_publisher(Float64, '/dynamics/position', 10)
        self.vel_pub = self.create_publisher(Float64, '/dynamics/velocity', 10)
        self.ctrl_sub = self.create_subscription(Float64, '/controller/command', self.ctrl_cb, 10)

        self.dt = 0.01  # seconds
        # define parameters
        m = 1.0
        k = 2.5
        c = 0.3

        x0 = 15.0
        v0 = 0.0
        a0 = 0.0

        self.declare_parameter('m', m)
        self.declare_parameter('k', k)
        self.declare_parameter('c', c)

        self.declare_parameter('x', x0)
        self.declare_parameter('v', v0)
        self.declare_parameter('a', a0)

        self.ctrl_cmd = 0

        time.sleep(1)

        self.m = self.get_parameter('m').get_parameter_value().double_value
        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.c = self.get_parameter('c').get_parameter_value().double_value

        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.v = self.get_parameter('v').get_parameter_value().double_value
        self.a = self.get_parameter('a').get_parameter_value().double_value

        time.sleep(1)

        self.timer = self.create_timer(self.dt, self.step)

    def ctrl_cb(self, msg):
        self.ctrl_cmd = msg.data

    def solve_ode(self):
        spring_force = self.k * self.x # Fs = k * x
        damper_force = self.c * self.v # Fc = c * v

        self.a = (self.ctrl_cmd - spring_force + damper_force) / self.m

        self.v += (self.a * self.dt) # Integral(a) = v

        self.x += (self.v * self.dt) # Integral(v) = x

    def step(self):

        self.solve_ode()

        pose = Float64()
        vel = Float64()
        pose.data = self.x
        vel.data = self.v
        self.get_logger().info(f"mass position: {pose.data}")

        self.pose_pub.publish(pose)
        self.vel_pub.publish(vel)
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = Dynamics()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()