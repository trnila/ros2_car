import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from . import CarAPI

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.api = CarAPI.CarApi()
        self.wheelbase = 1
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.on_twist, 10)

    def on_twist(self, message):
        print(message)
        self.api.set_motor_power(int(message.linear.x))
        self.api.set_steering_angle(-int(message.angular.z))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MinimalPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
