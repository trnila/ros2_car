import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from . import driver

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.driver = driver.Ultrasonic()
        self.publisher = self.create_publisher(Range, 'topic', 10)
        self.timer = self.create_timer(0.02, self.measure)

    def measure(self):
        msg = Range()
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.366519143
        msg.min_range = 1 / 100
        msg.max_range = 30 / 100
        msg.range = self.driver.measure() / 100

        print(msg)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(UltrasonicPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
