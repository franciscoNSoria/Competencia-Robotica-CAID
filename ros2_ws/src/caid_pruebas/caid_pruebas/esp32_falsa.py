"""Simula la ESP32 de control (micro-ROS) con los mismos topics."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool


class Esp32Falsa(Node):
    def __init__(self):
        super().__init__('esp32_falsa')
        log = self.get_logger()
        self.create_subscription(Twist, '/cmd_vel',
            lambda m: log.info(f'MOTORES  v={m.linear.x:.2f}  w={m.angular.z:.2f}'), 10)
        self.create_subscription(JointState, '/brazo/cmd',
            lambda m: log.info(f'BRAZO    {list(m.position)}'), 10)
        self.create_subscription(Bool, '/pinza/cmd',
            lambda m: log.info(f'PINZA    {"cerrar" if m.data else "abrir"}'), 10)
        self.pub = self.create_publisher(Range, '/distancia/frente', 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        r = Range()
        r.header.frame_id = 'sensor_frente'
        r.range = 0.30  # "pared a 30 cm", fijo
        self.pub.publish(r)


def main():
    rclpy.init()
    rclpy.spin(Esp32Falsa())
