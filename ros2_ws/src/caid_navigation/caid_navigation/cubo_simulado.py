"""
Detector de cubo simulado, para los mundos donde el cubo está en el piso (chico, maze_2).

Ahí la cámara no lo ve (mira horizontal), así que este nodo hace de percepción: conoce dónde está
el cubo en Gazebo y avisa cuando el robot entra a su misma celda, así nunca lo "ve" a través de una
pared. Publica la posición del cubo en el marco de la odometría, que nace en la pose de aparición.
En los escenarios e1 a e6 el cubo está a la altura de la cámara y lo detecta la visión.
"""
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class CuboSimulado(Node):

    def __init__(self):
        super().__init__('cubo_simulado')
        p = {
            'cubo_x': -1.8, 'cubo_y': 1.8,                       # m, en el mundo de Gazebo
            'robot_x': 2.25, 'robot_y': -0.9, 'robot_yaw': math.pi,  # pose de aparición
            'tam_celda': 0.45,
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        # Cubo en el marco de la odometría: trasladar y girar -yaw.
        dx = self.cfg['cubo_x'] - self.cfg['robot_x']
        dy = self.cfg['cubo_y'] - self.cfg['robot_y']
        c, s = math.cos(self.cfg['robot_yaw']), math.sin(self.cfg['robot_yaw'])
        self.cubo = Point(x=c * dx + s * dy, y=-s * dx + c * dy)
        self.celda_cubo = self.celda(self.cubo.x, self.cubo.y)

        self.pub = self.create_publisher(Point, '/nav/cubo_verde', 10)
        self.create_subscription(Odometry, '/sim/odom', self.on_odom, 10)

    def celda(self, x, y):
        tam = self.cfg['tam_celda']
        return (round(x / tam), round(y / tam))

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        if self.celda(pos.x, pos.y) == self.celda_cubo:
            self.pub.publish(self.cubo)


def main():
    rclpy.init()
    node = CuboSimulado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
