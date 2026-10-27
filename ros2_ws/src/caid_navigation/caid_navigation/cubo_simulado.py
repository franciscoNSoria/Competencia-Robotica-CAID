"""
Detector de cubo simulado: reemplaza a la percepción mientras no esté integrada (PER-04).

Conoce dónde está el cubo en Gazebo y avisa cuando el robot entra a su celda (a menos de
'alcance'); como el alcance es menor que una celda, nunca lo "ve" a través de una pared.
Publica la posición del cubo en el marco de la odometría, que nace en la pose de aparición.
Cuando el robot lo carga (/nav/carga = true), saca el cubo del mundo de Gazebo.
"""
import math
import subprocess

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


class CuboSimulado(Node):

    def __init__(self):
        super().__init__('cubo_simulado')
        p = {
            'cubo_x': -1.8, 'cubo_y': 1.8,                       # m, en el mundo de Gazebo
            'robot_x': 2.25, 'robot_y': -0.9, 'robot_yaw': math.pi,  # pose de aparición
            'alcance': 0.35,
            'modelo': 'cubo_verde',
            'mundo': 'ros2_world',
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        # Cubo en el marco de la odometría: trasladar y girar -yaw.
        dx = self.cfg['cubo_x'] - self.cfg['robot_x']
        dy = self.cfg['cubo_y'] - self.cfg['robot_y']
        c, s = math.cos(self.cfg['robot_yaw']), math.sin(self.cfg['robot_yaw'])
        self.cubo = Point(x=c * dx + s * dy, y=-s * dx + c * dy)
        self.cargado = False

        self.pub = self.create_publisher(Point, '/nav/cubo_verde', 10)
        self.create_subscription(Odometry, '/sim/odom', self.on_odom, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/nav/carga', self.on_carga, latched)

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        if not self.cargado and \
                math.hypot(pos.x - self.cubo.x, pos.y - self.cubo.y) < self.cfg['alcance']:
            self.pub.publish(self.cubo)

    def on_carga(self, msg):
        if msg.data and not self.cargado:
            self.cargado = True
            pedido = f'name: "{self.cfg["modelo"]}" type: MODEL'
            subprocess.Popen(
                ['gz', 'service', '-s', f'/world/{self.cfg["mundo"]}/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', pedido],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info('Cubo cargado: lo saco del mundo')


def main():
    rclpy.init()
    node = CuboSimulado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
