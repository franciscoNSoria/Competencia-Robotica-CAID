"""
Odometría del robot REAL: integra los flancos que publica firmware/banco/robot_4ruedas/main.c.

Ese firmware no calcula odometría, solo cuenta flancos: publica /wheel_ticks
(std_msgs/Int32MultiArray, [FL, FR, RL, RR] acumulados con signo, 20 Hz). Este nodo hace lo mismo
que caid_bringup/odom_ruedas.py (que es para la simulación, a partir de /sim/joint_states), pero
leyendo flancos reales en vez de simularlos desde un ángulo continuo de Gazebo.

flancos_por_vuelta sale directo del firmware (40). radio_rueda y separacion son los MEDIDOS el
3/11 (hardware-estado-y-calibraciones.md, sección 6): diámetro efectivo 64,5 mm (no el
geométrico de 67, hay desgaste/deformación) y trocha efectiva 291 mm (2,2 veces la geométrica
de 131: es un 4x4 que patina). Mantener estos dos números iguales a los que manda
caid_bringup/calibracion_robot.py por /robot_cmd: si la PC y el firmware usan valores
distintos, lo que el robot hace y lo que esta odometría cree que hizo no coinciden.

OJO con la separación: los 291 mm salen de girar 360°, no de 90° (los giros del laberinto).
El patinaje del arranque pesa más en un arco corto; puede no ser el mismo valor. Mientras no se
midan giros de 90°, esta es la mejor estimación que hay, no un dato firme.

Lee:     ticks_topic (std_msgs/Int32MultiArray), por defecto /wheel_ticks
Publica: odom_topic (nav_msgs/Odometry), por defecto /odom
"""
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class OdomTicks(Node):

    def __init__(self):
        super().__init__('odom_ticks')
        p = {
            'ticks_topic': '/wheel_ticks',
            'odom_topic': '/odom',
            'radio_rueda': 0.03225,       # m, diámetro efectivo medido (64,5 mm) / 2
            'flancos_por_vuelta': 40,     # firmware: 20 ranuras x 2 flancos
            # m, separación efectiva medida (giros de 360°, no 90°: ver el docstring).
            'separacion': 0.291,
            'orden_ruedas': ['FL', 'FR', 'RL', 'RR'],   # como las publica el firmware
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        flancos = int(self.cfg['flancos_por_vuelta'])
        self.m_por_flanco = 2 * math.pi * self.cfg['radio_rueda'] / flancos
        self.separacion = self.cfg['separacion']
        self.idx = {n: i for i, n in enumerate(self.cfg['orden_ruedas'])}

        self.x = self.y = self.th = 0.0
        self.previo = None               # (flancos izq promedio, flancos der promedio)
        self.pub = self.create_publisher(Odometry, self.cfg['odom_topic'], 10)
        self.create_subscription(Int32MultiArray, self.cfg['ticks_topic'], self.on_ticks, 10)
        self.get_logger().info(
            f'Escucho {self.cfg["ticks_topic"]}, publico {self.cfg["odom_topic"]} '
            f'({self.m_por_flanco * 1000:.3f} mm/flanco, separación {self.separacion * 1000:.0f} mm'
            ' -- VERIFICAR, no está calibrada)')

    def on_ticks(self, msg):
        if len(msg.data) < 4:
            return
        izq = (msg.data[self.idx['FL']] + msg.data[self.idx['RL']]) / 2.0
        der = (msg.data[self.idx['FR']] + msg.data[self.idx['RR']]) / 2.0
        if self.previo is None:
            self.previo = (izq, der)
            return
        d_izq = (izq - self.previo[0]) * self.m_por_flanco
        d_der = (der - self.previo[1]) * self.m_por_flanco
        self.previo = (izq, der)

        avance = (d_der + d_izq) / 2.0
        giro = (d_der - d_izq) / self.separacion
        self.x += avance * math.cos(self.th + giro / 2.0)
        self.y += avance * math.sin(self.th + giro / 2.0)
        self.th += giro

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = OdomTicks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
