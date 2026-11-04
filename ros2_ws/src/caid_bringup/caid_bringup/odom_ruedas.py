"""
Odometría como la del robot real: contando flancos de encoder, no la que regala Gazebo.

El plugin DiffDrive de Gazebo publica en /sim/odom una odometría casi perfecta: integra las
posiciones exactas de las ruedas con la separación exacta. En el robot real la odometría sale de
los encoders ópticos (20 ranuras x 2 flancos = 40 flancos por vuelta, medido en
docs/guias/caracterizacion-motores.md) y arrastra tres errores que la simulación no tenía:

1. **Cuantización**: un flanco son 2*pi*33,5 mm / 40 = 5,26 mm. Una celda de 400 mm son 76
   flancos y un giro de 90 grados son solo 30 flancos por rueda, así que un flanco de más o de
   menos en cada rueda ya mueve el rumbo 1,5 grados.
2. **Radio de rueda equivocado** (error sistemático de escala): las ruedas se deforman con el
   peso y el radio nominal nunca es el efectivo. Afecta todas las distancias por igual.
3. **Separación efectiva equivocada** (error sistemático de rumbo): es el grande. Este robot es
   un 4x4 sin diferencial, así que al girar las cuatro ruedas arrastran de costado y se comporta
   como si la trocha fuera mayor que los 131 mm geométricos (en gazebo.xacro está calibrada en
   204 mm, y ese valor cambia con el piso, la carga y la velocidad de giro). Si la separación que
   usa la cuenta se desvía un 10 %, cada giro de 90 grados se va 9 grados.

Por eso conviene esto antes que sumarle ruido al tópico de Gazebo: el error de un skid-steer no
es ruido blanco sobre la pose, es un error de escala en el rumbo que se acumula giro a giro, y es
exactamente lo que el navegador tiene que aguantar (no hay IMU: el rumbo se corrige con paredes).

Lee:     joint_states (sensor_msgs/JointState) con los ángulos de las cuatro ruedas
Publica: odom_topic (nav_msgs/Odometry), para que el explorador lo use en vez de /sim/odom

Con los tres errores en 0 queda una odometría exacta salvo la cuantización del encoder.
"""
import math
import random

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OdomRuedas(Node):

    def __init__(self):
        super().__init__('odom_ruedas')
        p = {
            'joint_states_topic': '/sim/joint_states',
            'odom_topic': '/sim/odom_ruedas',
            'ruedas_izq': ['wheel_fl_joint', 'wheel_rl_joint'],
            'ruedas_der': ['wheel_fr_joint', 'wheel_rr_joint'],
            'radio_rueda': 0.0335,        # m, hardware/cad/dimensions.yaml
            'separacion': 0.204,          # m, la efectiva calibrada en gazebo.xacro
            'flancos_por_vuelta': 40,     # 20 ranuras x 2 flancos (caracterizacion-motores.md)
            # Errores de la cuenta respecto de la realidad. 0 = la PC conoce el robot exacto.
            'error_radio': 0.0,           # +0,02 = la PC cree que la rueda es 2 % más grande
            'error_separacion': 0.0,      # +0,10 = cree la trocha 10 % mayor: gira de menos
            'desliz_giro': 0.0,           # desvío relativo al azar del giro de cada muestra
            'semilla': 1,
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        self.flancos = int(self.cfg['flancos_por_vuelta'])
        # Lo que cree la PC, que es lo único con lo que puede hacer la cuenta.
        self.m_por_flanco = (2 * math.pi * self.cfg['radio_rueda'] * (1 + self.cfg['error_radio'])
                             / self.flancos)
        self.separacion = self.cfg['separacion'] * (1 + self.cfg['error_separacion'])
        self.rng = random.Random(int(self.cfg['semilla']))

        self.x = self.y = self.th = 0.0
        self.previo = None               # (flancos izq, flancos der)
        self.pub = self.create_publisher(Odometry, self.cfg['odom_topic'], 10)
        self.create_subscription(JointState, self.cfg['joint_states_topic'], self.on_joints, 10)

    def cuenta(self, msg, nombres):
        """Flancos acumulados de un lado: promedio de sus ruedas, redondeado a flanco entero."""
        angulos = [msg.position[msg.name.index(n)] for n in nombres if n in msg.name]
        if not angulos:
            return None
        vueltas = sum(angulos) / len(angulos) / (2 * math.pi)
        return math.floor(vueltas * self.flancos)

    def on_joints(self, msg):
        izq = self.cuenta(msg, list(self.cfg['ruedas_izq']))
        der = self.cuenta(msg, list(self.cfg['ruedas_der']))
        if izq is None or der is None:
            return
        if self.previo is None:
            self.previo = (izq, der)
            return
        d_izq = (izq - self.previo[0]) * self.m_por_flanco
        d_der = (der - self.previo[1]) * self.m_por_flanco
        self.previo = (izq, der)

        avance = (d_der + d_izq) / 2.0
        giro = (d_der - d_izq) / self.separacion
        if self.cfg['desliz_giro'] > 0.0 and giro != 0.0:
            giro *= 1.0 + self.rng.gauss(0.0, self.cfg['desliz_giro'])
        # Integración por el punto medio del arco.
        self.x += avance * math.cos(self.th + giro / 2.0)
        self.y += avance * math.sin(self.th + giro / 2.0)
        self.th += giro

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = OdomRuedas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
