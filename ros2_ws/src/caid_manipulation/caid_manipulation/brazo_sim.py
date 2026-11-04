"""
Brazo simulado: en Gazebo hace lo mismo que la ESP32 del brazo real.

Escucha /brazo/cmd y publica /brazo/estado con el formato de firmware/brazo_sliders, así la
ventana de sliders (tools/brazo/brazo_sliders.py) y la carga (carga_brazo) mueven el brazo
simulado sin cambiar nada, igual que al real. Los grados nominales de cada servo pasan a ángulos
de las articulaciones del URDF con la calibración de cinematica.py y se mandan a los
controladores de posición de Gazebo (/sim/brazo/<articulación>/cmd).

Lee:     /brazo/cmd (std_msgs/Float32MultiArray, best effort)
Publica: /brazo/estado (std_msgs/Float32MultiArray, best effort, 10 Hz) y
         /sim/brazo/<articulación>/cmd (std_msgs/Float64, rad, 50 Hz)
"""
from caid_manipulation import cinematica
from caid_manipulation.servos import ServosBrazo
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64

ARTICULACIONES = ('arm_yaw_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint',
                  'arm_claw_a_joint', 'arm_claw_b_joint')
PERIODO = 0.02


class BrazoSim(Node):

    def __init__(self):
        super().__init__('brazo_sim')
        self.servos = ServosBrazo()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Float32MultiArray, '/brazo/cmd', self.on_cmd, qos)
        self.pub_estado = self.create_publisher(Float32MultiArray, '/brazo/estado', qos)
        self.pubs = {a: self.create_publisher(Float64, f'/sim/brazo/{a}/cmd', 10)
                     for a in ARTICULACIONES}
        self.create_timer(PERIODO, self.tick)
        self.create_timer(0.1, self.publicar_estado)
        self.get_logger().info('Brazo simulado listo: escucho /brazo/cmd como la ESP32')

    def on_cmd(self, msg):
        if not self.servos.orden(list(msg.data)):
            self.get_logger().warn('Orden descartada (incompleta o con NaN)')

    def tick(self):
        self.servos.paso(PERIODO)
        base, hombro, codo, pinza = self.servos.actual
        for nombre, valor in cinematica.servos_a_articulaciones(base, hombro, codo,
                                                                pinza).items():
            self.pubs[nombre].publish(Float64(data=float(valor)))

    def publicar_estado(self):
        self.pub_estado.publish(Float32MultiArray(data=self.servos.estado()))


def main():
    rclpy.init()
    node = BrazoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
