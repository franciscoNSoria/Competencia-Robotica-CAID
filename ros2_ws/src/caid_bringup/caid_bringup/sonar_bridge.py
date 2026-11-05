"""
Separa los 3 sonares de /wheel_state (firmware/banco/robot_4ruedas/main.c) en 3 Range.

El firmware real no publica un tópico por sonar: los manda empaquetados en /wheel_state
(Float32MultiArray, 34 valores, 10 Hz), en los índices [26] frontal, [27] izquierdo, [28]
derecho (m; -1 = sin eco). Este nodo los separa en 3 sensor_msgs/Range para que explorador.py
los lea exactamente igual que en la simulación (ahí cada sonar es su propio tópico).

Sin eco (-1) no es "pared pegada": es "nada dentro del rango del sonar" (4 m por software,
y menos en la práctica). Se publica como max_range (no como -1 ni como 0), así
explorador.distancia() (mediana de las últimas 3 lecturas) lo toma como paso libre, igual que
math.inf en la simulación.
"""
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

import rclpy
from rclpy.node import Node

IDX = {'frente': 26, 'izquierda': 27, 'derecha': 28}


class SonarBridge(Node):

    def __init__(self):
        super().__init__('sonar_bridge')
        p = {
            'wheel_state_topic': '/wheel_state',
            'frente_topic': '/distancia/frente',
            'izquierda_topic': '/distancia/izquierda',
            'derecha_topic': '/distancia/derecha',
            'max_range': 4.0,       # m, igual que sonar.h (rango valido 2 cm - 4 m)
            'min_range': 0.02,
            'fov': 0.2618,          # rad, 15 grados (HC-SR04 nominal)
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        self.pubs = {lado: self.create_publisher(Range, self.cfg[lado + '_topic'], 10)
                     for lado in IDX}
        self.create_subscription(Float32MultiArray, self.cfg['wheel_state_topic'],
                                 self.on_estado, 10)

    def on_estado(self, msg):
        if len(msg.data) < 29:
            return
        ahora = self.get_clock().now().to_msg()
        for lado, idx in IDX.items():
            d = msg.data[idx]
            r = Range()
            r.header.stamp = ahora
            r.header.frame_id = f'us_{lado}' if lado != 'frente' else 'us_front'
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = self.cfg['fov']
            r.min_range = self.cfg['min_range']
            r.max_range = self.cfg['max_range']
            r.range = self.cfg['max_range'] if d < 0.0 else d
            self.pubs[lado].publish(r)


def main():
    rclpy.init()
    node = SonarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
