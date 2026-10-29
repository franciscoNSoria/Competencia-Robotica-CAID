"""
Brazo simulado para probar la misión en Gazebo mientras el brazo real no está integrado (MAN/VIS).

Lee /mision/pedido (JSON {"accion", "color", "x", "y", "z"}, posición del objeto en base_link) y
responde en /mision/resultado (Bool) después de `demora` segundos:
- agarrar: si el objeto está al alcance (`alcance`, medido desde el eje del brazo), saca el modelo
  cubo_<color> del mundo de Gazebo (gz service .../remove) y responde True; si no, False.
- soltar: responde True si el canasto está al alcance (el cubo no se vuelve a crear).
"""
import json
import math
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

BRAZO_X = -0.0213     # eje del brazo en base_link (hardware/cad/dimensions.yaml)


class ManipulacionSimulada(Node):

    def __init__(self):
        super().__init__('manipulacion_simulada')
        self.declare_parameter('mundo', 'ros2_world')
        self.declare_parameter('alcance', 0.22)
        self.declare_parameter('demora', 2.0)
        self.pub = self.create_publisher(Bool, '/mision/resultado', 10)
        self.create_subscription(String, '/mision/pedido', self.on_pedido, 10)
        self.pendiente = None

    def on_pedido(self, msg):
        p = json.loads(msg.data)
        dist = math.hypot(p['x'] - BRAZO_X, p['y'])
        ok = dist <= self.get_parameter('alcance').value
        self.get_logger().info(f"{p['accion']} {p['color']}: a {dist:.3f} m del brazo -> "
                               f"{'ok' if ok else 'fuera de alcance'}")
        if ok and p['accion'] == 'agarrar':
            pedido = f'name: "cubo_{p["color"]}" type: MODEL'
            subprocess.Popen(
                ['gz', 'service', '-s', f'/world/{self.get_parameter("mundo").value}/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', pedido],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if self.pendiente is not None:
            self.pendiente.cancel()
        self.pendiente = self.create_timer(self.get_parameter('demora').value,
                                           lambda: self.responder(ok))

    def responder(self, ok):
        self.pendiente.cancel()
        self.pendiente = None
        self.pub.publish(Bool(data=ok))


def main():
    rclpy.init()
    node = ManipulacionSimulada()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
