"""
Mide la distancia focal fx de la cámara real con un objeto de ancho conocido.

Poné un rectángulo de color (rojo, verde o azul), más ancho que alto, de frente a la cámara y a
una distancia medida con cinta. El detector lo ve como "canasto" (o usá un cubo, tipo:=cubos).
Por semejanza de triángulos:  fx = ancho_en_px * distancia / ancho_real

Uso (con robot.launch.xml corriendo):
  ros2 run caid_vision medir_fx --ros-args -p ancho:=0.10 -p distancia:=0.50
  ros2 run caid_vision medir_fx --ros-args -p ancho:=0.10 -p distancia:=0.50 -p tipo:=cubos

Compara con la calibración de hardware/camara (fx ~ 947) y con el FOV de 60° de la simulación
(fx ~ 554). Medí a 2 o 3 distancias: si todas dan parecido, ese es el fx de tu cámara.
"""
import json
import math
import statistics

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

FX_CALIBRACION = 947.2    # hardware/camara/calibracion_esp32cam.yaml (640x480)
FX_SIMULACION = 554.3     # FOV 60° a 640 px (geometria.CAMARA_HFOV)
MUESTRAS = 30


class MedirFx(Node):
    def __init__(self):
        super().__init__('medir_fx')
        self.declare_parameter('ancho', 0.10)       # m, ancho real del objeto
        self.declare_parameter('distancia', 0.50)   # m, de la lente al frente del objeto
        self.declare_parameter('tipo', 'canastos')  # canastos | cubos
        self.anchos = []
        self.create_subscription(String, '/vision/detecciones', self.cb, 10)
        self.get_logger().info('Esperando el objeto... (centralo en la imagen, de frente)')

    def cb(self, msg):
        objs = json.loads(msg.data).get(self.get_parameter('tipo').value, [])
        if not objs:
            return
        w = max(objs, key=lambda o: o['area'])['caja'][2]   # el más grande
        self.anchos.append(w)
        if len(self.anchos) < MUESTRAS:
            return
        w = statistics.median(self.anchos)
        self.anchos.clear()
        ancho = self.get_parameter('ancho').value
        dist = self.get_parameter('distancia').value
        fx = w * dist / ancho
        hfov = math.degrees(2 * math.atan(320 / fx))
        d_cal = FX_CALIBRACION * ancho / w
        d_sim = FX_SIMULACION * ancho / w
        self.get_logger().info(
            f'ancho {w:.0f} px  ->  fx = {fx:.0f}  (FOV {hfov:.0f}°)\n'
            f'   con la calibración (fx {FX_CALIBRACION:.0f}) el robot calcularía '
            f'{d_cal * 100:.1f} cm'
            f'  ({(d_cal / dist - 1) * 100:+.0f}%)\n'
            f'   con la simulación  (fx {FX_SIMULACION:.0f}) calcularía {d_sim * 100:.1f} cm'
            f'  ({(d_sim / dist - 1) * 100:+.0f}%)\n'
            f'   real: {dist * 100:.1f} cm')


def main():
    rclpy.init()
    node = MedirFx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
