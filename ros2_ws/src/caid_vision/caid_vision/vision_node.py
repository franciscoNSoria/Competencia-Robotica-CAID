"""Vision CAID 2026: detecta flechas, cubos, canastos y zonas rojas.

La detección está en detector.py (sin ROS, con pruebas). Este nodo:
Lee:
  imagen_topic (sensor_msgs/Image), por defecto /camera/image_raw (/sim/camera/image_raw en Gazebo)
Publica:
  /vision/detecciones (String)  JSON con 'stamp' (s, hora de la imagen) y las detecciones en
                                píxeles (ver detector.py). Lo usa caid_navigation/navegador.
  /vision/cubo/<color> (Point)  x,y = centro en px, z = area           (como antes)
  /vision/flecha       (String) "color,direccion,angulo" confirmada N cuadros (como antes)
  /vision/debug        (Image)  imagen con todo dibujado
"""
import json

from caid_vision.detector import detectar
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

ANGULO_TOL = 45


def clasificar_direccion(ang):
    """Dirección en la imagen más cercana, solo si la inclinación está dentro de ANGULO_TOL."""
    for nombre, centro in (('derecha', 0), ('arriba', 90), ('izquierda', 180), ('abajo', -90)):
        if abs((ang - centro + 180) % 360 - 180) <= ANGULO_TOL:
            return nombre
    return 'diagonal'


class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('imagen_topic', '/camera/image_raw')
        self.declare_parameter('confirmar_frames', 5)
        self.declare_parameter('debug', True)
        self.br = CvBridge()
        self.create_subscription(Image, self.get_parameter('imagen_topic').value, self.cb,
                                 qos_profile_sensor_data)
        self.pub_det = self.create_publisher(String, '/vision/detecciones', 10)
        self.pubs = {c: self.create_publisher(Point, f'/vision/cubo/{c}', 10)
                     for c in ('rojo', 'verde', 'azul')}
        self.pub_flecha = self.create_publisher(String, '/vision/flecha', 10)
        self.debug = self.create_publisher(Image, '/vision/debug', 10)
        self.ultima_flecha = None
        self.cuenta_flecha = 0

    def cb(self, msg):
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        det = detectar(img)
        det['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub_det.publish(String(data=json.dumps(det)))
        for c in det['cubos']:
            x, y, w, h = c['caja']
            self.pubs[c['color']].publish(Point(x=float(x + w / 2), y=float(y + h / 2),
                                                z=float(c['area'])))
        self.publicar_flecha(det['flechas'])
        if self.get_parameter('debug').value:
            self.dibujar(img, det)
            self.debug.publish(self.br.cv2_to_imgmsg(img, 'bgr8'))

    def dibujar(self, img, det):
        blanco = (255, 255, 255)
        for f in det['flechas']:
            cola = tuple(int(v) for v in f['cola'])
            punta = tuple(int(v) for v in f['punta'])
            cv2.arrowedLine(img, cola, punta, blanco, 2, tipLength=0.3)
            cv2.putText(img, f['color'], (cola[0], cola[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        blanco, 1)
        for clave in ('cubos', 'canastos'):
            for c in det[clave]:
                x, y, w, h = c['caja']
                cv2.rectangle(img, (x, y), (x + w, y + h), blanco, 2)
                cv2.putText(img, f"{clave[:-1]} {c['color']}", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, blanco, 1)
        for u, v in det['zonas'][::5]:
            cv2.circle(img, (int(u), int(v)), 1, (0, 255, 255), -1)

    def publicar_flecha(self, flechas):
        """Publica la flecha más grande solo si se vio igual N cuadros seguidos (como antes)."""
        if not flechas:
            self.ultima_flecha, self.cuenta_flecha = None, 0
            return
        f = max(flechas, key=lambda f: f['area'])
        direccion = clasificar_direccion(f['angulo'])
        if direccion == 'diagonal':
            return
        clave = (f['color'], direccion)
        self.cuenta_flecha = self.cuenta_flecha + 1 if clave == self.ultima_flecha else 1
        self.ultima_flecha = clave
        if self.cuenta_flecha >= self.get_parameter('confirmar_frames').value:
            self.pub_flecha.publish(String(data=f"{f['color']},{direccion},{f['angulo']:.1f}"))


def main():
    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
