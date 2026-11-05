"""
Calibración de la lente de la ESP32-CAM con un tablero de ajedrez impreso.

Da la K (fx, fy, cx, cy) y la distorsión D de TU cámara, en el mismo formato que
hardware/camara/calibracion_esp32cam.yaml. Lo usan camara_node (camera_info) y el explorador
(distancias a canastos y flechas) con el parámetro calibracion.

Tablero: imprimí uno de 10 x 7 cuadros (9 x 6 esquinas internas), por ejemplo
https://markhedleyjones.com/projects/calibration-checkerboard-collection, pegalo sobre algo
plano y medí con regla el lado de un cuadro.

Uso (con robot.launch.xml corriendo):
  ros2 run caid_vision calibrar_lente --ros-args -p cuadro:=0.025
  (cuadro = lado de un cuadro en metros; columnas / filas = esquinas internas, 9 x 6 por defecto)

En la ventana:
  espacio   guardar la vista actual (solo si encontró el tablero: esquinas dibujadas en color)
  c         calcular con las vistas guardadas (hacen falta 12 o más)
  d         ver la imagen corregida (sin distorsión) para comparar
  g         guardar lente.yaml
  q / Esc   salir

Tomá 15 a 25 vistas: el tablero cerca y lejos, en el centro y en las 4 esquinas de la imagen,
inclinado hacia los costados y hacia arriba/abajo. Quieto en cada foto (si sale movido, no sirve).
"""
import time

from caid_vision.config_util import guardar_yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

FX_COMPANERO = 947.2    # hardware/camara/calibracion_esp32cam.yaml, para comparar
VISTAS_MIN = 12


class Lente(Node):
    def __init__(self):
        super().__init__('calibrar_lente')
        self.declare_parameter('topic', '/camera/image_raw/compressed')
        self.declare_parameter('columnas', 9)    # esquinas internas a lo ancho
        self.declare_parameter('filas', 6)       # esquinas internas a lo alto
        self.declare_parameter('cuadro', 0.025)  # m, lado de un cuadro
        self.declare_parameter('archivo', 'lente.yaml')
        self.patron = (self.get_parameter('columnas').value, self.get_parameter('filas').value)
        cuadro = self.get_parameter('cuadro').value
        obj = np.zeros((self.patron[0] * self.patron[1], 3), np.float32)
        obj[:, :2] = np.mgrid[0:self.patron[0], 0:self.patron[1]].T.reshape(-1, 2) * cuadro
        self.obj = obj
        self.create_subscription(CompressedImage, self.get_parameter('topic').value, self.cb,
                                 qos_profile_sensor_data)
        self.img = None
        self.vistas = []         # esquinas de cada vista guardada
        self.resultado = None
        self.corregir = False
        self.mensaje = 'Mostrá el tablero y apretá espacio cuando las esquinas salgan en color'

    def cb(self, msg):
        self.img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def buscar(self, gris):
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | \
            cv2.CALIB_CB_FAST_CHECK
        ok, esq = cv2.findChessboardCorners(gris, self.patron, flags)
        if not ok:
            return None
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        return cv2.cornerSubPix(gris, esq, (5, 5), (-1, -1), crit)

    def calcular(self):
        if len(self.vistas) < VISTAS_MIN:
            self.mensaje = f'Faltan vistas: {len(self.vistas)} de {VISTAS_MIN} como mínimo'
            return
        h, w = self.img.shape[:2]
        rms, K, D, _, _ = cv2.calibrateCamera([self.obj] * len(self.vistas), self.vistas,
                                              (w, h), None, None)
        fx, fy = K[0, 0], K[1, 1]
        hfov = np.degrees(2 * np.arctan(w / 2 / fx))
        self.resultado = {'K': K, 'D': D.ravel()[:5], 'rms': rms, 'w': w, 'h': h}
        calidad = ('bien' if rms < 0.5 else 'aceptable' if rms < 1.0
                   else 'MAL: repetí con más vistas')
        self.mensaje = (f'error {rms:.2f} px ({calidad})  fx {fx:.0f} fy {fy:.0f}  '
                        f'FOV {hfov:.0f} grados'
                        f'  (compañero: fx {FX_COMPANERO:.0f})')
        self.get_logger().info(self.mensaje)
        self.get_logger().info(f'cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}  D {np.round(D.ravel(), 4)}')

    def guardar(self):
        if self.resultado is None:
            self.mensaje = 'Primero c para calcular'
            return
        r = self.resultado
        datos = {
            'fecha': time.strftime('%Y-%m-%d %H:%M'),
            'vistas': len(self.vistas),
            'error_px': round(float(r['rms']), 3),
            'width': int(r['w']),
            'height': int(r['h']),
            'K': [float(x) for x in r['K'].ravel()],
            'D': [float(x) for x in r['D']],
        }
        cab = ('# Calibración de la lente de la ESP32-CAM (calibrar_lente, tablero de ajedrez).\n'
               '# Mismo formato que hardware/camara/calibracion_esp32cam.yaml.\n')
        escritas = guardar_yaml(self.get_parameter('archivo').value, datos, cab)
        self.mensaje = 'Guardado en ' + ' y '.join(escritas)
        self.get_logger().info(self.mensaje)

    def dibujar(self):
        img = self.img.copy()
        esq = self.buscar(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        self.esquinas = esq
        if self.corregir and self.resultado is not None:
            img = cv2.undistort(img, self.resultado['K'], self.resultado['D'])
        elif esq is not None:
            cv2.drawChessboardCorners(img, self.patron, esq, True)
        for v in self.vistas:      # dónde ya hay vistas: para cubrir toda la imagen
            x, y, w, h = cv2.boundingRect(v)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)
        lineas = [f'vistas {len(self.vistas)} (min {VISTAS_MIN})  tablero '
                  f'{"VISTO" if esq is not None else "no"}'
                  f'{"  [corregida]" if self.corregir else ""}',
                  'espacio guardar vista  c calcular  d ver corregida  g guardar  q salir',
                  self.mensaje]
        barra = np.zeros((22 * len(lineas) + 6, img.shape[1], 3), np.uint8)
        for i, t in enumerate(lineas):
            cv2.putText(barra, t, (6, 18 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1)
        return np.vstack([img, barra])

    def teclas(self, k):
        if k == ord(' '):
            if self.esquinas is None:
                self.mensaje = 'No veo el tablero entero: acercalo, iluminalo o enderezalo'
            else:
                self.vistas.append(self.esquinas)
                self.mensaje = f'Vista {len(self.vistas)} guardada'
        elif k == ord('c'):
            self.calcular()
        elif k == ord('d'):
            self.corregir = not self.corregir
        elif k == ord('g'):
            self.guardar()


def main():
    rclpy.init()
    n = Lente()
    n.esquinas = None
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
            if n.img is not None:
                cv2.imshow('calibrar lente', n.dibujar())
            k = cv2.waitKey(15) & 0xFF
            if k in (ord('q'), 27):
                break
            if k != 255:
                n.teclas(k)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.try_shutdown()
