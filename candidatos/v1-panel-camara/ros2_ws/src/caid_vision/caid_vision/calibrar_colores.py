"""
Calibración de colores con la cámara real: cubos, flechas y canastos rojo / verde / azul.

Uso (con robot.launch.xml corriendo, que publica /camera/image_raw/compressed):
  ros2 run caid_vision calibrar_colores
  ros2 run caid_vision calibrar_colores --ros-args -p camara:=192.168.18.66

En la ventana:
  mouse         arrastrar un recuadro ADENTRO del objeto (sin fondo)
  r / v / a     sumar ese recuadro como muestra de rojo / verde / azul (y recalcular su rango)
  1 / 2 / 3     ver la máscara de rojo / verde / azul (lo que el detector toma como ese color)
  0             ver las detecciones (cubos, flechas, canastos) con los rangos nuevos
  b             borrar las muestras del color que se está mirando (1/2/3) y volverlo de fábrica
  f             fijar la exposición y la ganancia de la cámara en los valores de ahora
  u             volver la exposición y la ganancia a automático
  [ / ]         exposición más corta / más larga (menos / más desenfoque al moverse); la
                ganancia queda en automático para compensar el brillo
  p             congelar / descongelar la imagen (para marcar un objeto con el robot andando)
  g             guardar colores.yaml (vision_node lo toma solo, sin reiniciar)
  q / Esc       salir (pregunta si hay cambios sin guardar)

Conviene tomar varias muestras de cada color: el cubo y la flecha de ese color, de frente y de
costado, cerca y lejos, con sombra y con luz, y con el robot andando (p congela la imagen para
poder marcar el objeto movido). El rango sale de todas juntas.
"""
import json
import time
import urllib.request

from caid_vision import detector
from caid_vision.config_util import (cargar_yaml, guardar_yaml, rangos_a_yaml,
                                     rangos_desde_yaml, ruta)
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

COLORES = ('rojo', 'verde', 'azul')
TECLA_MUESTRA = {ord('r'): 'rojo', ord('v'): 'verde', ord('a'): 'azul'}
TECLA_VER = {ord('1'): 'rojo', ord('2'): 'verde', ord('3'): 'azul', ord('0'): None}
BGR = {'rojo': (0, 0, 255), 'verde': (0, 200, 0), 'azul': (255, 80, 0)}

MARGEN_H = 4         # tono: se agranda el rango medido este tanto de cada lado
FACTOR_S = 0.85      # saturación mínima = percentil 5 medido x este factor
FACTOR_V = 0.80      # brillo mínimo = percentil 5 medido x este factor
# Nunca menos que esto: con S < 60 entran piel, madera y sombras grises (falsos). La cara en
# sombra de un cubo azul real da S ~70.
S_PISO, V_PISO = 60, 25
DH_LIMPIEZA = 18     # de cada recuadro se quedan los píxeles a +-18 del tono dominante
ANCHO_H_AVISO = 30   # un color de verdad ocupa menos que esto: si da más, la muestra tenía fondo

# Rangos de partida (los mismos de config/colores.yaml de fábrica). 'b' vuelve un color a esto.
RANGOS_BASE = {
    'rojo': [((0, 110, 55), (8, 255, 255)), ((172, 110, 55), (180, 255, 255))],
    'verde': [((40, 90, 55), (85, 255, 255))],
    'azul': [((88, 110, 40), (130, 255, 255))],
}


def limpiar(px):
    """Del recuadro, solo los píxeles del color dominante: saca fondo, bordes y sombras grises."""
    px = px[(px[:, 1] >= 40) & (px[:, 2] >= 20)]
    if len(px) < 30:
        return px[:0]
    # tono dominante con media circular (el rojo cruza el 0)
    ang = px[:, 0].astype(float) * (2 * np.pi / 180)
    centro = np.angle(np.mean(np.exp(1j * ang))) * 180 / (2 * np.pi) % 180
    dh = np.abs((px[:, 0].astype(float) - centro + 90) % 180 - 90)
    px = px[dh <= DH_LIMPIEZA]
    if len(px) < 30:
        return px[:0]
    return px[px[:, 1] >= 0.6 * np.median(px[:, 1])]


def rango_h(h):
    """Tramos de tono que cubren el 96% central de h. El rojo cruza el 0: puede dar dos tramos."""
    mejor = None
    for corrimiento in (0, 90):
        hs = (h + corrimiento) % 180
        lo, hi = np.percentile(hs, [2, 98])
        if mejor is None or hi - lo < mejor[1] - mejor[0]:
            mejor = (lo, hi, corrimiento)
    lo, hi, corrimiento = mejor
    lo, hi = lo - MARGEN_H - corrimiento, hi + MARGEN_H - corrimiento
    if hi <= 0:              # todo del lado alto (ej. 170-178): volver a 0-180
        lo, hi = lo + 180, hi + 180
    elif lo >= 180:
        lo, hi = lo - 180, hi - 180
    if lo < 0:
        return [(0, hi), (180 + lo, 180)]
    if hi > 180:
        return [(lo, 180), (0, hi - 180)]
    return [(lo, hi)]


def calcular_rango(px):
    """px: N x 3 HSV de las muestras de un color -> tramos ((h,s,v), (h,s,v)) del detector."""
    if len(px) < 30:
        return None
    s_min = max(S_PISO, int(np.percentile(px[:, 1], 5) * FACTOR_S))
    v_min = max(V_PISO, int(np.percentile(px[:, 2], 5) * FACTOR_V))
    tramos = []
    for lo, hi in rango_h(px[:, 0].astype(float)):
        tramos.append(((int(max(0, lo)), s_min, v_min), (int(min(180, round(hi))), 255, 255)))
    return tramos


def dentro(px, tramos):
    m = np.zeros(len(px), bool)
    for a, b in tramos:
        m |= np.all((px >= a) & (px <= b), axis=1)
    return m


class Calibrador(Node):
    def __init__(self):
        super().__init__('calibrar_colores')
        self.declare_parameter('topic', '/camera/image_raw/compressed')
        self.declare_parameter('camara', '192.168.18.66')   # IP de la ESP32-CAM (para fijar)
        self.declare_parameter('archivo', 'colores.yaml')
        self.archivo = self.get_parameter('archivo').value
        self.create_subscription(CompressedImage, self.get_parameter('topic').value, self.cb,
                                 qos_profile_sensor_data)
        previo = cargar_yaml(ruta(self.archivo)) or {}
        self.rangos = (rangos_desde_yaml(previo['rangos']) if previo.get('rangos')
                       else dict(detector.RANGOS))
        self.camara_cfg = dict(previo.get('camara') or {})
        detector.RANGOS = self.rangos
        detector.V_MIN = detector.V_MIN_AZUL = detector.S_MIN = 0
        self.muestras = {c: np.empty((0, 3), np.uint8) for c in COLORES}
        self.img = None
        self.congelada = False
        self.roi = None          # (x0, y0, x1, y1)
        self.arrastre = None
        self.ver = None          # None = detecciones, o un color = su máscara
        self.cambios = False
        self.mensaje = 'Arrastrá un recuadro adentro de un objeto y apretá r / v / a'
        self.get_logger().info(f'Rangos iniciales de {ruta(self.archivo)}')

    def cb(self, msg):
        if not self.congelada:
            self.img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    # ---------- acciones ----------
    def muestrear(self, color):
        if self.img is None or self.roi is None:
            self.mensaje = 'Primero arrastrá un recuadro con el mouse'
            return
        x0, y0, x1, y1 = self.roi
        hsv = limpiar(cv2.cvtColor(self.img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV).reshape(-1, 3))
        if calcular_rango(hsv) is None:     # no se suma: ensuciaría el rango
            self.mensaje = f'{color}: el recuadro casi no tiene color (¿es gris o está oscuro?)'
            return
        self.muestras[color] = np.vstack([self.muestras[color], hsv])
        tramos = calcular_rango(self.muestras[color])
        self.rangos[color] = tramos
        detector.RANGOS = self.rangos
        self.cambios = True
        self.ver = color
        txt = ' + '.join(f'H {a[0]}-{b[0]}' for a, b in tramos)
        s_min, v_min = tramos[0][0][1], tramos[0][0][2]
        self.mensaje = f'{color}: {len(self.muestras[color])} px -> {txt}, S>={s_min}, V>={v_min}'
        ancho = sum(b[0] - a[0] for a, b in tramos)
        if ancho > ANCHO_H_AVISO:
            self.mensaje = (f'OJO {color}: el tono va de un extremo a otro ({ancho}). Habia fondo '
                            'en algun recuadro: b y volver a tomar')
        self.get_logger().info(self.mensaje)
        self.revisar_cruces()

    def revisar_cruces(self):
        """Avisa si las muestras de un color caen en el rango de otro (se confundirían)."""
        for c in COLORES:
            if len(self.muestras[c]) == 0:
                continue
            for otro in COLORES:
                if otro == c:
                    continue
                frac = dentro(self.muestras[c], self.rangos[otro]).mean()
                if frac > 0.05:
                    self.get_logger().warn(f'El {frac:.0%} de las muestras de {c} cae en el '
                                           f'rango de {otro}: se van a confundir')

    def camara(self, var, val):
        url = f'http://{self.get_parameter("camara").value}/control?var={var}&val={val}'
        urllib.request.urlopen(url, timeout=2).read()

    def fijar_camara(self):
        try:
            url = f'http://{self.get_parameter("camara").value}/status'
            st = json.loads(urllib.request.urlopen(url, timeout=2).read())
            cfg = {'aec': 0, 'aec_value': int(st['aec_value']), 'agc': 0, 'gain': int(st['gain'])}
            for var, val in cfg.items():
                self.camara(var, val)
        except (OSError, ValueError, KeyError) as e:
            self.mensaje = f'No pude hablar con la cámara: {e}'
            return
        self.camara_cfg.update(cfg)
        self.cambios = True
        self.mensaje = f'Cámara fija: exposición {cfg["aec_value"]}, ganancia {cfg["gain"]}. ' \
                       'Volvé a tomar las muestras.'
        self.get_logger().info(self.mensaje)

    def exposicion(self, factor):
        """
        Exposición fija más corta (factor < 1) o más larga; la ganancia sigue en automático.

        Con exposición corta cada cuadro se toma más rápido y el objeto sale menos movido cuando
        el robot anda. A cambio la imagen es más oscura (la ganancia sube y mete ruido): hace
        falta buena luz.
        """
        try:
            actual = self.camara_cfg.get('aec_value')
            if actual is None:
                url = f'http://{self.get_parameter("camara").value}/status'
                actual = json.loads(urllib.request.urlopen(url, timeout=2).read())['aec_value']
            nuevo = int(min(1200, max(10, round(actual * factor))))
            self.camara('aec', 0)
            self.camara('aec_value', nuevo)
            self.camara('agc', 1)
        except (OSError, ValueError, KeyError) as e:
            self.mensaje = f'No pude hablar con la cámara: {e}'
            return
        for k in ('gain',):
            self.camara_cfg.pop(k, None)
        self.camara_cfg.update({'aec': 0, 'aec_value': nuevo, 'agc': 1})
        self.cambios = True
        self.mensaje = f'Exposición {nuevo} (más chico = menos desenfoque), ganancia automática'
        self.get_logger().info(self.mensaje)

    def camara_auto(self):
        try:
            self.camara('aec', 1)
            self.camara('agc', 1)
        except OSError as e:
            self.mensaje = f'No pude hablar con la cámara: {e}'
            return
        for k in ('aec', 'aec_value', 'agc', 'gain'):
            self.camara_cfg.pop(k, None)
        self.cambios = True
        self.mensaje = 'Exposición y ganancia en automático'

    def guardar(self):
        datos = {
            'fecha': time.strftime('%Y-%m-%d %H:%M'),
            'muestras_px': {c: int(len(self.muestras[c])) for c in COLORES},
            'rangos': rangos_a_yaml(self.rangos),
            'camara': self.camara_cfg,
        }
        cab = ('# Rangos de color del detector (HSV de OpenCV: H 0-180, S y V 0-255).\n'
               '# Escrito por `ros2 run caid_vision calibrar_colores`.\n'
               '# vision_node lo recarga solo cuando cambia.\n'
               '# camara: ajustes que camara_node le manda a la ESP32-CAM al conectarse.\n')
        escritas = guardar_yaml(self.archivo, datos, cab)
        self.cambios = False
        self.mensaje = 'Guardado en ' + ' y '.join(escritas)
        self.get_logger().info(self.mensaje)

    # ---------- ventana ----------
    def mouse(self, ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            self.arrastre = (x, y)
            self.roi = None
        elif ev == cv2.EVENT_MOUSEMOVE and self.arrastre:
            self.roi = self.normalizar(self.arrastre, (x, y))
        elif ev == cv2.EVENT_LBUTTONUP and self.arrastre:
            self.roi = self.normalizar(self.arrastre, (x, y))
            self.arrastre = None
            if self.roi[2] - self.roi[0] < 3 or self.roi[3] - self.roi[1] < 3:
                self.roi = None

    def normalizar(self, a, b):
        h, w = self.img.shape[:2]
        x0, x1 = sorted((max(0, min(w, a[0])), max(0, min(w, b[0]))))
        y0, y1 = sorted((max(0, min(h, a[1])), max(0, min(h, b[1]))))
        return x0, y0, x1, y1

    def dibujar(self):
        img = self.img.copy()
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        if self.ver is None:
            det = detector.detectar(self.img.copy())
            for f in det['flechas']:
                cola = tuple(int(v) for v in f['cola'])
                punta = tuple(int(v) for v in f['punta'])
                cv2.arrowedLine(img, cola, punta, BGR[f['color']], 3, tipLength=0.3)
                cv2.putText(img, f'flecha {f["color"]}', (cola[0], cola[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            for clave in ('cubos', 'canastos'):
                for c in det[clave]:
                    x, y, w, h = c['caja']
                    cv2.rectangle(img, (x, y), (x + w, y + h), BGR[c['color']], 2)
                    cv2.putText(img, f'{clave[:-1]} {c["color"]}', (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            vista = 'detecciones'
        else:
            m = detector.mascara(hsv, self.ver)
            gris = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR) // 3
            img = np.where(m[..., None] > 0, img, gris)
            vista = f'mascara {self.ver}'
        if self.roi:
            x0, y0, x1, y1 = self.roi
            cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), 1)
        n = ' '.join(f'{c[0]}:{len(self.muestras[c])}' for c in COLORES)
        fija = (f'exp {self.camara_cfg["aec_value"]}' if self.camara_cfg.get('aec') == 0
                else 'auto')
        if self.congelada:
            fija += '  [CONGELADA]'
        aviso = '  *sin guardar*' if self.cambios else ''
        lineas = [f'[{vista}]  muestras {n}  camara {fija}{aviso}',
                  'r/v/a muestra  1/2/3 mascara  0 detec.  b borrar  f fijar  u auto  '
                  '[ ] exposicion  p congelar  g guardar',
                  self.mensaje]
        barra = np.zeros((22 * len(lineas) + 6, img.shape[1], 3), np.uint8)
        for i, t in enumerate(lineas):
            cv2.putText(barra, t, (6, 18 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1)
        return np.vstack([img, barra])

    def teclas(self, k):
        if k in TECLA_MUESTRA:
            self.muestrear(TECLA_MUESTRA[k])
        elif k in TECLA_VER:
            self.ver = TECLA_VER[k]
        elif k == ord('b') and self.ver:
            self.muestras[self.ver] = np.empty((0, 3), np.uint8)
            self.rangos[self.ver] = list(RANGOS_BASE[self.ver])
            detector.RANGOS = self.rangos
            self.cambios = True
            self.mensaje = f'{self.ver}: muestras borradas, rango de fabrica. Toma muestras nuevas'
        elif k == ord('f'):
            self.fijar_camara()
        elif k == ord('['):
            self.exposicion(0.8)
        elif k == ord(']'):
            self.exposicion(1.25)
        elif k == ord('p'):
            self.congelada = not self.congelada
            self.mensaje = ('Imagen congelada: marcá el objeto y r/v/a' if self.congelada
                            else 'Imagen en vivo')
        elif k == ord('u'):
            self.camara_auto()
        elif k == ord('g'):
            self.guardar()


def esperando(topic):
    """Cartel mientras no llega ninguna imagen (casi siempre: el launch de la cámara no corre)."""
    img = np.zeros((200, 640, 3), np.uint8)
    for i, t in enumerate(('Esperando imagen de ' + topic,
                           '',
                           'Esta corriendo el launch de la camara en otra terminal?',
                           'ros2 launch caid_bringup robot.launch.xml \\',
                           '    fuente:=http://192.168.18.66:81/stream')):
        cv2.putText(img, t, (12, 32 + 32 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def main():
    rclpy.init()
    n = Calibrador()
    ventana = 'calibrar colores'
    cv2.namedWindow(ventana)
    cv2.setMouseCallback(ventana, n.mouse)
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
            if n.img is not None:
                cv2.imshow(ventana, n.dibujar())
            else:
                cv2.imshow(ventana, esperando(n.get_parameter('topic').value))
            k = cv2.waitKey(15) & 0xFF
            if k in (ord('q'), 27):
                if n.cambios:
                    n.mensaje = 'Hay cambios sin guardar: g para guardar, q otra vez para salir'
                    n.cambios = False
                    continue
                break
            if k != 255:
                n.teclas(k)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.try_shutdown()
