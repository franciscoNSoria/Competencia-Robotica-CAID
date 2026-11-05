#!/usr/bin/env python3
"""Robot en el laberinto, como el día del torneo: explora con los sonares, sigue las flechas verdes con la cámara, frena frente al
cubo verde, espera que lo agarres con los sliders (tecla y) y vuelve a la partida. La navegación está en torneo_logica.py.

    python3 tools/torneo/torneo.py                 # g arranca, y = cubo agarrado, ESPACIO frena, q sale
    python3 tools/torneo/torneo.py --sin-camara    # solo sonares (explora hasta el fondo)

Habla directo con la ESP32 (firmware/banco/robot_4ruedas): lee /wheel_state y /wheel_ticks, manda /cmd_vel y /robot_cmd. No hace
falta caid_bringup (ni calibracion_robot, ni odom_ticks, ni sonar_bridge): este programa manda la calibración y la guardia solo.
Para la cámara: camara_node + vision_node (ver README). Si no llegan detecciones, navega igual, sin flechas ni cubo.

Requisitos: ROS 2 Jazzy cargado, el agente micro-ROS corriendo y la ESP32 conectada. CERRAR cualquier otro programa que mande
/cmd_vel (robot_autonomo, el panel de ruedas, explorar_sin_camara): dos programas mandando velocidades se pisan.
Guarda un CSV de cada corrida en ~/caid/config/torneo_AAAAMMDD_HHMMSS.csv (para revisar después qué pasó)."""
import argparse
import csv
import datetime
import json
import math
import os
import select
import sys
import termios
import time
import tty

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import torneo_logica as tl  # noqa: E402
from caid_navigation import geometria  # noqa: E402
from caid_navigation.camara import Camara, ubicar_flecha  # noqa: E402

CARPETA = os.path.expanduser('~/caid/config')
CALIB_DEFECTO = dict(diametro_m=0.0645, trocha_m=0.291, frenado_m=0.0223, frenado_giro_m=0.0015)
N_ESTADO = 34
CUBO_LADO = 0.020
CAM_X = geometria.CAMARA_EJE_X + geometria.CAMARA_ADELANTE
AYUDA = "  g arrancar    y cubo agarrado    ESPACIO detener    q salir    ? ayuda"


def cargar_calibracion():
    """La de ~/caid/config/calibracion.json (la que guarda robot_teclado.py), o la medida el 3/11."""
    c = dict(CALIB_DEFECTO)
    try:
        with open(os.path.join(CARPETA, 'calibracion.json')) as f:
            c.update({k: float(v) for k, v in json.load(f).items() if k in c})
        return c, True
    except (OSError, ValueError):
        return c, False


class VistaDeParedes:
    """Lo que camara.ubicar_flecha necesita saber del mapa (igual que en caid_navigation/explorador.py)."""
    bloques = frozenset()
    bloqueos = frozenset()

    def __init__(self, mapa):
        self.mapa = mapa

    def abierto(self, celda, d):
        return self.mapa.abierto(celda, d)

    @property
    def libres(self):
        return self.mapa.visitadas | {tl.vecina(c, d) for (c, d), paso in self.mapa.lados.items() if paso}


class Torneo:
    """Todo menos ROS y la terminal."""

    def __init__(self, enlace, calib, params, usar_camara=True, hfov=math.radians(60), imprimir=print, carpeta=CARPETA):
        geometria.fijar_celda(tl.CELDA)
        self.enlace = enlace
        self.calib = calib
        self.imprimir = imprimir
        self.mpf = math.pi * calib['diametro_m'] / 40.0          # metros por flanco
        self.piloto = tl.Piloto(params, trocha=calib['trocha_m'], diametro=calib['diametro_m'], imprimir=imprimir)
        self.usar_camara = usar_camara
        self.hfov = hfov
        self.cam = None
        self.vista = VistaDeParedes(self.piloto.mapa)
        self.vistas_flecha = {}
        self._t_cfg = self._t_calib = -1e9
        self._g_ult = None
        self._t_y = None
        self.v = self.w = 0.0
        os.makedirs(carpeta, exist_ok=True)
        self.ruta_csv = os.path.join(carpeta, 'torneo_%s.csv' % datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        self._f = open(self.ruta_csv, 'w', newline='')
        self.csv = csv.writer(self._f)
        self.csv.writerow(['t', 'estado', 'modo', 'celda', 'rumbo', 'v', 'w', 'frente', 'izq', 'der', 'guardia', 'y_lat', 'psi_deg',
                           's', 'motivo'])
        self.t0 = time.time()

    # ------------------------------------------------------------------ telemetría
    def tele(self, ahora):
        e = self.enlace.estado
        if e is None or len(e) < N_ESTADO:
            return None
        nn = lambda m: None if m < 0 else float(m)
        return dict(frente=nn(e[26]), izq=nn(e[27]), der=nn(e[28]), guardia=int(round(e[29])), mov_activo=e[16] > 0.5,
                    mov_seq=int(round(e[19])), quieto=all(abs(v) < 3.0 for v in e[0:4]),
                    fresco=ahora - self.enlace.t_estado < 0.5, diam_mm=float(e[20]), trocha_mm=float(e[23]))

    def odometria(self):
        t = self.enlace.ticks
        if t is None or len(t) < 4:
            return None
        izq = (t[0] + t[2]) / 2.0 * self.mpf
        der = (t[1] + t[3]) / 2.0 * self.mpf
        return (izq + der) / 2.0, (der - izq) / self.calib['trocha_m']

    def calib_aplicada(self, tele):
        c = self.calib
        return abs(tele['diam_mm'] - 1000 * c['diametro_m']) < 0.05 and abs(tele['trocha_mm'] - 1000 * c['trocha_m']) < 0.5

    # ------------------------------------------------------------------ teclas
    def tecla(self, k, ahora):
        p = self.piloto
        if k == 'g':
            tele = self.tele(ahora)
            if tele is None or not tele['fresco']:
                self.imprimir('  No arranco: no llega /wheel_state de la ESP32 (¿agente? ¿firmware con sonares?).')
            elif not self.calib_aplicada(tele):
                self.imprimir('  No arranco: la ESP32 todavía no tiene la calibración (se manda sola; probá en 1 s).')
            elif tele['guardia'] == 0:
                self.imprimir('  No arranco: la guardia de la ESP32 todavía está apagada (se activa sola; probá en 1 s).')
            elif self.odometria() is None:
                self.imprimir('  No arranco: no llega /wheel_ticks.')
            elif p.estado in (tl.ESPERA, tl.DETENIDO, tl.FIN):
                p.iniciar(ahora)
                self.imprimir('  ARRANCA. ESPACIO frena.')
        elif k == 'y':
            if p.estado == tl.AGARRE:
                p.agarrado = True
                self.imprimir('  Cubo agarrado: vuelvo.')
            else:
                self.imprimir('  (la y vale cuando está frenado frente al cubo)')
        elif k in (' ', 's', '\x1b'):
            p.detener(ahora)
            self.parar()
        elif k == '?':
            self.imprimir(AYUDA)

    # ------------------------------------------------------------------ cámara
    def detecciones(self, ahora):
        d = self.enlace.det
        if not self.usar_camara or d is None or d[1] <= getattr(self, '_t_det', 0.0):
            return
        self._t_det = d[1]
        if ahora - d[1] > 0.5:
            return
        p = self.piloto
        if p.estado not in (tl.CORRER, tl.MEDIR, tl.VERIFICAR, tl.PARANDO, tl.ACERCAR):
            return                                 # girando, la pose de la foto no es confiable
        if self.cam is None:
            if self.enlace.tam_img is None:
                return
            self.cam = Camara(ancho=self.enlace.tam_img[0], alto=self.enlace.tam_img[1], hfov=self.hfov)
        det = d[0]
        pose = p.pose()
        tele = self.tele(ahora)
        for c in det.get('cubos', []):
            if c['color'] != p.p.color:
                continue
            x, y, w, h = c['caja']
            u = x + w / 2
            if abs(u - self.cam.cx) > 0.25 * self.cam.ancho or h < 4:
                continue
            dist_cam = self.cam.fy * CUBO_LADO / h
            p.ver_cubo(ahora, dist_cam - (tl.F_X - CAM_X) - CUBO_LADO / 2)
        if p.modo != 'EXPLORAR':
            return
        for f in det.get('flechas', []):
            if f['color'] != p.p.color:
                continue
            r = ubicar_flecha(self.vista, self.cam, pose, 0.0, f, tolerancia=0.35)
            if r is not None:
                clave = (r['celda'], r['dir'], r['largo'])
            else:
                clave = self.flecha_de_frente(f, tele, pose)
            if clave is None:
                continue
            self.vistas_flecha[clave] = self.vistas_flecha.get(clave, 0) + 1
            if self.vistas_flecha[clave] >= 3:
                p.flecha(clave[0], clave[1], clave[2], f['color'])

    def flecha_de_frente(self, f, tele, pose):
        """Si la geometría de la cámara no cierra (intrínsecos sin calibrar), una flecha casi horizontal en el medio de la imagen,
        con pared adelante a menos de 1,2 m: está en esa pared y manda doblar a la izquierda o a la derecha."""
        p = self.piloto
        if tele is None or tele['frente'] is None or tele['frente'] > 1.2 or p.estado not in (tl.CORRER, tl.MEDIR):
            return None
        (pu, pv), (cu, cv) = f['punta'], f['cola']
        dx, dy = pu - cu, pv - cv
        if abs(dy) > 0.58 * abs(dx) or abs((pu + cu) / 2 - self.cam.cx) > 0.25 * self.cam.ancho:
            return None
        cuello = f['cuello']
        punta = math.hypot(pu - cuello[0], pv - cuello[1])
        cuerpo = math.hypot(cuello[0] - cu, cuello[1] - cv)
        largo = 1 if cuerpo < 2.6 * punta else (2 if cuerpo < 4.8 * punta else 3)
        u = tl.PASOS[p.rumbo]
        x, y, _ = pose
        pared = (x + u[0] * (tl.F_X + tele['frente']), y + u[1] * (tl.F_X + tele['frente']))
        celda = (round((pared[0] - u[0] * tl.MEDIA) / tl.CELDA), round((pared[1] - u[1] * tl.MEDIA) / tl.CELDA))
        direccion = (p.rumbo + (3 if dx > 0 else 1)) % 4      # punta a la derecha de la imagen = doblar a la derecha
        return celda, direccion, largo

    # ------------------------------------------------------------------ cada 50 ms
    def paso(self, ahora):
        e = self.enlace
        tele = self.tele(ahora)
        if tele is not None and not self.calib_aplicada(tele) and ahora - self._t_calib >= 1.0:
            self._t_calib = ahora
            c = self.calib
            e.enviar_cmd(5, c['diametro_m'], c['trocha_m'], c['frenado_m'], c['frenado_giro_m'])
        g = self.piloto.guardia()
        if ahora - self._t_cfg >= 1.0 or g != self._g_ult:
            self._t_cfg, self._g_ult = ahora, g
            e.enviar_cmd(8, 1, g[0], g[1], 5.0)
        odo = self.odometria()
        if tele is None or odo is None:
            self.v = self.w = 0.0
            e.enviar_vel(0.0, 0.0)
            return
        self.detecciones(ahora)
        v, w, orden = self.piloto.paso(ahora, tele, odo[0], odo[1])
        self.v, self.w = v, w
        e.enviar_vel(v, w)
        if orden is not None:
            e.enviar_cmd(*orden)
        p = self.piloto
        fm = lambda m: '' if m is None else '%.3f' % m
        self.csv.writerow(['%.2f' % (ahora - self.t0), p.estado, p.modo, p.celda, p.rumbo, '%.3f' % v, '%.3f' % w,
                           fm(tele['frente']), fm(tele['izq']), fm(tele['der']), tele['guardia'], '%.3f' % p.obs.y,
                           '%.1f' % math.degrees(p.obs.psi), '%.3f' % odo[0], p.motivo])

    def linea(self, ahora):
        tele = self.tele(ahora)
        if tele is None:
            return 'ESP32 sin /wheel_state (¿agente? ¿firmware con sonares?)'
        fm = lambda m: ' -- ' if m is None else '%4.2f' % m
        p = self.piloto
        return '[%-9s] celda %-8s %s | F %s I %s D %s | guardia %d | y %+4.0f mm psi %+3.0f° | v %.2f w %+.2f' % (
            p.estado, p.celda, tl.NOMBRE_DIR[p.rumbo], fm(tele['frente']), fm(tele['izq']), fm(tele['der']), tele['guardia'],
            1000 * p.obs.y, math.degrees(p.obs.psi), self.v, self.w)

    def parar(self):
        self.v = self.w = 0.0
        self.enlace.enviar_vel(0.0, 0.0)
        self.enlace.enviar_cmd(4)

    def cerrar(self):
        try:
            for _ in range(3):
                self.parar()
            self.enlace.enviar_cmd(8, 0, 0.20, 0.40, 5.0)
        except Exception as ex:          # el contexto de ROS ya se cerró: la ESP32 igual frena a 1 s sin órdenes
            self.imprimir('  (no pude avisar a la ESP32 al salir: %s; las ruedas se frenan solas en 1 s)' % ex)
        self._f.close()


# ---------------------------------------------------------------------------------------------------------------------
# ROS 2 y terminal
# ---------------------------------------------------------------------------------------------------------------------
def crear_enlace_ros(usar_camara):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

    try:
        rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
    except (TypeError, AttributeError):
        rclpy.init()

    class Enlace(Node):
        def __init__(self):
            super().__init__('torneo')
            # best effort, igual que la ESP32: una suscripción "reliable" no recibe de un publicador "best effort"
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
            self.pub_vel = self.create_publisher(Twist, '/cmd_vel', qos)
            self.pub_cmd = self.create_publisher(Float32MultiArray, '/robot_cmd', qos)
            self.create_subscription(Float32MultiArray, '/wheel_state', self._estado, qos)
            self.create_subscription(Int32MultiArray, '/wheel_ticks', self._ticks, qos)
            self.estado = self.ticks = self.det = self.tam_img = None
            self.t_estado = 0.0
            if usar_camara:
                self.create_subscription(String, '/vision/detecciones', self._det, 10)
                self._sub_img = self.create_subscription(Image, '/camera/image_raw', self._img, qos_profile_sensor_data)

        def _estado(self, msg):
            self.estado = list(msg.data)
            self.t_estado = time.time()

        def _ticks(self, msg):
            self.ticks = list(msg.data)

        def _det(self, msg):
            try:
                self.det = (json.loads(msg.data), time.time())
            except ValueError:
                pass

        def _img(self, msg):
            if self.tam_img is None:
                self.tam_img = (msg.width, msg.height)

        def atender(self):
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0)

        def enviar_cmd(self, ident, *args):
            self.pub_cmd.publish(Float32MultiArray(data=[float(ident)] + [float(a) for a in args]))

        def enviar_vel(self, lineal, angular):
            m = Twist()
            m.linear.x = float(lineal)
            m.angular.z = float(angular)
            self.pub_vel.publish(m)

        def cerrar(self):
            self.destroy_node()
            rclpy.try_shutdown()

    return Enlace()


def tecla():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    d = tl.Parametros()
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--sin-camara', action='store_true', help='solo sonares: sin flechas ni cubo')
    ap.add_argument('--color', default=d.color, help='color de las flechas y del cubo (%s)' % d.color)
    ap.add_argument('--v', type=float, default=d.v_crucero, help='velocidad de crucero, m/s (%.2f)' % d.v_crucero)
    ap.add_argument('--parada-cubo', type=float, default=d.parada_cubo,
                    help='a cuánto del cubo frena, m (%.2f; la guardia no deja menos de 0,05)' % d.parada_cubo)
    ap.add_argument('--hfov', type=float, default=60.0, help='campo de visión horizontal de la cámara, grados (60)')
    a = ap.parse_args()
    params = tl.Parametros(v_crucero=a.v, color=a.color, parada_cubo=a.parada_cubo)

    calib, hay = cargar_calibracion()
    print('Calibración %s: diámetro %.1f mm, trocha %.0f mm' % (
        'de calibracion.json' if hay else 'POR DEFECTO (no hay ~/caid/config/calibracion.json)',
        1000 * calib['diametro_m'], 1000 * calib['trocha_m']))
    enlace = crear_enlace_ros(not a.sin_camara)
    interactivo = sys.stdin.isatty()

    def imprimir(m):
        print('\r\x1b[K' + m, flush=True)

    tor = Torneo(enlace, calib, params, usar_camara=not a.sin_camara, hfov=math.radians(a.hfov), imprimir=imprimir)
    term = termios.tcgetattr(sys.stdin) if interactivo else None
    if interactivo:
        tty.setcbreak(sys.stdin.fileno())
    print('Torneo: buscando el cubo %s%s. Teclas: g arrancar, y agarrado, ESPACIO detener, q salir.' % (
        a.color, ' (sin cámara)' if a.sin_camara else ''))
    print('Registro: %s' % tor.ruta_csv)
    t_paso = t_linea = 0.0
    try:
        while True:
            enlace.atender()
            ahora = time.time()
            k = tecla() if interactivo else None
            if k == 'q':
                break
            if k is not None:
                tor.tecla(k, ahora)
            if ahora - t_paso >= 0.05:
                t_paso = ahora
                tor.paso(ahora)
            if ahora - t_linea >= 0.25:
                t_linea = ahora
                print('\r\x1b[K' + tor.linea(ahora), end='', flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        tor.cerrar()
        if interactivo:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term)
        time.sleep(0.2)
        try:
            enlace.atender()
            enlace.cerrar()
        except Exception:
            pass
        print('\nListo. El robot quedó frenado y la guardia apagada.')


if __name__ == '__main__':
    main()
