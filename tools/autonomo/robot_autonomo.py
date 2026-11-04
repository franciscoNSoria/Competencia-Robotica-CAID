#!/usr/bin/env python3
"""Robot autonomo "vagabundo" desde la PC: avanza, y al ver un obstaculo con los sonares gira hasta ver libre. Por micro-ROS (Wi-Fi).

    python3 tools/autonomo/robot_autonomo.py                 # arranca con g, se detiene con ESPACIO
    python3 tools/autonomo/robot_autonomo.py --solo-guardia  # solo activa el freno por sonar: se maneja con el panel de ruedas

Requisitos: ROS 2 Jazzy cargado (source /opt/ros/jazzy/setup.bash), el agente micro-ROS corriendo y la ESP32 con el firmware de
firmware/banco/robot_4ruedas (version con sonares y guardia: publica 34 valores en /wheel_state). CERRAR el panel de ruedas y
cualquier otra herramienta que mande /cmd_vel: dos programas mandando velocidades se pisan.

Teclas:  g arrancar (o volver a armar despues de una detencion)   ESPACIO / s / Esc detener   ? ayuda   q salir

Seguridad: (1) el freno por sonar corre en la ESP32 (guardia.h) y esta activo mientras este programa lo pide (cada 1 s); (2) si falta
la telemetria, el programa se frena solo y la ESP32 igual frena las ruedas a 1 s sin ordenes; (3) un obstaculo que el sonar no ve
(patas finas, objetos de costado, vidrio, escalones) NO se evita: probar en un cuarto despejado, con una mano lista en ESPACIO.
Guarda un CSV de cada corrida en ~/caid/config/autonomo_AAAAMMDD_HHMMSS.csv."""
import argparse
import csv
import datetime
import json
import os
import random
import select
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autonomo_logica as al  # noqa: E402

CARPETA = os.path.expanduser('~/caid/config')
CALIB_DEFECTO = dict(diametro_m=0.065, trocha_m=0.300, frenado_m=0.0223, frenado_giro_m=0.0015)
N_ESTADO = 34                    # valores de /wheel_state del firmware con sonares
AYUDA = ('  g arrancar / volver a armar    ESPACIO, s o Esc detener    q salir\n'
         '  El freno por sonar de la ESP32 queda activo mientras este programa corre.')


def leer_estado(estado):
    """/wheel_state del firmware con sonares -> dict, o None si es el firmware viejo (26 valores) o todavia no llego."""
    if estado is None or len(estado) < N_ESTADO:
        return None
    nn = lambda m: None if m < 0 else float(m)
    return dict(frente=nn(estado[26]), izq=nn(estado[27]), der=nn(estado[28]), guardia=int(round(estado[29])),
                d_stop=float(estado[30]), d_lento=float(estado[31]), n_frente=int(round(estado[32])), ciego=float(estado[33]),
                vel=[float(v) for v in estado[0:4]], diam_mm=float(estado[20]), trocha_mm=float(estado[23]))


def cargar_calibracion(carpeta=CARPETA):
    """La calibracion del panel (calibracion.json), o los valores por defecto si no esta. -> (dict, hay_archivo)"""
    c = dict(CALIB_DEFECTO)
    try:
        with open(os.path.join(carpeta, 'calibracion.json')) as f:
            c.update({k: float(v) for k, v in json.load(f).items() if k in c})
        return c, True
    except (OSError, ValueError):
        return c, False


class Autonomo:
    """Todo menos ROS y la terminal: se prueba con un enlace falso (test_autonomo.py)."""

    def __init__(self, enlace, params=None, d_stop=0.20, d_lento=0.60, ciego_s=5.0, minutos=10.0, solo_guardia=False,
                 semilla=None, dejar_guardia=False, carpeta=None, imprimir=print, ahora=0.0, calib=None):
        self.enlace = enlace
        self.calib = calib
        self._t_calib = -1e9
        self.p = params or al.Parametros()
        self.d_stop, self.d_lento, self.ciego_s = d_stop, d_lento, ciego_s
        self.minutos = minutos
        self.solo_guardia = solo_guardia
        self.dejar_guardia = dejar_guardia
        self.imprimir = imprimir
        self.vag = al.Vagabundo(self.p, random.Random(semilla))
        self.t0 = ahora
        self._t_cfg = -1e9
        self._t_inicio = None
        self._t_fin = None
        self._visto = (self.vag.estado, self.vag.motivo)
        self.v, self.w = 0.0, 0.0
        self.csv = None
        self.ruta_csv = None
        if carpeta:
            os.makedirs(carpeta, exist_ok=True)
            self.ruta_csv = os.path.join(carpeta, 'autonomo_%s.csv' % datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
            self._f = open(self.ruta_csv, 'w', newline='')
            self.csv = csv.writer(self._f)
            self.csv.writerow(['t_s', 'estado', 'v_cmd', 'w_cmd', 'frente_m', 'izq_m', 'der_m', 'guardia', 'n_frente',
                               'vel_FL', 'vel_FR', 'vel_RL', 'vel_RR', 'motivo'])

    # ------------------------------------------------------------------ teclas
    def tecla(self, k, ahora):
        if k == 'g':
            if self.solo_guardia:
                self.imprimir('  Modo --solo-guardia: el programa no mueve el robot. Usa el panel de ruedas.')
                return
            tele = leer_estado(self.enlace.estado)
            if tele is None:
                self.imprimir('  No arranco: la ESP32 no manda los sonares (¿firmware viejo, o sin conexion?).')
            elif ahora - self.enlace.t_estado > self.p.sin_datos_s:
                self.imprimir('  No arranco: no llega telemetria de la ESP32.')
            elif not self.calib_aplicada(tele):
                self.imprimir('  No arranco: la ESP32 todavia no tiene la calibracion (se le manda sola; proba de nuevo en 1 s).')
            elif tele['guardia'] == al.G_APAGADA:
                self.imprimir('  No arranco: la guardia de la ESP32 todavia esta apagada (se activa sola en 1 s; proba de nuevo).')
            elif self.vag.estado in (al.AVANZAR, al.GIRAR):
                pass
            else:
                self._t_inicio = ahora
                self._t_fin = ahora + 60.0 * self.minutos
                self.vag.iniciar(ahora)
                self.imprimir('  ARRANCA (%.0f min como maximo). ESPACIO frena.' % self.minutos)
        elif k in (' ', 's', '\x1b'):
            if self.vag.estado in (al.AVANZAR, al.GIRAR):
                self.vag.detener(ahora)
            self.parar()
        elif k == '?':
            self.imprimir(AYUDA)

    # ------------------------------------------------------------------ cada 100 ms
    def paso(self, ahora):
        e = self.enlace
        tele = leer_estado(e.estado)
        edad = ahora - e.t_estado if e.t_estado is not None else 1e9
        frescos = tele is not None and edad < self.p.sin_datos_s

        if ahora - self._t_cfg >= 1.0:                    # la guardia se re-pide cada segundo: si la ESP32 se reinicia, vuelve sola
            self._t_cfg = ahora
            e.enviar_cmd(8, 1, self.d_stop, self.d_lento, self.ciego_s)
        if tele is not None and self.calib and not self.calib_aplicada(tele) and ahora - self._t_calib >= 1.0:
            self._t_calib = ahora              # sin el panel la ESP32 arranca con diametro 67 y trocha 131: los giros no andan
            c = self.calib
            e.enviar_cmd(5, c['diametro_m'], c['trocha_m'], c['frenado_m'], c['frenado_giro_m'])
        if self.solo_guardia:
            return
        corriendo = self.vag.estado in (al.AVANZAR, al.GIRAR)
        if corriendo:
            if tele is None:
                self.vag.detener(ahora, 'la ESP32 no manda los sonares')
            elif frescos and tele['guardia'] == al.G_APAGADA and ahora - self._t_inicio > 2.5:
                self.vag.detener(ahora, 'la guardia de la ESP32 esta apagada (¿se reinicio?): arma de nuevo con g')
            elif ahora >= self._t_fin:
                self.vag.detener(ahora, 'se cumplieron los %.0f min' % self.minutos)
        if tele is None:
            self.v, self.w = 0.0, 0.0
        else:
            self.v, self.w = self.vag.paso(ahora, tele['frente'], tele['izq'], tele['der'], tele['guardia'], tele['n_frente'],
                                           datos_frescos=frescos)
        e.enviar_vel(self.v, self.w)
        if (self.vag.estado, self.vag.motivo) != self._visto:
            self._visto = (self.vag.estado, self.vag.motivo)
            self.imprimir('\n  [%5.1f s] %s: %s' % (ahora - self.t0, self.vag.estado, self.vag.motivo))
        if self.csv and tele is not None and self.vag.estado != al.ESPERA:
            fm = lambda m: '' if m is None else '%.3f' % m
            self.csv.writerow(['%.2f' % (ahora - self.t0), self.vag.estado, '%.3f' % self.v, '%.3f' % self.w, fm(tele['frente']),
                               fm(tele['izq']), fm(tele['der']), tele['guardia'], tele['n_frente']] +
                              ['%.1f' % v for v in tele['vel']] + [self.vag.motivo])

    def calib_aplicada(self, tele):
        c = self.calib
        return (not c) or (abs(tele['diam_mm'] - 1000 * c['diametro_m']) < 0.05 and abs(tele['trocha_mm'] - 1000 * c['trocha_m']) < 0.5)

    def linea(self, ahora):
        tele = leer_estado(self.enlace.estado)
        if tele is None:
            return 'ESP32 sin sonares / sin conexion'
        fm = lambda m: '  -- ' if m is None else '%5.2f' % m
        corre = '' if self._t_inicio is None or self.vag.estado in (al.ESPERA, al.DETENIDO) else ' | %d:%02d' % divmod(int(ahora - self._t_inicio), 60)
        return '[%-8s] frente %s izq %s der %s m | guardia %-7s | v %+.2f w %+.2f%s | giros %d' % (
            self.vag.estado, fm(tele['frente']), fm(tele['izq']), fm(tele['der']), al.NOMBRES_GUARDIA.get(tele['guardia'], '?'),
            self.v, self.w, corre, self.vag.giros)

    # ------------------------------------------------------------------ salida
    def parar(self):
        self.v = self.w = 0.0
        self.enlace.enviar_vel(0.0, 0.0)
        self.enlace.enviar_cmd(4)

    def cerrar(self):
        try:
            for _ in range(3):
                self.parar()
            if not self.dejar_guardia:
                self.enlace.enviar_cmd(8, 0, self.d_stop, self.d_lento, self.ciego_s)
        except Exception as e:              # el contexto de ROS ya se cerro: la ESP32 igual frena a 1 s sin ordenes
            self.imprimir('  (no pude avisar a la ESP32 al salir: %s; las ruedas se frenan solas en 1 s)' % e)
        if self.csv:
            self._f.close()


# ---------------------------------------------------------------------------------------------------------------------
# ROS 2 y terminal (solo se importan al correr de verdad)
# ---------------------------------------------------------------------------------------------------------------------
def crear_enlace_ros():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray

    try:
        rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)     # Ctrl+C llega como KeyboardInterrupt, sin cerrar ROS antes
    except (TypeError, AttributeError):
        rclpy.init()

    class Enlace(Node):
        def __init__(self):
            super().__init__('pc_autonomo')
            # best effort, igual que la ESP32: una suscripcion "reliable" no recibe de un publicador "best effort"
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
            self.pub_vel = self.create_publisher(Twist, '/cmd_vel', qos)
            self.pub_cmd = self.create_publisher(Float32MultiArray, '/robot_cmd', qos)
            self.create_subscription(Float32MultiArray, '/wheel_state', self._al_llegar, qos)
            self.estado = None
            self.t_estado = None

        def _al_llegar(self, msg):
            self.estado = list(msg.data)
            self.t_estado = time.time()

        def atender(self):
            for _ in range(10):
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
    d = al.Parametros()
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--solo-guardia', action='store_true', help='solo activa el freno por sonar, sin mover el robot')
    ap.add_argument('--v', type=float, default=d.v_crucero, help='velocidad de crucero, m/s (%.2f)' % d.v_crucero)
    ap.add_argument('--giro', type=float, default=d.w_giro, help='velocidad de giro, rad/s (%.1f)' % d.w_giro)
    ap.add_argument('--evitar', type=float, default=d.d_evitar, help='distancia de frente a la que empieza a girar, m (%.2f)' % d.d_evitar)
    ap.add_argument('--libre', type=float, default=d.d_libre, help='distancia de frente para terminar un giro, m (%.2f)' % d.d_libre)
    ap.add_argument('--lateral', type=float, default=d.d_lateral, help='distancia lateral que lo empuja al otro lado, m (%.2f)' % d.d_lateral)
    ap.add_argument('--d-stop', type=float, default=0.20, help='la ESP32 frena a esta distancia del sonar frontal, m (0.20)')
    ap.add_argument('--d-lento', type=float, default=0.60, help='la ESP32 baja a 0,13 m/s desde esta distancia, m (0.60)')
    ap.add_argument('--ciego', type=float, default=5.0, help='sin ningun eco por mas de estos segundos la ESP32 no avanza (5)')
    ap.add_argument('--minutos', type=float, default=10.0, help='tope de una corrida (10)')
    ap.add_argument('--semilla', type=int, default=None)
    ap.add_argument('--dejar-guardia', action='store_true', help='al salir no apaga el freno por sonar de la ESP32')
    a = ap.parse_args()
    params = al.Parametros(v_crucero=a.v, w_giro=a.giro, d_evitar=a.evitar, d_libre=a.libre, d_lateral=a.lateral)

    enlace = crear_enlace_ros()
    calib, hay = cargar_calibracion()
    print('Calibracion %s: diametro %.1f mm, trocha %.0f mm' % ('de calibracion.json' if hay else 'POR DEFECTO (no encuentro calibracion.json)',
                                                              1000 * calib['diametro_m'], 1000 * calib['trocha_m']))
    ap_ = Autonomo(enlace, params, a.d_stop, a.d_lento, a.ciego, a.minutos, a.solo_guardia, a.semilla, a.dejar_guardia,
                   carpeta=CARPETA, ahora=time.time(), calib=calib)
    interactivo = sys.stdin.isatty()
    term = termios.tcgetattr(sys.stdin) if interactivo else None
    if interactivo:
        tty.setcbreak(sys.stdin.fileno())
    print('Robot autonomo%s. Teclas: g arrancar, ESPACIO detener, q salir, ? ayuda.' % (' (solo guardia)' if a.solo_guardia else ''))
    print('Registro: %s' % ap_.ruta_csv)
    t_paso = t_linea = 0.0
    try:
        while True:
            enlace.atender()
            ahora = time.time()
            k = tecla() if interactivo else None
            if k == 'q':
                break
            if k is not None:
                ap_.tecla(k, ahora)
            if ahora - t_paso >= 0.1:
                t_paso = ahora
                ap_.paso(ahora)
            if ahora - t_linea >= 0.2:
                t_linea = ahora
                print('\r\x1b[K' + ap_.linea(ahora), end='', flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        ap_.cerrar()
        if interactivo:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term)
        time.sleep(0.2)
        try:
            enlace.atender()
            enlace.cerrar()
        except Exception:
            pass
        print('\nListo. El robot quedo frenado%s.' % ('' if a.dejar_guardia else ' y el freno por sonar apagado'))


if __name__ == '__main__':
    main()
