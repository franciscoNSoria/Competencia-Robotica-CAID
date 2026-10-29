#!/usr/bin/env python3
"""Control del robot por teclado, inalambrico (ESP32 por micro-ROS), como el dia del torneo.

Teclas:
  c        calibracion de las 4 ruedas (EN EL AIRE): mide la velocidad real contra la curva programada,
           arma la curva de cada rueda y las iguala con el PI a una velocidad comun
  f / b    marcha adelante / atras de 1 m (EN EL PISO). Despues de cada una te pide el valor medido con cinta
  w s a d  adelante, atras, giro izquierda, giro derecha (manual).   Espacio o x: parar
  e        estado de las ruedas      h o ?  ayuda      q  salir (frena)
Durante una secuencia, 'q' o Espacio la cancelan y frenan.

Requisitos: ROS 2 Jazzy cargado (source /opt/ros/jazzy/setup.bash) y el agente micro-ROS corriendo en la PC.
La calibracion y el diametro efectivo se guardan en ~/caid/config/calibracion.json y se reenvian solos cuando la ESP32 se conecta.
"""
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
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int32MultiArray

RUEDAS = ['FL', 'FR', 'RL', 'RR']
CARPETA = os.path.expanduser('~/caid/config')
RUTA_CAL = os.path.join(CARPETA, 'calibracion.json')
RUTA_MARCHAS = os.path.join(CARPETA, 'marchas.csv')

DIAMETRO_M = 0.067          # CAD; el valor en uso viene de la ESP32 (estado[20]) y se corrige con la cinta
TROCHA_M = 0.131
FRENADO_M = 0.030
FLANCOS_POR_VUELTA = 40
V_COMUN_DEFECTO = 40.0      # flancos/s (1 vuelta/s = 0,21 m/s)
V_MARCHA = 40.0
NIVELES_DUTY = [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.45, 0.40, 0.36]   # de arriba hacia abajo: lo que arranca sigue girando
T_NIVEL_S, T_MEDIR_S = 2.8, 1.2
V_MANUAL, W_MANUAL = 0.20, 2.0

# Curva programada (flancos/s -> duty). Hasta 0,60 es la del firmware; los delanteros se extienden con lo medido en el banco
# (caracterizacion-motores.md) para poder comparar tambien a duty alto. Se compara con lo que mide la calibracion.
PROGRAMADA = {
    'FL': [(0.0, 0.34), (16.8, 0.36), (33.3, 0.40), (69.1, 0.50), (104.6, 0.60), (134.1, 0.70), (147.2, 0.80),
           (162.9, 0.85), (186.8, 0.90), (230.0, 1.00)],
    'FR': [(0.0, 0.34), (16.8, 0.36), (33.3, 0.40), (69.1, 0.50), (104.6, 0.60), (134.1, 0.70), (147.2, 0.80),
           (162.9, 0.85), (186.8, 0.90), (230.0, 1.00)],
    'RL': [(0.0, 0.36), (13.2, 0.40), (26.8, 0.50), (31.5, 0.60), (38.6, 0.70), (45.4, 0.75), (54.2, 0.80),
           (65.9, 0.85), (77.9, 0.90), (91.6, 1.00)],
    'RR': [(0.0, 0.50), (26.6, 0.50), (34.4, 0.60), (40.6, 0.70), (47.2, 0.75), (55.8, 0.80), (67.0, 0.85),
           (79.7, 0.90), (94.7, 1.00)],
}


class Cancelado(Exception):
    pass


def v_esperada(rueda, duty):
    """Velocidad que la curva programada da para ese duty (la curva esta en v -> duty)."""
    c = PROGRAMADA[rueda]
    if duty < c[0][1]:
        return 0.0
    for (v0, d0), (v1, d1) in zip(c, c[1:]):
        if d1 >= duty and d1 > d0:
            return v0 + (v1 - v0) * (duty - d0) / (d1 - d0) if duty >= d0 else v0
    return c[-1][0]


def pendiente(ts, ys):
    n = len(ts)
    if n < 2:
        return float('nan')
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den else float('nan')


class Robot(Node):
    def __init__(self):
        super().__init__('robot_teclado')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', qos)
        self.pub_cmd = self.create_publisher(Float32MultiArray, '/robot_cmd', qos)
        self.create_subscription(Int32MultiArray, '/wheel_ticks', self.cb_ticks, qos)
        self.create_subscription(Float32MultiArray, '/wheel_state', self.cb_estado, qos)
        self.ticks = deque(maxlen=6000)     # (t, [4])
        self.estado = deque(maxlen=3000)    # (t, [24])
        self.t_ultimo_estado = 0.0
        self.conectado = False
        self.reenviar_config = False
        self.manual = None                  # (lineal, angular) o None
        self.cal = self.cargar()
        self.fd = sys.stdin.fileno()
        self.term_orig = termios.tcgetattr(self.fd)

    # ---- entrada / salida
    def cb_ticks(self, msg):
        self.ticks.append((time.monotonic(), list(msg.data)))

    def cb_estado(self, msg):
        ahora = time.monotonic()
        if not self.conectado:
            self.conectado = True
            self.reenviar_config = True
            print('\n[ESP32 conectada]')
        self.t_ultimo_estado = ahora
        self.estado.append((ahora, list(msg.data)))

    def revisar_enlace(self):
        if self.conectado and time.monotonic() - self.t_ultimo_estado > 2.0:
            self.conectado = False
            print('\n[ESP32 sin datos hace 2 s]')

    def ultimo(self):
        return self.estado[-1][1] if self.estado else None

    def enviar(self, ident, *args):
        m = Float32MultiArray()
        m.data = [float(ident)] + [float(a) for a in args]
        self.pub_cmd.publish(m)

    def vel(self, lineal, angular):
        m = Twist()
        m.linear.x = float(lineal)
        m.angular.z = float(angular)
        self.pub_vel.publish(m)

    def tecla(self):
        if select.select([self.fd], [], [], 0)[0]:
            return os.read(self.fd, 1).decode(errors='ignore')
        return ''

    def preguntar(self, texto):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.term_orig)
        try:
            return input(texto)
        finally:
            tty.setcbreak(self.fd)

    def esperar(self, seg, enviar=None, hz=20.0):
        """Gira ROS durante `seg`, repitiendo `enviar()` a `hz`. 'q' o Espacio cancelan."""
        fin = time.monotonic() + seg
        prox = 0.0
        while time.monotonic() < fin:
            if enviar is not None and time.monotonic() >= prox:
                enviar()
                prox = time.monotonic() + 1.0 / hz
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.tecla() in ('q', ' '):
                self.enviar(4)
                raise Cancelado()

    def parar(self, seg=1.0):
        self.manual = None
        fin = time.monotonic() + seg
        while time.monotonic() < fin:
            self.enviar(4)
            self.vel(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- configuracion guardada
    def cargar(self):
        try:
            with open(RUTA_CAL) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def guardar(self):
        os.makedirs(CARPETA, exist_ok=True)
        with open(RUTA_CAL, 'w') as f:
            json.dump(self.cal, f, indent=2)

    def diametro(self):
        e = self.ultimo()
        return (e[20] / 1000.0) if e else self.cal.get('diametro_m', DIAMETRO_M)

    def flancos_por_m(self):
        return FLANCOS_POR_VUELTA / (math.pi * self.diametro())

    def mandar_config(self, diametro=None, frenado=None):
        d = diametro if diametro else self.cal.get('diametro_m', self.diametro())
        fr = frenado if frenado is not None else self.cal.get('frenado_m', FRENADO_M)
        self.cal['diametro_m'], self.cal['frenado_m'] = d, fr
        self.guardar()
        for _ in range(3):
            self.enviar(5, d, self.cal.get('trocha_m', TROCHA_M), fr)
            self.esperar(0.1)

    def mandar_curvas(self, curvas):
        for i, c in enumerate(curvas):
            pts = c['puntos'][:10]
            plano = [x for p in pts for x in p]
            for _ in range(3):
                self.enviar(6, i, c['d_min'], len(pts), *plano)
                self.esperar(0.1)

    def aplicar_guardado(self):
        if not self.cal:
            print('(no hay calibracion guardada: se usa la curva programada; apreta c para calibrar)')
            return
        print('Enviando la calibracion guardada a la ESP32...')
        self.mandar_config()
        if 'curvas' in self.cal:
            self.mandar_curvas(self.cal['curvas'])

    # ---- calibracion en el aire
    def medir_rueda(self, i):
        """Barre el duty de la rueda i, de arriba hacia abajo. Devuelve [(duty, flancos/s)]."""
        puntos = []
        for duty in NIVELES_DUTY:
            vec = [0.0] * 4
            vec[i] = duty
            self.esperar(T_NIVEL_S - T_MEDIR_S, lambda v=vec: self.enviar(1, *v))
            t0 = time.monotonic()
            self.esperar(T_MEDIR_S, lambda v=vec: self.enviar(1, *v))
            tk = [(t, d[i]) for t, d in self.ticks if t >= t0]
            v = pendiente([t for t, _ in tk], [d for _, d in tk])
            v = 0.0 if math.isnan(v) or v < 2.0 else v
            puntos.append((duty, v))
            print('   %s duty %.2f -> %6.1f flancos/s' % (RUEDAS[i], duty, v))
            if v == 0.0:
                break                      # dejo de girar: debajo tampoco sostiene
        self.parar(1.2)
        return puntos

    def calibrar(self):
        if not self.conectado:
            print('La ESP32 no esta conectada.')
            return
        if self.preguntar('\nRuedas EN EL AIRE y cables lejos. Enter para empezar (otra cosa cancela): ') != '':
            return
        medidas, fallas = {}, []
        for i, r in enumerate(RUEDAS):
            if not self.conectado:
                print('Se perdio la conexion con la ESP32 (reinicio por caida de tension?). Calibracion cancelada.')
                return
            print('\nRueda %s (barrido de duty, de 1,00 hacia abajo):' % r)
            puntos = self.medir_rueda(i)
            if not puntos or puntos[0][1] == 0.0:
                print('  %s NO GIRA ni con duty 1,00: revisar cableado de IA/IB, motor o encoder.' % r)
                fallas.append(r)
                continue
            medidas[r] = puntos
        if fallas:
            print('\nRuedas con falla: %s. No se envian curvas hasta que giren las 4.' % ', '.join(fallas))

        print('\nComparacion con la curva programada (flancos/s):')
        print('  duty | ' + ' | '.join('%-18s' % r for r in RUEDAS))
        for duty in NIVELES_DUTY:
            fila = []
            for r in RUEDAS:
                v = dict(medidas.get(r, [])).get(duty)
                if v is None:
                    fila.append('%-18s' % '-')
                else:
                    esp = v_esperada(r, duty)
                    dif = '%+4.0f%%' % (100.0 * (v - esp) / esp) if esp > 1.0 else '  n/a'
                    fila.append('%5.1f / %5.1f %s' % (v, esp, dif))
            print('  %.2f | %s' % (duty, ' | '.join(fila)))
        print('  (cada celda: medido / programado, diferencia)')
        if fallas:
            return

        curvas, vmax, vmin = [], [], []
        for r in RUEDAS:
            mov = sorted([(d, v) for d, v in medidas[r] if v > 0.0])          # duty ascendente
            d_min = mov[0][0]
            pts, ultimo = [(0.0, d_min)], 0.0
            for d, v in mov:
                if v > ultimo + 1.0:
                    pts.append((round(v, 1), d))
                    ultimo = v
            curvas.append({'rueda': r, 'd_min': d_min, 'puntos': pts})
            vmax.append(mov[-1][1])
            vmin.append(mov[0][1])
        v_max_comun = 0.85 * min(vmax)
        v_min_comun = max(vmin)
        v_comun = min(max(V_COMUN_DEFECTO, 1.1 * v_min_comun), v_max_comun)
        print('\nRango comun de las 4 ruedas: de %.1f a %.1f flancos/s (maximo con 15 %% de margen para el PI)' % (
            v_min_comun, v_max_comun))
        if v_comun < v_min_comun or v_max_comun <= v_min_comun:
            print('ATENCION: no hay un rango comun util. Las ruedas mas lentas no alcanzan a las otras.')
        print('Velocidad comun de verificacion: %.1f flancos/s (%.3f m/s)' % (v_comun, v_comun / self.flancos_por_m()))

        self.cal['curvas'] = curvas
        self.cal['v_comun'] = v_comun
        self.cal['rango_comun'] = [v_min_comun, v_max_comun]
        self.cal['fecha'] = datetime.datetime.now().isoformat(timespec='seconds')
        self.guardar()
        seq0 = self.ultimo()[22]
        self.mandar_curvas(curvas)
        seq1 = self.ultimo()[22]
        print('Curvas enviadas (la ESP32 acepto %d de 4).' % (seq1 - seq0))

        print('\nVerificacion: las 4 ruedas con PI a %.1f flancos/s durante 8 s...' % v_comun)
        v_ms = v_comun / self.flancos_por_m()
        self.esperar(5.0, lambda: self.enviar(2, v_ms, v_ms, v_ms, v_ms))
        t0 = time.monotonic()
        self.esperar(3.0, lambda: self.enviar(2, v_ms, v_ms, v_ms, v_ms))
        tk = [(t, d) for t, d in self.ticks if t >= t0]
        es = [d for t, d in self.estado if t >= t0]
        self.parar()
        vs = [pendiente([t for t, _ in tk], [d[i] for _, d in tk]) for i in range(4)]
        media = sum(vs) / 4.0
        print('  rueda | velocidad | error vs pedida | duty medio')
        for i, r in enumerate(RUEDAS):
            duty = sum(abs(d[8 + i]) for d in es) / len(es) if es else float('nan')
            print('  %-5s | %9.1f | %+14.1f%% | %.2f' % (r, vs[i], 100.0 * (vs[i] - v_comun) / v_comun, duty))
        disp = 100.0 * (max(vs) - min(vs)) / media
        print('Dispersion entre ruedas: %.1f %% -> %s' % (disp, 'IGUALADAS' if disp <= 5.0 else 'NO IGUALADAS (mas de 5 %)'))

    # ---- marchas de 1 m en el piso
    def marcha(self, dist):
        if not self.conectado:
            print('La ESP32 no esta conectada.')
            return
        nombre = 'adelante' if dist > 0 else 'atras'
        if self.preguntar('\nRobot EN EL PISO, en la marca de salida, con %.1f m libres %s. Enter para avanzar (otra cosa cancela): '
                          % (abs(dist) + 0.5, 'adelante' if dist > 0 else 'atras')) != '':
            return
        seq0 = self.ultimo()[19]
        activo = False
        fin = time.monotonic() + 30.0
        proxima = 0.0
        while time.monotonic() < fin:
            e = self.ultimo()
            if not activo and e and e[16] > 0.5:
                activo = True
            if not activo and time.monotonic() >= proxima:
                self.enviar(3, dist, V_MARCHA)       # se repite hasta que la ESP32 empieza (UDP puede perder uno)
                proxima = time.monotonic() + 0.3
            rclpy.spin_once(self, timeout_sec=0.02)
            e = self.ultimo()
            if e and activo:
                print('\r  recorrido %+.3f m   ' % e[17], end='', flush=True)
            if e and e[19] > seq0:
                break
            if self.tecla() in ('q', ' '):
                self.enviar(4)
                self.parar()
                print('\nCancelado.')
                return
        e = self.ultimo()
        if e is None or e[19] <= seq0:
            print('\nLa ESP32 no informo el final del movimiento.')
            return
        final = e[18]
        sobre = (final - dist) * (1 if dist > 0 else -1)         # positivo: paso de la meta
        print('\nMovimiento %s terminado. Calculado con los encoders: %+.3f m (pedido %+.3f m, diferencia %+.1f mm)' % (
            nombre, final, dist, sobre * 1000.0))

        frenado = self.cal.get('frenado_m', FRENADO_M)
        if abs(sobre) > 0.005:
            nuevo = min(0.09, max(0.0, frenado + sobre))
            print('Ajuste automatico del adelanto de frenado: %.1f mm -> %.1f mm' % (frenado * 1000, nuevo * 1000))
            self.mandar_config(frenado=nuevo)
            frenado = nuevo

        real_txt = self.preguntar('Cuantos metros avanzo DE VERDAD (con cinta)? (Enter = omitir): ').strip().replace(',', '.')
        real = None
        if real_txt:
            real = abs(float(real_txt))
            k = real / abs(final)
            nuevo_d = self.diametro() * k
            print('Factor real/calculado = %.4f -> diametro efectivo %.2f mm (antes %.2f mm)' % (
                k, nuevo_d * 1000, self.diametro() * 1000))
            self.mandar_config(diametro=nuevo_d)
        resto = None
        if dist < 0:
            txt = self.preguntar('A cuantos cm de la marca de salida quedo? (+ adelante de la marca, - detras; Enter = omitir): ')
            txt = txt.strip().replace(',', '.')
            resto = float(txt) if txt else None
            if resto is not None:
                print('Ida y vuelta: queda a %+.1f cm de la marca de salida.' % resto)
        os.makedirs(CARPETA, exist_ok=True)
        nuevo_archivo = not os.path.exists(RUTA_MARCHAS)
        with open(RUTA_MARCHAS, 'a', newline='') as f:
            w = csv.writer(f)
            if nuevo_archivo:
                w.writerow(['fecha', 'sentido', 'pedido_m', 'calculado_m', 'real_m', 'diametro_mm', 'frenado_mm',
                            'resto_cm'])
            w.writerow([datetime.datetime.now().isoformat(timespec='seconds'), nombre, dist, '%.4f' % final,
                        '' if real is None else real, '%.2f' % (self.diametro() * 1000), '%.1f' % (frenado * 1000),
                        '' if resto is None else resto])

    # ---- estado y ayuda
    def mostrar_estado(self):
        e = self.ultimo()
        if not e:
            print('Sin datos de la ESP32.')
            return
        print('\n  rueda | v medida | v pedida |  duty | bloqueada')
        for i, r in enumerate(RUEDAS):
            print('  %-5s | %8.1f | %8.1f | %+5.2f | %s' % (r, e[i], e[4 + i], e[8 + i], 'SI' if e[12 + i] > 0.5 else 'no'))
        print('  diametro efectivo %.2f mm, trocha %.1f mm, adelanto de frenado %.1f mm' % (e[20], e[23], e[21]))

    def ayuda(self):
        print(__doc__.split('Requisitos')[0])

    def teclado(self):
        self.ayuda()
        proximo_manual = 0.0
        print('Esperando a la ESP32 (agente micro-ROS corriendo y robot encendido)...')
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.revisar_enlace()
            if self.reenviar_config:
                self.reenviar_config = False
                self.aplicar_guardado()
            if self.manual is not None and time.monotonic() >= proximo_manual:
                self.vel(*self.manual)
                proximo_manual = time.monotonic() + 0.05
            t = self.tecla()
            if not t:
                continue
            try:
                if t == 'c':
                    self.manual = None
                    self.calibrar()
                elif t == 'f':
                    self.manual = None
                    self.marcha(+1.0)
                elif t == 'b':
                    self.manual = None
                    self.marcha(-1.0)
                elif t == 'w':
                    self.manual = (V_MANUAL, 0.0)
                elif t == 's':
                    self.manual = (-V_MANUAL, 0.0)
                elif t == 'a':
                    self.manual = (0.0, W_MANUAL)
                elif t == 'd':
                    self.manual = (0.0, -W_MANUAL)
                elif t in (' ', 'x'):
                    self.parar(0.5)
                elif t == 'e':
                    self.mostrar_estado()
                elif t in ('h', '?'):
                    self.ayuda()
                elif t == 'q':
                    break
            except Cancelado:
                self.parar()
                print('\nCancelado.')


def main():
    if not sys.stdin.isatty():
        sys.exit('Correlo en una terminal (necesita leer el teclado).')
    rclpy.init()
    robot = Robot()
    tty.setcbreak(robot.fd)
    try:
        robot.teclado()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            robot.parar(1.0)
        finally:
            termios.tcsetattr(robot.fd, termios.TCSADRAIN, robot.term_orig)
            robot.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
