#!/usr/bin/env python3
"""Banco y calibracion de las 4 ruedas por micro-ROS (corre en la PC, con el agente andando).

Pruebas (una por ejecucion):
  aire       cada rueda sola, adelante y atras, y despues las 4 juntas (ruedas en el aire)
  piso       las 4 juntas en el piso; compara con la prueba del aire (si hay un resultado guardado)
  metro      avanza 1 m calculado con los encoders; le pedis cuanto avanzo de verdad y calcula el diametro efectivo
  escalones  escalones de velocidad (respuesta del PI: tiempo de subida, sobreimpulso, error)

Siempre guarda los datos crudos de /wheel_ticks y /wheel_state en un CSV y el resumen en un JSON.
Uso:  python3 calibracion_ruedas.py aire
Si el programa se corta, el firmware frena solo a los 1000 ms sin ordenes.
"""
import argparse
import csv
import datetime
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int32MultiArray

RUEDAS = ['FL', 'FR', 'RL', 'RR']
DIAMETRO_M = 0.067          # el mismo que DIAMETRO_RUEDA_M del firmware; se corrige con la prueba del metro
FLANCOS_POR_VUELTA = 40
V_TEST_FLANCOS = 40.0       # 1 vuelta/s
TOLERANCIA = 0.10
PERIODO_ORDEN_S = 0.05      # las ordenes se repiten a 20 Hz (el firmware frena a los 1000 ms sin ordenes)

# Curva del firmware (sin carga): flancos/s -> duty. Sirve para comparar el duty esperado con el medido.
CURVAS = {
    'FL': [(0.0, 0.34), (16.8, 0.36), (33.3, 0.40), (69.1, 0.50), (104.6, 0.60)],
    'FR': [(0.0, 0.34), (16.8, 0.36), (33.3, 0.40), (69.1, 0.50), (104.6, 0.60)],
    'RL': [(0.0, 0.36), (13.2, 0.40), (26.8, 0.50), (31.5, 0.60), (38.6, 0.70), (45.4, 0.75), (54.2, 0.80),
           (65.9, 0.85), (77.9, 0.90), (91.6, 1.00)],
    'RR': [(0.0, 0.50), (26.6, 0.50), (34.4, 0.60), (40.6, 0.70), (47.2, 0.75), (55.8, 0.80), (67.0, 0.85),
           (79.7, 0.90), (94.7, 1.00)],
}


def flancos_por_m():
    return FLANCOS_POR_VUELTA / (math.pi * DIAMETRO_M)


def duty_curva(rueda, v):
    c = CURVAS[rueda]
    if v <= c[0][0]:
        return c[0][1]
    for (v0, d0), (v1, d1) in zip(c, c[1:]):
        if v <= v1:
            return d0 + (d1 - d0) * (v - v0) / (v1 - v0)
    return c[-1][1]


def pendiente(ts, ys):
    """Pendiente de la recta de minimos cuadrados (robusta al retardo variable del Wi-Fi)."""
    n = len(ts)
    if n < 2:
        return float('nan')
    mt, my = sum(ts) / n, sum(ys) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den else float('nan')


class Banco(Node):
    def __init__(self, carpeta, nombre):
        super().__init__('banco_ruedas')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', qos)
        self.pub_rueda = self.create_publisher(Float32MultiArray, '/wheel_cmd', qos)
        self.create_subscription(Int32MultiArray, '/wheel_ticks', self.cb_ticks, qos)
        self.create_subscription(Float32MultiArray, '/wheel_state', self.cb_estado, qos)
        self.t0 = time.monotonic()
        self.ticks = []     # (t, [4])
        self.estado = []    # (t, [16])
        os.makedirs(carpeta, exist_ok=True)
        self.ruta_csv = os.path.join(carpeta, nombre + '.csv')
        self.archivo = open(self.ruta_csv, 'w', newline='')
        self.csv = csv.writer(self.archivo)
        self.csv.writerow(['t_s', 'topic'] + ['d%d' % i for i in range(16)])

    def t(self):
        return time.monotonic() - self.t0

    def cb_ticks(self, msg):
        t = self.t()
        self.ticks.append((t, list(msg.data)))
        self.csv.writerow(['%.4f' % t, 'ticks'] + list(msg.data))

    def cb_estado(self, msg):
        t = self.t()
        self.estado.append((t, list(msg.data)))
        self.csv.writerow(['%.4f' % t, 'estado'] + ['%.3f' % x for x in msg.data])

    def girar(self, seg):
        rclpy.spin_once(self, timeout_sec=seg)

    def esperar_enlace(self, espera_s=20.0):
        print('Esperando datos de la ESP32 (/wheel_ticks y /wheel_state)...')
        fin = time.monotonic() + espera_s
        while time.monotonic() < fin:
            self.girar(0.1)
            if self.ticks and self.estado:
                print('Enlace OK.')
                return True
        print('No llegan datos: el agente esta corriendo y la ESP32 conectada?')
        return False

    def orden_vel(self, lineal, angular=0.0):
        m = Twist()
        m.linear.x = float(lineal)
        m.angular.z = float(angular)
        self.pub_vel.publish(m)

    def orden_ruedas(self, vel):
        m = Float32MultiArray()
        m.data = [float(v) for v in vel]
        self.pub_rueda.publish(m)

    def tramo(self, enviar, dur_s, hasta=None):
        """Repite la orden a 20 Hz durante dur_s (o hasta que hasta() sea verdadero). Devuelve (t_ini, t_fin)."""
        t_ini = self.t()
        fin = time.monotonic() + dur_s
        proxima = 0.0
        while time.monotonic() < fin:
            if time.monotonic() >= proxima:
                enviar()
                proxima = time.monotonic() + PERIODO_ORDEN_S
            self.girar(0.01)
            if hasta is not None and hasta():
                break
        return t_ini, self.t()

    def parar(self, seg=1.5):
        fin = time.monotonic() + seg
        while time.monotonic() < fin:
            self.orden_vel(0.0)
            self.orden_ruedas([0.0] * 4)
            self.girar(0.02)

    def ventana(self, t0, t1):
        """Velocidad por ticks (flancos/s, con signo), velocidad y duty del firmware en [t0, t1]."""
        tk = [(t, d) for t, d in self.ticks if t0 <= t <= t1]
        es = [d for t, d in self.estado if t0 <= t <= t1]
        res = {}
        for i, r in enumerate(RUEDAS):
            v_ticks = pendiente([t for t, _ in tk], [d[i] for _, d in tk])
            v_fw = sum(d[i] for d in es) / len(es) if es else float('nan')
            duty = sum(abs(d[8 + i]) for d in es) / len(es) if es else float('nan')
            bloq = max((d[12 + i] for d in es), default=0.0) > 0.5
            res[r] = {'v_ticks': v_ticks, 'v_fw': v_fw, 'duty': duty, 'bloqueada': bool(bloq)}
        return res

    def cerrar(self):
        self.archivo.close()


def veredicto(pedida, medida, bloqueada):
    if bloqueada or math.isnan(medida):
        return 'MAL'
    return 'OK' if abs(medida - pedida) <= TOLERANCIA * abs(pedida) else 'MAL'


def imprimir(titulo, res, pedidas, ruedas=RUEDAS):
    print('\n' + titulo)
    print('  rueda | pedida | por ticks | firmware | error  | duty medio | duty curva | resultado')
    for r in ruedas:
        d = res[r]
        ped = pedidas[r]
        err = 100.0 * (d['v_ticks'] - ped) / abs(ped) if ped else float('nan')
        print('  %-5s | %6.1f | %9.1f | %8.1f | %+5.1f%% | %10.2f | %10.2f | %s' % (
            r, ped, d['v_ticks'], d['v_fw'], err, d['duty'], duty_curva(r, abs(ped)),
            veredicto(ped, d['v_ticks'], d['bloqueada'])))


def prueba_aire(b, resumen):
    v_ms = V_TEST_FLANCOS / flancos_por_m()
    print('Ruedas EN EL AIRE. Velocidad de prueba %.1f flancos/s = %.3f m/s.' % (V_TEST_FLANCOS, v_ms))
    for sentido, nombre in ((+1, 'adelante'), (-1, 'atras')):
        for i, r in enumerate(RUEDAS):
            vel = [0.0] * 4
            vel[i] = sentido * v_ms
            print('Rueda %s %s...' % (r, nombre))
            t0, t1 = b.tramo(lambda v=vel: b.orden_ruedas(v), 7.0)
            res = b.ventana(t1 - 3.0, t1)
            b.parar()
            imprimir('%s %s' % (r, nombre), res, {r: sentido * V_TEST_FLANCOS}, [r])
            resumen['%s_%s' % (r, nombre)] = res[r]
    print('Las 4 juntas adelante...')
    t0, t1 = b.tramo(lambda: b.orden_vel(v_ms), 8.0)
    res = b.ventana(t1 - 3.0, t1)
    b.parar()
    imprimir('4 juntas (aire)', res, {r: V_TEST_FLANCOS for r in RUEDAS})
    resumen['juntas'] = res


def prueba_piso(b, resumen, carpeta):
    v_ms = V_TEST_FLANCOS / flancos_por_m()
    print('Robot EN EL PISO, con 1,5 m libres adelante. Velocidad %.1f flancos/s = %.3f m/s.' % (V_TEST_FLANCOS, v_ms))
    input('Enter para empezar...')
    t0, t1 = b.tramo(lambda: b.orden_vel(v_ms), 8.0)
    res = b.ventana(t1 - 3.0, t1)
    b.parar()
    imprimir('4 juntas (piso)', res, {r: V_TEST_FLANCOS for r in RUEDAS})
    resumen['juntas'] = res
    ruta = os.path.join(carpeta, 'resumen_aire.json')
    if os.path.exists(ruta):
        aire = json.load(open(ruta))['juntas']
        print('\nCarga del piso (duty piso / duty aire, a la misma velocidad):')
        for r in RUEDAS:
            print('  %-3s %.2f / %.2f = x%.2f' % (r, res[r]['duty'], aire[r]['duty'], res[r]['duty'] / aire[r]['duty']))
    else:
        print('(no hay resumen_aire.json para comparar: corre antes la prueba "aire" con el mismo --carpeta)')


def prueba_metro(b, resumen):
    global DIAMETRO_M
    v_ms = V_TEST_FLANCOS / flancos_por_m()
    objetivo_flancos = 1.0 * flancos_por_m()
    print('Robot en el piso, en la marca de salida, con 1,5 m libres. Se avanza 1 m calculado = %.1f flancos por rueda.'
          % objetivo_flancos)
    corridas = []
    while True:
        input('Enter para avanzar...')
        base = list(b.ticks[-1][1])

        def distancia_flancos():
            return sum(abs(a - c) for a, c in zip(b.ticks[-1][1], base)) / 4.0

        b.tramo(lambda: b.orden_vel(v_ms), 15.0, hasta=lambda: distancia_flancos() >= objetivo_flancos)
        b.parar(2.5)                         # que termine de rodar
        final = list(b.ticks[-1][1])
        por_rueda = [abs(a - c) for a, c in zip(final, base)]
        media = sum(por_rueda) / 4.0
        calculado = media / flancos_por_m()
        print('Flancos por rueda: %s  (media %.1f)  -> %.3f m calculados' % (por_rueda, media, calculado))
        real = input('Cuantos metros avanzo de verdad? (Enter = salir): ').strip().replace(',', '.')
        if not real:
            break
        real = float(real)
        k = real / calculado
        corridas.append({'flancos': por_rueda, 'calculado_m': calculado, 'real_m': real, 'k': k})
        print('Factor real/calculado = %.4f  -> diametro efectivo %.2f mm (ahora %.2f mm)' % (
            k, DIAMETRO_M * k * 1000, DIAMETRO_M * 1000))
        if input('Otra corrida? (s/n): ').strip().lower() != 's':
            break
    if corridas:
        k_med = sum(c['k'] for c in corridas) / len(corridas)
        resumen['corridas'] = corridas
        resumen['k_medio'] = k_med
        resumen['diametro_efectivo_mm'] = DIAMETRO_M * k_med * 1000
        print('\nPromedio de %d corridas: k = %.4f, diametro efectivo = %.2f mm.' % (len(corridas), k_med,
                                                                                    DIAMETRO_M * k_med * 1000))
        print('Poner ese diametro en DIAMETRO_RUEDA_M del firmware (en metros) y en DIAMETRO_M de este script.')


def prueba_escalones(b, resumen):
    niveles = [25.0, 40.0, 55.0, 70.0, 40.0, 0.0]
    print('Escalones de velocidad (flancos/s): %s, 6 s cada uno, las 4 ruedas juntas.' % niveles)
    for nivel in niveles:
        v_ms = nivel / flancos_por_m()
        t0, t1 = b.tramo(lambda v=v_ms: b.orden_vel(v), 6.0)
        es = [(t, d) for t, d in b.estado if t0 <= t <= t1]
        info = {}
        for i, r in enumerate(RUEDAS):
            serie = [(t - t0, d[i]) for t, d in es]
            fin = [v for t, v in serie if t > 4.0]
            estable = sum(fin) / len(fin) if fin else float('nan')
            subida = next((t for t, v in serie if nivel and v >= 0.9 * nivel), float('nan'))
            pico = max((v for t, v in serie), default=float('nan'))
            info[r] = {'estable': estable, 'subida_s': subida, 'pico': pico}
        resumen['escalon_%g' % nivel] = info
        print('\nEscalon a %g flancos/s:' % nivel)
        for r in RUEDAS:
            d = info[r]
            err = 100.0 * (d['estable'] - nivel) / nivel if nivel else float('nan')
            sobre = 100.0 * (d['pico'] - nivel) / nivel if nivel else float('nan')
            print('  %-3s estable %6.1f (%+5.1f%%)  subida al 90%% %5.2f s  sobreimpulso %+5.1f%%' % (
                r, d['estable'], err, d['subida_s'], sobre))
    b.parar()


def main():
    global DIAMETRO_M
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('prueba', choices=['aire', 'piso', 'metro', 'escalones'])
    ap.add_argument('--carpeta', default=os.path.expanduser('~/caid/mediciones'))
    ap.add_argument('--diametro-mm', type=float, default=DIAMETRO_M * 1000.0,
                    help='diametro usado para convertir flancos en metros (igual al del firmware)')
    args = ap.parse_args()
    DIAMETRO_M = args.diametro_mm / 1000.0

    rclpy.init()
    nombre = '%s_%s' % (datetime.datetime.now().strftime('%Y%m%d_%H%M%S'), args.prueba)
    b = Banco(args.carpeta, nombre)
    resumen = {'prueba': args.prueba, 'diametro_mm': DIAMETRO_M * 1000.0, 'v_test_flancos': V_TEST_FLANCOS}
    try:
        if not b.esperar_enlace():
            return
        {'aire': lambda: prueba_aire(b, resumen),
         'piso': lambda: prueba_piso(b, resumen, args.carpeta),
         'metro': lambda: prueba_metro(b, resumen),
         'escalones': lambda: prueba_escalones(b, resumen)}[args.prueba]()
    except KeyboardInterrupt:
        print('\nInterrumpido: frenando.')
    finally:
        b.parar(1.0)
        b.cerrar()
        ruta_json = os.path.join(args.carpeta, 'resumen_%s.json' % args.prueba)
        with open(ruta_json, 'w') as f:
            json.dump(resumen, f, indent=2)
        print('\nDatos crudos: %s\nResumen: %s' % (b.ruta_csv, ruta_json))
        b.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
