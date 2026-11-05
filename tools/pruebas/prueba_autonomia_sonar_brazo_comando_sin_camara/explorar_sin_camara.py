#!/usr/bin/env python3
"""
Explora un laberinto SOLO con los 3 sonares (sin cámara, sin flechas) hasta el fondo, se frena
a `distancia_parada` de la pared, y espera que escribas `y` + Enter en esta terminal (mientras
vos agarrás el cubo con tools/brazo/brazo_sliders.py). Confirmado, vuelve a la entrada.

Es una prueba de banco (ver el README de esta carpeta), no la estrategia de competencia: el
agarre a mano está prohibido en la prueba real (Categoría A, autonomía completa).

La exploración y la corrección de pose contra las paredes son una copia de
caid_navigation/explorador.py (MEDIR/GIRAR/AVANZAR y corregir_posicion), sin lo que necesita
cámara (flechas, cubos, canastos, servo de la cámara). Si el explorador completo cambia esa
lógica, no se actualiza sola acá: este archivo es para esta prueba puntual.

Lee:     odom_topic (nav_msgs/Odometry), frente/izquierda/derecha_topic (sensor_msgs/Range)
Publica: cmd_vel_topic (Twist), /nav/mapa (String, dibujo del mapa)
"""
import math
import statistics
import sys
import threading
from collections import deque

from caid_navigation.mapa import Mapa, PASOS
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

LADOS = ('frente', 'izquierda', 'derecha')


def ang_dif(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def limitar(x, tope):
    return max(-tope, min(tope, x))


class ExplorarSinCamara(Node):

    def __init__(self):
        super().__init__('explorar_sin_camara')
        p = {
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'frente_topic': '/distancia/frente',
            'izquierda_topic': '/distancia/izquierda',
            'derecha_topic': '/distancia/derecha',
            'tam_celda': 0.40,
            'sensor_frente_x': 0.1399,
            'sensor_lateral_y': 0.0815,
            'umbral_pared': 0.25,
            'distancia_parada': 0.03,    # m, a esto del sonar frontal se frena (3 cm pedidos)
            'vel_lineal': 0.15,
            'vel_angular': 0.8,
            'frenar_giro': 0.03,         # m; igual que el freno de seguridad de explorador.py
            'tiempo_medir': 0.3,
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        self.pub_cmd = self.create_publisher(Twist, self.cfg['cmd_vel_topic'], 10)
        self.pub_mapa = self.create_publisher(String, '/nav/mapa', 10)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, 10)
        self.lecturas = {lado: deque(maxlen=3) for lado in LADOS}
        for lado in LADOS:
            self.create_subscription(
                Range, self.cfg[lado + '_topic'],
                lambda m, lado=lado: self.lecturas[lado].append(m.range), 10)

        self.odom = None
        self.corr = [0.0, 0.0, 0.0]
        self.mapa = Mapa()
        self.mapa.cerrar((0, 0), 2)      # atrás está la salida: no se vuelve a explorar sola
        self.celda = (0, 0)
        self.estado = 'ESPERA'
        self.modo = 'EXPLORAR'           # EXPLORAR -> FRENANDO -> ESPERANDO -> VOLVIENDO -> FIN
        self.t_estado = self.ahora()
        self.objetivo = None
        self.muestras_rumbo = []
        self.agarrado = threading.Event()
        self._ult_debug_avanzar = 0.0
        threading.Thread(target=self._leer_teclado, daemon=True).start()
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Arranco: exploro con sonar, sin cámara ni flechas')

    def _leer_teclado(self):
        print("\n>>> Cuando hayas agarrado el cubo con los sliders, escribí 'y' y Enter acá <<<\n")
        while rclpy.ok():
            try:
                linea = input()
            except EOFError:
                return
            if linea.strip().lower() == 'y':
                self.agarrado.set()

    # ---------- lo mismo que explorador.py ----------

    def pose(self):
        x, y, th = self.odom
        return x + self.corr[0], y + self.corr[1], th + self.corr[2]

    def distancia(self, lado):
        datos = self.lecturas[lado]
        return statistics.median(datos) if datos else math.inf

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rumbo(self):
        return round(self.pose()[2] / (math.pi / 2)) % 4

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

    def mover(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    def cambiar(self, estado, objetivo=None):
        self.estado = estado
        self.objetivo = objetivo
        self.t_estado = self.ahora()
        self.muestras_rumbo = []
        if estado in ('MEDIR', 'GIRAR', 'TERMINADO'):
            self.mover(0.0, 0.0)
        if estado == 'MEDIR':
            for d in self.lecturas.values():
                d.clear()

    def corregir_posicion(self, d):
        u, n = PASOS[d], PASOS[(d + 1) % 4]
        x, y, th = self.pose()
        tam = self.cfg['tam_celda']
        d_th = ang_dif(th, d * math.pi / 2)
        if abs(d_th) > 0.15:
            return
        a = x * u[0] + y * u[1]
        lat = x * n[0] + y * n[1]
        nominal = tam / 2 - self.cfg['sensor_lateral_y']
        medidas = []
        izq, der = self.distancia('izquierda'), self.distancia('derecha')
        if izq < self.cfg['umbral_pared']:
            medidas.append(nominal - izq)
        if der < self.cfg['umbral_pared']:
            medidas.append(der - nominal)
        if medidas:
            e_med = sum(medidas) / len(medidas)
            centro = round(lat / tam) * tam
            delta = 0.1 * (centro + e_med - lat)
            self.corr[0] += delta * n[0]
            self.corr[1] += delta * n[1]
        f = self.distancia('frente')
        if f < 0.40:
            pared = a + self.cfg['sensor_frente_x'] + f
            pared = (round(pared / tam - 0.5) + 0.5) * tam
            a_med = pared - self.cfg['sensor_frente_x'] - f
            if abs(a_med - a) < 0.1:
                self.corr[0] += 0.1 * (a_med - a) * u[0]
                self.corr[1] += 0.1 * (a_med - a) * u[1]

    def tick_girar(self):
        cerca = min(self.distancia(lado) for lado in LADOS)
        if cerca < self.cfg['frenar_giro']:
            self.get_logger().warn(f'Girando, sonar a {cerca:.3f} m: freno y vuelvo a medir')
            self.mover(0.0, 0.0)
            self.cambiar('MEDIR')
            return
        th = self.pose()[2]
        err = ang_dif(self.objetivo * math.pi / 2, th)
        if abs(err) < 0.03:
            self.cambiar('MEDIR')
            return
        w = limitar(2.0 * err, self.cfg['vel_angular'])
        self.mover(0.0, math.copysign(max(abs(w), 0.25), w))

    def tick_avanzar(self):
        d = self.rumbo()
        u, n = PASOS[d], PASOS[(d + 1) % 4]
        x, y, th = self.pose()
        tam = self.cfg['tam_celda']
        s = (self.objetivo[0] * u[0] + self.objetivo[1] * u[1]) * tam - (x * u[0] + y * u[1])
        e = (x * n[0] + y * n[1]) - (self.objetivo[0] * n[0] + self.objetivo[1] * n[1]) * tam
        d_th = ang_dif(th, d * math.pi / 2)
        v = min(self.cfg['vel_lineal'], max(0.04, 1.5 * s))
        w = limitar(3.0 * (math.atan(-2.0 * e) - d_th), self.cfg['vel_angular'])
        if self.ahora() - self._ult_debug_avanzar >= 1.0:
            self._ult_debug_avanzar = self.ahora()
            self.get_logger().info(
                f'  [avanzar] objetivo={self.objetivo} d={d} pose=({x:.3f},{y:.3f},{th:.3f}) '
                f's={s:.3f} e={e:.3f} d_th={d_th:.3f} v={v:.3f} w={w:.3f} '
                f'frente={self.distancia("frente"):.3f}')
        if s < 0.01:
            self.mapa.recorrer(self.celda, d)
            self.celda = self.objetivo
            self.registrar_y_decidir()
            return
        if self.distancia('frente') < 0.07:
            self.get_logger().warn('Pared inesperada adelante, freno')
            celda = (round(x / tam), round(y / tam))
            if celda != self.objetivo:
                self.mapa.bloquear(self.celda, d)
            self.celda = celda
            self.cambiar('MEDIR')
            return
        self.mover(v, w)
        self.corregir_posicion(d)

    def tick_retroceder(self):
        """Antes de girar para volver: alejarse de la pared, si no el freno de GIRAR no suelta
        nunca (está a propósito a `distancia_parada` de la pared, adentro de `frenar_giro`)."""
        if self.distancia('frente') > self.cfg['frenar_giro'] + 0.07:
            self.mover(0.0, 0.0)
            self.cambiar('MEDIR')
            return
        self.mover(-min(self.cfg['vel_lineal'], 0.08), 0.0)

    def tick_acercarse(self):
        """Avanza recto hasta que el sonar frontal mida distancia_parada."""
        d = self.rumbo()
        f = self.distancia('frente')
        if f <= self.cfg['distancia_parada']:
            self.mover(0.0, 0.0)
            self.modo = 'ESPERANDO'
            self.get_logger().info(
                f"Frenado a {f:.3f} m del frente. Agarrá el cubo con los sliders y escribí 'y'")
            return
        self.mover(min(self.cfg['vel_lineal'], max(0.03, 1.0 * (f - self.cfg['distancia_parada']))),
                   0.0)
        self.corregir_posicion(d)

    # ---------- la misión de esta prueba ----------

    def registrar_y_decidir(self):
        dist = [self.distancia(lado) for lado in LADOS]
        hay_paso = [m > self.cfg['umbral_pared'] for m in dist]
        rumbo = self.rumbo()
        self.mapa.registrar(self.celda, rumbo, *hay_paso)
        self.pub_mapa.publish(String(data=self.mapa.dibujo()))
        self.get_logger().info(
            f'{self.modo} celda {self.celda} rumbo {rumbo} paso {hay_paso} '
            f'({", ".join(f"{m:.3f}" for m in dist)})')

        if self.modo == 'EXPLORAR':
            if not hay_paso[0] and not self.mapa.queda_por_explorar():
                self.get_logger().info('Nada más por explorar y pared adelante: asumo el fondo')
                self.modo = 'ACERCANDO'
                self.cambiar('ACERCAR')
                return
            d = self.mapa.decidir(self.celda, rumbo)
            if d is None:
                self.get_logger().warn('No sé a dónde ir: me quedo quieto')
                self.cambiar('TERMINADO')
                return
        else:   # VOLVIENDO
            if self.celda == self.mapa.inicio:
                self.get_logger().info('De vuelta en la entrada. Fin de la prueba')
                self.cambiar('TERMINADO')
                return
            d = self.mapa.decidir(self.celda, rumbo, destino=self.mapa.inicio)
            if d is None:
                self.get_logger().warn('No conozco el camino de vuelta: me quedo quieto')
                self.cambiar('TERMINADO')
                return

        if d == rumbo and self.mapa.abierto(self.celda, d):
            self.cambiar('AVANZAR', (self.celda[0] + PASOS[d][0], self.celda[1] + PASOS[d][1]))
        else:
            self.cambiar('GIRAR', d)

    def tick(self):
        if self.estado == 'ESPERA':
            if self.odom and all(self.lecturas.values()):
                self.get_logger().info('Sensores y odometría listos: empiezo a explorar')
                self.cambiar('MEDIR')
        elif self.estado == 'MEDIR':
            self.corregir_posicion(self.rumbo())
            listo = all(len(d) == 3 for d in self.lecturas.values())
            if self.ahora() - self._ult_debug_avanzar >= 1.0:
                self._ult_debug_avanzar = self.ahora()
                largos = {lado: len(self.lecturas[lado]) for lado in LADOS}
                self.get_logger().info(
                    f'  [medir] hace {self.ahora() - self.t_estado:.2f}s, listo={listo}, '
                    f'muestras={largos}')
            if self.ahora() - self.t_estado >= self.cfg['tiempo_medir'] and listo:
                self.registrar_y_decidir()
        elif self.estado == 'GIRAR':
            self.tick_girar()
        elif self.estado == 'AVANZAR':
            self.tick_avanzar()
        elif self.estado == 'ACERCAR':
            self.tick_acercarse()
            if self.modo == 'ESPERANDO':
                self.estado = 'ESPERANDO'
        elif self.estado == 'RETROCEDER':
            self.tick_retroceder()
        elif self.estado == 'ESPERANDO':
            if self.agarrado.is_set():
                self.get_logger().info('Confirmado: me alejo de la pared y vuelvo a la entrada')
                self.modo = 'VOLVIENDO'
                self.cambiar('RETROCEDER')


def main():
    rclpy.init()
    node = ExplorarSinCamara()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.mover(0.0, 0.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
