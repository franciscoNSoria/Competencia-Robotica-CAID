"""
Misión de prueba: entrar al laberinto, encontrar el cubo verde, cargarlo y volver a salir.

Explora celda por celda con los 3 ultrasonidos y la odometría de las ruedas. El robot arranca en
la entrada, mirando hacia adentro: el lado de atrás de la celda de inicio es la salida y no se
explora. Cuando le avisan dónde está el cubo va hasta él, simula la carga y vuelve a salir.

Lee:     odometría (nav_msgs/Odometry), ultrasonidos frente/izquierda/derecha (sensor_msgs/Range)
         y /nav/cubo_verde (geometry_msgs/Point, posición del cubo en el marco de la odometría)
Publica: /cmd_vel (Twist), /nav/carga (Bool), /nav/mapa (String, dibujo del mapa),
         /nav/mapa_rviz (MarkerArray) y la TF map -> base_footprint con la pose corregida
Los nombres de los tópicos son parámetros: por defecto apuntan a la simulación (/sim/...).

No hay IMU: el rumbo sale de la odometría y se corrige con las paredes.
- Posición lateral: con las distancias a las paredes de los costados.
- Posición longitudinal: con la distancia a la pared de enfrente.
- Rumbo: con la pendiente de la distancia lateral mientras el robot avanza.

Máquina de estados: ESPERA -> MEDIR -> (GIRAR -> MEDIR | AVANZAR | CARGAR) ... -> TERMINADO
Modo de la misión:   BUSCAR (explorar o ir al cubo) -> VOLVER (a la entrada) -> SALIR (una celda
afuera)
"""
from collections import deque
import math
import statistics

from caid_navigation.mapa import Mapa, PASOS, vecina
from caid_navigation.marcadores import marcadores
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

LADOS = ('frente', 'izquierda', 'derecha')


def ang_dif(a, b):
    """Devuelve a - b llevado a [-pi, pi)."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def limitar(x, tope):
    return max(-tope, min(tope, x))


class Explorador(Node):

    def __init__(self):
        super().__init__('explorador')
        p = {
            'cmd_vel_topic': '/sim/cmd_vel',
            'odom_topic': '/sim/odom',
            'frente_topic': '/sim/ultrasonic/front',
            'izquierda_topic': '/sim/ultrasonic/left',
            'derecha_topic': '/sim/ultrasonic/right',
            'cubo_topic': '/nav/cubo_verde',
            'tam_celda': 0.45,          # m, lado de la celda del laberinto
            'sensor_frente_x': 0.1399,  # m, desde el centro del robot (dimensions.yaml)
            'sensor_lateral_y': 0.0815,
            'umbral_pared': 0.30,       # m, más cerca que esto = pared en esa celda
            'vel_lineal': 0.20,         # m/s
            'vel_angular': 1.0,         # rad/s
            'frenar_frente': 0.07,      # m, distancia al frente para frenar sí o sí
            'tiempo_medir': 0.3,        # s quieto antes de medir
            'corregir_rumbo': True,
            'tiempo_cargar': 2.0,       # s, simulación de la carga del cubo
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        self.pub_cmd = self.create_publisher(Twist, self.cfg['cmd_vel_topic'], 10)
        self.pub_mapa = self.create_publisher(String, '/nav/mapa', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_carga = self.create_publisher(Bool, '/nav/carga', latched)
        self.pub_rviz = self.create_publisher(MarkerArray, '/nav/mapa_rviz', latched)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, 10)
        self.create_subscription(Point, self.cfg['cubo_topic'], self.on_cubo, 10)
        self.lecturas = {lado: deque(maxlen=3) for lado in LADOS}
        for lado in LADOS:
            self.create_subscription(
                Range, self.cfg[lado + '_topic'],
                lambda m, lado=lado: self.lecturas[lado].append(m.range), 10)

        self.odom = None                  # (x, y, yaw) crudos
        self.corr = [0.0, 0.0, 0.0]       # corrección sumada a la odometría
        self.mapa = Mapa()
        self.mapa.marcar_lado((0, 0), 2, False)   # atrás está la salida: no se explora
        self.celda = (0, 0)
        self.modo = 'BUSCAR'
        self.cubo = None                  # celda del cubo, cuando se lo vio
        self.rastro = []                  # poses corregidas, para RViz
        self.estado = 'ESPERA'
        self.t_estado = self.ahora()
        self.objetivo = None              # celda (AVANZAR) o dirección (GIRAR)
        self.muestras_rumbo = []
        self.pub_carga.publish(Bool(data=False))
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publicar_rviz)

    # ---------- entradas ----------

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        x, y, th = self.pose()
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(th / 2)
        tf.transform.rotation.w = math.cos(th / 2)
        self.tf.sendTransform(tf)
        if not self.rastro or math.dist(self.rastro[-1], (x, y)) > 0.05:
            self.rastro.append((x, y))

    def on_cubo(self, msg):
        if self.cubo is None and self.modo == 'BUSCAR':
            tam = self.cfg['tam_celda']
            self.cubo = (round(msg.x / tam), round(msg.y / tam))
            self.get_logger().info(f'Cubo verde a la vista, en la celda {self.cubo}')

    def pose(self):
        x, y, th = self.odom
        return x + self.corr[0], y + self.corr[1], th + self.corr[2]

    def distancia(self, lado):
        """Mediana de las últimas 3 lecturas; inf si no hay eco."""
        datos = self.lecturas[lado]
        return statistics.median(datos) if datos else math.inf

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rumbo(self):
        return round(self.pose()[2] / (math.pi / 2)) % 4

    # ---------- estados ----------

    def cambiar(self, estado, objetivo=None):
        self.estado = estado
        self.objetivo = objetivo
        self.t_estado = self.ahora()
        self.muestras_rumbo = []
        if estado in ('MEDIR', 'GIRAR', 'CARGAR', 'TERMINADO'):
            self.mover(0.0, 0.0)
        if estado == 'MEDIR':
            for d in self.lecturas.values():
                d.clear()

    def tick(self):
        if self.estado == 'ESPERA':
            if self.odom and all(self.lecturas.values()):
                self.get_logger().info('Sensores y odometría listos: empiezo a explorar')
                self.cambiar('MEDIR')
        elif self.estado == 'MEDIR':
            self.corregir_posicion(self.rumbo())
            if self.ahora() - self.t_estado >= self.cfg['tiempo_medir'] and \
                    all(len(d) == 3 for d in self.lecturas.values()):
                self.registrar_y_decidir()
        elif self.estado == 'GIRAR':
            self.tick_girar()
        elif self.estado == 'AVANZAR':
            self.tick_avanzar()
        elif self.estado == 'CARGAR':
            if self.ahora() - self.t_estado >= self.cfg['tiempo_cargar']:
                self.pub_carga.publish(Bool(data=True))
                self.get_logger().info('Cubo verde cargado: vuelvo a la entrada')
                self.modo = 'VOLVER'
                self.cambiar('MEDIR')

    def registrar_y_decidir(self):
        hay_paso = [self.distancia(lado) > self.cfg['umbral_pared'] for lado in LADOS]
        rumbo = self.rumbo()
        self.mapa.registrar(self.celda, rumbo, *hay_paso)
        if self.modo != 'SALIR':
            self.mapa.marcar_lado(self.mapa.inicio, 2, False)   # la salida no se explora
        self.pub_mapa.publish(String(data=self.mapa.dibujo()))

        if self.modo == 'BUSCAR' and self.celda == self.cubo:
            self.get_logger().info('Llegué al cubo verde: cargando')
            self.cambiar('CARGAR')
            return
        if self.modo == 'VOLVER' and self.celda == self.mapa.inicio:
            self.get_logger().info('En la entrada: salgo del laberinto')
            self.modo = 'SALIR'
            self.mapa.marcar_lado(self.mapa.inicio, 2, True)

        destino = {'BUSCAR': self.cubo, 'VOLVER': self.mapa.inicio,
                   'SALIR': vecina(self.mapa.inicio, 2)}[self.modo]
        d = self.mapa.decidir(self.celda, rumbo, destino)
        self.get_logger().info(
            f'{self.modo} celda {self.celda} rumbo {rumbo} paso {hay_paso} -> {d}')
        if d is None and self.modo == 'BUSCAR':
            self.get_logger().warn('Exploré todo y no encontré el cubo: vuelvo a la entrada')
            self.modo = 'VOLVER'
            self.registrar_y_decidir()
        elif d is None:
            self.get_logger().info('Misión terminada, afuera del laberinto:\n'
                                   + self.mapa.dibujo())
            self.cambiar('TERMINADO')
            self.publicar_rviz()
        elif d == rumbo and self.mapa.abierto(self.celda, d):
            self.cambiar('AVANZAR', vecina(self.celda, d))
        else:
            self.cambiar('GIRAR', d)

    def tick_girar(self):
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

        if s < 0.01:
            self.celda = self.objetivo
            self.registrar_y_decidir()
            return
        if self.distancia('frente') < self.cfg['frenar_frente']:
            # Pared inesperada: me quedo en la celda donde estoy y vuelvo a medir.
            self.get_logger().warn('Pared inesperada adelante, freno')
            self.celda = (round(x / tam), round(y / tam))
            self.cambiar('MEDIR')
            return

        v = min(self.cfg['vel_lineal'], max(0.04, 1.5 * s))
        w = 3.0 * (math.atan(-2.0 * e) - d_th)
        self.mover(v, limitar(w, self.cfg['vel_angular']))
        self.corregir_posicion(d)

    def publicar_rviz(self):
        self.pub_rviz.publish(marcadores(
            self.mapa, self.cfg['tam_celda'], self.rastro,
            None if self.modo != 'BUSCAR' else self.cubo,
            self.get_clock().now().to_msg()))

    def mover(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    # ---------- correcciones con las paredes ----------

    def corregir_posicion(self, d):
        """Acerca la pose de la odometría a lo que dicen las paredes, de a poco."""
        u, n = PASOS[d], PASOS[(d + 1) % 4]
        x, y, th = self.pose()
        tam = self.cfg['tam_celda']
        d_th = ang_dif(th, d * math.pi / 2)
        if abs(d_th) > 0.15:
            return
        a = x * u[0] + y * u[1]
        lat = x * n[0] + y * n[1]

        # Lateral: con el robot centrado, cada sensor ve la pared a tam/2 - sensor_lateral_y.
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
            self.estimar_rumbo(a, e_med, d_th)
        else:
            self.muestras_rumbo = []

        # Longitudinal: la pared de enfrente está en un borde de celda, (k + 1/2) * tam.
        f = self.distancia('frente')
        if f < 0.40:
            pared = a + self.cfg['sensor_frente_x'] + f
            pared = (round(pared / tam - 0.5) + 0.5) * tam
            a_med = pared - self.cfg['sensor_frente_x'] - f
            if abs(a_med - a) < 0.1:
                self.corr[0] += 0.1 * (a_med - a) * u[0]
                self.corr[1] += 0.1 * (a_med - a) * u[1]

    def estimar_rumbo(self, a, e_med, d_th):
        """
        Si el rumbo de la odometría está corrido en delta, el robot se va acercando a una pared.

        Con 25 cm de avance junto a pared se ajusta la recta e(a): su pendiente es el seno del
        rumbo real respecto del pasillo, y delta = rumbo estimado - rumbo real.
        """
        if self.estado != 'AVANZAR' or not self.cfg['corregir_rumbo']:
            return
        self.muestras_rumbo.append((a, e_med, d_th))
        m = self.muestras_rumbo
        if abs(m[-1][0] - m[0][0]) < 0.25:
            return
        n = len(m)
        a_med = sum(s[0] for s in m) / n
        e_prom = sum(s[1] for s in m) / n
        var = sum((s[0] - a_med) ** 2 for s in m)
        pendiente = sum((s[0] - a_med) * (s[1] - e_prom) for s in m) / var
        # La pendiente se mide sobre el avance a lo largo de u, que crece al avanzar.
        if abs(pendiente) < 0.3:
            delta = sum(s[2] for s in m) / n - math.asin(pendiente)
            self.corr[2] -= 0.3 * delta
        self.muestras_rumbo = []


def main():
    rclpy.init()
    node = Explorador()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.mover(0.0, 0.0)
    node.destroy_node()
    rclpy.try_shutdown()
