"""
Misión de prueba: buscar los cubos de a uno, cargarlo, sacarlo del laberinto y volver a entrar.

Explora celda por celda con los 3 ultrasonidos y la odometría de las ruedas. El robot arranca en
la entrada, mirando hacia adentro: el lado de atrás de la celda de inicio es la salida y no se
explora. Busca los colores en el orden del parámetro colores. Cuando ve un cubo de cerca con la
cámara (o le avisa /nav/cubo_verde en los mundos con el cubo en el piso), al llegar al centro de
esa celda pide la action /cargar (caid_interfaces/action/Cargar) y muestra su avance. Con el cubo
arriba vuelve a la entrada, sale una celda (la entrega), vuelve a entrar y sigue con el próximo.
Los cubos de otros colores que ve en el camino quedan anotados para después; el mapa no se borra.

Flechas: de las detecciones de la cámara (/vision/detecciones) guarda las de los tres colores y al
decidir usa solo las del color que busca ahora.
Cada una se ubica en una celda del mapa con la geometría de la cámara (camara.ubicar_flecha) y,
al decidir en esa celda, su dirección va antes que la regla de exploración (Mapa.decidir).

Lee:     odometría (nav_msgs/Odometry), ultrasonidos frente/izquierda/derecha (sensor_msgs/Range)
         /nav/cubo_verde (geometry_msgs/Point, posición del cubo en el marco de la odometría)
         y /vision/detecciones (String, JSON de caid_vision)
Publica: /cmd_vel (Twist), /nav/mapa (String, dibujo del mapa),
         /nav/mapa_rviz (MarkerArray) y la TF map -> base_footprint con la pose corregida
Los nombres de los tópicos son parámetros: por defecto apuntan a la simulación (/sim/...).

Si se pierde (la odometría falla), no se detiene:
1. Relocalización: en una celda conocida, si 2 o más lados medidos contradicen el mapa, prueba las
   celdas vecinas (±1) y se corre a la única donde todo coincide.
2. Referencias: un canasto ya ubicado que la cámara ve en otra celda (3 veces) corrige la pose.
3. Volver a mapear: si no se relocaliza 3 veces seguidas o no conoce un camino, borra el mapa y
   explora otra vez desde donde está, sin perder lo entregado ni lo que lleva.

No hay IMU: el rumbo sale de la odometría y se corrige con las paredes.
- Posición lateral: con las distancias a las paredes de los costados.
- Posición longitudinal: con la distancia a la pared de enfrente.
- Rumbo: con la pendiente de la distancia lateral mientras el robot avanza.

Máquina de estados: ESPERA -> MEDIR -> (GIRAR -> MEDIR | AVANZAR | CARGAR) ... -> TERMINADO
Modo de la misión:   BUSCAR (explorar o ir al cubo) -> VOLVER (a la entrada) -> SALIR (una celda
afuera: entrega) -> ENTRAR (a la entrada) -> BUSCAR el próximo ... -> TERMINADO
"""
from collections import Counter, deque
import json
import math
import statistics

from caid_interfaces.action import Cargar
from caid_navigation import geometria
from caid_navigation.camara import Camara, ubicar_flecha
from caid_navigation.mapa import Mapa, PASOS, vecina
from caid_navigation.marcadores import marcadores
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray
import yaml

LADOS = ('frente', 'izquierda', 'derecha')
CANASTO_ANCHO = 0.044    # m, Canasto_Ficticio.stl
NOMBRE_DIR = ('+x', '+y', '-x', '-y')


class VistaDeParedes:
    """Lo que camara.ubicar_flecha necesita saber del mapa: dónde hay pared y qué es libre."""

    bloques = frozenset()
    bloqueos = frozenset()

    def __init__(self, mapa):
        self.mapa = mapa

    def abierto(self, celda, d):
        return self.mapa.abierto(celda, d)

    @property
    def libres(self):
        return self.mapa.visitadas | {
            vecina(c, d) for (c, d), paso in self.mapa.lados.items() if paso}


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
            'detecciones_topic': '/vision/detecciones',
            'colores': ['verde', 'rojo', 'azul'],   # orden en que se buscan los cubos
            # Entrega en el canasto de su color, que se busca con la cámara. False = entrega en
            # la celda de atrás de la partida (mundos sin canastos, como chico y maze_2).
            'canastos_con_camara': False,
            'confirmar_canasto': 2,     # imágenes con el canasto en la misma celda para anotarlo
            # Flecha: con la primera imagen cercana queda anotada en su celda (hasta un remapeo).
            # Dirección y largo se siguen reevaluando: vale la mayoría de las últimas
            # ventana_flecha imágenes cercanas.
            'ventana_flecha': 10,
            # De lejos la flecha solo se ve: dirección y tamaño se votan recién con la cámara a
            # esta distancia (m) o menos de la pared de la flecha.
            'distancia_decision_flecha': 0.50,
            'tam_celda': 0.45,          # m, lado de la celda del laberinto
            'sensor_frente_x': 0.1399,  # m, desde el centro del robot (dimensions.yaml)
            'sensor_lateral_y': 0.0815,
            'umbral_pared': 0.30,       # m, más cerca que esto = pared en esa celda
            'vel_lineal': 0.20,         # m/s
            'vel_angular': 1.0,         # rad/s
            'frenar_frente': 0.07,      # m, distancia al frente para frenar sí o sí
            'tiempo_medir': 0.3,        # s quieto antes de medir
            'corregir_rumbo': True,
            'area_cubo_cerca': 500.0,   # px², cubo de 20 mm a ~0,5 m: "lo tengo enfrente"
            # Celdas [x_min, x_max, y_min, y_max] en el marco de la odometría; afuera es pared.
            # Todo en 0 = sin límites. Hace falta con áreas abiertas sin paredes (E6).
            'limites': [0, 0, 0, 0],
            # YAML de calibración de la cámara real (K a 640x480, ver hardware/camara). Vacío =
            # FOV de 60° de la simulación.
            'calibracion': '',
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        self.pub_cmd = self.create_publisher(Twist, self.cfg['cmd_vel_topic'], 10)
        self.pub_mapa = self.create_publisher(String, '/nav/mapa', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cargar = ActionClient(self, Cargar, '/cargar')
        self.pub_rviz = self.create_publisher(MarkerArray, '/nav/mapa_rviz', latched)
        # /nav/estado: qué busca, flechas anotadas y la última decisión de giro (JSON)
        self.pub_estado = self.create_publisher(String, '/nav/estado', latched)
        self.ultima_decision = None
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.cfg['odom_topic'], self.on_odom, 10)
        self.create_subscription(Point, self.cfg['cubo_topic'], self.on_cubo, 10)
        self.create_subscription(String, self.cfg['detecciones_topic'], self.on_detecciones, 10)
        self.lecturas = {lado: deque(maxlen=3) for lado in LADOS}
        for lado in LADOS:
            self.create_subscription(
                Range, self.cfg[lado + '_topic'],
                lambda m, lado=lado: self.lecturas[lado].append(m.range), 10)

        self.odom = None                  # (x, y, yaw) crudos
        self.corr = [0.0, 0.0, 0.0]       # corrección sumada a la odometría
        self.canastos = {}                # color -> celda del canasto, vista con la cámara
        self.votos_canasto = {}           # (color, celda) -> imágenes que lo mostraron ahí
        self.perdidas = 0                 # celdas seguidas que no coinciden con el mapa
        self.remapeos = 0                 # veces que borró el mapa sin poder seguir
        limites = tuple(self.cfg['limites'])
        self.mapa = Mapa(limites=limites if any(limites) else None)
        self.mapa.cerrar((0, 0), 2)       # atrás está la salida: no se explora
        self.celda = (0, 0)
        self.modo = 'BUSCAR'
        self.cubos = {}                   # color -> celda donde se carga (frente al cubo)
        self.dir_cubo = {}                # color -> hacia dónde quedaba el cubo desde esa celda
        self.vistos = set()               # colores vistos de cerca desde la última celda
        self.entregados = []
        self.agotados = set()             # colores que no aparecieron explorando todo
        self.llevando = None              # color del cubo cargado
        self.rastro = []                  # poses corregidas, para RViz
        self.cam = self.crear_camara(self.cfg['calibracion'])
        self.vista = VistaDeParedes(self.mapa)
        self.flechas = {}                 # celda -> {color: dirección de la flecha}
        self.vistas_flecha = {}           # (celda, color) -> últimas (dirección, largo) vistas
        self.largo_flecha = {}            # (celda, color) -> largo fijo: 1 = P3, 2 = P2, 3 = P1
        geometria.fijar_celda(self.cfg['tam_celda'])
        self.estado = 'ESPERA'
        self.t_estado = self.ahora()
        self.objetivo = None              # celda (AVANZAR) o dirección (GIRAR)
        self.muestras_rumbo = []
        self.carga = None                 # None, 'enviada' o el resultado (True / False)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publicar_rviz)

    def crear_camara(self, ruta):
        """Cámara con la K calibrada (si hay YAML) o con el FOV de la simulación."""
        if not ruta:
            return Camara()
        with open(ruta) as fh:
            d = yaml.safe_load(fh)
        k = [float(x) for x in d['K']]
        self.get_logger().info(f'Calibración de cámara: fx={k[0]:.1f} fy={k[4]:.1f} '
                               f'cx={k[2]:.1f} cy={k[5]:.1f} ({d["width"]}x{d["height"]})')
        return Camara(ancho=d['width'], alto=d['height'], fx=k[0], fy=k[4], cx=k[2], cy=k[5])

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
        if 'verde' not in self.cubos and 'verde' not in self.entregados \
                and self.llevando != 'verde':
            tam = self.cfg['tam_celda']
            self.cubos['verde'] = (round(msg.x / tam), round(msg.y / tam))
            self.get_logger().info(f'Cubo verde a la vista, en la celda {self.cubos["verde"]}')

    def buscado(self):
        """Color que se busca ahora: el primero que ya se ubicó o, si no, el primero que falta."""
        faltan = [c for c in self.cfg['colores'] if c not in self.entregados
                  and c != self.llevando and c not in self.agotados]
        ubicados = [c for c in faltan if c in self.cubos]
        return (ubicados or faltan or [None])[0]

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

    def on_detecciones(self, msg):
        if self.odom is None or self.estado in ('ESPERA', 'CARGAR'):
            return
        det = json.loads(msg.data)
        for c in det.get('cubos', []):
            col = c['color']
            if col not in self.cfg['colores'] or col in self.entregados or col in self.cubos \
                    or col == self.llevando or col in self.vistos \
                    or c['area'] < self.cfg['area_cubo_cerca']:
                continue
            self.vistos.add(col)
            self.get_logger().info(
                f'Cámara: cubo {col} enfrente ({c["area"]:.0f} px²), lo anoto en la próxima celda')
        # Girando, la pose de la imagen no es confiable; la cámara queda siempre al frente.
        if self.estado == 'GIRAR':
            return
        for k in det.get('canastos', []):
            self.ubicar_canasto(k)
        for f in det.get('flechas', []):
            if f['color'] not in self.cfg['colores']:
                continue
            r = ubicar_flecha(self.vista, self.cam, self.pose(), 0.0, f)
            if r is None:
                continue
            self.votar_flecha(r['celda'], f['color'], r['dir'], r['distancia'], r['largo'])

    def votar_flecha(self, celda, color, direccion, distancia, largo=None):
        """
        Suma una lectura cercana de la flecha.

        La flecha queda anotada con la primera y no se borra; dirección y largo son la mayoría de
        las últimas ventana_flecha lecturas (se siguen reevaluando). De lejos (más de
        distancia_decision_flecha) no vota: la medida no es confiable.
        """
        if distancia > self.cfg['distancia_decision_flecha']:
            return                          # se ve, pero todavía está lejos para decidir
        lecturas = self.vistas_flecha.setdefault(
            (celda, color), deque(maxlen=self.cfg['ventana_flecha']))
        lecturas.append((direccion, largo))
        dirs = Counter(d for d, _ in lecturas)
        largos = Counter(lg for _, lg in lecturas if lg is not None)
        nueva = dirs.most_common(1)[0][0]
        nuevo_largo = largos.most_common(1)[0][0] if largos else None
        pistas = self.flechas.setdefault(celda, {})
        antes = (pistas.get(color), self.largo_flecha.get((celda, color)))
        pistas[color] = nueva
        self.largo_flecha[(celda, color)] = nuevo_largo
        if antes != (nueva, nuevo_largo):
            nombre = {1: 'P3', 2: 'P2', 3: 'P1'}.get(nuevo_largo, '?')
            que = 'anotada' if antes[0] is None else 'cambia'
            self.get_logger().info(
                f'Flecha {color} {nombre} en la celda {celda}: apunta a {NOMBRE_DIR[nueva]} '
                f'(a {distancia:.2f} m, {dirs[nueva]} de {len(lecturas)} lecturas) -> {que}')

    def ubicar_canasto(self, k):
        """
        Celda del canasto a partir de su caja en la imagen.

        La distancia sale del ancho (el canasto mide 44 mm) y la dirección, de la columna del
        centro de la caja. Con la pose del robot da un punto del mapa; su celda se anota cuando
        la misma aparece en confirmar_canasto imágenes.
        """
        col = k['color']
        if col not in self.cfg['colores']:
            return
        x, _, w, _ = k['caja']
        ang = math.atan((self.cam.cx - (x + w / 2)) / self.cam.fx)      # + a la izquierda
        dist = self.cam.fx * CANASTO_ANCHO / max(w, 1) / math.cos(ang) + CANASTO_ANCHO / 2
        pose = self.pose()
        ox, oy, _ = self.cam.origen(pose, 0.0)
        tam = self.cfg['tam_celda']
        px = ox + dist * math.cos(pose[2] + ang)
        py = oy + dist * math.sin(pose[2] + ang)
        celda = (round(px / tam), round(py / tam))
        clave = (col, celda)
        self.votos_canasto[clave] = self.votos_canasto.get(clave, 0) + 1
        conocido = self.canastos.get(col)
        if conocido is None and self.votos_canasto[clave] >= self.cfg['confirmar_canasto']:
            self.canastos[col] = celda
            self.get_logger().info(f'Cámara: canasto {col} en la celda {celda} (a {dist:.2f} m)')
        elif conocido not in (None, celda) and self.votos_canasto[clave] >= 3 \
                and self.estado == 'MEDIR':
            # Los canastos no se mueven: si se lo ve en otra celda, el que se corrió es el robot.
            self.votos_canasto[clave] = 0
            self.desplazar(conocido[0] - celda[0], conocido[1] - celda[1],
                           f'el canasto {col}')

    # ---------- recuperación ----------

    def desplazar(self, dx, dy, motivo):
        """Corre la pose estimada (dx, dy) celdas: el robot estaba en otra celda."""
        tam = self.cfg['tam_celda']
        self.corr[0] += dx * tam
        self.corr[1] += dy * tam
        self.celda = (self.celda[0] + dx, self.celda[1] + dy)
        if self.estado == 'AVANZAR':
            self.objetivo = (self.objetivo[0] + dx, self.objetivo[1] + dy)
        self.get_logger().warn(f'Me relocalicé con {motivo}: estoy en la celda {self.celda}')

    def relocalizar(self, rumbo, hay_paso):
        """Busca la única celda vecina visitada donde lo medido coincide con el mapa."""
        candidatas = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                c = (self.celda[0] + dx, self.celda[1] + dy)
                if (dx, dy) == (0, 0) or c not in self.mapa.visitadas:
                    continue
                coinciden, contradicen = self.mapa.comparar(c, rumbo, *hay_paso)
                if contradicen == 0 and coinciden >= 2:
                    candidatas.append((dx, dy))
        if len(candidatas) != 1:
            return False
        self.desplazar(*candidatas[0], 'las paredes')
        return True

    def remapear(self, motivo):
        """Borra el mapa y explora otra vez desde acá; la misión (entregas, carga) sigue."""
        self.remapeos += 1
        self.perdidas = 0
        self.get_logger().warn(f'{motivo}: borro el mapa y vuelvo a mapear desde la celda '
                               f'{self.celda} (vez {self.remapeos})')
        self.mapa.reiniciar()
        for tabla in (self.cubos, self.dir_cubo, self.canastos, self.votos_canasto,
                      self.flechas, self.vistas_flecha, self.largo_flecha):
            tabla.clear()
        self.agotados.clear()
        self.cambiar('MEDIR')

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
            self.tick_cargar()

    def registrar_y_decidir(self):
        dist = [self.distancia(lado) for lado in LADOS]
        hay_paso = [m > self.cfg['umbral_pared'] for m in dist]
        rumbo = self.rumbo()
        if self.celda in self.mapa.visitadas:
            _, contradicen = self.mapa.comparar(self.celda, rumbo, *hay_paso)
            if contradicen >= 2 and not self.relocalizar(rumbo, hay_paso):
                self.perdidas += 1
                if self.perdidas >= 3:
                    self.remapear('Lo que mido no coincide con el mapa')
                    return
            elif contradicen == 0:
                self.perdidas = 0
        self.mapa.registrar(self.celda, rumbo, *hay_paso)
        self.pub_mapa.publish(String(data=self.mapa.dibujo()))
        for col in self.vistos:
            if col not in self.cubos:
                self.cubos[col] = self.celda
                self.dir_cubo[col] = rumbo
                self.get_logger().info(f'Cubo {col}: se carga desde la celda {self.celda}')
        self.vistos.clear()

        col = self.buscado()
        if self.modo == 'BUSCAR' and col is None:
            self.get_logger().info('No queda ningún cubo por buscar: salgo del laberinto')
            self.modo = 'VOLVER'
        if self.modo == 'BUSCAR' and self.cubos.get(col) == self.celda:
            self.get_logger().info(f'Llegué al cubo {col}: pido /cargar')
            self.carga = None
            self.cambiar('CARGAR', col)
            return
        if self.modo == 'VOLVER' and self.celda == self.mapa.inicio:
            self.get_logger().info('En la entrada: salgo del laberinto')
            self.modo = 'SALIR'
            self.mapa.abrir(self.mapa.inicio, 2)
        # Entrega: en la celda del canasto de su color, si ya lo vio la cámara. Mientras no lo
        # encuentre, afuera es None y explora (en SALIR). Sin canastos: la celda de atrás.
        afuera = vecina(self.mapa.inicio, 2)
        if self.cfg['canastos_con_camara'] and self.llevando:
            afuera = self.canastos.get(self.llevando)
        if self.modo == 'SALIR' and self.celda == afuera:
            if self.llevando:
                self.entregados.append(self.llevando)
                self.get_logger().info(f'Entregué el cubo {self.llevando} '
                                       f'({len(self.entregados)} de {len(self.cfg["colores"])})')
                self.llevando = None
            if self.mapa.queda_por_explorar():
                self.agotados.clear()     # hay lugares nuevos: los que no aparecieron, otra vez
            if self.buscado() is None:
                self.get_logger().info('Misión terminada, afuera del laberinto:\n'
                                       + self.mapa.dibujo())
                self.cambiar('TERMINADO')
                self.publicar_rviz()
                return
            self.get_logger().info(f'Vuelvo a entrar a buscar el cubo {self.buscado()}')
            self.modo = 'ENTRAR'
        if self.modo == 'ENTRAR' and self.celda == self.mapa.inicio:
            self.mapa.cerrar(self.mapa.inicio, 2)
            self.modo = 'BUSCAR'
            col = self.buscado()
            self.get_logger().info(f'Adentro otra vez: busco el cubo {col}')

        destino = {'BUSCAR': self.cubos.get(col), 'VOLVER': self.mapa.inicio,
                   'SALIR': afuera, 'ENTRAR': self.mapa.inicio}[self.modo]
        preferida = self.flechas.get(self.celda, {}).get(col) if self.modo == 'BUSCAR' else None
        d = self.mapa.decidir(self.celda, rumbo, destino, preferida)
        if d is None and destino is not None \
                and self.modo in ('BUSCAR', 'SALIR', 'VOLVER', 'ENTRAR'):
            # Sé dónde está el cubo (o el canasto) pero todavía no conozco un camino: exploro.
            d = self.mapa.decidir(self.celda, rumbo, preferida=preferida)
        if preferida is not None and d == preferida:
            self.get_logger().info(f'Sigo la flecha {col} hacia {NOMBRE_DIR[d]}')
        if d is None:
            motivo = 'nada'
        elif preferida is not None and d == preferida:
            motivo = f'flecha {col}'
        elif destino is not None and self.mapa.camino(self.celda, lambda c: c == destino):
            motivo = 'camino al destino'
        else:
            motivo = 'tremaux' + (f' (flecha {col} ignorada: ese camino ya se recorrió o '
                                  'tiene pared)' if preferida is not None else '')
        self.ultima_decision = {
            'celda': list(self.celda), 'rumbo': NOMBRE_DIR[rumbo], 'modo': self.modo,
            'busca': col, 'gira_hacia': NOMBRE_DIR[d] if d is not None else None,
            'motivo': motivo,
            'flecha_en_celda': NOMBRE_DIR[preferida] if preferida is not None else None}
        self.publicar_estado()
        self.get_logger().info(
            f'{self.modo} {col or ""} celda {self.celda} rumbo {rumbo} paso {hay_paso} '
            f'({", ".join(f"{m:.3f}" for m in dist)}) -> {d}')
        if d is None and self.modo == 'BUSCAR':
            self.get_logger().warn(f'Exploré todo y no encontré el cubo {col}: queda para '
                                   'después')
            self.agotados.add(col)
            if self.buscado() is None:
                self.modo = 'VOLVER'
            self.registrar_y_decidir()
        elif d is None and self.modo == 'SALIR' and self.llevando and afuera is None:
            self.get_logger().warn(f'No encontré el canasto {self.llevando}: lo entrego en la '
                                   'celda de atrás de la partida')
            self.canastos[self.llevando] = vecina(self.mapa.inicio, 2)
            self.registrar_y_decidir()
        elif d is None and self.remapeos < 3:
            self.remapear(f'{self.modo}: no conozco el camino')
        elif d is None:
            self.get_logger().error(f'{self.modo}: me perdí {self.remapeos} veces, me detengo')
            self.cambiar('TERMINADO')
        elif d == rumbo and self.mapa.abierto(self.celda, d):
            self.cambiar('AVANZAR', vecina(self.celda, d))
        else:
            self.cambiar('GIRAR', d)

    def tick_cargar(self):
        if self.carga is None:
            if not self.cargar.server_is_ready():
                return      # todavía no arrancó el servidor de /cargar
            self.carga = 'enviada'
            pedido = Cargar.Goal(color=self.objetivo)
            envio = self.cargar.send_goal_async(pedido, feedback_callback=self.on_avance_carga)
            envio.add_done_callback(self.on_carga_aceptada)
        elif self.carga is True:
            col = self.objetivo
            self.get_logger().info(f'Cubo {col} cargado: vuelvo a la entrada')
            self.llevando = col
            del self.cubos[col]
            # Se fue el pilar: lo medido en esta celda y en la del cubo se vuelve a medir.
            self.mapa.olvidar(self.celda)
            if col in self.dir_cubo:
                self.mapa.olvidar(vecina(self.celda, self.dir_cubo.pop(col)))
            self.modo = 'VOLVER'
            self.cambiar('MEDIR')
        elif self.carga is False:
            self.get_logger().warn(f'No se pudo cargar el cubo {self.objetivo}: sigo explorando')
            self.cubos.pop(self.objetivo, None)
            self.cambiar('MEDIR')

    def on_avance_carga(self, msg):
        f = msg.feedback
        if f.etapa != getattr(self, 'etapa_carga', None) or f.progreso >= 1.0:
            self.etapa_carga = f.etapa
            self.get_logger().info(f'Carga: {f.etapa} ({f.progreso * 100:.0f} %)')

    def on_carga_aceptada(self, envio):
        meta = envio.result()
        if not meta.accepted:
            self.carga = False
            return
        meta.get_result_async().add_done_callback(
            lambda fut: setattr(self, 'carga', bool(fut.result().result.exito)))

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
            self.mapa.recorrer(self.celda, d)
            self.celda = self.objetivo
            self.registrar_y_decidir()
            return
        if self.distancia('frente') < self.cfg['frenar_frente']:
            # Pared inesperada: me quedo en la celda donde estoy y vuelvo a medir.
            self.get_logger().warn('Pared inesperada adelante, freno')
            celda = (round(x / tam), round(y / tam))
            if celda == self.objetivo:
                self.mapa.recorrer(self.celda, d)
            else:
                # No llegó a entrar: ese paso queda cerrado aunque después se lo mida libre.
                self.mapa.bloquear(self.celda, d)
            self.celda = celda
            self.cambiar('MEDIR')
            return

        v = min(self.cfg['vel_lineal'], max(0.04, 1.5 * s))
        w = 3.0 * (math.atan(-2.0 * e) - d_th)
        self.mover(v, limitar(w, self.cfg['vel_angular']))
        self.corregir_posicion(d)

    def publicar_estado(self):
        """JSON en /nav/estado: qué busca, en qué orden, flechas anotadas y última decisión."""
        nombres = {1: 'P3', 2: 'P2', 3: 'P1'}
        flechas = [{'celda': list(c), 'color': col, 'apunta': NOMBRE_DIR[d],
                    'largo': nombres.get(self.largo_flecha.get((c, col)))}
                   for c, pistas in sorted(self.flechas.items()) for col, d in pistas.items()]
        estado = {
            'busca': self.buscado(), 'orden_colores': list(self.cfg['colores']),
            'entregados': list(self.entregados), 'llevando': self.llevando,
            'modo': self.modo, 'estado': self.estado, 'celda': list(self.celda),
            'flechas': flechas, 'ultima_decision': self.ultima_decision,
        }
        self.pub_estado.publish(String(data=json.dumps(estado, ensure_ascii=False)))

    def publicar_rviz(self):
        self.publicar_estado()
        self.pub_rviz.publish(marcadores(
            self.mapa, self.cfg['tam_celda'], self.rastro,
            self.cubos.get(self.buscado()) if self.modo == 'BUSCAR' else None,
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
