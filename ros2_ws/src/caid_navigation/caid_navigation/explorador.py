"""
Misión de prueba: buscar los cubos de a uno, cargarlo, sacarlo del laberinto y volver a entrar.

Explora celda por celda con los 3 ultrasonidos y la odometría de las ruedas. El robot arranca en
la zona de partida, junto a los canastos, mirando hacia la entrada del laberinto. Busca los
colores en el orden del parámetro colores. Cuando ve un cubo de cerca con la cámara (o le avisa
/nav/cubo_verde en los mundos con el cubo en el piso), al llegar al centro de esa celda pide la
action /cargar (caid_interfaces/action/Cargar) y muestra su avance. Con el cubo arriba sale a la
zona de partida, lo deja en el canasto de su color desde la celda de al lado, vuelve a entrar
hasta el punto de reingreso y sigue con el próximo. Los cubos de otros colores que ve en el
camino quedan anotados para después; el mapa no se borra.

Para probar la lógica sin Gazebo: tools/sim/mision_rapida.py corre este mismo nodo en segundos.

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

Cámara sobre servo: en un cruce, antes de decidir, la gira a varios ángulos (MIRAR) para leer
las flechas de las paredes de los costados, que de frente no se ven. Las flechas reales son
chicas y solo se leen a menos de 0,45-0,80 m, así que mirar de frente nada más pierde la mitad.

Partida, puerta y punto de reingreso (la zona de partida en la pista real es abierta, sin
paredes de referencia):
- Al arrancar (RECONOCER) mira alrededor con el servo y media vuelta para ubicar los canastos,
  con la pose perfecta de la partida. Los canastos solo se aceptan con el robot en la zona.
- La puerta es la primera celda con pared a los dos lados. Buscando no sale por ahí.
- El punto de reingreso es la primera bifurcación pasando la puerta, con el rumbo que traía.
  Toda ruta hacia adentro pasa por ahí. Después de cada entrega vuelve, se orienta igual,
  verifica las paredes contra el mapa y sigue buscando.

Máquina de estados: ESPERA -> (RECONOCER) -> MEDIR -> (MIRAR -> MEDIR | GIRAR -> MEDIR |
AVANZAR | CARGAR) ... -> TERMINADO
Modo de la misión:   BUSCAR (explorar o ir al cubo) -> VOLVER (a la partida) -> SALIR (al lado
del canasto de su color: entrega) -> ENTRAR (al punto de reingreso) -> BUSCAR el próximo ...
-> TERMINADO
"""
from collections import deque
import json
import math
import statistics

from caid_interfaces.action import Cargar, Dejar
from caid_navigation import geometria
from caid_navigation.camara import Camara, ubicar_flecha
from caid_navigation.mapa import Mapa, PASOS, seguir_guia, vecina
from caid_navigation.marcadores import marcadores
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

LADOS = ('frente', 'izquierda', 'derecha')
CANASTO_ANCHO = 0.044    # m, Canasto_Ficticio.stl
CANASTO_ALTO = 0.020     # m
CUBO_LADO = 0.020        # m, reglamento
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
            'camara_pan_topic': '/sim/camera_pan/cmd',
            'joint_states_topic': '/sim/joint_states',
            # En un cruce, girar la cámara para leer las flechas de las paredes de los costados:
            # la flecha solo se lee a menos de 0,45-0,80 m, así que de frente se pierden las que
            # están en las paredes laterales. False = la cámara mira siempre al frente.
            'usar_servo_camara': True,
            'angulos_camara': [0.0, 0.95, -0.95],   # rad; con 60° de FOV cubren unos ±84°
            'tiempo_servo': 1.0,        # s, tope de espera a que el servo llegue
            'tiempo_mirar': 0.3,        # s mirando en cada ángulo, ya frenado el servo
            'tolerancia_servo': 0.05,   # rad para darlo por llegado
            'colores': ['verde', 'rojo', 'azul'],   # orden en que se buscan los cubos
            # Entrega en el canasto de su color, que se busca con la cámara. False = entrega en
            # la celda de atrás de la partida (mundos sin canastos, como chico y maze_2).
            'canastos_con_camara': False,
            'confirmar_canasto': 2,     # imágenes con el canasto en la misma celda para anotarlo
            'confirmar_flecha': 2,      # imágenes iguales para usar una flecha
            # Usar el largo de la flecha (1, 2 o 3 tramos rectos hasta el objetivo) para seguir
            # el camino varias celdas sin desviarse por las ramas laterales. False = la flecha
            # solo manda en la celda donde se la ve.
            'usar_largo_flecha': True,
            'tam_celda': 0.45,          # m, lado de la celda del laberinto
            'sensor_frente_x': 0.1399,  # m, desde el centro del robot (dimensions.yaml)
            'sensor_lateral_y': 0.0815,
            'umbral_pared': 0.30,       # m, más cerca que esto = pared en esa celda
            'vel_lineal': 0.20,         # m/s
            'vel_angular': 1.0,         # rad/s
            # m, si algún sonar mide menos que esto GIRANDO, frena: significa que el robot no
            # está centrado (la celda da margen para girar en el lugar, pero poco: con 400 mm de
            # celda y 322 mm de diagonal del robot sobran solo 39 mm por lado). Un valor bajo
            # (0,03) solo frena ante un choque ya encima; es un último recurso, no evita el golpe
            # inicial, pero evita seguir empujando contra la pared.
            'frenar_giro': 0.03,
            # m, distancia al frente para frenar sí o sí. 0 = automática: 2 cm menos que lo que
            # mide el sensor de frente con el robot centrado y pared adelante. Con 0,07 fijo y
            # celda de 0,40 (que da 0,060 centrado) frenaba en cada fondo de pasillo.
            'frenar_frente': 0.0,
            # Al arrancar, mirar alrededor (servo y media vuelta) para ubicar los canastos con la
            # pose perfecta de la partida. Solo con canastos_con_camara.
            'reconocer_al_inicio': True,
            'canasto_dist_max': 1.8,    # m; más lejos, el ancho en píxeles no alcanza para medir
            # Altura de la base del canasto sobre el piso (el borde a 108,5 y 20 mm de alto). La
            # distancia al canasto sale de la altura de su borde de abajo en la imagen.
            'canasto_base_z': 0.0885,
            # Acercamiento para agarrar o dejar: el robot frena sobre el eje del pasillo donde el
            # objeto queda a esta distancia del eje de la base del brazo. Con los límites del
            # firmware el brazo agarra a la altura del cubo entre 193 y 237 mm
            # (caid_manipulation/cinematica.py). Con 205 mm, un cubo de frente queda a 3 cm del
            # paragolpes, como pidió el equipo.
            'alcance_brazo': 0.205,
            'eje_brazo_x': -0.0213,     # m, eje de giro de la base del brazo en base_link
            'vel_acercar': 0.08,        # m/s
            'tiempo_medir': 0.3,        # s quieto antes de medir
            'corregir_rumbo': True,
            'area_cubo_cerca': 500.0,   # px², cubo de 20 mm a ~0,5 m: "lo tengo enfrente"
            # Celdas [x_min, x_max, y_min, y_max] en el marco de la odometría; afuera es pared.
            # Todo en 0 = sin límites. Hace falta con áreas abiertas sin paredes (E6).
            'limites': [0, 0, 0, 0],
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}
        if not self.cfg['frenar_frente']:
            self.cfg['frenar_frente'] = (self.cfg['tam_celda'] / 2 - self.cfg['sensor_frente_x']
                                         - 0.02)

        self.pub_cmd = self.create_publisher(Twist, self.cfg['cmd_vel_topic'], 10)
        self.pub_mapa = self.create_publisher(String, '/nav/mapa', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cargar = ActionClient(self, Cargar, '/cargar')
        self.dejar = ActionClient(self, Dejar, '/dejar')
        self.accion = None                # 'cargar' o 'dejar' mientras CARGAR
        self.pos_cubo = {}                # color -> posiciones estimadas por la cámara (mapa)
        self.pos_canasto = {}             # color -> posiciones estimadas por la cámara (mapa)
        self.pub_rviz = self.create_publisher(MarkerArray, '/nav/mapa_rviz', latched)
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
        self.celda = (0, 0)
        # La zona de partida y el laberinto se separan por la puerta: la primera celda con pared
        # a los dos lados. Buscando, no se sale por ahí (no se explora la zona abierta).
        self.puerta = None                # (celda de adentro, dirección hacia afuera)
        self.adentro = False              # el robot arranca en la zona de partida
        # Punto de reingreso: la primera bifurcación pasando la puerta, con el rumbo que traía.
        # Toda ruta hacia adentro pasa por ahí, así que ir no cuesta nada, y como tiene paredes
        # sirve para volver a anclar la pose después de andar por la zona abierta.
        self.ancla = None                 # (celda, rumbo)
        self.ancla_es_cruce = False       # False mientras el ancla es la puerta (provisoria)
        self.despues = None               # estado al que vuelve MIRAR o GIRAR (por RECONOCER)
        self.plan_inicio = []
        self.modo = 'BUSCAR'
        self.cubos = {}                   # color -> celda donde se carga (frente al cubo)
        self.dir_cubo = {}                # color -> hacia dónde quedaba el cubo desde esa celda
        self.vistos = set()               # colores vistos de cerca desde la última celda
        self.entregados = []
        self.agotados = set()             # colores que no aparecieron explorando todo
        self.llevando = None              # color del cubo cargado
        self.rastro = []                  # poses corregidas, para RViz
        self.cam = Camara()
        self.vista = VistaDeParedes(self.mapa)
        self.pub_pan = self.create_publisher(Float64, self.cfg['camara_pan_topic'], 10)
        self.create_subscription(JointState, self.cfg['joint_states_topic'], self.on_joints, 10)
        self.pan = 0.0                    # ángulo real del servo de la cámara
        self.pan_pedido = 0.0
        self.barrido = []                 # ángulos que faltan mirar en esta celda
        self.t_asentado = None            # cuándo llegó el servo al ángulo pedido
        self.mirado = set()               # (celda, color) ya barridos con la cámara
        self.flechas = {}                 # celda -> {color: (dirección, largo) de la flecha}
        self.guia = None                  # flecha que se está siguiendo: {color, dir, tramos}
        # Flechas ya seguidas en esta búsqueda: (celda, color). Una flecha manda aunque el paso
        # ya esté recorrido, pero una sola vez por búsqueda: así no hay ciclos.
        self.seguidas = set()
        self.vistas_flecha = {}           # (celda, dirección) -> imágenes que la mostraron
        geometria.fijar_celda(self.cfg['tam_celda'])
        self.estado = 'ESPERA'
        self.t_estado = self.ahora()
        self.objetivo = None              # celda (AVANZAR) o dirección (GIRAR)
        self.muestras_rumbo = []
        self.carga = None                 # None, 'enviada' o el resultado (True / False)
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

    def on_joints(self, msg):
        if 'camera_pan_joint' in msg.name:
            self.pan = msg.position[msg.name.index('camera_pan_joint')]

    def servo_quieto(self):
        """True si el servo ya llegó al ángulo pedido: recién ahí la imagen sirve para medir."""
        return abs(self.pan - self.pan_pedido) < self.cfg['tolerancia_servo']

    def mirar_a(self, angulo):
        self.pan_pedido = float(angulo)
        self.t_asentado = None
        self.pub_pan.publish(Float64(data=self.pan_pedido))

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
            if col in self.cfg['colores'] and col not in self.entregados and col != self.llevando \
                    and self.estado != 'GIRAR' and self.servo_quieto():
                # Dónde está, para acercarse justo antes de agarrar (la celda sola no alcanza).
                px, py, dist = self.ubicar_objeto(c['caja'], CUBO_LADO, CUBO_LADO)
                if dist < 1.0:
                    lista = self.pos_cubo.setdefault(col, [])
                    lista.append((px, py))
                    del lista[:-15]
            if col not in self.cfg['colores'] or col in self.entregados or col in self.cubos \
                    or col == self.llevando or col in self.vistos \
                    or c['area'] < self.cfg['area_cubo_cerca']:
                continue
            self.vistos.add(col)
            self.get_logger().info(
                f'Cámara: cubo {col} enfrente ({c["area"]:.0f} px²), lo anoto en la próxima celda')
        # Girando, la pose de la imagen no es confiable. Y si el servo todavía se está moviendo,
        # el ángulo con el que se proyectaría el rayo no es el de la foto.
        if self.estado == 'GIRAR' or not self.servo_quieto():
            return
        for k in det.get('canastos', []):
            self.ubicar_canasto(k)
        for f in det.get('flechas', []):
            if f['color'] not in self.cfg['colores']:
                continue
            r = ubicar_flecha(self.vista, self.cam, self.pose(), self.pan, f)
            if r is None:
                continue
            clave = (r['celda'], f['color'], r['dir'], r['largo'])
            self.vistas_flecha[clave] = self.vistas_flecha.get(clave, 0) + 1
            pistas = self.flechas.setdefault(r['celda'], {})
            if self.vistas_flecha[clave] >= self.cfg['confirmar_flecha'] \
                    and pistas.get(f['color']) != (r['dir'], r['largo']):
                pistas[f['color']] = (r['dir'], r['largo'])
                tramos = 'tres o más' if r['largo'] >= 3 else str(r['largo'])
                self.get_logger().info(
                    f'Flecha {f["color"]} en la celda {r["celda"]}: apunta a '
                    f'{NOMBRE_DIR[r["dir"]]}, {tramos} tramo(s) recto(s) '
                    f'(a {r["distancia"]:.2f} m)')

    def ubicar_canasto(self, k):
        """
        Celda del canasto a partir de su caja en la imagen.

        La distancia sale del ancho (el canasto mide 44 mm) y la dirección, de la columna del
        centro de la caja. Con la pose del robot da un punto del mapa; su celda se anota cuando
        la misma aparece en confirmar_canasto imágenes.

        Solo con el robot en la zona de partida: los canastos están ahí, y adentro del laberinto
        una flecha o un pedazo de algo del mismo color se confundió con un canasto (pasó en E2:
        un "canasto verde" adentro, y después la relocalización movió al robot 14 celdas).
        """
        col = k['color']
        if col not in self.cfg['colores'] or self.adentro:
            return
        punto = self.ubicar_por_base(k['caja'], self.cfg['canasto_base_z'], CANASTO_ANCHO)
        if punto is None:
            return
        px, py, dist = punto
        if dist > self.cfg['canasto_dist_max']:
            return
        tam = self.cfg['tam_celda']
        celda = (round(px / tam), round(py / tam))
        if self.canastos.get(col) in (None, celda):
            lista = self.pos_canasto.setdefault(col, [])
            lista.append((px, py))
            del lista[:-15]
        self.votar_canasto(col, celda, dist)

    def ubicar_por_base(self, caja, z_base, prof_m):
        """
        Centro de un objeto apoyado a una altura conocida, por la fila de su borde de abajo.

        Para el canasto: la cámara (138,5 mm) lo ve desde arriba (borde a 108,5), así que al alto
        del frente se le suma la cara de arriba y parece un 20 % más alto. El borde de abajo de la
        caja es siempre el borde inferior más cercano, a z_base del piso, se lo mire como se lo
        mire: el rayo de ese píxel cortado con ese plano da el punto. None si está sobre el
        horizonte. Devuelve (x, y, distancia).
        """
        x, y, w, h = caja
        pose = self.pose()
        o, d = self.cam.rayo(pose, self.pan, x + w / 2, y + h)
        # La cámara mide la altura desde el piso (camara.py), igual que z_base.
        if d[2] > -1e-4 or o[2] <= z_base:
            return None
        t = (z_base - o[2]) / d[2]
        nx, ny = o[0] + t * d[0], o[1] + t * d[1]
        horiz = math.hypot(d[0], d[1])
        cx, cy = nx + prof_m / 2 * d[0] / horiz, ny + prof_m / 2 * d[1] / horiz
        return cx, cy, math.hypot(cx - o[0], cy - o[1])

    def ubicar_objeto(self, caja, alto_m, prof_m):
        """
        Punto del mapa del centro de un objeto de alto conocido, a partir de su caja en la imagen.

        La distancia sale del ALTO en píxeles, no del ancho: visto en diagonal, un cubo o un
        canasto muestran dos caras y parecen más anchos (y más cerca). Con el ancho, el cubo de E2
        quedaba estimado 110 mm más cerca de lo que estaba. La dirección sale de la columna del
        centro de la caja; prof_m es el fondo del objeto, para llegar a su centro.
        Devuelve (x, y, distancia).
        """
        x, _, w, h = caja
        ang = math.atan((self.cam.cx - (x + w / 2)) / self.cam.fx)      # + a la izquierda
        dist = self.cam.fy * alto_m / max(h, 1) / math.cos(ang) + prof_m / 2
        pose = self.pose()
        ox, oy, _ = self.cam.origen(pose, self.pan)
        return (ox + dist * math.cos(pose[2] + self.pan + ang),
                oy + dist * math.sin(pose[2] + self.pan + ang), dist)

    def votar_canasto(self, col, celda, dist):
        clave = (col, celda)
        self.votos_canasto[clave] = self.votos_canasto.get(clave, 0) + 1
        conocido = self.canastos.get(col)
        if conocido is None and self.votos_canasto[clave] >= self.cfg['confirmar_canasto']:
            self.canastos[col] = celda
            self.get_logger().info(f'Cámara: canasto {col} en la celda {celda} (a {dist:.2f} m)')
        elif conocido not in (None, celda) and self.votos_canasto[clave] >= 3 \
                and self.estado in ('MEDIR', 'MIRAR'):
            self.votos_canasto[clave] = 0
            dx, dy = conocido[0] - celda[0], conocido[1] - celda[1]
            if max(abs(dx), abs(dy)) > 1:
                # Los canastos no se mueven, pero la odometría tampoco se equivoca en varias
                # celdas dentro de la zona de partida: una de las dos vistas está mal. No se toca.
                self.get_logger().warn(f'Veo el canasto {col} en {celda} y lo tenía en '
                                       f'{conocido}: demasiado lejos para ser un error mío, '
                                       'no me relocalizo')
                return
            # Si se lo ve una celda corrido, el que se corrió es el robot.
            self.desplazar(dx, dy, f'el canasto {col}')

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
        self.guia = None
        self.mirado.clear()
        # Los canastos, la puerta y el punto de reingreso quedan: se ubicaron con buena pose y
        # el marco de la odometría no cambia al borrar el mapa.
        for tabla in (self.cubos, self.dir_cubo, self.votos_canasto,
                      self.flechas, self.vistas_flecha):
            tabla.clear()
        self.agotados.clear()
        self.cambiar('MEDIR')

    # ---------- estados ----------

    def cambiar(self, estado, objetivo=None):
        self.estado = estado
        self.objetivo = objetivo
        self.t_estado = self.ahora()
        self.muestras_rumbo = []
        if estado in ('MEDIR', 'MIRAR', 'GIRAR', 'CARGAR', 'TERMINADO', 'RECONOCER'):
            self.mover(0.0, 0.0)
        if estado == 'MEDIR':
            for d in self.lecturas.values():
                d.clear()

    def tick(self):
        if self.estado == 'ESPERA':
            if self.odom and all(self.lecturas.values()):
                self.get_logger().info('Sensores y odometría listos: empiezo a explorar')
                if self.cfg['reconocer_al_inicio'] and self.cfg['canastos_con_camara'] \
                        and self.cfg['usar_servo_camara']:
                    self.plan_inicio = ['mirar', ('girar', 2), 'mirar', ('girar', 0)]
                    self.cambiar('RECONOCER')
                else:
                    self.cambiar('MEDIR')
        elif self.estado == 'RECONOCER':
            self.tick_reconocer()
        elif self.estado == 'MEDIR':
            self.corregir_posicion(self.rumbo())
            if self.ahora() - self.t_estado >= self.cfg['tiempo_medir'] and \
                    all(len(d) == 3 for d in self.lecturas.values()):
                if self.debe_mirar():
                    self.empezar_barrido()
                else:
                    self.registrar_y_decidir()
        elif self.estado == 'MIRAR':
            self.tick_mirar()
        elif self.estado == 'GIRAR':
            self.tick_girar()
        elif self.estado == 'AVANZAR':
            self.tick_avanzar()
        elif self.estado == 'CARGAR':
            self.tick_cargar()
        elif self.estado in ('ACERCAR', 'ALEJAR'):
            self.tick_eje()

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
                # Con la posición que da la cámara se anota la celda del cubo; sin ella (cubo
                # avisado por /nav/cubo_verde), la de adelante como antes: se carga desde acá.
                celda = self.celda_del_cubo(col)
                if celda is None:
                    self.cubos[col] = self.celda
                    self.dir_cubo[col] = rumbo
                else:
                    self.cubos[col] = celda
                self.get_logger().info(f'Cubo {col}: se carga en la celda {self.cubos[col]}')
        self.vistos.clear()
        self.anotar_puerta_y_ancla(rumbo, hay_paso)

        col = self.buscado()
        if self.modo == 'BUSCAR' and col is None:
            self.get_logger().info('No queda ningún cubo por buscar: salgo del laberinto')
            self.modo = 'VOLVER'
        if self.modo == 'BUSCAR' and self.cubos.get(col) == self.celda:
            if col in self.dir_cubo:
                hacia = self.dir_cubo[col]
            else:
                hacia = self.lado_del_cubo(col, rumbo)
            if hacia != rumbo:
                self.cambiar('GIRAR', hacia)          # de frente al lado del cubo
                return
            self.get_logger().info(f'Llegué al cubo {col}: me acerco para agarrarlo')
            self.acercarse('cargar', col, self.donde_cubo(col, rumbo), rumbo)
            return
        if self.modo == 'VOLVER' and self.celda == self.mapa.inicio:
            self.get_logger().info('En la partida: voy al canasto')
            self.modo = 'SALIR'
        if self.modo == 'SALIR':
            entrega = self.lado_de_entrega(rumbo)
            if entrega is not None and entrega != rumbo:
                self.cambiar('GIRAR', entrega)        # de frente al canasto para dejar el cubo
                return
            if entrega is not None:
                k = self.objetivo_entrega()
                if self.cfg['canastos_con_camara'] and self.llevando and k is not None:
                    self.get_logger().info(f'Junto al canasto {self.llevando}: me acerco para '
                                           'dejar el cubo')
                    self.acercarse('dejar', self.llevando, self.donde_canasto(self.llevando, k),
                                   rumbo)
                    return
                if self.entregar():
                    return
        if self.modo == 'ENTRAR':
            punto = self.ancla or (self.mapa.inicio, None)
            if self.celda == punto[0]:
                if punto[1] is not None and rumbo != punto[1]:
                    self.get_logger().info('En el punto de reingreso: me oriento como la '
                                           'primera vez')
                    self.cambiar('GIRAR', punto[1])
                    return
                self.modo = 'BUSCAR'
                col = self.buscado()
                self.seguidas.clear()       # búsqueda nueva: las flechas vuelven a mandar
                self.get_logger().info(f'En el punto de reingreso {self.celda}, rumbo {rumbo}: '
                                       f'busco el cubo {col}')

        self.ajustar_puerta()
        destino = {'BUSCAR': self.cubos.get(col), 'VOLVER': self.mapa.inicio, 'SALIR': None,
                   'ENTRAR': (self.ancla or (self.mapa.inicio, None))[0]}[self.modo]
        preferida = self.decidir_por_flecha(col)
        insistir = preferida is not None and (self.celda, col) not in self.seguidas
        if self.modo == 'SALIR':
            d = self.paso_hacia_canasto(rumbo)
        else:
            d = self.mapa.decidir(self.celda, rumbo, destino, preferida, insistir)
            if d is None and destino is not None:
                # Sé dónde está el cubo (o el punto) pero todavía no conozco un camino: exploro.
                d = self.mapa.decidir(self.celda, rumbo, preferida=preferida, insistir=insistir)

        if preferida is not None and d == preferida:
            quedan = f", quedan {self.guia['tramos']} tramo(s)" if self.guia else ''
            self.get_logger().info(f'Sigo la flecha {col} hacia {NOMBRE_DIR[d]}{quedan}')
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
        elif d is None and self.modo == 'SALIR' and self.llevando \
                and self.objetivo_entrega() is None:
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

    # ---------- partida, puerta y punto de reingreso ----------

    def tick_reconocer(self):
        """Al arrancar: barrido al frente, media vuelta, barrido atrás y de vuelta al frente."""
        if not self.plan_inicio:
            faltan = [c for c in self.cfg['colores'] if c not in self.canastos]
            self.get_logger().info(f'Reconocimiento terminado: canastos {self.canastos}'
                                   + (f', no vi {faltan}' if faltan else ''))
            self.cambiar('MEDIR')
            return
        paso = self.plan_inicio.pop(0)
        self.despues = 'RECONOCER'
        if paso == 'mirar':
            self.barrido = list(self.cfg['angulos_camara'])
            self.cambiar('MIRAR')
            self.mirar_a(self.barrido.pop(0))
        else:
            self.cambiar('GIRAR', paso[1])

    def anotar_puerta_y_ancla(self, rumbo, hay_paso):
        """La puerta es la primera celda con pared a los dos lados; el ancla, el primer cruce."""
        if self.modo != 'BUSCAR':
            return
        if self.puerta is None:
            if not hay_paso[1] and not hay_paso[2]:
                self.puerta = (self.celda, (rumbo + 2) % 4)
                self.adentro = True
                self.ancla = (self.celda, rumbo)          # provisoria, hasta el primer cruce
                self.get_logger().info(f'Entrada del laberinto en la celda {self.celda}: '
                                       'buscando, no salgo por acá')
            return
        abiertos = sum(1 for d in range(4) if self.mapa.abierto(self.celda, d))
        if not self.ancla_es_cruce and self.adentro and abiertos >= 3:
            self.ancla = (self.celda, rumbo)
            self.ancla_es_cruce = True
            self.get_logger().info(f'Primera bifurcación en {self.celda}, mirando a '
                                   f'{NOMBRE_DIR[rumbo]}: es el punto de reingreso')

    def ajustar_puerta(self):
        """Buscando, la puerta se cierra; afuera, entregando, también: no se mete al laberinto."""
        if self.puerta is None:
            return
        cerrar = self.modo == 'BUSCAR' or (self.modo == 'SALIR' and not self.adentro)
        (self.mapa.cerrar if cerrar else self.mapa.abrir)(*self.puerta)

    def cruzar(self, desde, d):
        """Anota que el robot pasó de desde a su vecina en d y si con eso cruzó la puerta."""
        self.mapa.recorrer(desde, d)
        # La flecha de esta celda queda seguida recién al salir por donde dice (girar hacia ella
        # y volver a medir en la misma celda no cuenta). Si sale por otro lado, la guía ya no vale.
        col = self.buscado()
        pista = self.flechas.get(desde, {}).get(col)
        if pista is not None and pista[0] == d:
            self.seguidas.add((desde, col))
        if self.guia is not None and d != self.guia['dir']:
            self.guia = None
        if self.puerta is None:
            return
        celda, d_afuera = self.puerta
        if desde == celda and d == d_afuera:
            self.adentro = False
            self.get_logger().info('Salí del laberinto por la puerta')
        elif vecina(desde, d) == celda and d == (d_afuera + 2) % 4:
            self.adentro = True
            self.get_logger().info('Entré al laberinto por la puerta')

    def objetivo_entrega(self):
        """Celda del canasto del cubo que lleva (sin canastos reales: la de atrás de la partida)."""
        if self.cfg['canastos_con_camara'] and self.llevando:
            return self.canastos.get(self.llevando)
        return vecina(self.mapa.inicio, 2)

    def lado_de_entrega(self, rumbo):
        """Dirección hacia el canasto si ya está al lado (o el rumbo, si ya está en la celda)."""
        k = self.objetivo_entrega()
        if k is None:
            return None
        if not self.cfg['canastos_con_camara']:
            return rumbo if self.celda == k else None    # mundos sin canastos: se entra a la celda
        for d in range(4):
            if vecina(self.celda, d) == k:
                return d
        return None

    def paso_hacia_canasto(self, rumbo):
        """Primer paso hacia la celda de al lado del canasto, esquivando los otros canastos."""
        k = self.objetivo_entrega()
        if k is None:
            return self.mapa.decidir(self.celda, rumbo)    # todavía no lo vi: exploro la zona
        if self.cfg['canastos_con_camara']:
            otros = set(self.canastos.values())
            ruta = self.mapa.camino(
                self.celda, lambda c: any(vecina(c, d) == k for d in range(4)) and c not in otros,
                optimista=True, evitar=otros)
        else:
            ruta = self.mapa.camino(self.celda, lambda c: c == k, optimista=True)
        return ruta[0] if ruta else None

    def entregar(self):
        """Deja el cubo. Devuelve True si con eso terminó la misión."""
        if self.llevando:
            self.entregados.append(self.llevando)
            self.get_logger().info(f'Entregué el cubo {self.llevando} '
                                   f'({len(self.entregados)} de {len(self.cfg["colores"])})')
            self.llevando = None
        # ¿Queda algo por explorar adentro? Se mira desde el punto de reingreso, con la puerta
        # cerrada: la zona abierta tiene lados sin medir por todos lados y no cuenta.
        if self.puerta:
            self.mapa.cerrar(*self.puerta)
        desde = (self.ancla or (self.mapa.inicio, None))[0]
        if self.mapa.camino(desde, lambda c: c in self.mapa.visitadas
                            and bool(self.mapa.pendientes(c))) is not None:
            self.agotados.clear()     # hay lugares nuevos: los que no aparecieron, otra vez
        if self.buscado() is None:
            self.get_logger().info('Misión terminada, en la zona de partida:\n'
                                   + self.mapa.dibujo())
            self.cambiar('TERMINADO')
            self.publicar_rviz()
            return True
        destino = (self.ancla or (self.mapa.inicio, None))[0]
        self.get_logger().info(f'Vuelvo al punto de reingreso {destino} a buscar el cubo '
                               f'{self.buscado()}')
        self.modo = 'ENTRAR'
        return False

    def debe_mirar(self):
        """
        ¿Vale la pena barrer con la cámara antes de decidir en esta celda?

        Buscando un cubo (o al llegar al punto de reingreso): solo en un cruce, si todavía no se
        barrió esta celda para ese color y no se conoce ya una flecha de ese color acá. De frente
        la cámara solo ve la pared del fondo, y con las flechas chicas el alcance de lectura es
        menos de una celda. En la zona de partida, entregando: si todavía no se sabe dónde está
        el canasto del cubo que lleva.
        """
        if not self.cfg['usar_servo_camara']:
            return False
        if self.modo == 'SALIR' and not self.adentro and self.cfg['canastos_con_camara'] \
                and self.llevando and self.llevando not in self.canastos:
            return (self.celda, 'canasto') not in self.mirado
        en_ancla = self.modo == 'ENTRAR' and self.ancla and self.celda == self.ancla[0] \
            and self.rumbo() == self.ancla[1]
        col = self.buscado()
        if (self.modo != 'BUSCAR' and not en_ancla) or col is None:
            return False
        if (self.celda, col) in self.mirado or col in self.flechas.get(self.celda, {}):
            return False
        if col in self.cubos:
            return False        # ya sé dónde está: voy por el camino más corto, no hace falta
        # Dos de los tres lados libres: una T, una rama lateral o una esquina recién doblada (el
        # pasillo por el que llegó queda al costado). Las flechas están pegadas a las paredes,
        # así que en un cruce de cuatro o en una sala abierta (tres lados libres) no hay dónde
        # leer. La esquina importa por los cubos: uno pegado a la pared lateral de la celda que
        # sigue a una esquina queda a 30,3 grados del eje al terminar de girar, y la cámara ve 30.
        # Sin barrer no entra nunca en cuadro (en E1 completo, sin servo, el robot no veía dos).
        abiertos = sum(1 for lado in LADOS
                       if self.distancia(lado) > self.cfg['umbral_pared'])
        return abiertos == 2

    def empezar_barrido(self):
        # Por color: la flecha del próximo cubo puede estar en la misma pared de este cruce.
        que = 'canasto' if self.modo == 'SALIR' else self.buscado()
        self.mirado.add((self.celda, que))
        self.barrido = [a for a in self.cfg['angulos_camara']]
        self.get_logger().info(
            f'En {self.celda} miro con la cámara a {len(self.barrido)} ángulos'
            + (' buscando el canasto' if que == 'canasto' else ''))
        self.cambiar('MIRAR')
        self.mirar_a(self.barrido.pop(0))

    def tick_mirar(self):
        """Barrido de la cámara: en cada ángulo espera a que el servo pare y mira un rato."""
        ahora = self.ahora()
        if self.t_asentado is None:
            if self.servo_quieto():
                self.t_asentado = ahora
            elif ahora - self.t_estado > self.cfg['tiempo_servo']:
                self.get_logger().warn(
                    f'El servo no llegó a {self.pan_pedido:.2f} rad (está en {self.pan:.2f}), sigo')
                self.t_asentado = ahora
            return
        if ahora - self.t_asentado < self.cfg['tiempo_mirar']:
            return
        if self.barrido:
            self.t_estado = ahora
            self.mirar_a(self.barrido.pop(0))
            return
        if self.pan_pedido != 0.0:
            self.t_estado = ahora
            self.mirar_a(0.0)       # volver al frente antes de moverse
            return
        self.cambiar(self.despues or 'MEDIR')
        self.despues = None

    def decidir_por_flecha(self, col):
        """Dirección que mandan las flechas del color buscado, o None.

        La flecha de esta celda arranca (o reemplaza) la guía. Con usar_largo_flecha, la guía
        sigue valiendo en las celdas siguientes mientras duren los tramos rectos que anunció el
        largo: así el robot no se desvía por cada rama lateral del camino (ver mapa.seguir_guia).
        """
        if self.modo != 'BUSCAR' or col is None:
            self.guia = None
            return None
        pista = self.flechas.get(self.celda, {}).get(col)
        if pista is not None:
            self.guia = {'color': col, 'dir': pista[0], 'tramos': pista[1]}
        if self.guia is None or self.guia['color'] != col:
            self.guia = None
            return None
        if not self.cfg['usar_largo_flecha']:
            return pista[0] if pista is not None else None
        preferida, self.guia = seguir_guia(self.mapa, self.celda, self.guia)
        return preferida

    # ---------- agarrar y dejar ----------

    def celda_del_cubo(self, col):
        lista = self.pos_cubo.get(col)
        if not lista:
            return None
        tam = self.cfg['tam_celda']
        x, y = self.donde_cubo(col, 0)
        return (round(x / tam), round(y / tam))

    def lado_del_cubo(self, col, rumbo):
        """
        Hacia dónde mirar, parado en la celda del cubo, para tenerlo de frente.

        El cubo está sobre su cilindro, pegado a una pared de la celda: el robot mira hacia esa
        pared y lo agarra de frente (3 cm del paragolpes). Si está casi en el centro, sigue como va.
        """
        tam = self.cfg['tam_celda']
        x, y = self.donde_cubo(col, rumbo)
        ox, oy = x - self.celda[0] * tam, y - self.celda[1] * tam
        if math.hypot(ox, oy) < 0.06:
            return rumbo
        return max(range(4), key=lambda d: ox * PASOS[d][0] + oy * PASOS[d][1])

    def donde_cubo(self, col, rumbo):
        """Centro del cubo en el mapa: la mediana de lo que vio la cámara, o la celda siguiente."""
        lista = self.pos_cubo.get(col)
        if lista:
            return (statistics.median(p[0] for p in lista),
                    statistics.median(p[1] for p in lista))
        tam = self.cfg['tam_celda']
        c = vecina(self.celda, rumbo)
        return (c[0] * tam, c[1] * tam)

    def donde_canasto(self, col, celda):
        lista = self.pos_canasto.get(col)
        if lista:
            return (statistics.median(p[0] for p in lista),
                    statistics.median(p[1] for p in lista))
        tam = self.cfg['tam_celda']
        return (celda[0] * tam, celda[1] * tam)

    def acercarse(self, accion, col, punto, rumbo):
        """
        Avanza (o retrocede) por el eje del pasillo hasta tener el objeto al alcance del brazo.

        Con el objeto a "a" metros por delante y "l" al costado del centro de la celda, el robot
        frena donde la distancia del eje de la base del brazo al objeto es alcance_brazo.
        """
        tam = self.cfg['tam_celda']
        u, n = PASOS[rumbo], PASOS[(rumbo + 1) % 4]
        cx, cy = self.celda[0] * tam, self.celda[1] * tam
        a = (punto[0] - cx) * u[0] + (punto[1] - cy) * u[1]
        lat = (punto[0] - cx) * n[0] + (punto[1] - cy) * n[1]
        r = self.cfg['alcance_brazo']
        adelante = math.sqrt(max(r * r - lat * lat, 0.0))
        avance = limitar(a - self.cfg['eje_brazo_x'] - adelante, 0.45)
        self.accion = (accion, col, punto)
        self.carga = None
        self.get_logger().info(f'{accion}: {col} a {a:.2f} m adelante y {lat:+.2f} m al costado; '
                               f'avanzo {avance:+.2f} m')
        self.cambiar('ACERCAR', {'punto': (cx + avance * u[0], cy + avance * u[1]),
                                 'rumbo': rumbo})

    def tick_eje(self):
        """Ir por el eje del pasillo a un punto (adelante o atrás), despacio, sin girar."""
        d = self.objetivo['rumbo']
        u, n = PASOS[d], PASOS[(d + 1) % 4]
        x, y, th = self.pose()
        px, py = self.objetivo['punto']
        s = (px - x) * u[0] + (py - y) * u[1]
        e = (x - px) * n[0] + (y - py) * n[1]
        d_th = ang_dif(th, d * math.pi / 2)
        frente = self.distancia('frente')
        if abs(s) < 0.005 or (s > 0 and frente < 0.015):
            self.mover(0.0, 0.0)
            self.al_llegar_eje()
            return
        sentido = 1.0 if s > 0 else -1.0
        v = sentido * min(self.cfg['vel_acercar'], max(0.02, 1.5 * abs(s)))
        w = 3.0 * (math.atan(-2.0 * e * sentido) - d_th)
        self.mover(v, limitar(w, self.cfg['vel_angular']))
        self.corregir_posicion(d)

    def al_llegar_eje(self):
        if self.estado == 'ALEJAR':
            terminar = self.objetivo.get('entregar', False)
            self.cambiar('MEDIR')
            if terminar:
                self.entregar()
            return
        # Llegó al punto de agarre o de entrega: pide la acción al brazo.
        self.cambiar('CARGAR', self.accion[1])

    def objeto_en_base_link(self, punto):
        x, y, th = self.pose()
        dx, dy = punto[0] - x, punto[1] - y
        c, s = math.cos(th), math.sin(th)
        return c * dx + s * dy, -s * dx + c * dy

    def volver_al_centro(self, entregar=False):
        tam = self.cfg['tam_celda']
        self.cambiar('ALEJAR', {'punto': (self.celda[0] * tam, self.celda[1] * tam),
                                'rumbo': self.rumbo(), 'entregar': entregar})

    def tick_cargar(self):
        accion, col, punto = self.accion
        cliente, tipo = (self.cargar, Cargar) if accion == 'cargar' else (self.dejar, Dejar)
        if self.carga is None:
            if not cliente.server_is_ready():
                return      # todavía no arrancó el servidor de la acción
            self.carga = 'enviada'
            bx, by = self.objeto_en_base_link(punto)
            self.get_logger().info(f'Pido /{accion} {col}: objeto en base_link '
                                   f'({bx:.3f}, {by:.3f})')
            pedido = tipo.Goal(color=col, x=float(bx), y=float(by))
            envio = cliente.send_goal_async(pedido, feedback_callback=self.on_avance_carga)
            envio.add_done_callback(self.on_carga_aceptada)
        elif self.carga is True and accion == 'cargar':
            self.get_logger().info(f'Cubo {col} cargado: vuelvo a la entrada')
            self.llevando = col
            del self.cubos[col]
            self.pos_cubo.pop(col, None)
            # Lo medido en esta celda y en la del cubo se vuelve a medir.
            self.mapa.olvidar(self.celda)
            if col in self.dir_cubo:
                self.mapa.olvidar(vecina(self.celda, self.dir_cubo.pop(col)))
            self.modo = 'VOLVER'
            self.volver_al_centro()
        elif self.carga is True:
            self.get_logger().info(f'Cubo {col} en su canasto')
            self.volver_al_centro(entregar=True)
        elif self.carga is False and accion == 'cargar':
            self.get_logger().warn(f'No se pudo cargar el cubo {col}: sigo explorando')
            self.cubos.pop(col, None)
            self.volver_al_centro()
        elif self.carga is False:
            self.get_logger().warn(f'No se pudo dejar el cubo {col} en el canasto: lo intento '
                                   'de nuevo')
            self.volver_al_centro()

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
        cerca = min(self.distancia(lado) for lado in LADOS)
        if cerca < self.cfg['frenar_giro']:
            self.get_logger().warn(f'Girando, sonar a {cerca:.3f} m: freno y vuelvo a medir '
                                   '(no estoy centrado)')
            self.mover(0.0, 0.0)
            self.cambiar('MEDIR')
            return
        th = self.pose()[2]
        err = ang_dif(self.objetivo * math.pi / 2, th)
        if abs(err) < 0.03:
            self.cambiar(self.despues or 'MEDIR')
            self.despues = None
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
            self.cruzar(self.celda, d)
            self.celda = self.objetivo
            # Llegando a un cruce avanzando también hay que mirar: antes solo se barría desde
            # MEDIR (después de girar), y los cruces a los que se llega de frente se salteaban.
            if self.debe_mirar():
                self.empezar_barrido()
            else:
                self.registrar_y_decidir()
            return
        if self.distancia('frente') < self.cfg['frenar_frente']:
            # Pared inesperada: me quedo en la celda donde estoy y vuelvo a medir.
            self.get_logger().warn('Pared inesperada adelante, freno')
            celda = (round(x / tam), round(y / tam))
            if celda == self.objetivo:
                self.cruzar(self.celda, d)
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

    def publicar_rviz(self):
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
