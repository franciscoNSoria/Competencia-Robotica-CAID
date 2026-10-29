"""
Nodo de la misión completa: explora el laberinto, busca los cubos, los lleva a su canasto y vuelve.

Envoltorio de ROS 2 de mision.Navegador (toda la lógica está ahí, sin ROS, probada con el
simulador 2D). Este nodo solo traduce tópicos.

Lee:
  odom_topic        nav_msgs/Odometry       odometría de las ruedas (marco odom, nace en la partida)
  frente/izquierda/derecha_topic  sensor_msgs/Range   ultrasonidos
  /vision/detecciones  std_msgs/String     JSON de caid_vision (píxeles de flechas, cubos, ...)
  joint_states_topic sensor_msgs/JointState  ángulo real del servo de la cámara
  /mision/resultado  std_msgs/Bool         respuesta del brazo (agarró / soltó)
Publica:
  cmd_vel_topic      geometry_msgs/Twist
  pan_topic          std_msgs/Float64      ángulo pedido al servo de la cámara (rad)
  /mision/pedido     std_msgs/String       JSON {"accion": "agarrar"|"soltar", "color", "x", "y", "z"}
                                           (posición del objeto en base_link, m)
  /nav/estado        std_msgs/String       último mensaje del navegador
  /nav/mapa          std_msgs/String       mapa en texto
  /nav/mapa_rviz     visualization_msgs/MarkerArray
  TF map -> base_footprint (pose corregida con las paredes)
"""
from collections import deque
import json
import math

from caid_navigation.geometria import CELDA
from caid_navigation.marcadores import marcadores
from caid_navigation.mision import Navegador, PARAMETROS
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Bool, Float64, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

LADOS = ('frente', 'izquierda', 'derecha')


class NavegadorNode(Node):

    def __init__(self):
        super().__init__('navegador')
        topicos = {
            'cmd_vel_topic': '/sim/cmd_vel',
            'odom_topic': '/sim/odom',
            'frente_topic': '/sim/ultrasonic/front',
            'izquierda_topic': '/sim/ultrasonic/left',
            'derecha_topic': '/sim/ultrasonic/right',
            'pan_topic': '/sim/camera_pan/cmd',
            'joint_states_topic': '/sim/joint_states',
            'pan_joint': 'camera_pan_joint',
            # Qué cubos indica cada color de flecha (JSON). Ver docs/arquitectura/laberintos.md.
            'significado_colores': '{"rojo": ["rojo"], "verde": ["verde"], "azul": ["azul"], '
                                   '"gris": []}',
        }
        for nombre, valor in topicos.items():
            self.declare_parameter(nombre, valor)
        for nombre, valor in PARAMETROS.items():
            self.declare_parameter(nombre, valor)
        cfg = {n: self.get_parameter(n).value for n in topicos}
        params = {n: self.get_parameter(n).value for n in PARAMETROS}
        self.nav = Navegador(params, json.loads(cfg['significado_colores']))
        self.pan_joint = cfg['pan_joint']

        self.pub_cmd = self.create_publisher(Twist, cfg['cmd_vel_topic'], 10)
        self.pub_pan = self.create_publisher(Float64, cfg['pan_topic'], 10)
        self.pub_pedido = self.create_publisher(String, '/mision/pedido', 10)
        self.pub_estado = self.create_publisher(String, '/nav/estado', 10)
        self.pub_mapa = self.create_publisher(String, '/nav/mapa', 10)
        self.pub_rviz = self.create_publisher(MarkerArray, '/nav/mapa_rviz', 10)
        self.tf = TransformBroadcaster(self)

        self.odom = None
        self.rangos = {lado: math.inf for lado in LADOS}
        self.pan_real = 0.0
        self.historia = deque(maxlen=200)       # (t, pose, pan) para fechar las imágenes
        self.mensajes_vistos = 0
        self.rastro = []
        self.create_subscription(Odometry, cfg['odom_topic'], self.on_odom, 20)
        for lado in LADOS:
            self.create_subscription(
                Range, cfg[lado + '_topic'],
                lambda m, lado=lado: self.rangos.__setitem__(
                    lado, m.range if math.isfinite(m.range) else math.inf),
                qos_profile_sensor_data)
        self.create_subscription(JointState, cfg['joint_states_topic'], self.on_joints, 10)
        self.create_subscription(String, '/vision/detecciones', self.on_vision, 10)
        self.create_subscription(Bool, '/mision/resultado', self.on_resultado, 10)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publicar_mapa)
        self.get_logger().info('Navegador listo: espero odometría y ultrasonidos')

    def ahora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_joints(self, msg):
        if self.pan_joint in msg.name:
            self.pan_real = msg.position[msg.name.index(self.pan_joint)]

    def on_resultado(self, msg):
        self.nav.resultado_manipulacion(msg.data)

    def on_vision(self, msg):
        try:
            datos = json.loads(msg.data)
        except ValueError:
            return
        t_img = datos.get('stamp', self.ahora())
        if not self.historia:
            return
        # Pose y ángulo de cámara en el momento de la imagen (el más cercano guardado).
        t, pose, pan = min(self.historia, key=lambda h: abs(h[0] - t_img))
        if abs(t - t_img) > 0.3:
            return
        self.nav.vision(t, datos, pose, pan)

    def tick(self):
        if self.odom is None:
            return
        t = self.ahora()
        v, w, pan = self.nav.actualizar(t, self.odom, dict(self.rangos), self.pan_real)
        pose = self.nav.pose
        self.historia.append((t, pose, self.pan_real))
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)
        self.pub_pan.publish(Float64(data=float(pan)))
        while self.nav.eventos:
            accion, color, rel = self.nav.eventos.pop(0)
            self.pub_pedido.publish(String(data=json.dumps(
                {'accion': accion, 'color': color, 'x': rel[0], 'y': rel[1], 'z': rel[2]})))
        while self.mensajes_vistos < len(self.nav.mensajes):
            marca, texto = self.nav.mensajes[self.mensajes_vistos]
            self.mensajes_vistos += 1
            self.get_logger().info(f'[{marca:6.1f} s] {texto}')
            self.pub_estado.publish(String(data=texto))
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = pose[0]
        tf.transform.translation.y = pose[1]
        tf.transform.rotation.z = math.sin(pose[2] / 2)
        tf.transform.rotation.w = math.cos(pose[2] / 2)
        self.tf.sendTransform(tf)
        if not self.rastro or math.dist(self.rastro[-1], pose[:2]) > 0.05:
            self.rastro.append(pose[:2])

    def publicar_mapa(self):
        con = self.nav.con
        self.pub_mapa.publish(String(data=con.dibujo(self.nav.pose)))
        cubo = None
        if self.nav.cubos:
            o = next(iter(self.nav.cubos.values()))
            cubo = (round(o.x / CELDA), round(o.y / CELDA))
        self.pub_rviz.publish(marcadores(con.mapa, CELDA, self.rastro, cubo,
                                         self.get_clock().now().to_msg()))

    def detener(self):
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = NavegadorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.detener()
    node.destroy_node()
    rclpy.try_shutdown()
