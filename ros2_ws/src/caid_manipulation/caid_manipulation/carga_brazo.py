"""
Cargar y dejar cubos con el brazo: servidores de /cargar y /dejar (caid_interfaces).

Mueve el brazo por /brazo/cmd y mira /brazo/estado, igual con el brazo real (ESP32) que con el
simulado (brazo_sim): el mismo nodo sirve para los dos. Con la posición del objeto en base_link
que manda la navegación, calcula los ángulos con la cinemática calibrada (cinematica.py) y hace:

  cargar: abrir pinza, girar base, acercar (4 cm arriba del cubo), bajar, cerrar pinza, levantar
          y volver a la pose de transporte (el reposo seguro con la base al frente)
  dejar:  girar base, acercar, bajar sobre el canasto, abrir pinza, levantar y volver

En la simulación (simulacion:=true), al cerrar la pinza saca el cubo del mundo y crea una copia
(cubo_<color>_agarrado) en la punta de la pinza, que la unión desmontable del robot engancha sola
(caid_description/urdf/gazebo.xacro); al abrirla, la suelta y cae en el canasto.
"""
import math
import re
import subprocess
import tempfile
import threading
import time

from caid_interfaces.action import Cargar, Dejar
from caid_manipulation import cinematica as k
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32MultiArray

COLORES_RGB = {'rojo': '1 0 0', 'verde': '0 0.7 0.1', 'azul': '0 0.2 1'}
CUBO_SDF = """<sdf version="1.9"><model name="{nombre}"><link name="link">
<inertial><mass>0.0099</mass><inertia><ixx>6.6e-07</ixx><iyy>6.6e-07</iyy><izz>6.6e-07</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name="c"><geometry><box><size>0.02 0.02 0.02</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.02 0.02 0.02</size></box></geometry>
<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
</link></model></sdf>"""
ALTURA_BASE_LINK = 0.1085      # m: base_link sobre el piso (cara de arriba del 2.o piso)


class CargaBrazo(Node):

    def __init__(self):
        super().__init__('carga_brazo')
        p = {
            'simulacion': True,
            'mundo': 'ros2_world',
            'velocidad': 60.0,          # grados/s de los servos durante la maniobra
            'altura_cubo': 0.010,       # m sobre base_link: centro del cubo (apoya a 108,5 mm)
            'altura_soltar': 0.035,     # m sobre base_link: la punta sobre el borde del canasto
            'subir': 0.04,              # m por encima para acercar y levantar
            'tolerancia': 1.5,          # grados para dar por llegado cada servo
            'tiempo_max_paso': 12.0,    # s
        }
        for n, v in p.items():
            self.declare_parameter(n, v)
        self.cfg = {n: self.get_parameter(n).value for n in p}
        grupo = ReentrantCallbackGroup()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_cmd = self.create_publisher(Float32MultiArray, '/brazo/cmd', qos)
        self.create_subscription(Float32MultiArray, '/brazo/estado', self.on_estado, qos,
                                 callback_group=grupo)
        self.pub_soltar = {c: self.create_publisher(Empty, f'/sim/pinza/soltar/{c}', 10)
                           for c in COLORES_RGB}
        self.estado = None
        self.cerrojo = threading.Lock()
        ActionServer(self, Cargar, '/cargar', self.cargar, callback_group=grupo)
        ActionServer(self, Dejar, '/dejar', self.dejar, callback_group=grupo)
        self.get_logger().info('Carga con brazo lista: /cargar y /dejar')

    def on_estado(self, msg):
        if len(msg.data) >= 4:
            self.estado = list(msg.data)

    # ---------- movimientos ----------

    def mover(self, goal, fb, etapa, progreso, base, hombro, codo, pinza):
        """Manda la pose y espera a que /brazo/estado diga que llegó. False si no llegó."""
        objetivo = [base, hombro, codo, pinza]
        fb.etapa, fb.progreso = etapa, float(progreso)
        goal.publish_feedback(fb)
        t0 = time.time()
        while time.time() - t0 < self.cfg['tiempo_max_paso']:
            orden = [float(v) for v in objetivo] + [15.0, float(self.cfg['velocidad'])]
            self.pub_cmd.publish(Float32MultiArray(data=orden))
            e = self.estado
            if e is not None and all(abs(e[i] - objetivo[i]) <= self.cfg['tolerancia']
                                     for i in range(4)):
                return True
            time.sleep(0.1)
        self.get_logger().warn(f'{etapa}: el brazo no llegó a {objetivo} (está en '
                               f'{None if self.estado is None else self.estado[:4]})')
        return False

    def plan(self, x, y, z):
        """(base, hombro, codo) para la punta en (x, y, z) y 'subir' más arriba, o None."""
        abajo = k.ik(x, y, z)
        arriba = k.ik(x, y, z + self.cfg['subir'])
        if abajo is None or arriba is None:
            return None
        lim = {'hombro': (10.0, 125.0), 'codo': (80.0, 200.0), 'base': (0.0, 180.0)}
        for sol in (abajo, arriba):
            for nombre, v in zip(('base', 'hombro', 'codo'), sol):
                if not lim[nombre][0] <= v <= lim[nombre][1]:
                    return None
        return abajo, arriba

    def fallar(self, goal, tipo, mensaje):
        self.get_logger().warn(mensaje)
        goal.abort()
        r = tipo.Result()
        r.exito, r.mensaje = False, mensaje
        return r

    # ---------- acciones ----------

    def cargar(self, goal):
        color, x, y = goal.request.color, goal.request.x, goal.request.y
        with self.cerrojo:
            plan = self.plan(x, y, self.cfg['altura_cubo'])
            r = math.hypot(x - k.BASE_X / 1000.0, y)
            if plan is None:
                return self.fallar(goal, Cargar, f'cubo {color} fuera de alcance: está a '
                                   f'{r * 1000:.0f} mm del eje del brazo (alcanza 193 a 237)')
            (b, h, c), (_, ha, ca) = plan
            self.get_logger().info(f'Cargo el cubo {color} a {r * 1000:.0f} mm del eje: '
                                   f'base {b:.1f}, hombro {h:.1f}, codo {c:.1f}')
            fb, ab, ce = Cargar.Feedback(), k.PINZA_ABIERTA, k.PINZA_CERRADA
            pasos = [('abrir pinza', 0.1, k.BASE_REPOSO, 60.0, 180.0, ab),
                     ('girar base', 0.25, b, 60.0, 180.0, ab),
                     ('acercar', 0.4, b, ha, ca, ab),
                     ('bajar', 0.55, b, h, c, ab),
                     ('cerrar pinza', 0.7, b, h, c, ce)]
            for paso in pasos:
                if not self.mover(goal, fb, *paso):
                    return self.fallar(goal, Cargar, f'no pude {paso[0]}')
            if self.cfg['simulacion']:
                self.agarrar_en_gazebo(color)
            for paso in (('levantar', 0.85, b, ha, ca, ce),
                         ('transporte', 0.95, b, 60.0, 180.0, ce),
                         ('transporte', 1.0, k.BASE_REPOSO, 60.0, 180.0, ce)):
                if not self.mover(goal, fb, *paso):
                    return self.fallar(goal, Cargar, f'no pude {paso[0]}')
            goal.succeed()
            res = Cargar.Result()
            res.exito, res.mensaje = True, f'cubo {color} cargado'
            return res

    def dejar(self, goal):
        color, x, y = goal.request.color, goal.request.x, goal.request.y
        with self.cerrojo:
            plan = self.plan(x, y, self.cfg['altura_soltar'])
            r = math.hypot(x - k.BASE_X / 1000.0, y)
            if plan is None:
                return self.fallar(goal, Dejar, f'canasto {color} fuera de alcance: está a '
                                   f'{r * 1000:.0f} mm del eje del brazo')
            (b, h, c), (_, ha, ca) = plan
            self.get_logger().info(f'Dejo el cubo {color} en su canasto a {r * 1000:.0f} mm')
            fb, ab, ce = Dejar.Feedback(), k.PINZA_ABIERTA, k.PINZA_CERRADA
            for paso in (('girar base', 0.15, b, 60.0, 180.0, ce),
                         ('acercar', 0.3, b, ha, ca, ce),
                         ('bajar', 0.45, b, h, c, ce),
                         ('abrir pinza', 0.6, b, h, c, ab)):
                if not self.mover(goal, fb, *paso):
                    return self.fallar(goal, Dejar, f'no pude {paso[0]}')
            if self.cfg['simulacion']:
                self.pub_soltar[color].publish(Empty())
            for paso in (('levantar', 0.75, b, ha, ca, ab),
                         ('transporte', 0.9, b, 60.0, 180.0, ab),
                         ('transporte', 1.0, k.BASE_REPOSO, 60.0, 180.0, k.REPOSO['pinza'])):
                if not self.mover(goal, fb, *paso):
                    return self.fallar(goal, Dejar, f'no pude {paso[0]}')
            goal.succeed()
            res = Dejar.Result()
            res.exito, res.mensaje = True, f'cubo {color} dejado'
            return res

    # ---------- solo en Gazebo ----------

    def pose_gz(self, modelo):
        """(x, y, z, yaw) de un modelo de Gazebo, o None."""
        try:
            salida = subprocess.run(['gz', 'model', '-m', modelo, '-p'], capture_output=True,
                                    text=True, timeout=10, check=False).stdout
        except (OSError, subprocess.TimeoutExpired):
            return None
        m = re.search(r'Pose[^\n]*\n\s*\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]\s*\n\s*'
                      r'\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]', salida)
        if not m:
            return None
        v = [float(g) for g in m.groups()]
        return v[0], v[1], v[2], v[5]

    def gz_servicio(self, servicio, reqtype, req):
        subprocess.run(['gz', 'service', '-s', f'/world/{self.cfg["mundo"]}/{servicio}',
                        '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '3000', '--req', req],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=8,
                       check=False)

    def agarrar_en_gazebo(self, color):
        """Saca el cubo del mundo y pone su copia en la punta de la pinza (la engancha el robot)."""
        robot = self.pose_gz('caid_bot')
        if robot is None or self.estado is None:
            self.get_logger().warn('No pude leer la pose del robot en Gazebo: el cubo no se mueve')
            return
        tx, ty, tz = k.punta(*self.estado[:3])
        c, s = math.cos(robot[3]), math.sin(robot[3])
        wx, wy = robot[0] + c * tx - s * ty, robot[1] + s * tx + c * ty
        wz = robot[2] + ALTURA_BASE_LINK + tz
        candidatos = [m for m in (f'cubo_{color}', f'destino_{color}')
                      if self.pose_gz(m) is not None]
        if candidatos:
            def dist2(m):
                p = self.pose_gz(m)
                return (p[0] - wx) ** 2 + (p[1] - wy) ** 2
            self.gz_servicio('remove', 'gz.msgs.Entity',
                             f'name: "{min(candidatos, key=dist2)}" type: MODEL')
        with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
            f.write(CUBO_SDF.format(nombre=f'cubo_{color}_agarrado', rgb=COLORES_RGB[color]))
        self.gz_servicio('create', 'gz.msgs.EntityFactory',
                         f'sdf_filename: "{f.name}" pose: {{position: {{x: {wx:.4f} y: {wy:.4f} '
                         f'z: {wz:.4f}}}}}')
        self.get_logger().info(f'Cubo {color} en la pinza ({wx:.3f}, {wy:.3f}, {wz:.3f})')


def main():
    rclpy.init()
    node = CargaBrazo()
    ejecutor = MultiThreadedExecutor()
    ejecutor.add_node(node)
    try:
        ejecutor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
