"""
Servidor de la action /cargar (caid_interfaces/action/Cargar), simulado.

Reemplaza al brazo mientras MAN no está integrado: recorre las etapas de un agarre con su avance
(feedback) y al terminar saca del mundo de Gazebo el cubo y su pilar (el pilar está en el medio
del pasillo: si quedara, cerraría el paso), como si el robot lo hubiera levantado. Si hay más
de un cubo de ese color (cudar tiene cubo_<color> y destino_<color>), saca el más cercano al
robot: el que tiene enfrente.
"""
import re
import subprocess
import time

from caid_interfaces.action import Cargar
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

ETAPAS = ('acercar', 'bajar brazo', 'cerrar pinza', 'levantar')


class CargaSimulada(Node):

    def __init__(self):
        super().__init__('carga_simulada')
        self.declare_parameter('duracion', 3.0)       # s de reloj de pared, todas las etapas
        self.declare_parameter('mundo', 'ros2_world')
        self.duracion = self.get_parameter('duracion').value
        self.mundo = self.get_parameter('mundo').value
        ActionServer(self, Cargar, '/cargar', self.ejecutar)

    def ejecutar(self, goal):
        color = goal.request.color
        self.get_logger().info(f'Cargando el cubo {color}')
        avance = Cargar.Feedback()
        pasos = 10 * len(ETAPAS)
        for i in range(pasos):
            avance.etapa = ETAPAS[i * len(ETAPAS) // pasos]
            avance.progreso = (i + 1) / pasos
            goal.publish_feedback(avance)
            time.sleep(self.duracion / pasos)
        modelo = self.mas_cercano([f'cubo_{color}', f'destino_{color}'])
        for nombre in (modelo, f'soporte_{modelo}'):
            self.sacar_del_mundo(nombre)
        goal.succeed()
        resultado = Cargar.Result()
        resultado.exito = True
        resultado.mensaje = f'cubo {color} cargado'
        return resultado

    def mas_cercano(self, modelos):
        """De los modelos que existen en el mundo, el más cercano al robot (o el primero)."""
        robot = self.pose('caid_bot')
        poses = [(m, self.pose(m)) for m in modelos]
        poses = [(m, p) for m, p in poses if p]
        if not robot or not poses:
            return modelos[0]
        return min(poses, key=lambda mp: (mp[1][0] - robot[0]) ** 2
                   + (mp[1][1] - robot[1]) ** 2)[0]

    def pose(self, modelo):
        """(x, y) de un modelo de Gazebo, o None si no existe."""
        salida = subprocess.run(['gz', 'model', '-m', modelo, '-p'], capture_output=True,
                                text=True, timeout=10, check=False).stdout
        m = re.search(r'Pose.*?\n\s*\[([-\d.e]+) ([-\d.e]+) ', salida)
        return (float(m.group(1)), float(m.group(2))) if m else None

    def sacar_del_mundo(self, modelo):
        # Espera la respuesta: el robot vuelve a medir enseguida y no tiene que ver el pilar.
        subprocess.run(
            ['gz', 'service', '-s', f'/world/{self.mundo}/remove',
             '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', f'name: "{modelo}" type: MODEL'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5, check=False)


def main():
    rclpy.init()
    node = CargaSimulada()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
