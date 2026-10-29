"""
Servidor de la action /cargar (caid_interfaces/action/Cargar), simulado.

Reemplaza al brazo mientras MAN no está integrado: recorre las etapas de un agarre con su avance
(feedback) y al terminar saca del mundo de Gazebo el cubo y su pilar (el pilar está en el medio
del pasillo: si quedara, cerraría el paso), como si el robot lo hubiera levantado.
"""
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
        for modelo in (f'cubo_{color}', f'soporte_cubo_{color}'):
            self.sacar_del_mundo(modelo)
        goal.succeed()
        resultado = Cargar.Result()
        resultado.exito = True
        resultado.mensaje = f'cubo {color} cargado'
        return resultado

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
