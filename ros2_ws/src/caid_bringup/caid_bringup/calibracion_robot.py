"""
Manda al firmware (robot_4ruedas) la calibración y activa la guardia por sonar.

La ESP32 no guarda nada al apagarse: arranca con diámetro 67 mm y trocha 131 mm (geométricos,
CAD), y la guardia (freno local por sonar) apagada. Alguien tiene que mandarle la calibración
real por /robot_cmd antes de moverla: sin eso, el rumbo y la velocidad de giro que calcula el
firmware a partir de /cmd_vel no corresponden a lo que de verdad va a girar el robot (la trocha
efectiva, medida con 10 giros de 360° el 3/11, es de 291 mm: 2,2 veces la geométrica, porque es
un 4x4 que patina).

OJO: esos 291 mm salen de girar 360°, en 6 de 10 corridas válidas (las otras 4 salieron muy
cortas, causa sin identificar) y CON la aceleración/rampa de un giro largo. El laberinto gira
90°, donde el patinaje del arranque pesa más: puede no ser la misma trocha efectiva. Hasta medir
giros de 90° (pendiente del equipo), este valor es la mejor estimación que hay, no un dato firme.

Manda [5, diametro, trocha, frenado] una vez al arrancar (varias veces seguidas, por si el
suscriptor de la ESP32 todavía no está listo: no hay confirmación, /wheel_state con cfg_seq
creciendo es la única forma de verificarlo a mano) y [8, 1, d_stop, d_lento] cada 1 s mientras
el nodo esté vivo, para tener la guardia siempre activa (es el hábito ya usado en
tools/autonomo/robot_autonomo.py; el firmware no la apaga sola, pero re-mandarla no hace daño
y cubre un reinicio de la ESP32 a mitad de la prueba).
"""
from std_msgs.msg import Float32MultiArray

import rclpy
from rclpy.node import Node


class CalibracionRobot(Node):

    def __init__(self):
        super().__init__('calibracion_robot')
        p = {
            'robot_cmd_topic': '/robot_cmd',
            # Medidos el 3/11 (hardware-estado-y-calibraciones.md, sección 6). Mantener iguales
            # a caid_bringup/odom_ticks.py: los dos lados (firmware y odometría de la PC) tienen
            # que usar los mismos números, o lo que el robot hace y lo que cree que hizo difieren.
            'diametro_m': 0.0645,
            'trocha_m': 0.291,
            'frenado_m': 0.030,
            # Guardia (freno local por sonar, guardia.h): libre desde d_lento, 0,13 m/s entre
            # d_stop y d_lento, 0 por debajo de d_stop. Medido en el piso el 4/11: frenó con la
            # lectura en 16-17 cm con d_stop=0,20 m (3-4 cm de margen real sobre el umbral).
            'd_stop_m': 0.20,
            'd_lento_m': 0.40,
        }
        for nombre, valor in p.items():
            self.declare_parameter(nombre, valor)
        self.cfg = {n: self.get_parameter(n).value for n in p}

        self.pub = self.create_publisher(Float32MultiArray, self.cfg['robot_cmd_topic'], 10)
        self.create_timer(0.3, self.mandar_calibracion_inicial)
        self.envios_calibracion = 0
        self.create_timer(1.0, self.mandar_guardia)
        self.get_logger().info(
            f"Calibrando: diametro {self.cfg['diametro_m'] * 1000:.1f} mm, trocha "
            f"{self.cfg['trocha_m'] * 1000:.0f} mm (sin confirmar en giros de 90°). Guardia: "
            f"frena a {self.cfg['d_stop_m']:.2f} m, lenta desde {self.cfg['d_lento_m']:.2f} m")

    def mandar_calibracion_inicial(self):
        if self.envios_calibracion >= 10:
            return
        self.envios_calibracion += 1
        self.pub.publish(Float32MultiArray(
            data=[5.0, self.cfg['diametro_m'], self.cfg['trocha_m'], self.cfg['frenado_m']]))

    def mandar_guardia(self):
        self.pub.publish(Float32MultiArray(
            data=[8.0, 1.0, self.cfg['d_stop_m'], self.cfg['d_lento_m']]))


def main():
    rclpy.init()
    node = CalibracionRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
