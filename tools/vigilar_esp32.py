#!/usr/bin/env python3
"""Vigila la ESP32 de las ruedas SIN grabar nada: avisa cuando se corta /wheel_state y cuando la ESP32 se REINICIO.

  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  source /opt/ros/jazzy/setup.bash
  python3 tools/vigilar_esp32.py

Se reinicio si el contador [32] de /wheel_state (mediciones terminadas del sonar frontal, siempre sube) BAJA: arranca de cero con cada
reinicio. Si en cambio solo hay silencio y despues vuelve el contador sin bajar, fue el Wi-Fi / el agente, no la placa.
Necesita el agente micro-ROS corriendo y que la ESP32 llegue a publicar (si nunca llega, mira el log del agente y el ping)."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

CONTADOR = 32                     # mediciones terminadas del frontal (sube siempre; vuelve a 0 al reiniciarse)
SILENCIO_S = 1.0                  # sin datos mas que esto = corte


class Vigia(Node):
    def __init__(self):
        super().__init__('vigilar_esp32')
        self.create_subscription(Float32MultiArray, '/wheel_state', self.al_llegar, qos_profile_sensor_data)
        self.t0 = time.monotonic()
        self.ultimo = None            # monotonic del ultimo mensaje
        self.cont = None
        self.n = self.cortes = self.reinicios = 0
        self.create_timer(0.5, self.revisar)
        self.t_info = 0.0
        self.get_logger().info('Esperando /wheel_state (la ESP32 tiene que llegar a publicar)...')

    def t(self):
        return time.monotonic() - self.t0

    def al_llegar(self, msg):
        ahora = time.monotonic()
        if self.ultimo is not None and ahora - self.ultimo > SILENCIO_S:
            self.cortes += 1
            print('[%6.1fs] VOLVIERON los datos tras %.1f s de silencio' % (self.t(), ahora - self.ultimo))
        if self.ultimo is None:
            print('[%6.1fs] LLEGO el primer dato: la ESP32 esta publicando' % self.t())
        self.ultimo = ahora
        self.n += 1
        c = msg.data[CONTADOR] if len(msg.data) > CONTADOR else None
        if c is not None and self.cont is not None and c < self.cont:
            self.reinicios += 1
            print('[%6.1fs] *** SE REINICIO LA ESP32 (el contador bajo de %d a %d) ***' % (self.t(), self.cont, c))
        if c is not None:
            self.cont = c
        self.frente = msg.data[26] if len(msg.data) > 26 else None

    def revisar(self):
        ahora = time.monotonic()
        if self.ultimo is not None and ahora - self.ultimo > SILENCIO_S and ahora - self.t_info > 2.0:
            self.t_info = ahora
            print('[%6.1fs] SIN DATOS hace %.1f s' % (self.t(), ahora - self.ultimo))
        elif self.ultimo is not None and ahora - self.t_info > 5.0:
            self.t_info = ahora
            print('[%6.1fs] ok | %d mensajes | cortes %d | reinicios %d | contador %s | sonar frente %s' % (
                self.t(), self.n, self.cortes, self.reinicios, int(self.cont) if self.cont is not None else '?',
                '--' if self.frente is None or self.frente < 0 else '%.0f cm' % (100 * self.frente)))


def main():
    rclpy.init()
    nodo = Vigia()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print('\nResumen: %d mensajes, %d cortes, %d reinicios en %.0f s' % (nodo.n, nodo.cortes, nodo.reinicios, nodo.t()))


if __name__ == '__main__':
    main()
