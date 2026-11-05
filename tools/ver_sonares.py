#!/usr/bin/env python3
"""Muestra los 3 sonares que manda la ESP32 de las ruedas en /wheel_state, en cm y con nombre.

/wheel_state (std_msgs/Float32MultiArray, 34 valores, 10 Hz, firmware/banco/robot_4ruedas):
  [26] frente   [27] izquierdo   [28] derecho   en metros; -1 = sin eco.

Uso (con el agente de micro-ROS corriendo y la ESP32 conectada):
  python3 tools/ver_sonares.py
"""
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

FRENTE, IZQ, DER = 26, 27, 28


def cm(x):
    return '  sin eco' if x < 0 else f'{x * 100:6.1f} cm'


def main():
    rclpy.init()
    nodo = rclpy.create_node('ver_sonares')

    def cb(msg):
        d = msg.data
        if len(d) <= DER:
            print(f'El firmware no manda sonares ({len(d)} valores; hacen falta 34)')
            return
        print(f'frente {cm(d[FRENTE])}   |   izquierda {cm(d[IZQ])}   |   derecha {cm(d[DER])}',
              flush=True)

    # la ESP32 publica en best effort: con la QoS por defecto (reliable) no llega nada
    nodo.create_subscription(Float32MultiArray, '/wheel_state', cb, qos_profile_sensor_data)
    print('Escuchando /wheel_state (Ctrl+C para salir)...', flush=True)
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
