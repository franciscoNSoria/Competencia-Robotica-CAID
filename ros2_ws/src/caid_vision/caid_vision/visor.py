"""Visor rápido: muestra un topic CompressedImage (JPEG) en una ventana de OpenCV.

Uso:
  ros2 run caid_vision visor                       # /vision/debug/compressed
  ros2 run caid_vision visor --ros-args -p topic:=/camera/image_raw/compressed
Tecla q o Esc en la ventana para salir. El título muestra los fps y el retraso.
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Visor(Node):
    def __init__(self):
        super().__init__('visor')
        self.declare_parameter('topic', '/vision/debug/compressed')
        self.topic = self.get_parameter('topic').value
        self.create_subscription(CompressedImage, self.topic, self.cb, qos_profile_sensor_data)
        self.img = None
        self.cuadros = 0
        self.t0 = time.time()
        self.fps = 0.0
        self.retraso_ms = 0.0
        self.get_logger().info(f'Mirando {self.topic}')

    def cb(self, msg):
        self.img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        ahora = self.get_clock().now().nanoseconds * 1e-9
        self.retraso_ms = (ahora - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)) * 1000
        self.cuadros += 1
        if time.time() - self.t0 >= 1.0:
            self.fps = self.cuadros / (time.time() - self.t0)
            self.cuadros, self.t0 = 0, time.time()


def main():
    rclpy.init()
    v = Visor()
    try:
        while rclpy.ok():
            rclpy.spin_once(v, timeout_sec=0.01)
            if v.img is not None:
                img = v.img.copy()
                cv2.putText(img, f'{v.fps:.1f} fps  retraso {v.retraso_ms:.0f} ms', (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow(v.topic, img)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    v.destroy_node()
    rclpy.try_shutdown()
