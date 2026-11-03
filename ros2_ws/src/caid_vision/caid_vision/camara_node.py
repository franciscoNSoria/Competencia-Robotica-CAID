"""
Nodo de ROS 2 -> publica imgenes de una cámara (webcam,permite cambio a ESP32-CAM)
en el topic /camera/image_raw y metadatos en /camera/camera_info.
"""
import os
import threading
import time
import cv2  # Libreria para OpenCV
from cv_bridge import CvBridge  # Puente para convertir imágenes de OpenCV a mensajes ROS

# Importaciones necesarias para ROS 2 en Python
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class Camara(Node):
    def __init__(self):
        super().__init__('camara_node')
        
        # 1. DECLARACIÓN DE PARÁMETROS CONFIGURABLES
        # - fuente: '0' para webcam local, o una URL tipo 'http://IP:81/stream' para ESP32-CAM
        # - fps: Tasa de fotogramas por segundo deseada
        # - calibracion: Ruta al archivo YAML con la matriz de calibración de la lente
        self.declare_parameter('fuente', '0', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('fps', 15)
        self.declare_parameter('calibracion', '')
        
        # Parsear el parámetro 'fuente': si es un número entero lo convierte, si es URL lo deja como string
        f = str(self.get_parameter('fuente').value)
        self.fuente = int(f) if f.isdigit() else f
        
        # Último cuadro leído por el hilo lector (y su hora de captura)
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = None
        self.nuevo = False

        # Inicializar la cámara y cargar los datos de calibración si existen
        self.abrir()
        self.info = self.cargar_info(self.get_parameter('calibracion').value)
        
        # 2. CREACIÓN DE PUBLICADORES
        # Publica la imagen en bruto y la información intrínseca de la cámara a 10 Hz de cola de mensajes
        # QoS de sensor (best effort, cola 1): si alguien va lento, se descartan cuadros viejos
        self.pub_img = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info',
                                              qos_profile_sensor_data)
        # Versión JPEG (~40 KB en vez de 900 KB): pasa por DDS sin perder cuadros
        self.pub_jpg = self.create_publisher(CompressedImage, '/camera/image_raw/compressed',
                                             qos_profile_sensor_data)
        
        # Inicializar la herramienta de conversión de formatos OpenCV <-> ROS
        self.br = CvBridge()
        
        # 3. TEMPORIZADOR (TIMER)
        # Llama a la función 'tick' repetidamente en función de los FPS configurados
        self.create_timer(1.0 / self.get_parameter('fps').value, self.tick)

        # 4. HILO LECTOR
        # Lee el stream sin parar y se queda solo con el cuadro más nuevo. Si leyéramos solo
        # en el timer, los cuadros que manda la ESP32 de más se acumulan en el buffer de
        # OpenCV/FFmpeg y la imagen llega cada vez más atrasada.
        threading.Thread(target=self.leer, daemon=True).start()

    def abrir(self):
        """Abre la fuente de video usando OpenCV y fija una resolución estándar."""
        self.cap = cv2.VideoCapture(self.fuente)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # algunos backends lo respetan
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.get_logger().info(f'Fuente: {self.fuente}  abierta={self.cap.isOpened()}')

    def cargar_info(self, ruta):
        """Lee el archivo YAML de calibración y configura el mensaje CameraInfo de ROS."""
        info = CameraInfo()
        if ruta and os.path.exists(ruta):
            with open(ruta) as fh:
                d = yaml.safe_load(fh)
            info.width, info.height = d['width'], d['height']
            info.k = [float(x) for x in d['K']]  # Matriz intrínseca
            info.d = [float(x) for x in d['D']]  # Coeficientes de distorsión
            info.distortion_model = 'plumb_bob'
            self.get_logger().info('Calibracion cargada')
        return info

    def leer(self):
        """Hilo: lee cuadros todo el tiempo y guarda el último."""
        while rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('Sin imagen, reconectando...')
                self.cap.release()
                time.sleep(0.5)
                self.abrir()
                continue
            stamp = self.get_clock().now().to_msg()  # hora de captura, no de publicación
            with self.lock:
                self.frame, self.stamp, self.nuevo = frame, stamp, True

    def tick(self):
        """Publica el cuadro más nuevo (si llegó uno desde la última vez)."""
        with self.lock:
            if not self.nuevo:
                return
            frame, stamp, self.nuevo = self.frame, self.stamp, False

        self.info.header.stamp = stamp
        self.info.header.frame_id = 'camara'
        self.pub_info.publish(self.info)

        # Solo se arma cada versión si alguien la está escuchando
        if self.pub_jpg.get_subscription_count() > 0:
            jpg = self.br.cv2_to_compressed_imgmsg(frame, 'jpg')
            jpg.header = self.info.header
            self.pub_jpg.publish(jpg)
        if self.pub_img.get_subscription_count() > 0:
            msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
            msg.header = self.info.header
            self.pub_img.publish(msg)


def main():
    """Función principal para inicializar y ejecutar el nodo en ROS 2."""
    rclpy.init()
    rclpy.spin(Camara())