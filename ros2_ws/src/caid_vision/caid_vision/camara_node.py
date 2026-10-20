"""
Nodo de ROS 2 -> publica imgenes de una cámara (webcam,permite cambio a ESP32-CAM)
en el topic /camera/image_raw y metadatos en /camera/camera_info.
"""
import os
import cv2  # Libreria para OpenCV
from cv_bridge import CvBridge  # Puente para convertir imágenes de OpenCV a mensajes ROS

# Importaciones necesarias para ROS 2 en Python
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo


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
        
        # Inicializar la cámara y cargar los datos de calibración si existen
        self.abrir()
        self.info = self.cargar_info(self.get_parameter('calibracion').value)
        
        # 2. CREACIÓN DE PUBLICADORES
        # Publica la imagen en bruto y la información intrínseca de la cámara a 10 Hz de cola de mensajes
        self.pub_img = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Inicializar la herramienta de conversión de formatos OpenCV <-> ROS
        self.br = CvBridge()
        
        # 3. TEMPORIZADOR (TIMER)
        # Llama a la función 'tick' repetidamente en función de los FPS configurados
        self.create_timer(1.0 / self.get_parameter('fps').value, self.tick)

    def abrir(self):
        """Abre la fuente de video usando OpenCV y fija una resolución estándar."""
        self.cap = cv2.VideoCapture(self.fuente)
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

    def tick(self):
        """Ciclo principal que captura, convierte y publica cada fotograma."""
        ok, frame = self.cap.read()
        
        # Control de errores: si la cámara se desconecta, libera y reintenta abrir
        if not ok:
            self.get_logger().warn('Sin imagen, reconectando...')
            self.cap.release()
            self.abrir()
            return
        
        # Convertir la matriz de OpenCV (BGR8) a un mensaje ROS de tipo sensor_msgs/Image
        msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
        
        # Asignar marca de tiempo actual y el marco de referencia ('frame_id')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camara'
        
        # Sincronizar el encabezado de la información de la cámara con el de la imagen
        self.info.header = msg.header
        
        # Publicar ambos mensajes en sus respectivos tópicos
        self.pub_img.publish(msg)
        self.pub_info.publish(self.info)


def main():
    """Función principal para inicializar y ejecutar el nodo en ROS 2."""
    rclpy.init()
    rclpy.spin(Camara())