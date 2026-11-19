"""
Vision CAID 2026: detecta flechas, cubos, canastos y zonas rojas.

La detección está en detector.py (sin ROS, con pruebas). Este nodo:
Lee:
  colores      (parámetro) YAML de calibrar_colores con los rangos HSV; se recarga solo cuando
               cambia el archivo. Vacío = rangos de detector.py (los de la simulación).
  imagen_topic (sensor_msgs/Image), por defecto /camera/image_raw (/sim/camera/image_raw en
               Gazebo).
               Si termina en /compressed se lee como CompressedImage (JPEG, mucho más liviano).
Publica:
  /vision/detecciones (String)  JSON con 'stamp' (s, hora de la imagen) y las detecciones en
                                píxeles (ver detector.py). Lo usa caid_navigation/navegador.
  /vision/proximo_giro (String) JSON con el PRÓXIMO GIRO: {"giro": "derecha" | "izquierda",
                                "color", "tamano", "distancia", "busca"}. Sale de la flecha FIJA
                                (a distancia_fija, 30 cm) del color que busca el explorador
                                (/nav/estado) o, si no se sabe, de la más cercana. Solo se
                                publica cuando hay giro: sin flecha que seguir, no sale nada
  /vision/cubo/<color> (Point)  x,y = centro en px, z = area           (como antes)
  /vision/flecha       (String) "color,direccion,angulo,tamaño" de cada flecha anotada
                                (validador.py): de lejos solo se ve; a distancia_decision o menos
                                queda anotada para siempre, y su dirección y tamaño (P3 / P2 / P1)
                                son la mayoría de las últimas ventana_flecha lecturas cercanas
  /vision/debug/compressed (CompressedImage) imagen con todo dibujado (JPEG, para rqt_image_view)
  /vision/debug        (Image)  lo mismo sin comprimir, solo si alguien lo está mirando
"""
import json
import math
import os

from caid_vision import detector
from caid_vision.config_util import cargar_yaml, rangos_desde_yaml
from caid_vision.detector import detectar
from caid_vision.validador import medir_flecha, ValidadorFlechas
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


def clasificar_direccion(ang):
    """
    Solo derecha o izquierda: el lado hacia el que apunta, aunque esté inclinada.

    ang: grados en la imagen (0 = derecha, 90 = arriba, 180 = izquierda). Una flecha hacia arriba
    a la derecha (ej. 60) cuenta como derecha; hacia arriba a la izquierda (ej. 120), izquierda.
    """
    return 'derecha' if math.cos(math.radians(ang)) >= 0 else 'izquierda'


def cartel(img, texto, esquina, escala, color):
    """Texto grueso con fondo negro, para que se lea sobre cualquier imagen."""
    fuente, grosor = cv2.FONT_HERSHEY_SIMPLEX, max(1, int(round(escala * 2.2)))
    (w, h), base = cv2.getTextSize(texto, fuente, escala, grosor)
    x = max(2, min(int(esquina[0]), img.shape[1] - w - 4))
    y = max(h + 4, int(esquina[1]))
    cv2.rectangle(img, (x - 4, y - h - 5), (x + w + 4, y + base + 3), (0, 0, 0), -1)
    cv2.putText(img, texto, (x, y), fuente, escala, color, grosor, cv2.LINE_AA)


class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('imagen_topic', '/camera/image_raw')
        self.declare_parameter('ventana_flecha', 10)    # lecturas cercanas que votan la dirección
        self.declare_parameter('olvido_flecha_s', 1.5)  # sin verla este tiempo, se olvida
        # Las flechas se deciden recién a esta distancia (m) o menos: de lejos la medida no sirve
        self.declare_parameter('distancia_decision', 0.50)   # desde acá vota (m)
        self.declare_parameter('distancia_fija', 0.30)       # acá queda FIJA y no duda más (m)
        self.declare_parameter('color_buscado', '')          # si no llega /nav/estado
        self.declare_parameter('vigencia_giro_s', 2.0)       # fija y sin verla: deja de mandar
        # fx para medir la distancia: 0 = el de la calibración de lente (si hay) o FOV de 60°
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('calibracion', '')
        self.declare_parameter('debug', True)
        self.declare_parameter('colores', '')
        # Filtros contra detecciones falsas; se cambian en vivo con ros2 param set.
        # v_min / v_min_azul / s_min: -1 = lo que diga colores.yaml (o detector.py sin YAML)
        self.declare_parameter('v_min', -1)        # brillo mínimo de rojo y verde (0-255)
        self.declare_parameter('v_min_azul', -1)   # brillo mínimo del azul
        self.declare_parameter('s_min', -1)        # saturación mínima (0-255)
        self.declare_parameter('area_min_cubo', detector.AREA_MIN_CUBO)
        self.declare_parameter('area_min_flecha', detector.AREA_MIN_FLECHA)
        self.declare_parameter('area_min_canasto', detector.AREA_MIN_CANASTO)
        self.br = CvBridge()
        topic = self.get_parameter('imagen_topic').value
        tipo = CompressedImage if topic.endswith('/compressed') else Image
        self.create_subscription(tipo, topic, self.cb, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(String, '/vision/detecciones', 10)
        self.pubs = {c: self.create_publisher(Point, f'/vision/cubo/{c}', 10)
                     for c in ('rojo', 'verde', 'azul')}
        self.pub_flecha = self.create_publisher(String, '/vision/flecha', 10)
        self.pub_giro = self.create_publisher(String, '/vision/proximo_giro', 10)
        self.busca = None                 # color que busca el explorador (de /nav/estado)
        self.create_subscription(String, '/nav/estado', self.on_estado, 10)
        self.dist_flecha = {}             # id -> última distancia medida (m)
        self.proximo = None               # último giro decidido (None = no hay)
        self.debug = self.create_publisher(Image, '/vision/debug', qos_profile_sensor_data)
        self.debug_jpg = self.create_publisher(CompressedImage, '/vision/debug/compressed',
                                               qos_profile_sensor_data)
        self.validador = ValidadorFlechas(self.get_parameter('ventana_flecha').value,
                                          self.get_parameter('olvido_flecha_s').value)
        self.ids_flecha = []      # id de la flecha seguida para cada flecha del cuadro actual
        self.medidas = []         # (distancia, cuerpo_mm, tamaño) de cada flecha del cuadro
        self.fx_cache = None
        self.avisadas = {}        # id -> (dirección, tamaño) ya anunciados en el log
        self.mtime_colores = None
        self.create_timer(1.0, self.cargar_colores)
        self.cargar_colores()

    def cargar_colores(self):
        """Lee colores.yaml si es la primera vez o si cambió (calibrar_colores lo reescribe)."""
        ruta = self.get_parameter('colores').value
        if not ruta:
            return
        try:
            mtime = os.path.getmtime(ruta)
        except OSError:
            if self.mtime_colores is None:
                self.get_logger().warn(f'No está {ruta}: uso los rangos de detector.py')
                self.mtime_colores = -1
            return
        if mtime == self.mtime_colores:
            return
        self.mtime_colores = mtime
        d = cargar_yaml(ruta) or {}
        if not d.get('rangos'):
            return
        detector.RANGOS = rangos_desde_yaml(d['rangos'])
        # Los rangos calibrados ya traen su brillo/saturación mínimos medidos
        detector.V_MIN = detector.V_MIN_AZUL = detector.S_MIN = 0
        resumen = ', '.join(f'{c} {len(t)} tramo(s)' for c, t in detector.RANGOS.items())
        self.get_logger().info(f'Colores cargados de {ruta}: {resumen}')

    def cb(self, msg):
        if isinstance(msg, CompressedImage):
            img = self.br.compressed_imgmsg_to_cv2(msg, 'bgr8')
        else:
            img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        self.aplicar_filtros()
        det = detectar(img)
        det['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub_det.publish(String(data=json.dumps(det)))
        for c in det['cubos']:
            x, y, w, h = c['caja']
            self.pubs[c['color']].publish(Point(x=float(x + w / 2), y=float(y + h / 2),
                                                z=float(c['area'])))
        self.publicar_flecha(det['flechas'], det['stamp'], img.shape[1])
        if self.get_parameter('debug').value:
            mira_jpg = self.debug_jpg.get_subscription_count() > 0
            mira_raw = self.debug.get_subscription_count() > 0
            if not (mira_jpg or mira_raw):
                return  # nadie mira: no gastar en dibujar
            self.dibujar(img, det)
            if mira_jpg:
                out = self.br.cv2_to_compressed_imgmsg(img, 'jpg')
                out.header = msg.header
                self.debug_jpg.publish(out)
            if mira_raw:
                out = self.br.cv2_to_imgmsg(img, 'bgr8')
                out.header = msg.header
                self.debug.publish(out)

    def aplicar_filtros(self):
        g = self.get_parameter
        for param, atributo in (('v_min', 'V_MIN'), ('v_min_azul', 'V_MIN_AZUL'),
                                ('s_min', 'S_MIN')):
            v = int(g(param).value)
            if v >= 0:
                setattr(detector, atributo, v)
        detector.AREA_MIN_CUBO = int(g('area_min_cubo').value)
        detector.AREA_MIN_FLECHA = int(g('area_min_flecha').value)
        detector.AREA_MIN_CANASTO = int(g('area_min_canasto').value)

    def dibujar(self, img, det):
        blanco = (255, 255, 255)
        verde_ok = (0, 255, 0)
        amarillo = (0, 220, 255)
        for f, k, (dist, _, tam) in zip(det['flechas'], self.ids_flecha, self.medidas):
            cola = tuple(int(v) for v in f['cola'])
            punta = tuple(int(v) for v in f['punta'])
            dec = self.validador.decision(k)
            d_txt = f'{dist:.2f} m' if dist is not None else '?'
            if dec:         # anotada: dirección y tamaño = mayoría de las últimas lecturas
                direccion, nd, td, tamano, nt, tt = dec
                color_linea = verde_ok if self.validador.es_fija(k) else amarillo
                cv2.arrowedLine(img, cola, punta, (0, 0, 0), 9, tipLength=0.3)
                cv2.arrowedLine(img, cola, punta, color_linea, 5, tipLength=0.3)
                arriba = min(f['punta'][1], f['cola'][1], *(h[1] for h in f.get('hombros', [])))
                x = int(min(f['punta'][0], f['cola'][0]))
                cartel(img, f'{direccion.upper()} {tamano or "?"}', (x, int(arriba) - 12), 1.0,
                       verde_ok)
                estado = 'FIJA' if self.validador.es_fija(k) else f'votando {nd}/{td}'
                cartel(img, f"{f['color']} {d_txt}  {estado}",
                       (x, int(arriba) - 12 + 26), 0.5, verde_ok)
            else:           # lejos: solo se avisa que hay una flecha
                cv2.arrowedLine(img, cola, punta, blanco, 1, tipLength=0.3)
                cv2.putText(img, f"flecha {f['color']} a {d_txt}", (cola[0], cola[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, blanco, 1)
        # Resumen arriba: las últimas 4 flechas anotadas (no se borran; la dirección se reevalúa)
        for i, (k, color, direccion, tam, _) in enumerate(self.validador.presentes()[-4:]):
            cartel(img, f'{color.upper()}: {direccion.upper()} {tam}', (8, 34 + 36 * i),
                   0.85, verde_ok)
        p = self.proximo
        if p:
            cartel(img, f"PROXIMO GIRO: {p['giro'].upper()}  (flecha {p['color']} {p['tamano']})",
                   (8, img.shape[0] - 12), 0.8, (0, 255, 255))
        for clave in ('cubos', 'canastos'):
            for c in det[clave]:
                x, y, w, h = c['caja']
                cv2.rectangle(img, (x, y), (x + w, y + h), blanco, 2)
                cv2.putText(img, f"{clave[:-1]} {c['color']}", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, blanco, 1)
        for u, v in det['zonas'][::5]:
            cv2.circle(img, (int(u), int(v)), 1, (0, 255, 255), -1)

    def fx(self, ancho_img):
        """Distancia focal en px para este ancho: parámetro, lente calibrada o FOV 60°."""
        if self.fx_cache and self.fx_cache[0] == ancho_img:
            return self.fx_cache[1]
        fx = float(self.get_parameter('fx').value)
        if fx <= 0:
            d = cargar_yaml(self.get_parameter('calibracion').value) or {}
            if d.get('K'):
                fx = float(d['K'][0]) * ancho_img / float(d.get('width', ancho_img))
            else:
                fx = (ancho_img / 2) / math.tan(math.radians(30))
        self.fx_cache = (ancho_img, fx)
        return fx

    def on_estado(self, msg):
        try:
            self.busca = json.loads(msg.data).get('busca')
        except ValueError:
            pass

    def elegir_proximo(self, t):
        """Próximo giro: la flecha FIJA, vista hace poco, del color buscado (o la más cercana)."""
        busca = self.busca or self.get_parameter('color_buscado').value or None
        vigencia = self.get_parameter('vigencia_giro_s').value
        candidatas = []
        for k, color, direccion, tam, _ in self.validador.presentes():
            reciente = t - self.validador.visto(k) <= vigencia
            if self.validador.es_fija(k) and reciente and busca in (None, color):
                candidatas.append((self.dist_flecha.get(k, 9.9), k, color, direccion, tam))
        if not candidatas:
            return None
        dist, _, color, direccion, tam = min(candidatas)
        return {'giro': direccion, 'color': color, 'tamano': tam,
                'distancia': round(dist, 3), 'busca': busca}

    def publicar_flecha(self, flechas, t, ancho_img):
        """Mide cada flecha, vota solo las que están cerca y publica las anotadas."""
        fx = self.fx(ancho_img)
        lim = self.get_parameter('distancia_decision').value
        self.medidas = [medir_flecha(f, fx) for f in flechas]
        entradas = []
        for f, (dist, _, tam) in zip(flechas, self.medidas):
            cerca = dist is not None and dist <= lim
            entradas.append((clasificar_direccion(f['angulo']), tam, cerca, f))
        self.ids_flecha = self.validador.actualizar(t, entradas)
        lim_fija = self.get_parameter('distancia_fija').value
        for k, (dist, _, _) in zip(self.ids_flecha, self.medidas):
            if dist is None:
                continue
            self.dist_flecha[k] = dist
            if dist <= lim_fija and self.validador.fijar(k):
                d = self.validador.decision(k)
                self.get_logger().info(f'Flecha a {dist:.2f} m: FIJA {d[0]} {d[3]} '
                                       f'({d[1]} de {d[2]} lecturas)')
        for k, color, direccion, tam, f in self.validador.presentes():
            antes = self.avisadas.get(k)
            if antes != (direccion, tam):
                self.avisadas[k] = (direccion, tam)
                que = 'anotada' if antes is None else f'cambia (antes {antes[0]} {antes[1]})'
                self.get_logger().info(f'Flecha {color} -> {direccion}, {tam}: {que}')
            self.pub_flecha.publish(String(data=f"{color},{direccion},{f['angulo']:.1f},{tam}"))
        proximo = self.elegir_proximo(t)
        antes = self.proximo and (self.proximo['giro'], self.proximo['color'])
        if proximo and (proximo['giro'], proximo['color']) != antes:
            self.get_logger().info(f"PRÓXIMO GIRO: {proximo['giro'].upper()} "
                                   f"(flecha {proximo['color']} {proximo['tamano']})")
        self.proximo = proximo
        if proximo:
            self.pub_giro.publish(String(data=json.dumps(proximo)))


def main():
    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
