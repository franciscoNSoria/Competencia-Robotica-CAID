"""Vision CAID 2026: detecta cubos y flechas rojo/verde/azul.
Publica:
  /vision/cubo/<color>  (Point)   x,y = centro en px, z = area
  /vision/flecha        (String)  "color,direccion,segmentos,angulo"  (confirmada N cuadros)
  /vision/debug         (Image)   imagen con todo dibujado
"""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge

RANGOS = {  # HSV -> ajustar con la luz real
    'rojo':  [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    'verde': [((40, 70, 70), (85, 255, 255))],
    'azul':  [((95, 120, 70), (130, 255, 255))],
}
AREA_MIN_CUBO = 200
AREA_MIN_FLECHA = 500

# Flechas (reglamento): punta 70 mm de ancho, cuerpo L = 30 / 60 / 100 mm.
# Largo del cuerpo / ancho de punta:  30/70=0.43   60/70=0.86   100/70=1.43
UMBRALES_LARGO = (0.64, 1.15)

# Tolerancia de inclinacion (grados): hasta cuanto se puede inclinar una flecha
# respecto de derecha/arriba/izquierda/abajo y seguir contando como esa direccion.
# Reglamento: inclinacion maxima +-30.  Con 45 nunca queda "diagonal".
ANGULO_TOL = 45


# =============================== CUBOS ===============================
def es_cubo(c):
    """Cuadrado visto de frente: macizo (sin entradas), 4 lados y largo ~ ancho."""
    hull = cv2.convexHull(c)
    solidez = cv2.contourArea(c) / max(cv2.contourArea(hull), 1)
    if solidez < 0.9:            # un cuadrado llena su contorno; una flecha tiene "entradas" (~0.72-0.88)
        return False
    (_, _), (w, h), _ = cv2.minAreaRect(c)     # rectangulo que gira con el objeto
    if min(w, h) == 0 or max(w, h) / min(w, h) > 1.33:
        return False
    if cv2.contourArea(c) / (w * h) < 0.85:    # llena su rectangulo (un circulo da ~0.78)
        return False
    aprox = cv2.approxPolyDP(hull, 0.06 * cv2.arcLength(hull, True), True)
    return 4 <= len(aprox) <= 5     # 5 tolera esquinas pixeladas en cubos chicos/lejanos


# ============================== FLECHAS ==============================
def analizar_flecha(c):
    """c: contorno de OpenCV. Devuelve dict o None si no parece flecha."""
    area = cv2.contourArea(c)
    if area < 300:
        return None
    hull_idx = cv2.convexHull(c, returnPoints=False)
    if hull_idx is None or len(hull_idx) < 4:
        return None
    try:
        defectos = cv2.convexityDefects(c, hull_idx)
    except cv2.error:
        return None
    if defectos is None or len(defectos) < 2:
        return None
    # 1) Los dos "huecos" mas profundos son el cuello (donde el cuerpo se une a la punta)
    defectos = sorted(defectos.reshape(-1, 4), key=lambda d: d[3], reverse=True)
    escala = math.sqrt(area)
    d1, d2 = defectos[0], defectos[1]
    if d2[3] / 256.0 < 0.06 * escala:      # hueco muy chico -> es un rectangulo, no flecha
        return None
    n1 = c[d1[2]][0].astype(float)
    n2 = c[d2[2]][0].astype(float)
    cuello = (n1 + n2) / 2
    # 2) Eje aproximado: perpendicular a la linea entre los dos puntos del cuello
    t = n2 - n1
    if np.linalg.norm(t) < 1e-6:
        return None
    t /= np.linalg.norm(t)
    eje0 = np.array([-t[1], t[0]])
    P = c[:, 0, :].astype(float)
    a0 = (P - cuello) @ eje0
    if a0.max() <= 0 or a0.min() >= 0:
        return None

    # 3) Probar las dos hipotesis (punta de un lado o del otro). Para cada una,
    #    el eje preciso va del cuello al punto mas lejano (la supuesta punta).
    #    La correcta es la que tiene la punta afinada y el cuerpo ancho al final.
    def evaluar(lado):
        tip = P[np.argmax(a0 * lado)]
        eje = tip - cuello
        if np.linalg.norm(eje) < 1e-6:
            return None
        eje /= np.linalg.norm(eje)
        perp = np.array([-eje[1], eje[0]])
        a = (P - cuello) @ eje
        lat_s = (P - cuello) @ perp
        lat = np.abs(lat_s)
        # una flecha es simetrica respecto de su eje: igual de ancha a cada lado
        simetria = min(lat_s.max(), -lat_s.min()) / max(lat_s.max(), -lat_s.min(), 1e-6)
        if a.min() >= 0:
            return None
        ancho_tip = lat[a > 0.8 * a.max()].max()
        ancho_cola = lat[a < 0.8 * a.min()].max()
        # las esquinas de la punta (lo mas ancho) tienen que estar cerca del cuello
        i_ancho = np.argmax(lat)
        if abs(a[i_ancho]) > 0.35 * a.max():
            return None
        if ancho_cola / lat.max() > 0.75:
            return None
        return dict(score=ancho_cola / max(ancho_tip, 1e-6) * simetria ** 2, eje=eje, a=a, lat=lat,
                    ancho_cola=ancho_cola, tip=tip)
    hip = [h for h in (evaluar(1), evaluar(-1)) if h]
    if not hip:
        return None
    h = max(hip, key=lambda x: x['score'])
    if h['score'] < 0.9:
        return None              # ningun lado se afina -> no es flecha
    eje, a, lat = h['eje'], h['a'], h['lat']
    ancho_punta = 2 * lat.max()             # las dos esquinas de la punta son lo mas ancho
    ancho_cuerpo = 2 * h['ancho_cola']
    largo_cuerpo = -a.min()
    if ancho_punta <= 0 or ancho_cuerpo / ancho_punta > 0.75:
        return None              # punta y cuerpo casi iguales -> no es flecha
    rel = largo_cuerpo / ancho_punta
    segmentos = 1 if rel < UMBRALES_LARGO[0] else (2 if rel < UMBRALES_LARGO[1] else 3)
    # 4) Angulo hacia donde apunta (0 = derecha, 90 = arriba, en grados, imagen)
    ang = math.degrees(math.atan2(-eje[1], eje[0]))
    direccion = clasificar_direccion(ang)
    return {
        'direccion': direccion,
        'angulo': ang,
        'segmentos': segmentos,
        'relacion': rel,
        'centro': tuple(map(float, cuello)),
        'punta': tuple(map(float, h['tip'])),
    }


def clasificar_direccion(ang):
    """Direccion mas cercana, solo si la inclinacion esta dentro de ANGULO_TOL."""
    for nombre, centro in (('derecha', 0), ('arriba', 90), ('izquierda', 180), ('abajo', -90)):
        desvio = abs((ang - centro + 180) % 360 - 180)
        if desvio <= ANGULO_TOL:
            return nombre
    return 'diagonal'          # demasiado inclinada: no se usa para navegar


# =============================== NODO ================================
class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('confirmar_frames', 5)
        self.br = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.pubs = {c: self.create_publisher(Point, f'/vision/cubo/{c}', 10) for c in RANGOS}
        self.pub_flecha = self.create_publisher(String, '/vision/flecha', 10)
        self.debug = self.create_publisher(Image, '/vision/debug', 10)
        self.kernel = np.ones((5, 5), np.uint8)
        self.ultima_flecha = None
        self.cuenta_flecha = 0

    def cb(self, msg):
        img = self.br.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        flechas = []

        for color, rangos in RANGOS.items():
            # 1) Mascara del color (una sola vez, sirve para cubos y flechas)
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for a, b in rangos:
                mask |= cv2.inRange(hsv, np.array(a), np.array(b))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # 2) Cada mancha: es cubo? es flecha? o nada?
            cubo_publicado = False
            for c in sorted(cnts, key=cv2.contourArea, reverse=True):
                area = cv2.contourArea(c)
                if area < AREA_MIN_CUBO:
                    break
                if es_cubo(c):
                    if not cubo_publicado:       # solo el cubo mas grande de cada color
                        x, y, w, h = cv2.boundingRect(c)
                        self.pubs[color].publish(Point(x=x + w / 2, y=y + h / 2, z=float(area)))
                        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 255), 2)
                        cv2.putText(img, f'cubo {color}', (x, y - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cubo_publicado = True
                    continue
                if area >= AREA_MIN_FLECHA:
                    f = analizar_flecha(c)
                    if f:
                        f['color'], f['area'] = color, area
                        flechas.append(f)
                        cx, cy = map(int, f['centro'])
                        px, py = map(int, f['punta'])
                        cv2.drawContours(img, [c], -1, (255, 255, 255), 2)
                        cv2.arrowedLine(img, (cx, cy), (px, py), (255, 255, 255), 2, tipLength=0.3)
                        cv2.putText(img, f"{color} {f['direccion']} x{f['segmentos']} {f['angulo']:.0f}",
                                    (cx - 60, cy - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        self.publicar_flecha(flechas)
        self.debug.publish(self.br.cv2_to_imgmsg(img, 'bgr8'))

    def publicar_flecha(self, flechas):
        """Publica la flecha mas cercana solo si se vio igual N cuadros seguidos."""
        flechas = [f for f in flechas if f['direccion'] != 'diagonal']
        if not flechas:
            self.ultima_flecha, self.cuenta_flecha = None, 0
            return
        f = max(flechas, key=lambda f: f['area'])       # la mas grande = la mas cercana
        clave = (f['color'], f['direccion'], f['segmentos'])
        self.cuenta_flecha = self.cuenta_flecha + 1 if clave == self.ultima_flecha else 1
        self.ultima_flecha = clave
        if self.cuenta_flecha >= self.get_parameter('confirmar_frames').value:
            texto = f"{f['color']},{f['direccion']},{f['segmentos']},{f['angulo']:.1f}"
            self.pub_flecha.publish(String(data=texto))


def main():
    rclpy.init()
    rclpy.spin(Vision())
