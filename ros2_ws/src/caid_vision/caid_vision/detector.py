"""
Detección de flechas, cubos, canastos y zonas rojas en una imagen BGR (OpenCV clásico, sin ROS).

Entrega píxeles, no posiciones: la ubicación en el laberinto la calcula la navegación con la pose
del robot y el ángulo de la cámara (caid_navigation/camara.py). Formato (el mismo que publica
vision_node en /vision/detecciones, como JSON):

    {'flechas':  [{'color', 'punta': (u, v), 'cuello': (u, v), 'cola': (u, v), 'area',
                   'angulo', 'hombros': [(u, v), (u, v)]}],   # hombros: base de la punta
     'cubos':    [{'color', 'caja': (x, y, w, h), 'area'}],
     'canastos': [{'color', 'caja': (x, y, w, h), 'area'}],
     'zonas':    [(u, v), ...]}          # píxeles rojos del piso (líneas de zona prohibida)

Las funciones es_cubo y analizar_flecha vienen de vision_node.py (PER, Martín); acá se agregó la
cola de la flecha (hace falta para medir el largo del cuerpo en milímetros sobre la pared).
"""
import math

import cv2
import numpy as np

RANGOS = {  # HSV de OpenCV (H 0-180). Ajustar con la luz real.
    'rojo': [((0, 110, 40), (8, 255, 255)), ((172, 110, 40), (180, 255, 255))],
    'verde': [((40, 90, 40), (85, 255, 255))],
    'azul': [((88, 110, 40), (130, 255, 255))],   # el cubo azul real da H 89-109
}
# Brillo (V) y saturación (S) mínimos para todos los colores, además de los de RANGOS. Con la
# ESP32-CAM, las zonas oscuras (monitor, sombras) tienen un tinte verde/azul con V bajo y salían
# cubos y flechas falsos por toda la imagen. vision_node los cambia en vivo (parámetros v_min,
# s_min, area_min_*).
S_MIN = 0
AREA_MIN_CUBO = 150       # px² (antes 60: una manchita de 8x8 ya era un cubo)
AREA_MIN_FLECHA = 150     # px²; flecha chica nueva (30 mm, 3,45 cm²) a ~70 cm: ~200 px²
AREA_MIN_CANASTO = 250    # antes 120
# Una flecha nunca es maciza: los "hombros" entre punta y cuerpo le bajan la solidez (área /
# envolvente convexa). Flechas del reglamento (nov. 2026: cuerpo 10 mm de ancho, punta 29 mm de
# base y 10 mm de alto, largo total 30 / 60 / 95 mm): 0,64 / 0,58 / 0,55. Un cubo es ~1,0; girado
# se ve como un rombo y antes salía como flecha diagonal.
SOLIDEZ_MAX_FLECHA = 0.88
# Una flecha tiene exactamente 2 entradas profundas (los hombros, ~0,3 de la raíz del área) y el
# resto del borde casi liso. Las manchas de ruido de las zonas oscuras tienen hombros flojos
# (0,13-0,16) o varias entradas (la 3ª de 0,09 a 0,29). Probado con flechas chicas y oscuras
# (JPEG, ruido): con estos dos cortes se pierde 1 de 324 y no queda ninguna falsa en las fotos.
HOMBRO_MIN = 0.13    # el más profundo (flecha larga nueva de lejos, cuerpo de ~9 px: 0,13-0,20)
HOMBRO2_MIN = 0.10   # el otro: en una flecha chica (~50 px) uno sale flojo por los píxeles (0,13)
TERCERA_ENTRADA_MAX = 0.15   # 0,11 rechazaba la flecha azul real (punta serruchada: 0,14)
# Los hombros son esquinas agudas: el desenfoque, el JPEG y el filtro de la máscara los redondean
# y la base de la punta se mide ~8-12 px más corta, sin importar el tamaño (probado de 8 a 30
# px/cm). Se corre cada hombro esto hacia afuera para que caid_navigation/camara.py mida bien
# los 29 mm.
HOMBRO_CORRECCION_PX = 5.0
V_MIN = 55        # rojo y verde
V_MIN_AZUL = 40   # el azul es oscuro por naturaleza: el cubo azul real da V 47-92 (mediana 61)


def mascara(hsv, color):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for a, b in RANGOS[color]:
        v_min = V_MIN_AZUL if color == 'azul' else V_MIN
        a = (a[0], max(a[1], S_MIN), max(a[2], v_min))
        m |= cv2.inRange(hsv, np.array(a), np.array(b))
    return m


def es_cubo(c):
    """Cuadrado visto de frente: macizo (sin entradas), 4 lados y largo ~ ancho."""
    hull = cv2.convexHull(c)
    solidez = cv2.contourArea(c) / max(cv2.contourArea(hull), 1)
    if solidez < 0.85:      # con poca luz y JPEG los bordes salen serruchados (real: 0,90-0,91)
        return False
    (_, _), (w, h), _ = cv2.minAreaRect(c)
    # Girado 45° sobre su eje vertical se ve sqrt(2) = 1,41 veces más ancho que alto
    if min(w, h) == 0 or max(w, h) / min(w, h) > 1.5:
        return False
    if cv2.contourArea(c) / (w * h) < 0.75:   # real: 0,83-0,85; una flecha da 0,47-0,62
        return False
    aprox = cv2.approxPolyDP(hull, 0.06 * cv2.arcLength(hull, True), True)
    return 4 <= len(aprox) <= 6


def es_canasto(c):
    """Canasto visto de costado y un poco desde arriba: ancho >= 1,4 veces el alto y macizo."""
    x, y, w, h = cv2.boundingRect(c)
    hull = cv2.convexHull(c)
    solidez = cv2.contourArea(c) / max(cv2.contourArea(hull), 1)
    return w >= 1.4 * h and solidez > 0.85 and cv2.contourArea(c) / max(w * h, 1) > 0.7


def analizar_flecha(c):
    """c: contorno de OpenCV. Devuelve dict (punta, cuello, cola, segmentos...) o None."""
    area = cv2.contourArea(c)
    if area < 150:
        return None
    if area / max(cv2.contourArea(cv2.convexHull(c)), 1) > SOLIDEZ_MAX_FLECHA:
        return None     # demasiado maciza para ser flecha (cubo, canasto, mancha)
    hull_idx = cv2.convexHull(c, returnPoints=False)
    if hull_idx is None or len(hull_idx) < 4:
        return None
    try:
        defectos = cv2.convexityDefects(c, hull_idx)
    except cv2.error:
        return None
    if defectos is None or len(defectos) < 2:
        return None
    defectos = sorted(defectos.reshape(-1, 4), key=lambda d: d[3], reverse=True)
    escala = math.sqrt(area)
    d1, d2 = defectos[0], defectos[1]
    if d1[3] / 256.0 < HOMBRO_MIN * escala or d2[3] / 256.0 < HOMBRO2_MIN * escala:
        return None
    if len(defectos) > 2 and defectos[2][3] / 256.0 > TERCERA_ENTRADA_MAX * escala:
        return None     # borde irregular: mancha de ruido, no flecha
    n1 = c[d1[2]][0].astype(float)
    n2 = c[d2[2]][0].astype(float)
    cuello = (n1 + n2) / 2
    t = n2 - n1
    if np.linalg.norm(t) < 1e-6:
        return None
    t /= np.linalg.norm(t)
    eje0 = np.array([-t[1], t[0]])
    P = c[:, 0, :].astype(float)
    a0 = (P - cuello) @ eje0
    if a0.max() <= 0 or a0.min() >= 0:
        return None

    # ¿De qué lado del cuello está la cabeza? Cada entrada (esquina cuerpo-cabeza) queda entre dos
    # puntos de la envolvente: el hombro de la cabeza, pegado al cuello, y la esquina de la cola,
    # al final del cuerpo. La cola está del lado del punto lejano y la punta del otro. No mide
    # anchos (la cabeza nueva mide 1 cm de alto: pocos píxeles) y sirve para cualquier largo.
    lados, hombros = [], []
    for d in (d1, d2):
        q0, q1 = c[d[0]][0].astype(float), c[d[1]][0].astype(float)
        e0, e1 = (q0 - cuello) @ eje0, (q1 - cuello) @ eje0
        lejos, cerca = (e0, e1) if abs(e0) > abs(e1) else (e1, e0)
        if abs(cerca) > 0.6 * abs(lejos):
            return None     # la entrada no está entre hombro y cola: no es una flecha
        lados.append(-1 if lejos > 0 else 1)
        hombros.append(q1 if abs(e0) > abs(e1) else q0)   # el punto cercano es el hombro
    if lados[0] != lados[1]:
        return None         # las dos entradas no coinciden: mejor no adivinar la dirección
    lado = lados[0]
    # Eje: de la cola al cuello (el cuerpo es largo y recto). Desde el cuello hacia el punto más
    # adelantado fallaba: con la cabeza corta y ancha, a veces ese punto era un hombro.
    s0 = a0 * lado                       # + hacia la cabeza, - hacia la cola
    extremo_cola = P[s0 <= 0.85 * s0.min()]
    eje = cuello - extremo_cola.mean(axis=0)
    if np.linalg.norm(eje) < 1e-6:
        return None
    eje /= np.linalg.norm(eje)
    a = (P - cuello) @ eje
    tip = P[np.argmax(a)]
    if a.min() >= 0:
        return None
    # Cola: punto medio del extremo del cuerpo (los puntos con a cerca del mínimo).
    extremo = P[a < a.min() + 0.15 * (a.max() - a.min()) * 0.3]
    cola = extremo.mean(axis=0) if len(extremo) else cuello + eje * a.min()
    cola = cuello + eje * float((cola - cuello) @ eje)
    perp = np.array([-eje[1], eje[0]])
    hombros = [h + perp * np.sign((h - cuello) @ perp) * HOMBRO_CORRECCION_PX for h in hombros]
    return {'punta': tuple(map(float, tip)), 'cuello': tuple(map(float, cuello)),
            'cola': tuple(map(float, cola)), 'angulo': math.degrees(math.atan2(-eje[1], eje[0])),
            'hombros': [tuple(map(float, h)) for h in hombros]}


def detectar(img, horizonte=None, max_puntos_zona=400):
    """Detecciones de una imagen BGR (ver el formato arriba)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    alto = img.shape[0]
    horizonte = alto / 2 if horizonte is None else horizonte
    kernel = np.ones((3, 3), np.uint8)
    salida = {'flechas': [], 'cubos': [], 'canastos': [], 'zonas': []}
    for color in RANGOS:
        m = mascara(hsv, color)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        usados = np.zeros_like(m)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < AREA_MIN_CUBO:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if x <= 1 or x + w >= img.shape[1] - 1:
                continue        # cortado por el borde: la caja no sirve para medir
            # Primero cubo (prueba estricta: macizo y cuadrado), después flecha
            if es_cubo(c):
                salida['cubos'].append({'color': color, 'caja': (x, y, w, h), 'area': float(area)})
                cv2.drawContours(usados, [c], -1, 255, -1)
                continue
            if area >= AREA_MIN_FLECHA:
                f = analizar_flecha(c)
                if f:
                    f.update(color=color, area=float(area))
                    salida['flechas'].append(f)
                    cv2.drawContours(usados, [c], -1, 255, -1)
                    continue
            if area >= AREA_MIN_CANASTO and es_canasto(c) and y + h > horizonte:
                salida['canastos'].append({'color': color, 'caja': (x, y, w, h),
                                           'area': float(area)})
                cv2.drawContours(usados, [c], -1, 255, -1)
        if color == 'rojo':
            # Líneas de zona prohibida: rojo del piso que no es flecha, cubo ni canasto. Una zona
            # es un contorno (hueco) o una banda angosta vista en perspectiva; un pedazo de flecha
            # cortado por el borde de la imagen es macizo: se descarta.
            crudo = mascara(hsv, 'rojo')
            crudo[usados > 0] = 0
            crudo[:int(horizonte) + 2] = 0
            n, etiquetas, stats, _ = cv2.connectedComponentsWithStats(crudo)
            for i in range(1, n):
                x, y, w, h, area = stats[i]
                relleno = area / max(w * h, 1)
                if area < 15 or (relleno > 0.45 and h > 0.3 * w):
                    crudo[etiquetas == i] = 0
            vs, us = np.nonzero(crudo)
            if len(us) > max_puntos_zona:
                paso = len(us) // max_puntos_zona + 1
                us, vs = us[::paso], vs[::paso]
            salida['zonas'] = [(float(u), float(v)) for u, v in zip(us, vs)]
    return salida
