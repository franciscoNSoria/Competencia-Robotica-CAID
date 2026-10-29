"""
Detección de flechas, cubos, canastos y zonas rojas en una imagen BGR (OpenCV clásico, sin ROS).

Entrega píxeles, no posiciones: la ubicación en el laberinto la calcula la navegación con la pose
del robot y el ángulo de la cámara (caid_navigation/camara.py). Formato (el mismo que publica
vision_node en /vision/detecciones, como JSON):

    {'flechas':  [{'color', 'punta': (u, v), 'cuello': (u, v), 'cola': (u, v), 'area'}],
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
    'azul': [((95, 110, 40), (130, 255, 255))],
}
AREA_MIN_CUBO = 60
AREA_MIN_FLECHA = 250
AREA_MIN_CANASTO = 120


def mascara(hsv, color):
    m = np.zeros(hsv.shape[:2], np.uint8)
    for a, b in RANGOS[color]:
        m |= cv2.inRange(hsv, np.array(a), np.array(b))
    return m


def es_cubo(c):
    """Cuadrado visto de frente: macizo (sin entradas), 4 lados y largo ~ ancho."""
    hull = cv2.convexHull(c)
    solidez = cv2.contourArea(c) / max(cv2.contourArea(hull), 1)
    if solidez < 0.9:
        return False
    (_, _), (w, h), _ = cv2.minAreaRect(c)
    if min(w, h) == 0 or max(w, h) / min(w, h) > 1.33:
        return False
    if cv2.contourArea(c) / (w * h) < 0.85:
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
    if d2[3] / 256.0 < 0.06 * escala:
        return None
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
        simetria = min(lat_s.max(), -lat_s.min()) / max(lat_s.max(), -lat_s.min(), 1e-6)
        if a.min() >= 0:
            return None
        ancho_tip = lat[a > 0.8 * a.max()].max()
        ancho_cola = lat[a < 0.8 * a.min()].max()
        i_ancho = np.argmax(lat)
        if abs(a[i_ancho]) > 0.35 * a.max():
            return None
        if ancho_cola / lat.max() > 0.75:
            return None
        return dict(score=ancho_cola / max(ancho_tip, 1e-6) * simetria ** 2, eje=eje, a=a,
                    lat=lat, ancho_cola=ancho_cola, tip=tip)
    hip = [h for h in (evaluar(1), evaluar(-1)) if h]
    if not hip:
        return None
    h = max(hip, key=lambda x: x['score'])
    if h['score'] < 0.9:
        return None
    eje, a, lat = h['eje'], h['a'], h['lat']
    ancho_punta = 2 * lat.max()
    ancho_cuerpo = 2 * h['ancho_cola']
    if ancho_punta <= 0 or ancho_cuerpo / ancho_punta > 0.75:
        return None
    # Cola: punto medio del extremo del cuerpo (los puntos con a cerca del mínimo).
    extremo = P[a < a.min() + 0.15 * (a.max() - a.min()) * 0.3]
    cola = extremo.mean(axis=0) if len(extremo) else cuello + eje * a.min()
    cola = cuello + eje * float((cola - cuello) @ eje)
    return {'punta': tuple(map(float, h['tip'])), 'cuello': tuple(map(float, cuello)),
            'cola': tuple(map(float, cola)), 'angulo': math.degrees(math.atan2(-eje[1], eje[0]))}


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
            if area >= AREA_MIN_FLECHA:
                f = analizar_flecha(c)
                if f:
                    f.update(color=color, area=float(area))
                    salida['flechas'].append(f)
                    cv2.drawContours(usados, [c], -1, 255, -1)
                    continue
            if es_cubo(c):
                salida['cubos'].append({'color': color, 'caja': (x, y, w, h), 'area': float(area)})
                cv2.drawContours(usados, [c], -1, 255, -1)
            elif area >= AREA_MIN_CANASTO and es_canasto(c) and y + h > horizonte:
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
