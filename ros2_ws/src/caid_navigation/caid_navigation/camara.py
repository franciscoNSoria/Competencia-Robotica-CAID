"""
De los píxeles de la cámara al mapa (núcleo puro, sin ROS).

Modelo de cámara estenopeica sin inclinación, sobre el servo de giro (pan). Con la pose del robot y
el ángulo del servo, cada píxel es un rayo en el marco `map`. Con ese rayo:

- **Flechas**: el rayo del cuello de la flecha se cruza con las caras de los bloques que encuentra
  (recorriendo la grilla). Se vuelven a proyectar la punta, el cuello y la cola sobre el plano de
  cada cara candidata y se queda la cara en la que la punta mide lo que dice el reglamento (50 mm
  a lo largo del eje). En esa cara se miden en milímetros el largo del cuerpo (30 / 60 / 100 mm) y
  hacia qué lado apunta a lo largo de la pared, sin importar la perspectiva ni la inclinación.
- **Cubos y canastos**: la distancia sale del borde de abajo del objeto (apoya a una altura
conocida,
  más abajo que la cámara) o, si está muy cerca del horizonte, de su ancho.
- **Zonas rojas**: los píxeles rojos del piso se proyectan al piso; las celdas cuyo centro queda
  dentro del rectángulo rojo son prohibidas.
"""
import math

from caid_navigation.geometria import (
    a_mundo, CAMARA_ADELANTE, CAMARA_ALTO, CAMARA_ALTURA, CAMARA_ANCHO, CAMARA_EJE_X, CAMARA_HFOV,
    celda_de, centro, MEDIA, PASOS)

CUBO_LADO = 0.020
CUBO_BASE_Z = 0.1085          # O: los cubos apoyan a la altura de la base del brazo
CANASTO_ANCHO = 0.044
CANASTO_BASE_Z = 0.0885       # P: borde del canasto a 108,5 mm, canasto de 20 mm de alto
FLECHA_PUNTA_LARGO = 0.050    # R: alto de la punta (a lo largo del eje)
LARGO_CUERPO = (0.045, 0.080)  # cortes entre 30 / 60 / 100 mm


class Camara:

    def __init__(self, ancho=CAMARA_ANCHO, alto=CAMARA_ALTO, hfov=CAMARA_HFOV, fx=None, fy=None,
                 cx=None, cy=None, altura=CAMARA_ALTURA):
        self.ancho, self.alto = ancho, alto
        self.fx = fx or (ancho / 2) / math.tan(hfov / 2)
        self.fy = fy or self.fx
        self.cx = ancho / 2 if cx is None else cx
        self.cy = alto / 2 if cy is None else cy
        self.altura = altura

    def origen(self, pose, pan):
        px = CAMARA_EJE_X + CAMARA_ADELANTE * math.cos(pan)
        py = CAMARA_ADELANTE * math.sin(pan)
        x, y = a_mundo(pose, px, py)
        return x, y, self.altura

    def rayo(self, pose, pan, u, v):
        """Origen (x, y, z) y dirección (sin normalizar) del píxel (u, v) en el marco map."""
        psi = pose[2] + pan
        f = (math.cos(psi), math.sin(psi))
        der = (math.sin(psi), -math.cos(psi))
        xc = (u - self.cx) / self.fx
        yc = (v - self.cy) / self.fy
        return self.origen(pose, pan), (f[0] + xc * der[0], f[1] + xc * der[1], -yc)

    def proyectar(self, pose, pan, p):
        """Píxel (u, v) de un punto 3D del mapa, o None si está detrás de la cámara."""
        o = self.origen(pose, pan)
        psi = pose[2] + pan
        dx, dy, dz = p[0] - o[0], p[1] - o[1], p[2] - o[2]
        z = dx * math.cos(psi) + dy * math.sin(psi)
        if z < 0.02:
            return None
        x = dx * math.sin(psi) - dy * math.cos(psi)
        return self.cx + self.fx * x / z, self.cy - self.fy * dz / z

    def suelo(self, pose, pan, u, v):
        """Punto del piso que ve el píxel, o None si está sobre el horizonte."""
        o, d = self.rayo(pose, pan, u, v)
        if d[2] > -1e-4:
            return None
        t = -o[2] / d[2]
        return o[0] + t * d[0], o[1] + t * d[1]

    def a_distancia(self, pose, pan, u, dist):
        """Punto horizontal a `dist` m (en planta) en la dirección de la columna u."""
        o, d = self.rayo(pose, pan, u, self.cy)
        n = math.hypot(d[0], d[1])
        return o[0] + dist * d[0] / n, o[1] + dist * d[1] / n


# ---------------------------------------------------------------------------------------------
# Recorrido de la grilla
# ---------------------------------------------------------------------------------------------
def caras_en_rayo(con, ox, oy, dx, dy, alcance=3.0):
    """
    Caras de bloque (o de celda desconocida) que cruza el rayo en planta, en orden.

    Devuelve [(t, celda_libre, d, conocida)]: el rayo entra en la celda vecina de `celda_libre` por
    su lado d a la distancia t. Termina en el primer bloque conocido.
    """
    n = math.hypot(dx, dy)
    dx, dy = dx / n, dy / n
    celda = celda_de(ox, oy)
    salida = []
    t = 0.0
    while t < alcance:
        cx, cy = centro(celda)
        tx = ((cx + math.copysign(MEDIA, dx)) - ox) / dx if abs(dx) > 1e-9 else math.inf
        ty = ((cy + math.copysign(MEDIA, dy)) - oy) / dy if abs(dy) > 1e-9 else math.inf
        if tx < ty:
            t, d = tx, 0 if dx > 0 else 2
        else:
            t, d = ty, 1 if dy > 0 else 3
        sig = (celda[0] + PASOS[d][0], celda[1] + PASOS[d][1])
        if sig in con.bloques or con.abierto(celda, d) is False:
            if (celda, d) not in con.bloqueos:
                salida.append((t, celda, d, True))
                return salida
        elif sig not in con.libres:
            salida.append((t, celda, d, False))
        celda = sig
    return salida


def _en_plano(o, d, celda, dpared):
    """Intersección del rayo (o, d) con el plano de la cara dpared de la celda (a 1,5 mm)."""
    u = PASOS[dpared]
    cx, cy = centro(celda)
    plano = cx * u[0] + cy * u[1] + MEDIA - 0.0015
    den = d[0] * u[0] + d[1] * u[1]
    if abs(den) < 1e-6:
        return None
    t = (plano - (o[0] * u[0] + o[1] * u[1])) / den
    if t <= 0:
        return None
    return (o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2])


def ubicar_flecha(con, cam, pose, pan, det, tolerancia=0.25):
    """
    det: {'color', 'punta': (u, v), 'cuello': (u, v), 'cola': (u, v)}.

    Devuelve {'celda', 'pared', 'dir', 'largo', 'color', 'xyz', 'error'} o None.
    """
    o, d = cam.rayo(pose, pan, *det['cuello'])
    mejor = None
    nd = math.hypot(d[0], d[1])
    for t, celda, dpared, _ in caras_en_rayo(con, o[0], o[1], d[0], d[1]):
        normal = PASOS[dpared]
        if abs(d[0] * normal[0] + d[1] * normal[1]) / nd < 0.4:
            continue        # pared vista muy de costado (más de 66°): la medida no es confiable
        pts = []
        for clave in ('punta', 'cuello', 'cola'):
            oo, dd = cam.rayo(pose, pan, *det[clave])
            p = _en_plano(oo, dd, celda, dpared)
            if p is None:
                break
            pts.append(p)
        if len(pts) < 3:
            continue
        punta, cuello, cola = pts
        largo_punta = math.dist(punta, cuello)
        error = abs(largo_punta / FLECHA_PUNTA_LARGO - 1)
        if error > tolerancia or (mejor is not None and error >= mejor['error']):
            continue
        if not (0.0 < cuello[2] < 0.30):
            continue
        tang = PASOS[(dpared + 1) % 4]
        eje = (punta[0] - cola[0], punta[1] - cola[1], punta[2] - cola[2])
        s = eje[0] * tang[0] + eje[1] * tang[1]
        if abs(s) < 0.5 * math.sqrt(sum(a * a for a in eje)):
            continue        # más de 60° de inclinación: no es una flecha del reglamento
        cuerpo = math.dist(cuello, cola)
        largo = 1 if cuerpo < LARGO_CUERPO[0] else (2 if cuerpo < LARGO_CUERPO[1] else 3)
        direccion = (dpared + 1) % 4 if s > 0 else (dpared + 3) % 4
        mejor = {'celda': celda, 'pared': dpared, 'dir': direccion,
                 'largo': largo, 'color': det['color'], 'xyz': cuello, 'error': error,
                 'cuerpo': cuerpo, 'distancia': t}
    return mejor


def ubicar_objeto(con, cam, pose, pan, caja, base_z, ancho_real):
    """
    Posición en planta de un cubo o canasto a partir de su caja en la imagen (x, y, w, h).

    Distancia: con el borde de abajo (el objeto apoya a base_z, más abajo que la cámara); si el
    borde queda a menos de 6 px del horizonte se usa el ancho. Devuelve (x, y, distancia) o None.
    """
    x, y, w, h = caja
    u = x + w / 2
    v_abajo = y + h
    caida = cam.altura - base_z
    if v_abajo - cam.cy > 6:
        prof = cam.fy * caida / (v_abajo - cam.cy)     # profundidad del borde de abajo más cercano
    else:
        prof = cam.fx * ancho_real / max(w, 1)
    # De profundidad (eje óptico) a distancia en planta por la columna u, y al centro del objeto.
    dist = prof / math.cos(math.atan((u - cam.cx) / cam.fx)) + ancho_real / 2
    o = cam.origen(pose, pan)
    px, py = cam.a_distancia(pose, pan, u, dist)
    # No puede estar detrás de una pared conocida: si el rayo choca antes, se acerca a la pared.
    _, d = cam.rayo(pose, pan, u, cam.cy)
    caras = [c for c in caras_en_rayo(con, o[0], o[1], d[0], d[1], alcance=dist + 0.3) if c[3]]
    if caras and caras[-1][0] < math.hypot(px - o[0], py - o[1]) - 0.01:
        t = caras[-1][0] - 0.01
        n = math.hypot(d[0], d[1])
        px, py = o[0] + t * d[0] / n, o[1] + t * d[1] / n
    return px, py, dist


def forma_de_cubo(cam, caja, corte=0.43, tolerancia_px=2.0):
    """
    True si una caja detectada como cubo tiene la forma vertical de un cubo apoyado a 108,5 mm.

    Con la cámara sin inclinación, un objeto entre las alturas z0 (abajo) y z1 (arriba), más abajo
    que la cámara, da (v_arriba - cy) / (v_abajo - cy) = (h_cam - z1) / (h_cam - z0) a cualquier
    distancia. Cubo: el borde de arriba que se ve es la arista de atrás, 2 cm más lejos, así que da
    10 / 30 * Z / (Z + 0,02) = 0,31-0,33. Canasto: 30 / 50 = 0,6. Un pedazo de flecha (en la pared,
    más alto) da menos. Se acepta de 0,32 - 2 px / (v_abajo - cy) (cuantización del borde de
    arriba, que queda a pocos píxeles del horizonte) hasta 0,43. Medido en el render (S): cubos
    0,24-0,33, canastos 0,52-0,58, pedazo de flecha 0,21 a 28 px.
    """
    _, y, _, h = caja
    abajo = y + h - cam.cy
    if abajo <= 2:
        return True     # sobre el horizonte: no se puede decir nada
    razon = (y - cam.cy) / abajo
    return 0.32 - tolerancia_px / abajo <= razon <= corte


def zonas_en_suelo(con, cam, pose, pan, puntos, minimo=4):
    """
    Votos de zona roja por celda en esta imagen: {celda: votos}.

    puntos: [(u, v)] de píxeles rojos del piso. Las líneas de una zona están entre 5 y 25 mm
    adentro
    del borde de su celda; con 2 o 3 cm de error de pose no se puede saber de qué lado del borde
    cae
    una línea suelta. Pero la zona es un rectángulo (convexo): la envolvente convexa de cualquier
    parte de su contorno queda adentro del rectángulo. Votan los puntos de esa envolvente que están
    a más de 3 cm de su borde. Una sola línea recta no vota (su envolvente es muy angosta).
    """
    o = cam.origen(pose, pan)
    suelo = []
    for u, v in puntos:
        p = cam.suelo(pose, pan, u, v)
        if p is None:
            continue
        dist = math.hypot(p[0] - o[0], p[1] - o[1])
        if dist > 1.6 or celda_de(*p) in con.bloques:
            continue
        caras = [c for c in caras_en_rayo(con, o[0], o[1], p[0] - o[0], p[1] - o[1], dist + 0.1)
                 if c[3]]
        if caras and caras[-1][0] < dist - 0.005:
            continue
        suelo.append(p)
    votos = {}
    for grupo in _agrupar(suelo, 0.06):
        if len(grupo) < 10:
            continue
        casco = _envolvente(grupo)
        if len(casco) < 3 or _area(casco) < 0.006:
            continue
        xs = [q[0] for q in casco]
        ys = [q[1] for q in casco]
        x = min(xs)
        while x <= max(xs):
            y = min(ys)
            while y <= max(ys):
                if _adentro(casco, (x, y), 0.03):
                    c = celda_de(x, y)
                    votos[c] = votos.get(c, 0) + 1
                y += 0.02
            x += 0.02
    return {c: n for c, n in votos.items() if n >= minimo and c not in con.bloques}


def _agrupar(puntos, sep):
    grupos = []
    for p in puntos:
        cerca = [g for g in grupos if any(abs(p[0] - q[0]) < sep and abs(p[1] - q[1]) < sep
                                          for q in g)]
        if not cerca:
            grupos.append([p])
            continue
        base = cerca[0]
        base.append(p)
        for g in cerca[1:]:
            base.extend(g)
            grupos.remove(g)
    return grupos


def _envolvente(pts):
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts

    def cruz(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    abajo, arriba = [], []
    for p in pts:
        while len(abajo) >= 2 and cruz(abajo[-2], abajo[-1], p) <= 0:
            abajo.pop()
        abajo.append(p)
    for p in reversed(pts):
        while len(arriba) >= 2 and cruz(arriba[-2], arriba[-1], p) <= 0:
            arriba.pop()
        arriba.append(p)
    return abajo[:-1] + arriba[:-1]


def _area(casco):
    return abs(sum(casco[i][0] * casco[i - 1][1] - casco[i - 1][0] * casco[i][1]
                   for i in range(len(casco)))) / 2


def _adentro(casco, p, margen):
    """True si p está dentro del polígono convexo (antihorario) a más de `margen` de sus lados."""
    n = len(casco)
    for i in range(n):
        a, b = casco[i], casco[(i + 1) % n]
        largo = math.hypot(b[0] - a[0], b[1] - a[1])
        if largo < 1e-9:
            continue
        if ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / largo < margen:
            return False
    return True


def celda_visible(con, cam, pose, pan, celda, margen=20):
    """
    True si se ve el piso de la mitad lejana de la celda (dentro de la imagen y sin pared delante).

    La cámara no ve el piso a menos de 0,33 m: desde la celda vecina solo alcanza la parte lejana
    de
    la celda, que es donde se mira (11 cm más allá del centro, en la dirección robot -> celda).
    """
    cx, cy = centro(celda)
    rx, ry = pose[0], pose[1]
    dd = math.hypot(cx - rx, cy - ry)
    if dd > 1e-6:
        cx, cy = cx + 0.11 * (cx - rx) / dd, cy + 0.11 * (cy - ry) / dd
    pix = cam.proyectar(pose, pan, (cx, cy, 0.0))
    if pix is None:
        return False
    u, v = pix
    if not (margen < u < cam.ancho - margen and cam.cy + 4 < v < cam.alto - 2):
        return False
    o = cam.origen(pose, pan)
    dist = math.hypot(cx - o[0], cy - o[1])
    caras = [c for c in caras_en_rayo(con, o[0], o[1], cx - o[0], cy - o[1], dist + 0.2) if c[3]]
    return not (caras and caras[-1][0] < dist - 0.01)
