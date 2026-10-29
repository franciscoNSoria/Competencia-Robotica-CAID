"""
De los píxeles de la cámara al mapa (núcleo puro, sin ROS).

Modelo de cámara estenopeica sin inclinación, sobre el servo de giro (pan). Con la pose del robot y
el ángulo del servo, cada píxel es un rayo en el marco `map`. Con ese rayo:

- **Flechas**: el rayo del cuello de la flecha se cruza con las caras de los bloques que encuentra
  (recorriendo la grilla). Se vuelven a proyectar la punta, el cuello y la cola sobre el plano de
  cada cara candidata y se queda la cara en la que la punta mide lo que dice el reglamento (50 mm
  a lo largo del eje). En esa cara se miden en milímetros el largo del cuerpo (30 / 60 / 100 mm) y
  hacia qué lado apunta a lo largo de la pared, sin importar la perspectiva ni la inclinación.

Viene del navegador de la rama claude/happy-sagan-hzyge3, recortado a la parte de flechas.
"""
import math

from caid_navigation import geometria
from caid_navigation.geometria import (
    a_mundo, CAMARA_ADELANTE, CAMARA_ALTO, CAMARA_ALTURA, CAMARA_ANCHO, CAMARA_EJE_X, CAMARA_HFOV,
    celda_de, centro, PASOS)

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
        tx = ((cx + math.copysign(geometria.MEDIA, dx)) - ox) / dx if abs(dx) > 1e-9 else math.inf
        ty = ((cy + math.copysign(geometria.MEDIA, dy)) - oy) / dy if abs(dy) > 1e-9 else math.inf
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
    plano = cx * u[0] + cy * u[1] + geometria.MEDIA - 0.0015
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
