"""
Giros en el lugar dentro de una celda de 0,32 m (núcleo puro, sin ROS).

El robot mide 278 x 163 mm: sus esquinas delanteras quedan a 162 mm del centro de giro y las
traseras
a 160 mm. El pasillo deja 160 mm a cada lado del eje, así que el robot **no puede girar centrado**:
roza la pared del frente o la de atrás (ver docs/arquitectura/navegacion.md).

La salida es girar corrido del centro: frenar antes o después del centro (`s`, a lo largo del
pasillo
por el que llega) y llegar corrido hacia un costado (`lat`). Este módulo busca el corrimiento más
chico
con el que el giro completo, ensanchado con un margen, no toca ningún bloque. Un giro de 180° en un
pasillo o en un callejón no tiene solución: ahí el robot retrocede.

Los bloques se pasan como cajas en el marco de la celda (centro de la celda en el origen y x hacia
donde mira el robot antes de girar), así el resultado vale para cualquier rumbo.
"""
from functools import lru_cache
import math

from caid_navigation.geometria import (
    CELDA, HUELLA_X, HUELLA_Y, MEDIA, poligono_toca_caja)

# Corrimientos que se prueban (m). |lat| queda limitado por el pasillo: 160 - 81,5 = 78,5 mm.
S_PRUEBA = [i / 100 for i in range(-6, 7)]
L_PRUEBA = [i / 100 for i in range(-5, 6)]
PASOS_ANGULO = 24


def _huella(s, lat, ang, margen):
    c, si = math.cos(ang), math.sin(ang)
    x0, x1 = HUELLA_X[0] - margen, HUELLA_X[1] + margen
    y0, y1 = HUELLA_Y[0] - margen, HUELLA_Y[1] + margen
    return [(s + c * px - si * py, lat + si * px + c * py)
            for px, py in ((x1, y1), (x1, y0), (x0, y0), (x0, y1))]


def barrido_libre(cajas, s, lat, ang0, ang1, margen):
    """True si el robot en (s, lat) puede girar de ang0 a ang1 sin tocar ninguna caja."""
    for k in range(PASOS_ANGULO + 1):
        ang = ang0 + (ang1 - ang0) * k / PASOS_ANGULO
        poli = _huella(s, lat, ang, margen)
        if any(poligono_toca_caja(poli, caja) for caja in cajas):
            return False
    return True


# El rumbo estimado puede estar corrido unos grados: el barrido de la huella se extiende ±3° más
# allá del giro pedido.
DUDA_RUMBO = 0.05


MARGENES = (0.04, 0.03, 0.022, 0.016, 0.012, 0.008, 0.006, 0.004)


@lru_cache(maxsize=8192)
def holgura(cajas, s, lat, giro):
    """Mayor margen de MARGENES con el que el giro en (s, lat) no toca nada (0 si ninguno)."""
    extra = math.copysign(DUDA_RUMBO, giro)
    for m in MARGENES:
        if barrido_libre(cajas, s, lat, -extra, giro + extra, m):
            return m
    return 0.0


@lru_cache(maxsize=8192)
def buscar_giro(cajas, giro, margen, l_fijo=None, s_actual=0.0):
    """
    Corrimiento (s, lat) para girar `giro` rad desde rumbo 0 con la mayor holgura posible (al
    menos `margen`), o None si no hay. Entre dos con la misma holgura, el más chico.

    cajas: tupla de cajas (xmin, ymin, xmax, ymax) en el marco de la celda.
    l_fijo: si el robot ya está quieto, su lateral no se puede cambiar sin moverse: solo se busca
    s.
    """
    if l_fijo is None:
        candidatos = [(si, li) for si in S_PRUEBA for li in L_PRUEBA]
    else:
        candidatos = [(si, l_fijo) for si in S_PRUEBA + [s_actual]]
    mejor = None
    for si, li in candidatos:
        h = holgura(cajas, si, li, giro)
        if h < margen - 1e-9:
            continue
        costo = abs(si - s_actual) if l_fijo is not None else math.hypot(si, li)
        clave = (-h, round(costo, 3))
        if mejor is None or clave < mejor[0]:
            mejor = (clave, (si, li))
    return None if mejor is None else mejor[1]


@lru_cache(maxsize=4096)
def angulo_barrido(cajas, s, lat, maximo, margen):
    """Mayor ángulo a <= maximo tal que el robot en (s, lat) puede girar de -a a +a, o 0."""
    a = maximo
    while a >= 0.12:
        if barrido_libre(cajas, s, lat, -a - DUDA_RUMBO, a + DUDA_RUMBO, margen):
            return a
        a -= 0.06
    return 0.0


def cajas_vecinas(ocupada, interiores=()):
    """
    Cajas del entorno de una celda, en el marco de la celda con rumbo 0.

    ocupada(dx, dy) -> True si la celda vecina (dx, dy), en ese marco, es un bloque o no se conoce.
    interiores: cajas extra en el mismo marco (soportes, obstáculos dentro de la celda).
    """
    cajas = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if (dx or dy) and ocupada(dx, dy):
                cx, cy = dx * CELDA, dy * CELDA
                cajas.append((round(cx - MEDIA, 4), round(cy - MEDIA, 4),
                              round(cx + MEDIA, 4), round(cy + MEDIA, 4)))
    cajas.extend(tuple(round(v, 3) for v in c) for c in interiores)
    return tuple(sorted(cajas))
