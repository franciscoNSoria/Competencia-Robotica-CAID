"""
Geometría del robot y del laberinto (núcleo puro, sin ROS).

Marcos:
- `map`: nace en el centro de la celda de partida, con x hacia donde mira el robot al arrancar.
  La celda (i, j) tiene su centro en (i * CELDA, j * CELDA).
- `base_link`: x adelante, y a la izquierda, en el centro de la placa (hardware/cad/dimensions.yaml).

Direcciones absolutas: 0 = +x, 1 = +y, 2 = -x, 3 = -y (antihorario, como el yaw). Es la misma
convención que mapa.py.

Las medidas salen de hardware/cad/dimensions.yaml (en metros). Las que dicen P son supuestos.
"""
import math

CELDA = 0.32                 # m, pasillo y bloque de pared (docs/arquitectura/laberintos.md)
MEDIA = CELDA / 2

# Huella del robot en base_link: paredes frontal y trasera y ancho del paragolpes.
HUELLA_X = (-0.1382, 0.1399)
HUELLA_Y = (-0.0815, 0.0815)

# Ultrasonidos: (x, y, yaw, altura sobre el piso) en base_link. El cono simulado es de 15°.
SONARES = {
    'frente': (0.1399, 0.0, 0.0, 0.080),
    'izquierda': (0.096, 0.0815, math.pi / 2, 0.0738),
    'derecha': (0.096, -0.0815, -math.pi / 2, 0.0738),
}
CONO_SONAR = math.radians(15.0)

# Cámara: eje del servo de giro en (0.094, 0), centro óptico 7 mm por delante del eje, a 138,5 mm
# del piso. FOV horizontal 60° (gazebo.xacro), imagen de 640 x 480.
CAMARA_EJE_X = 0.094
CAMARA_ADELANTE = 0.00705
CAMARA_ALTURA = 0.1385
CAMARA_HFOV = 1.047
CAMARA_ANCHO = 640
CAMARA_ALTO = 480

PASOS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def ang_dif(a, b):
    """Devuelve a - b llevado a [-pi, pi)."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def limitar(x, tope):
    return max(-tope, min(tope, x))


def vecina(celda, d, n=1):
    return (celda[0] + n * PASOS[d][0], celda[1] + n * PASOS[d][1])


def centro(celda):
    return (celda[0] * CELDA, celda[1] * CELDA)


def celda_de(x, y):
    return (round(x / CELDA), round(y / CELDA))


def rumbo_de(yaw):
    """Dirección absoluta más cercana al yaw."""
    return round(yaw / (math.pi / 2)) % 4


def yaw_de(d):
    return ang_dif(d * math.pi / 2, 0.0)


def a_mundo(pose, px, py):
    """Punto (px, py) de base_link expresado en el marco de la pose (x, y, yaw)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return x + c * px - s * py, y + s * px + c * py


def a_robot(pose, wx, wy):
    """Punto (wx, wy) del mundo expresado en base_link."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = wx - x, wy - y
    return c * dx + s * dy, -s * dx + c * dy


def esquinas_huella(pose, margen=0.0):
    x0, x1 = HUELLA_X[0] - margen, HUELLA_X[1] + margen
    y0, y1 = HUELLA_Y[0] - margen, HUELLA_Y[1] + margen
    return [a_mundo(pose, px, py) for px, py in ((x1, y1), (x1, y0), (x0, y0), (x0, y1))]


def caja_de_celda(celda):
    cx, cy = centro(celda)
    return (cx - MEDIA, cy - MEDIA, cx + MEDIA, cy + MEDIA)


def poligono_toca_caja(puntos, caja):
    """Separación de ejes entre un polígono convexo y una caja alineada (xmin, ymin, xmax, ymax)."""
    xs = [p[0] for p in puntos]
    ys = [p[1] for p in puntos]
    if max(xs) <= caja[0] or min(xs) >= caja[2] or max(ys) <= caja[1] or min(ys) >= caja[3]:
        return False
    vertices = ((caja[0], caja[1]), (caja[2], caja[1]), (caja[2], caja[3]), (caja[0], caja[3]))
    n = len(puntos)
    for i in range(n):
        ax, ay = puntos[i]
        bx, by = puntos[(i + 1) % n]
        nx, ny = by - ay, ax - bx
        proy = [nx * px + ny * py for px, py in puntos]
        proy_c = [nx * vx + ny * vy for vx, vy in vertices]
        if max(proy) <= min(proy_c) or max(proy_c) <= min(proy):
            return False
    return True


def rayo_caja(ox, oy, dx, dy, caja):
    """Distancia a lo largo del rayo hasta la caja (None si no la toca). (dx, dy) unitario."""
    tmin, tmax = 0.0, math.inf
    for o, d, lo, hi in ((ox, dx, caja[0], caja[2]), (oy, dy, caja[1], caja[3])):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin
