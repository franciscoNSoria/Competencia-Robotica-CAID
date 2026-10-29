"""
Qué dicen los ultrasonidos del laberinto (núcleo puro, sin ROS).

- `RegistroLateral`: mientras el robot pasa por una celda, junta lo que miden los ultrasonidos de los
  costados cuando el sensor está cerca del centro de la celda, y al salir marca esos dos lados
  (pared si la mediana es menor que 0,20 m). Los sensores están 96 mm por delante del centro del
  robot, así que cada celda se registra antes de que el robot llegue a su centro.
- `leer_frente`: con el ultrasonido de frente decide hasta dónde sigue abierto el pasillo. El cono
  de 15° toca las paredes laterales de un pasillo de 0,32 m a 1,2 m, así que solo se usa hasta ~1 m.
  Si el eco no cae en un borde de celda, hay un obstáculo dentro de una celda.
- `Barrido`: mapa fino (1 cm) de la celda de enfrente armado girando el robot ±25° con el
  ultrasonido de frente. Dice si se puede pasar y con qué corrimiento lateral (un bloque de 8 cm
  pegado a una pared deja pasar al robot solo si va corrido), o si el paso está cerrado.
"""
import math

from caid_navigation.geometria import (
    CELDA, CONO_SONAR, HUELLA_Y, MEDIA, PASOS, SONARES, a_mundo, ang_dif, celda_de, centro,
    rumbo_de, yaw_de)

UMBRAL_PARED = 0.20


class RegistroLateral:

    def __init__(self, ventana=0.11):
        self.ventana = ventana
        self.datos = {}     # (celda, d) -> lecturas

    def actualizar(self, con, pose, rangos):
        """Devuelve la lista de (celda, d, abierto) que se cerraron en este paso."""
        x, y, th = pose
        h = rumbo_de(th)
        listos = []
        if abs(ang_dif(th, yaw_de(h))) > 0.15:
            return self._vaciar(con, listos)
        u = PASOS[h]
        activos = set()
        for lado, d_rel in (('izquierda', 1), ('derecha', 3)):
            sx, sy = a_mundo(pose, *SONARES[lado][:2])
            celda = celda_de(sx, sy)
            cx, cy = centro(celda)
            if abs((sx - cx) * u[0] + (sy - cy) * u[1]) <= self.ventana:
                clave = (celda, (h + d_rel) % 4)
                activos.add(clave)
                r = rangos.get(lado, math.inf)
                self.datos.setdefault(clave, []).append(r if math.isfinite(r) else 9.9)
        for clave in list(self.datos):
            if clave not in activos or len(self.datos[clave]) >= 6:
                listos.append(self._cerrar(con, clave))
        return listos

    def _vaciar(self, con, listos):
        for clave in list(self.datos):
            listos.append(self._cerrar(con, clave))
        return listos

    def _cerrar(self, con, clave):
        lecturas = sorted(self.datos.pop(clave))
        mediana = lecturas[len(lecturas) // 2]
        abierto = mediana > UMBRAL_PARED
        con.marcar(clave[0], clave[1], abierto, cerca=len(lecturas) >= 2)
        return clave[0], clave[1], abierto


def alcance_confiable(lateral, err_rumbo, duda=0.04):
    """Hasta qué distancia del sensor el cono de 15° no toca las paredes del pasillo."""
    libre = MEDIA - abs(lateral) - 0.015
    return max(0.0, libre / math.tan(CONO_SONAR / 2 + abs(err_rumbo) + duda))


def leer_frente(pose, f, tolerancia=0.045):
    """
    Interpreta la lectura de frente con el robot alineado al eje.

    Devuelve (abiertos, cerrado, sospecha):
    - abiertos: lista de (celda, d) que siguen abiertos hacia adelante;
    - cerrado: (celda, d) con pared en ese borde, o None;
    - sospecha: (celda, distancia) si el eco cae dentro de una celda (obstáculo), o None.
    """
    x, y, th = pose
    h = rumbo_de(th)
    err = ang_dif(th, yaw_de(h))
    if abs(err) > 0.10:
        return [], None, None
    u, n = PASOS[h], PASOS[(h + 1) % 4]
    sx, sy = a_mundo(pose, *SONARES['frente'][:2])
    celda0 = celda_de(x, y)
    cx, cy = centro(celda0)
    lat = (x - cx) * n[0] + (y - cy) * n[1]
    a_s = (sx - cx) * u[0] + (sy - cy) * u[1]       # sensor respecto del centro de celda0
    alcance = min(1.0, alcance_confiable(lat, err))
    if not math.isfinite(f):
        f = 99.0
    abiertos = []
    cerrado = None
    sospecha = None
    celda = celda0
    for k in range(6):
        borde = (k + 0.5) * CELDA - a_s           # distancia del sensor al borde k
        if borde < -0.02:
            celda = (celda[0] + u[0], celda[1] + u[1])
            continue
        if f < borde - tolerancia:
            # Eco antes del borde: algo dentro de esta celda (si está dentro del alcance confiable).
            if f <= alcance:
                sospecha = (celda, f)
            break
        if abs(f - borde) <= tolerancia:
            if f <= alcance:
                cerrado = (celda, h)
            break
        if borde + tolerancia > alcance:
            break
        abiertos.append((celda, h))
        celda = (celda[0] + u[0], celda[1] + u[1])
    return abiertos, cerrado, sospecha


class Barrido:
    """
    Celda de enfrente vista con el ultrasonido de frente mientras el robot gira en el lugar.

    Una lectura es "de pared" si no es más corta que la pared más cercana posible: el pasillo de la
    celda actual y la de enfrente cerrado por los costados y por el fondo. Esas lecturas solo liberan
    espacio. Una lectura más corta tiene que venir de un obstáculo dentro del pasillo: deja un arco de
    "posible obstáculo" a esa distancia, y lo ocupado es lo del arco que ninguna otra lectura liberó.
    """

    RES = 0.01

    def __init__(self, celda, h):
        self.celda = celda          # celda donde está el robot
        self.h = h
        self.objetivo = (celda[0] + PASOS[h][0], celda[1] + PASOS[h][1])
        self.libre = set()
        self.arcos = []
        self.lecturas = 0

    def _local(self, wx, wy):
        """Punto del mundo en el marco de la celda del robot (x adelante por h)."""
        cx, cy = centro(self.celda)
        u, n = PASOS[self.h], PASOS[(self.h + 1) % 4]
        return ((wx - cx) * u[0] + (wy - cy) * u[1], (wx - cx) * n[0] + (wy - cy) * n[1])

    def agregar(self, pose, f):
        sx, sy = a_mundo(pose, *SONARES['frente'][:2])
        ox, oy = self._local(sx, sy)
        yaw = ang_dif(pose[2], yaw_de(self.h))
        f_eff = min(f if math.isfinite(f) else 9.0, 0.8)
        R = self.RES
        arco = []
        for k in range(5):
            a = yaw + CONO_SONAR * (k / 4 - 0.5)
            c, s = math.cos(a), math.sin(a)
            t = 0.0
            while t < f_eff - 0.012:
                self.libre.add((round((ox + c * t) / R), round((oy + s * t) / R)))
                t += R / 2
            if math.isfinite(f) and f < 0.8:
                arco.append((round((ox + c * f) / R), round((oy + s * f) / R)))
        # El rumbo estimado puede estar corrido unos grados: la pared más cercana posible se calcula
        # con ±3° de error y la lectura tiene que ser 3 cm más corta que eso.
        pared = min(self._pared_minima(ox, oy, yaw + e) for e in (-0.05, 0.0, 0.05))
        if arco and math.isfinite(f) and f < pared - 0.03:
            self.arcos.append(arco)
        self.lecturas += 1

    @staticmethod
    def _pared_minima(ox, oy, yaw):
        """Distancia del sensor a la pared más cercana posible del pasillo (5 rayos del cono)."""
        mejor = math.inf
        for k in range(5):
            a = yaw + CONO_SONAR * (k / 4 - 0.5)
            c, s = math.cos(a), math.sin(a)
            t = math.inf
            if c > 1e-6:
                t = min(t, (3 * MEDIA - ox) / c)
            if s > 1e-6:
                t = min(t, (MEDIA - oy) / s)
            elif s < -1e-6:
                t = min(t, (-MEDIA - oy) / s)
            mejor = min(mejor, t)
        return mejor

    def ocupados(self):
        """
        Puntos (x, y) en metros, marco de la celda del robot, que quedan como obstáculo.

        Si hay puntos que marcan dos o más lecturas (en cuadros de 2 cm), se usan solo esos: el
        "ensanche" del cono cae en un lugar distinto en cada lectura y así se descarta. Si ninguno
        se repite (un obstáculo visto de pasada), se usan todos.
        """
        R = self.RES
        puntos = {}
        votos = {}
        for i, arco in enumerate(self.arcos):
            for p in set(arco):
                if p in self.libre:
                    continue
                x, y = p[0] * R, p[1] * R
                if not (MEDIA - 0.005 < x < 3 * MEDIA - 0.01 and abs(y) < MEDIA - 0.015):
                    continue
                cuadro = (round(x / 0.02), round(y / 0.02))
                votos.setdefault(cuadro, set()).add(i)
                puntos[(x, y)] = cuadro
        repetidos = [p for p, c in puntos.items() if len(votos[c]) >= 2]
        return sorted(repetidos or puntos)

    def ocupados_centrales(self, limite=0.13):
        """Ocupados lejos de las paredes del pasillo: los que se guardan como obstáculo."""
        return [p for p in self.ocupados() if abs(p[1]) < limite]

    def resultado(self, margen=0.025, max_desvio=0.065):
        """
        (pasable, desvio, puntos): desvio en m hacia la izquierda del eje h.

        pasable = False si con ningún corrimiento lateral hasta max_desvio la franja del robot
        (ancho 163 mm + margen a cada lado) queda libre de puntos ocupados.
        """
        pts = self.ocupados()
        if not pts:
            return True, 0.0, pts
        mitad = HUELLA_Y[1] + margen
        opciones = sorted((i / 200 for i in range(-int(max_desvio * 200), int(max_desvio * 200) + 1)),
                          key=abs)
        for l in opciones:
            if MEDIA - abs(l) - mitad < 0.004:
                continue
            if all(not (l - mitad < y < l + mitad) for _, y in pts):
                return True, l, pts
        return False, None, pts
