"""
Todo lo que el robot sabe del laberinto en la ronda actual (núcleo puro, sin ROS).

Empieza vacío en cada ronda (el reglamento no admite un mapa programado de antemano) y se llena con
los ultrasonidos y la cámara:

- `mapa`: los lados de cada celda (paso, pared o desconocido) y las celdas visitadas (mapa.py).
- `bloques` y `libres`: qué celdas son bloque de pared y cuáles son piso. Sirven para saber si el
  robot puede girar en una celda (giros.py).
- `prohibidas`: celdas con zona roja; nunca se entra.
- `cajas`: obstáculos dentro de una celda (soportes, bloques chicos, canastos), en el marco `map`.
- `desvio`: corrimiento lateral para cruzar una celda con un obstáculo pegado a una pared.
- `bloqueos`: pasos cerrados por un obstáculo (no por un bloque de pared).
"""
import math

from caid_navigation.geometria import CELDA, MEDIA, PASOS, centro, poligono_toca_caja, vecina
from caid_navigation.giros import angulo_barrido, buscar_giro, cajas_vecinas
from caid_navigation.mapa import Mapa

# Rotación de (dx, dy) del marco de una celda con rumbo h al marco map.
ROT = {0: lambda x, y: (x, y), 1: lambda x, y: (-y, x),
       2: lambda x, y: (-x, -y), 3: lambda x, y: (y, -x)}


class Conocimiento:

    def __init__(self, margen_giro=0.010):
        self.mapa = Mapa((0, 0))
        self.bloques = set()
        self.libres = {(0, 0)}
        self.prohibidas = set()
        self.cajas = {}
        self.desvio = {}
        self.bloqueos = set()
        self.votos = {}             # lado -> suma de votos (+ paso, - pared)
        self.giros_imposibles = set()   # (celda, rumbo, nuevo) que fallaron en la práctica
        self.margen_giro = margen_giro
        self.version = 0            # sube con cada cambio: el plan se rehace si cambió

    # ---------- escritura ----------

    def marcar(self, celda, d, abierto, cerca=True):
        """
        Registra una medida de un lado. Cada medida vota (2 de cerca, 1 de lejos) y el estado es
        el signo de la suma: una lectura aislada mal asignada no da vuelta un lado medido varias
        veces.
        """
        clave = (celda, d)
        if clave in self.bloqueos:
            return
        n = vecina(celda, d)
        canon = min(clave, (n, (d + 2) % 4))
        voto = (2 if cerca else 1) * (1 if abierto else -1)
        total = max(-6, min(6, self.votos.get(canon, 0) + voto))
        self.votos[canon] = total
        if total == 0:
            return
        estado = total > 0
        previo = self.mapa.abierto(celda, d)
        if previo == estado:
            return
        self.version += 1
        self.mapa.marcar_lado(celda, d, estado)
        if estado:
            self.libres.update((celda, n))
            self.bloques.discard(n)
        else:
            self.bloques.add(n)
            self.libres.discard(n)
            self.libres.add(celda)

    def bloquear(self, celda, d):
        """Cierra un paso por un obstáculo: la celda de enfrente sigue siendo piso."""
        if self.mapa.abierto(celda, d) is not False or (celda, d) not in self.bloqueos:
            self.version += 1
        self.mapa.marcar_lado(celda, d, False)
        n = vecina(celda, d)
        self.bloqueos.update(((celda, d), (n, (d + 2) % 4)))
        self.bloques.discard(n)

    def prohibir(self, celda):
        if celda not in self.prohibidas and celda != (0, 0):
            self.prohibidas.add(celda)
            self.version += 1
            return True
        return False

    def agregar_caja(self, celda, caja):
        self.cajas.setdefault(celda, []).append(caja)
        self.version += 1

    def visitar(self, celda):
        if celda not in self.mapa.visitadas:
            self.mapa.visitadas.add(celda)
            self.libres.add(celda)
            self.version += 1

    # ---------- lectura ----------

    def abierto(self, celda, d):
        return self.mapa.abierto(celda, d)

    def transitable(self, celda, d):
        return self.mapa.abierto(celda, d) is True and vecina(celda, d) not in self.prohibidas

    def visitada(self, celda):
        return celda in self.mapa.visitadas

    def ocupada(self, celda):
        """Para los giros: bloque, celda prohibida o desconocida."""
        return celda not in self.libres or celda in self.bloques

    def cajas_giro(self, celda, h):
        """Cajas del entorno de la celda en su marco con rumbo h (bloques, desconocidas, obstáculos)."""
        rot = ROT[h]
        inv = ROT[(4 - h) % 4]
        cx, cy = centro(celda)
        interiores = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                ax, ay = rot(dx, dy)
                for caja in self.cajas.get((celda[0] + ax, celda[1] + ay), ()):
                    esquinas = [inv(x - cx, y - cy) for x in (caja[0], caja[2])
                                for y in (caja[1], caja[3])]
                    xs = [e[0] for e in esquinas]
                    ys = [e[1] for e in esquinas]
                    interiores.append((min(xs), min(ys), max(xs), max(ys)))

        def ocupada(dx, dy):
            ax, ay = rot(dx, dy)
            return self.ocupada((celda[0] + ax, celda[1] + ay))
        return cajas_vecinas(ocupada, interiores)

    def barrido_posible(self, celda, h, s, l, maximo, margen=None):
        """Ángulo (rad) que el robot en (s, l) puede barrer a cada lado sin tocar nada."""
        return angulo_barrido(self.cajas_giro(celda, h), round(s, 3), round(l, 3), maximo,
                              self.margen_giro if margen is None else margen)

    def giro(self, celda, h, giro, l_fijo=None, s_actual=0.0, margen=None):
        """
        Corrimiento (s, l) para girar `giro` rad en la celda con rumbo inicial h, o None.

        Con l_fijo (robot quieto con ese lateral) solo se busca dónde frenar a lo largo del eje.
        """
        if l_fijo is not None:
            l_fijo, s_actual = round(l_fijo, 3), round(s_actual, 3)
        return buscar_giro(self.cajas_giro(celda, h), giro,
                           self.margen_giro if margen is None else margen, l_fijo, s_actual)

    def huella_libre(self, x, y, yaw, margen=0.01):
        """True si el robot en (x, y, yaw) no toca bloques, celdas desconocidas ni obstáculos."""
        from caid_navigation.geometria import caja_de_celda, esquinas_huella
        poli = esquinas_huella((x, y, yaw), margen)
        c0 = (round(x / CELDA), round(y / CELDA))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                c = (c0[0] + dx, c0[1] + dy)
                if self.ocupada(c) and poligono_toca_caja(poli, caja_de_celda(c)):
                    return False
                for caja in self.cajas.get(c, ()):
                    if poligono_toca_caja(poli, caja):
                        return False
        return True

    def libres_laterales(self, celda, eje, margen=0.02):
        """
        Tramos [(lo, hi)] donde puede ir el centro del robot al cruzar la celda por el eje
        (0 = x, 1 = y), en lateral respecto del eje de la celda (hacia +y en el eje x, hacia -x en
        el eje y). Descuenta las paredes y los obstáculos conocidos de la celda.
        """
        cx, cy = centro(celda)
        mitad = 0.0815 + margen
        lim = MEDIA - mitad
        prohibidos = []
        for caja in self.cajas.get(celda, ()):
            if eje == 0:
                b0, b1 = caja[1] - cy, caja[3] - cy
            else:
                b0, b1 = -(caja[2] - cx), -(caja[0] - cx)
            prohibidos.append((b0 - mitad, b1 + mitad))
        tramos = [(-lim, lim)]
        for p0, p1 in prohibidos:
            nuevos = []
            for t0, t1 in tramos:
                if p1 <= t0 or p0 >= t1:
                    nuevos.append((t0, t1))
                    continue
                if p0 > t0:
                    nuevos.append((t0, p0))
                if p1 < t1:
                    nuevos.append((p1, t1))
            tramos = nuevos
        return tramos

    def desvio_de(self, celda, eje):
        """Corrimiento lateral (m, a la izquierda del eje +x o +y) para cruzar la celda."""
        return self.desvio.get((celda, eje), 0.0)

    def junto_a_pared(self, celda):
        """True si alguna de las 8 celdas vecinas es un bloque conocido."""
        return any((celda[0] + dx, celda[1] + dy) in self.bloques
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy)

    def lados_abiertos(self, celda):
        return [d for d in range(4) if self.mapa.abierto(celda, d)]

    def distancia_a(self, celda, x, y):
        cx, cy = centro(celda)
        return math.hypot(x - cx, y - cy)

    def dibujo(self, pose=None):
        """Mapa en texto (para el log y las pruebas): # bloque, . visitada, Z prohibida, ? sin ver."""
        celdas = self.libres | self.bloques | self.prohibidas
        if not celdas:
            return ''
        xs = [c[0] for c in celdas]
        ys = [c[1] for c in celdas]
        robot = None
        if pose is not None:
            robot = (round(pose[0] / CELDA), round(pose[1] / CELDA))
        filas = []
        for y in range(max(ys), min(ys) - 1, -1):
            fila = ''
            for x in range(min(xs), max(xs) + 1):
                c = (x, y)
                if c == robot:
                    fila += 'R'
                elif c == (0, 0):
                    fila += 'S'
                elif c in self.prohibidas:
                    fila += 'Z'
                elif c in self.bloques:
                    fila += '#'
                elif c in self.mapa.visitadas:
                    fila += 'o' if c in self.cajas else '.'
                elif c in self.libres:
                    fila += ','
                else:
                    fila += ' '
            filas.append(fila)
        return '\n'.join(filas)


def eje_de(d):
    """0 si la dirección va por x, 1 si va por y."""
    return d % 2


def paso_vector(d):
    return PASOS[d]
