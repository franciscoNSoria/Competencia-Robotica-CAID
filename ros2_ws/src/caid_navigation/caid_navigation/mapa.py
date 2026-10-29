"""
Mapa de celdas y exploración estilo Trémaux (núcleo puro, sin ROS).

El laberinto se ve como una grilla de celdas. Cada lado de una celda es pared, paso o desconocido.
La exploración marca las celdas visitadas y en cada celda elige:

1. un paso hacia una celda no visitada, al lado de la actual (prefiere seguir derecho);
2. si no hay, el camino más corto (BFS) hasta la celda más cercana con algo por explorar;
3. si ya no queda nada, el camino más corto de vuelta al inicio.

Es la idea de Trémaux (marcar lo recorrido y no repetirlo) adaptada a un robot que sabe dónde está:
funciona con islas porque nunca sigue una pared y, después de cualquier desvío, retoma desde
la frontera más cercana.

Direcciones absolutas: 0 = +x, 1 = +y, 2 = -x, 3 = -y (sentido antihorario, como el yaw).
"""
from collections import deque

PASOS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def vecina(celda, d):
    dx, dy = PASOS[d]
    return (celda[0] + dx, celda[1] + dy)


def girar(rumbo, relativo):
    """Devuelve la dirección absoluta; relativo: 0 = frente, 1 = izq., 2 = atrás, 3 = der."""
    return (rumbo + relativo) % 4


class Mapa:

    def __init__(self, inicio=(0, 0), limites=None):
        self.inicio = inicio
        self.limites = limites  # (x_min, x_max, y_min, y_max): afuera se trata como pared
        self.lados = {}        # (celda, d) -> True si hay paso, False si hay pared
        self.visitadas = set()
        self.recorridos = set()  # pasos por los que pasó el robot: ninguna medición los cierra
        self.cerrados = set()    # pasos que no se usan aunque estén libres (la salida)
        self.bloqueados = set()  # pasos donde el robot frenó antes de entrar: pared hasta olvidar
        self.votos = {}          # paso -> [veces medido como pared, veces medido como paso]

    def dentro(self, celda):
        if self.limites is None:
            return True
        x_min, x_max, y_min, y_max = self.limites
        return x_min <= celda[0] <= x_max and y_min <= celda[1] <= y_max

    def abierto(self, celda, d):
        """Devuelve True si hay paso, False si hay pared y None si no se sabe."""
        return self.lados.get((celda, d))

    def marcar_lado(self, celda, d, hay_paso):
        """
        Suma una medición del lado. Gana la mayoría (en empate, la última).

        Así un obstáculo chico que a veces se ve y a veces no, no abre y cierra el paso en cada
        pasada (eso dejaba a la exploración yendo y viniendo entre dos ramas).
        """
        paso = frozenset((celda, vecina(celda, d)))
        if paso in self.recorridos:
            return
        if paso in self.bloqueados or not self.dentro(vecina(celda, d)):
            hay_paso = False
        else:
            votos = self.votos.setdefault(paso, [0, 0])
            votos[1 if hay_paso else 0] += 1
            if votos[0] != votos[1]:
                hay_paso = votos[1] > votos[0]
        self._fijar(celda, d, hay_paso)

    def _fijar(self, celda, d, hay_paso):
        self.lados[(celda, d)] = hay_paso
        self.lados[(vecina(celda, d), (d + 2) % 4)] = hay_paso

    def bloquear(self, celda, d):
        """El robot frenó antes de pasar: es pared, la midan como la midan, hasta olvidar."""
        self.bloqueados.add(frozenset((celda, vecina(celda, d))))
        self._fijar(celda, d, False)

    def recorrer(self, desde, d):
        """Anota que el robot pasó de desde a su vecina en d: ese lado es paso, sí o sí."""
        self._fijar(desde, d, True)
        self.recorridos.add(frozenset((desde, vecina(desde, d))))

    def cerrar(self, celda, d):
        """No explorar ni planificar por ese lado aunque esté libre (la salida, por ejemplo)."""
        self.cerrados.add(frozenset((celda, vecina(celda, d))))

    def abrir(self, celda, d):
        self.cerrados.discard(frozenset((celda, vecina(celda, d))))

    def olvidar(self, celda):
        """Borra lo medido en la celda (salvo los pasos recorridos) para volver a medirla."""
        for d in range(4):
            paso = frozenset((celda, vecina(celda, d)))
            if paso not in self.recorridos:
                self.lados.pop((celda, d), None)
                self.lados.pop((vecina(celda, d), (d + 2) % 4), None)
                self.votos.pop(paso, None)
                self.bloqueados.discard(paso)

    def _usable(self, celda, d):
        return frozenset((celda, vecina(celda, d))) not in self.cerrados

    def registrar(self, celda, rumbo, frente, izquierda, derecha):
        """Guarda lo que ven los tres ultrasonidos en el centro de la celda (True = paso)."""
        self.visitadas.add(celda)
        self.marcar_lado(celda, girar(rumbo, 0), frente)
        self.marcar_lado(celda, girar(rumbo, 1), izquierda)
        self.marcar_lado(celda, girar(rumbo, 3), derecha)

    def pendientes(self, celda):
        """Lados de la celda que todavía hay que explorar."""
        out = []
        for d in range(4):
            estado = self.abierto(celda, d)
            if not self._usable(celda, d):
                continue
            if estado is None or (estado and vecina(celda, d) not in self.visitadas):
                out.append(d)
        return out

    def queda_por_explorar(self):
        """Dice si alguna celda alcanzable desde el inicio tiene algo pendiente."""
        return self.camino(self.inicio,
                           lambda c: c in self.visitadas and bool(self.pendientes(c))) is not None

    def camino(self, desde, objetivo):
        """
        Busca el camino más corto (BFS) por pasos conocidos.

        Devuelve las direcciones hasta la primera celda que cumple objetivo(celda): [] si desde ya
        lo cumple y None si no se llega.
        """
        previo = {desde: None}
        cola = deque([desde])
        while cola:
            c = cola.popleft()
            if objetivo(c):
                pasos = []
                while previo[c] is not None:
                    c, d = previo[c]
                    pasos.append(d)
                return pasos[::-1]
            for d in range(4):
                n = vecina(c, d)
                if self.abierto(c, d) and self._usable(c, d) and n not in previo:
                    previo[n] = (c, d)
                    cola.append(n)
        return None

    def decidir(self, celda, rumbo, destino=None, preferida=None):
        """
        Elige la próxima dirección absoluta; None si ya exploró todo y está en el inicio.

        Si la dirección elegida es un lado desconocido, el robot solo tiene que girar hacia él y
        volver a medir: no hay que avanzar a ciegas.
        Con destino, en vez de explorar va por el camino más corto conocido hasta esa celda y
        devuelve None al llegar o si todavía no se conoce un camino.
        preferida es la dirección de una flecha del color buscado en esta celda: va antes que la
        exploración si está pendiente o es un paso por el que todavía no pasó.
        """
        if destino is not None:
            ruta = self.camino(celda, lambda c: c == destino)
            return ruta[0] if ruta else None
        locales = self.pendientes(celda)
        if preferida is not None and (preferida in locales or (
                self.abierto(celda, preferida)
                and frozenset((celda, vecina(celda, preferida))) not in self.recorridos)):
            return preferida
        if locales:
            # Frente, derecha, izquierda, atrás: menos giros primero.
            for rel in (0, 3, 1, 2):
                if girar(rumbo, rel) in locales:
                    return girar(rumbo, rel)
        ruta = self.camino(celda, lambda c: c in self.visitadas and self.pendientes(c))
        if ruta is None:
            ruta = self.camino(celda, lambda c: c == self.inicio)
        return ruta[0] if ruta else None

    def dibujo(self):
        """Dibujo del mapa: # pared, . paso o celda visitada, ? desconocido, I inicio."""
        if not self.visitadas:
            return ''
        xs = [c[0] for c in self.visitadas]
        ys = [c[1] for c in self.visitadas]
        filas = []
        for y in range(max(ys), min(ys) - 1, -1):
            arriba, medio = '', ''
            for x in range(min(xs), max(xs) + 1):
                c = (x, y)
                arriba += '+' + self._simbolo(c, 1, '-')
                medio += self._simbolo(c, 2, '|')
                if c == self.inicio:
                    medio += 'I'
                else:
                    medio += '.' if c in self.visitadas else ' '
            filas.append(arriba + '+')
            filas.append(medio + self._simbolo((max(xs), y), 0, '|'))
        filas.append('+' + '+'.join(self._simbolo((x, min(ys)), 3, '-')
                                    for x in range(min(xs), max(xs) + 1)) + '+')
        return '\n'.join(filas)

    def _simbolo(self, celda, d, pared):
        estado = self.abierto(celda, d)
        if estado is None:
            return '?'
        return ' ' if estado else pared
