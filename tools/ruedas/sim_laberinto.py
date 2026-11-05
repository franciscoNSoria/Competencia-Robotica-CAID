"""Simulador de laberinto de celdas de 40 cm para probar laberinto_ruedas.py (usa el robot simulado de test_guiado.py).

Formato del texto (el de tools/tremaux/laberintos.py): '#' bloque de pared, '.' libre, 'S' partida mirando a +x. Fila 0 arriba.
Mapa: celda = (columna - columna_S, fila_S - fila), asi +y es hacia arriba del dibujo.
OJO: es un modelo; el piso real manda.
"""
import math
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', 'tremaux'))
import test_guiado as tg  # noqa: E402
import laberintos  # noqa: E402

CELDA = 0.40


class RobotLaberinto(tg.RobotPasillo):
    def __init__(self, texto, **kw):
        super().__init__(**kw)
        filas = [l for l in texto.split('\n') if l.strip()]
        self.filas = filas
        self.nf, self.nc = len(filas), max(len(f) for f in filas)
        sc = sf = None
        for r, fila in enumerate(filas):
            for c, ch in enumerate(fila):
                if ch == 'S':
                    sf, sc = r, c
        self.sc, self.sf = sc, sf
        self.x, self.y = 0.0, 0.0                     # el origen es el centro de la celda S
        seg = []
        for r in range(-1, self.nf + 1):
            for c in range(-1, self.nc + 1):
                if self._pared(c, r):
                    x0, y0 = (c - sc - 0.5) * CELDA, (sf - r - 0.5) * CELDA
                    x1, y1 = x0 + CELDA, y0 + CELDA
                    seg += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
        self.segmentos = seg
        self.pared_x = None
        self._sync()

    def _pared(self, c, r):
        if r < 0 or r >= self.nf or c < 0 or c >= len(self.filas[r]):
            return True
        return self.filas[r][c] == '#'

    def pared_en(self, x, y):
        c = int(math.floor(x / CELDA + 0.5 + self.sc))
        r = int(math.floor(self.sf - y / CELDA + 0.5))
        return self._pared(c, r)

    def _cuerpo(self):
        cs, sn = math.cos(self.th), math.sin(self.th)
        L, A = tg.LARGO / 2, tg.ANCHO / 2
        puntos = [(a, b) for a in (-L, -L / 2, 0, L / 2, L) for b in (-A, A)] + [(L, 0), (-L, 0)]
        for a, b in puntos:
            if self.pared_en(self.x + a * cs - b * sn, self.y + a * sn + b * cs):
                self.choco = True

    def celda(self):
        return (int(math.floor(self.x / CELDA + 0.5)), int(math.floor(self.y / CELDA + 0.5)))


def correr(nombre_o_texto, semilla=1, t_max=900.0, ganancia_giro=0.74, retardo=0.0, factor_der=1.0, imprimir=None, corr90=10.0,
           corr180=0.0, vel_dps=153.0, ruido=0.002, cada_tick=None):
    """cada_tick(robot, rec): se llama en cada paso (para inyectar flechas y cubos de una camara simulada)."""
    import tempfile
    import laberinto_ruedas as lr
    import ruedas_logica as rl
    texto = laberintos.todos().get(nombre_o_texto, nombre_o_texto)
    robot = RobotLaberinto(texto, semilla=semilla, ganancia_giro=ganancia_giro, retardo=retardo, factor_der=factor_der, ruido=ruido)
    log = []
    imp = imprimir or log.append
    carpeta = tempfile.mkdtemp()
    c = rl.Controlador(robot, carpeta=carpeta, reloj=lambda: 100.0 + robot.t, imprimir=lambda m: None)
    rec = lr.Recorrido(c, vel_dps=vel_dps, corr90=corr90, corr180=corr180, imprimir=imp, reloj=lambda: 100.0 + robot.t)
    for _ in range(8):
        robot.avanzar(0.05)
        c.tick()
    rec.arrancar()
    while robot.t < t_max and not rec.terminado() and not robot.choco:
        robot.avanzar(0.05)
        c.tick()
        rec.tick()
        if cada_tick:
            cada_tick(robot, rec)
    c.cerrar()
    return robot, rec, log
