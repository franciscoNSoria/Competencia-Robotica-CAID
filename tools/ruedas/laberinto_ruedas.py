"""Trémaux por celdas con las piezas del panel de ruedas que ya andan en el piso (sin camara, sin ROS: se prueba con sim_laberinto.py).

Por cada celda de 40 cm:
  1. MIDE quieto en el centro: frente, izquierda y derecha con los sonares -> Mapa de caid_navigation (el Tremaux del repo).
  2. DECIDE con mapa.decidir (sigue derecho si puede, si no el camino mas corto a lo que falta; al terminar vuelve a la partida).
  3. GIRA con Controlador.girar(±(90 + sumador del Giro 90 del panel)) o giro_180_hacia_la_pared (dos giros de 90).
  4. AVANZA UNA CELDA con Controlador.enderezar_avanzando (LINEA RECTA enderezando: corrige en marcha, se separa de la pared).
Si algo sale mal (no logra separarse de una pared, el pivote no arranca, una pared lateral no deja girar) se DETIENE y lo dice:
preferible a chocar. Se sigue la posicion a lo largo del pasillo (`pos`, m respecto del centro de la celda) con la odometria y
se la corrige con la pared de enfrente cuando se la ve.
"""
import os
import statistics
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', '..', 'ros2_ws', 'src', 'caid_navigation', 'caid_navigation'))
import ruedas_logica as rl  # noqa: E402
from mapa import Mapa, vecina  # noqa: E402

CELDA = 0.40
F_X = 0.1399                          # el sonar frontal esta 14 cm adelante del centro
F_CENTRO = CELDA / 2 - F_X            # lo que lee el frontal en el centro de la celda mirando una pared (6 cm)
FRENTE_PARED_M = 0.16                 # el frontal lee menos que esto: hay pared de enfrente (abierto lee 40 cm o mas)
FRENTE_REFERENCIA_M = 1.2             # el frontal sirve de referencia hasta esta distancia (el HC-SR04 es poco confiable mas lejos)
LATERAL_PARED_M = 0.25                # un lateral lee menos que esto: hay pared (a 8,15 cm del eje, centrado lee ~12 cm)
POS_TOLERANCIA_M = 0.03               # pasado o corto del centro mas que esto: se corre antes de girar
T_ASENTAR_S, T_MEDIR_S = 0.4, 0.9      # quieto: espera que se asiente y mide hasta aqui (antes 0,7 y 1,5 s: tardaba mucho en decidir)
NOMBRES = ('+x', '+y', '-x', '-y')


class Recorrido:
    def __init__(self, c, vel_dps=153.0, corr90=0.0, corr180=0.0, v=rl.V_LENTA_MS, frenada_m=0.03, imprimir=print, reloj=None):
        self.c, self.imprimir = c, imprimir
        self.vel_dps, self.corr90, self.corr180, self.v, self.frenada_m = vel_dps, corr90, corr180, v, frenada_m
        self.reloj = reloj or c.reloj
        self.mapa = Mapa(inicio=(0, 0))
        self.celda, self.rumbo, self.pos = (0, 0), 0, 0.0
        self.estado, self.motivo = 'espera', ''
        self.t_estado = 0.0
        self.muestras = []
        self.s0 = 0.0
        self.intentos = {}
        self.pasos = 0
        self.cuenta_celdas = 0
        c.ajustar_frenado = False

    # ---- control
    def arrancar(self):
        self.pos, self.intentos = 0.0, {}
        self._ir('medir', 'arranco')
        self.imprimir('== Trémaux con las piezas validadas: celda %s mirando %s ==' % (self.celda, NOMBRES[self.rumbo]))

    def detener(self, motivo='detenido'):
        self.c.parar()
        self.motivo = motivo
        self.estado = 'detenido'
        self.imprimir('  ! DETENIDO: ' + motivo)

    def terminado(self):
        return self.estado in ('fin', 'detenido')

    def _ir(self, estado, texto=None):
        self.estado, self.t_estado = estado, self.reloj()
        self.muestras = []
        if texto:
            self.imprimir('  [%s] %s' % (estado, texto))

    def _ocupado(self):
        c = self.c
        return c.maniobra is not None or c.marcha is not None or c.guiado is not None or bool(c.cola) or bool(c.secuencia)

    # ---- bucle (llamar despues de Controlador.tick)
    def tick(self):
        if self.estado in ('espera', 'fin', 'detenido') or self._ocupado():
            return
        ahora = self.reloj()
        getattr(self, '_' + self.estado)(ahora)

    # ---- 1. medir y decidir
    def _medir(self, ahora):
        son = self.c.sonares()
        if son is None:
            self.detener('la ESP32 no informa los sonares')
            return
        if ahora - self.t_estado < T_ASENTAR_S:
            return
        self.muestras.append(son)
        if ahora - self.t_estado < T_MEDIR_S or len(self.muestras) < 5:
            return
        f, i, d = (self._mediana([m[k] for m in self.muestras]) for k in range(3))
        frente = f is None or f > FRENTE_PARED_M
        izq = i is None or i > LATERAL_PARED_M
        der = d is None or d > LATERAL_PARED_M
        if f is not None and f < FRENTE_REFERENCIA_M:
            self.pos = self._pos_por_frente(f)           # las paredes estan en los bordes de las celdas: el frontal da donde estoy
        self.mapa.registrar(self.celda, self.rumbo, frente, izq, der)
        self.imprimir('  celda %s mirando %s: frente %s | izq %s | der %s | pos %+.1f cm  (f=%s i=%s d=%s)' % (
            self.celda, NOMBRES[self.rumbo], 'paso' if frente else 'PARED', 'paso' if izq else 'PARED', 'paso' if der else 'PARED',
            100 * self.pos, self._cm(f), self._cm(i), self._cm(d)))
        dire = self.mapa.decidir(self.celda, self.rumbo)
        if dire is None:
            self.estado = 'fin'
            self.imprimir('== TERMINE: %s ==' % ('recorri todo y volvi a la partida' if self.celda == self.mapa.inicio
                                                 else 'no hay camino a la partida desde %s' % (self.celda,)))
            return
        self.dire = dire
        rel = (dire - self.rumbo) % 4
        self.pasos += 1
        if self.pasos > 400:
            self.detener('mas de 400 decisiones: algo no anda')
            return
        if rel == 0:
            self._ir('avanzar', 'sigo derecho hacia %s' % (vecina(self.celda, dire),))
            return
        # corto del centro del cruce: avanza (nunca retrocede); si frenó a 10 cm de la pared de enfrente fue a proposito
        k = (self.celda, self.rumbo)
        if self.pos < -POS_TOLERANCIA_M and not (f is not None and f < 0.2) and self.intentos.get(k, 0) < 1:
            self.intentos[k] = 1
            self._corrida(-self.pos, 'estoy %.0f cm corto del centro: avanzo antes de girar' % abs(100 * self.pos))
            return
        self._ir('girar', 'giro 90° a la %s' % ('izquierda' if rel == 1 else 'derecha') if rel in (1, 3) else 'callejon: dos giros de 90°')

    def _corrida(self, metros, texto):
        """Avanza `metros` enderezandose y vuelve a medir."""
        self.imprimir('  ' + texto)
        self._s_ini = self.c.odo.s
        self.c.enderezar_avanzando(metros, self.v)
        self._ir('corrida_fin')

    def _corrida_fin(self, ahora):
        self.pos += self.c.odo.s - self._s_ini
        self._revisar_fin()
        if self.estado != 'detenido':
            self._ir('medir')

    def _revisar_fin(self):
        t = getattr(self.c, 'ultimo_fin', '') or ''
        self.c.ultimo_fin = ''
        if t.startswith('ME DETENGO') or t.startswith('ABORTADO') or t.startswith('Se cortaron'):
            self.detener(t)

    # ---- 2. girar
    def _girar(self, ahora):
        rel = (self.dire - self.rumbo) % 4
        self._s_ini = self.c.odo.s
        if rel == 2:
            self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)
            self.nuevo_rumbo = self.dire
            self._ir('girar_fin')
            return
        grados = (90.0 + self.corr90) * (1 if rel == 1 else -1)
        if not self.c.girar(grados, self.vel_dps):
            self.detener('no pude empezar el giro')
            return
        self.nuevo_rumbo = self.dire
        self._ir('girar_fin')

    def _girar_fin(self, ahora):
        rel = (self.nuevo_rumbo - self.rumbo) % 4
        avance = self.c.odo.s - self._s_ini
        self.pos = -(self.pos + avance) if rel == 2 else 0.0      # media vuelta: lo que estaba adelante queda atras
        self.rumbo = self.nuevo_rumbo
        self._revisar_fin()
        if self.estado != 'detenido':
            self._ir('medir', 'giro hecho: miro %s' % NOMBRES[self.rumbo])

    # ---- 3. avanzar una celda
    def _avanzar(self, ahora):
        dist = max(0.10, CELDA - self.pos - self.frenada_m)
        self._s_ini = self.c.odo.s
        if not self.c.enderezar_avanzando(dist, self.v):
            self.mapa.bloquear(self.celda, self.rumbo)            # frente pegado a una pared: no hay paso
            self.imprimir('  no avanzo: pared de enfrente; la marco y vuelvo a decidir')
            self._ir('medir')
            return
        self._ir('avanzar_fin')

    def _avanzar_fin(self, ahora):
        self._revisar_fin()
        if self.estado == 'detenido':
            return
        hecho = self.pos + (self.c.odo.s - self._s_ini)            # lugar donde quedo respecto del centro de la celda de partida
        if hecho >= CELDA - 0.12:                                  # llego (o casi: la pared de enfrente lo hizo frenar)
            self.mapa.recorrer(self.celda, self.rumbo)
            self.celda = vecina(self.celda, self.rumbo)
            self.pos = hecho - CELDA
            self.cuenta_celdas += 1
        else:
            self.mapa.bloquear(self.celda, self.rumbo)
            self.pos = hecho
            self.imprimir('  frene a %.0f cm: algo bloquea el paso, lo marco como pared' % (100 * hecho))
        self._ir('medir')

    # ---- util
    @staticmethod
    def _pos_por_frente(f):
        """Posicion a lo largo del pasillo (m respecto del centro de la celda) con la distancia a la pared de enfrente, que cae
        siempre en un borde de celda: centrado, el centro del cuerpo esta a 20 cm + k x 40 cm de ella."""
        r = (f + F_X - CELDA / 2) % CELDA
        pos = -r if r <= CELDA / 2 else CELDA - r
        return max(-0.15, min(0.15, pos))

    @staticmethod
    def _mediana(v):
        v = [9.0 if x is None else x for x in v]
        m = statistics.median(v)
        return None if m >= 9.0 else m

    @staticmethod
    def _cm(m):
        return '--' if m is None else '%.1f' % (100 * m)
