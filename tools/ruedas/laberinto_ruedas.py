"""Trémaux por celdas con las piezas del panel de ruedas que ya andan en el piso (sin camara, sin ROS: se prueba con sim_laberinto.py).

Por cada celda de 40 cm:
  1. MIDE quieto en el centro: frente, izquierda y derecha con los sonares -> Mapa de caid_navigation (el Tremaux del repo).
  2. DECIDE con mapa.decidir (sigue derecho si puede, si no el camino mas corto a lo que falta; al terminar vuelve a la partida).
     Si la camara FIJO una flecha (/vision/proximo_giro, ver recibir_flecha) el giro de esta celda lo manda la flecha, una sola
     vez; si ese lado tiene pared se la ignora y sigue Trémaux. Sigue siempre el mismo color: el primero que adopta.
OBJETIVO (por ahora): llegar al CUBO del color que sigue. Cuando la camara lo ve a DIST_CUBO_M o menos (recibir_cubo) frena en
el acto y termina. No hay vuelta a la partida ni fin por el mapa: el mapa de celdas solo sirve para moverse entre flechas.
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
from mapa import girar, Mapa, vecina  # noqa: E402

CELDA = 0.40
F_X = 0.1399                          # el sonar frontal esta 14 cm adelante del centro
F_CENTRO = CELDA / 2 - F_X            # lo que lee el frontal en el centro de la celda mirando una pared (6 cm)
FRENTE_PARED_M = 0.16                 # el frontal lee menos que esto: hay pared de enfrente (abierto lee 40 cm o mas)
FRENTE_REFERENCIA_M = 1.2             # el frontal sirve de referencia hasta esta distancia (el HC-SR04 es poco confiable mas lejos)
LATERAL_PARED_M = 0.25                # un lateral lee menos que esto: hay pared (a 8,15 cm del eje, centrado lee ~12 cm)
POS_TOLERANCIA_M = 0.03               # pasado o corto del centro mas que esto: se corre antes de girar
T_ASENTAR_S, T_MEDIR_S = 0.4, 0.9      # quieto: espera que se asiente y mide hasta aqui (antes 0,7 y 1,5 s: tardaba mucho en decidir)
NOMBRES = ('+x', '+y', '-x', '-y')
VIGENCIA_FLECHA_S = 3.0               # el ultimo giro de la vision mas viejo que esto no se usa
DIST_CUBO_M = 0.20                    # cubo del color seguido a esto o menos de la camara: llegue
CUBO_LADO_M = 0.020
FX_CAMARA_PX = 950.0                  # calibracion_esp32cam.yaml (640x480)
SORDO_TRAS_GIRO_S = 2.5               # tras girar, la vision sigue mandando la flecha vieja hasta 2 s (vigencia_giro_s): se ignora


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
        self.busca = None                 # color de la flecha que se sigue (el primero que llega; la vision prefiere la P3)
        self.flecha = None                # ultimo giro de la vision: {'giro', 'color', 'tamano', 't'}
        self.sordo_hasta = 0.0
        self.dist_cubo, self.fx = DIST_CUBO_M, FX_CAMARA_PX
        self.al_adoptar_color = None      # callback(color): laberinto_real avisa a la vision por /nav/estado
        c.ajustar_frenado = False

    # ---- flechas (las manda laberinto_real desde /vision/proximo_giro)
    def recibir_flecha(self, datos):
        """datos: el JSON de /vision/proximo_giro ya leido ({'giro': 'derecha'|'izquierda', 'color', 'tamano', ...})."""
        ahora = self.reloj()
        giro, color = datos.get('giro'), datos.get('color')
        if giro not in ('derecha', 'izquierda') or ahora < self.sordo_hasta:
            return
        if self.busca is None and color:
            self.busca = color
            self.imprimir('  flecha: sigo el color %s (%s)' % (color, datos.get('tamano', '?')))
            if self.al_adoptar_color:
                self.al_adoptar_color(color)
        if color != self.busca:
            return
        nueva = self.flecha is None or self.flecha['giro'] != giro
        self.flecha = {'giro': giro, 'color': color, 'tamano': datos.get('tamano'), 't': ahora}
        if nueva:
            self.imprimir('  flecha %s %s: el proximo giro es a la %s' % (color, datos.get('tamano', '?'), giro.upper()))

    def recibir_cubo(self, color, area):
        """Cubo visto por la camara (area en px²). Si es del color seguido (o todavia no hay color) y esta cerca: llegue."""
        if self.estado in ('espera', 'fin', 'detenido') or area <= 0:
            return
        if self.busca is not None and color != self.busca:
            return
        dist = self.fx * CUBO_LADO_M / area ** 0.5
        if dist > self.dist_cubo:
            return
        self.c.parar()
        self.estado = 'fin'
        self.motivo = 'cubo %s' % color
        self.imprimir('== LLEGUE AL CUBO %s (a ~%.0f cm, celda %s) ==' % (color.upper(), 100 * dist, self.celda))

    def _preferida_flecha(self, ahora):
        """Direccion absoluta que pide la flecha vigente, o None."""
        f = self.flecha
        if f is None or ahora - f['t'] > VIGENCIA_FLECHA_S:
            return None
        return girar(self.rumbo, 1 if f['giro'] == 'izquierda' else 3)

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
        pref = self._preferida_flecha(ahora)
        dire = self.mapa.decidir(self.celda, self.rumbo, preferida=pref, insistir=True)
        f_usada = None
        if pref is not None:
            fl = f_usada = self.flecha
            self.flecha = None                                     # una flecha manda un solo giro
            if dire == pref:
                self.imprimir('  sigo la flecha %s: giro a la %s' % (fl['color'], fl['giro']))
            elif frente:                                           # la flecha esta en una pared mas adelante: es para otro cruce
                self.flecha, f_usada = fl, None
            else:
                self.imprimir('  flecha %s a la %s ignorada: ese lado tiene pared' % (fl['color'], fl['giro']))
        if dire is None:
            self.detener('recorri todo el laberinto y no llegue al cubo%s' % (' ' + self.busca if self.busca else ''))
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
            if f_usada is not None:                                # todavia no giro: la flecha sigue valiendo
                self.flecha = dict(f_usada, t=ahora)
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
        self.flecha, self.sordo_hasta = None, ahora + SORDO_TRAS_GIRO_S
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
