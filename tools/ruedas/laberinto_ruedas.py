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
import math
import os
import statistics
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', '..', 'ros2_ws', 'src', 'caid_navigation', 'caid_navigation'))
import ruedas_logica as rl  # noqa: E402
from mapa import Mapa, vecina  # noqa: E402
from seguidor import Seguidor  # noqa: E402   (solo su compactar(): une rectos y cancela las idas y vueltas a un callejon)

CELDA = 0.40
F_X = 0.1399                          # el sonar frontal esta 14 cm adelante del centro
F_CENTRO = CELDA / 2 - F_X            # lo que lee el frontal en el centro de la celda mirando una pared (6 cm)
FRENTE_GIRO_M = 0.19                  # el avance frena cuando el frontal lee esto; por inercia queda a ~15 cm y ahi gira
FRENTE_PARED_M = 0.22                 # el frontal lee menos que esto: hay pared de enfrente (abierto lee 40 cm o mas)
FRENTE_REFERENCIA_M = 1.2             # el frontal sirve de referencia hasta esta distancia (el HC-SR04 es poco confiable mas lejos)
LATERAL_PARED_M = 0.25                # un lateral lee menos que esto: hay pared (a 8,15 cm del eje, centrado lee ~12 cm)
POS_TOLERANCIA_M = 0.03               # pasado o corto del centro mas que esto: se corre antes de girar
T_ASENTAR_S, T_MEDIR_S = 0.4, 0.9      # quieto: espera que se asiente y mide hasta aqui (antes 0,7 y 1,5 s: tardaba mucho en decidir)
NOMBRES = ('+x', '+y', '-x', '-y')
PARADA_CUBO_M = 0.10                  # se detiene a esto del cubo (nunca mas cerca de 10 cm)
CUBO_STOP_FRENTE_M = 0.08             # mientras se acerca al cubo el frontal no lo frena a 15 cm (eso es para paredes): solo a esto, por seguridad
V_CUBO_MS = 0.08
CUBO_REACCION_M = 0.50                # el cubo a esta distancia o menos: empieza a acercarse despacio
CUBO_ANTES_M = 0.10                   # el avance guiado lo deja a esta distancia; el resto lo hace una orden de distancia
CUBO_LADO_M = 0.020
FX_CAMARA_PX = 950.0
_SIN_CUBO = ('espera', 'fin', 'detenido', 'esperar_y', 'cubo_fino', 'cubo_llego', 'media_cubo', 'media_cubo_fin', 'canasto', 'esperar_canasto', 'pausa_reentrar', 'reentrar_giro', 'retorno')


class Recorrido:
    def __init__(self, c, vel_dps=153.0, corr90=0.0, corr180=0.0, v=rl.V_LENTA_MS, frenada_m=0.03, imprimir=print, reloj=None, cuenta_giros=False):
        self.c, self.imprimir = c, imprimir
        self.cuenta_giros = cuenta_giros       # True: anota todos los rectos y giros hasta el cubo y vuelve repitiendolos al reves, a ciegas
        self.log, self.s_tramo0, self.plan = [], 0.0, None
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
        self.fx = FX_CAMARA_PX
        self.parada_cubo_m = PARADA_CUBO_M
        self.camino = [('celda', (0, 0), 0)]   # lo andado: celdas visitadas y giros, para el mapa del cubo
        self.cubo_lect, self.cubo_color, self.cubo_frenando = [], None, False
        self.ignora = set()                    # colores ya entregados: sus cubos (y sus flechas, para la vision) no cuentan
        self.al_cambiar_estado = None          # callback(ignora): laberinto_real avisa a la vision por /nav/estado
        self.entregado = False                 # el cubo en curso ya se dejo en el canasto
        self.volviendo = False                 # con el cubo ya 'capturado': vuelve a la partida (donde esta el canasto)
        c.ajustar_frenado = False

    # ---- control
    def arrancar(self):
        self.pos, self.intentos, self.volviendo = 0.0, {}, False
        self.log, self.s_tramo0, self.rumbo0 = [], self.c.odo.s, self.rumbo
        self._ir('medir', 'arranco')
        self.imprimir('== Trémaux con las piezas validadas: celda %s mirando %s ==' % (self.celda, NOMBRES[self.rumbo]))

    def confirmar(self):
        """Tecla Y: frente al cubo, da media vuelta, cuenta el mapa y vuelve a la partida; en la partida (canasto), cierra la entrega."""
        if self.estado == 'esperar_canasto':
            color = (self.cubo_color or '?')
            self.imprimir('  OK: cubo %s entregado en el canasto.' % color.upper())
            if self.cubo_color:
                self.ignora.add(self.cubo_color)               # ese color ya salio: ni su cubo ni sus flechas cuentan de ahora en mas
                self._avisar_estado()
            self.entregado = True
            self._pausa_reentrar('PAUSA: cubo %s entregado.' % color.upper())
            return True
        if self.estado == 'pausa_reentrar':
            self.imprimir('  OK: giro 180° y vuelvo a entrar a buscar el resto (ignoro: %s).' % (', '.join(sorted(self.ignora)) or 'nada'))
            self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)
            self._s_ini = self.c.odo.s
            self._ir('reentrar_giro')
            return True
        if self.estado != 'esperar_y':
            self.imprimir('  (el OK solo vale cuando estoy esperando frente al cubo)')
            return False
        self.imprimir('  OK: giro 180° y te cuento el camino que hice.')
        self._ir('media_cubo')
        return True

    def _avisar_estado(self):
        if self.al_cambiar_estado:
            self.al_cambiar_estado(sorted(self.ignora))

    def _pausa_reentrar(self, texto):
        self.imprimir('== %s Esperando el OK (tecla Y): con el OK giro 180° y vuelvo a entrar a buscar el resto de los cubos ==' % texto)
        self._ir('pausa_reentrar')

    def _reentrar_giro(self, ahora):
        """Termino el 180° en la partida: borra lo del cubo anterior y vuelve a explorar (el mapa que ya conoce se conserva)."""
        avance = self.c.odo.s - self._s_ini
        self.pos = -(self.pos + avance)
        self.rumbo = (self.rumbo + 2) % 4
        self.volviendo, self.entregado = False, False
        self.cubo_frenando, self.cubo_lect, self.cubo_color = False, [], None
        self.camino, self.cuenta_celdas, self.intentos = [('celda', self.celda, self.rumbo)], 0, {}
        self.log, self.s_tramo0, self.rumbo0 = [], self.c.odo.s, self.rumbo
        self._revisar_fin()
        if self.estado != 'detenido':
            self._ir('medir', 'vuelvo a entrar: celda %s mirando %s, busco el siguiente cubo' % (self.celda, NOMBRES[self.rumbo]))

    def recibir_cubo(self, color, area):
        """/vision/cubo/<color> (area en px2): si esta cerca, deja de explorar y se acerca despacio hasta pararse."""
        if self.volviendo or color in self.ignora or (self.estado in _SIN_CUBO and self.estado != 'cubo_fino') or area <= 0:
            return
        dist = self.fx * CUBO_LADO_M / math.sqrt(area)
        ahora = self.reloj()
        self.cubo_color = color
        self.cubo_lect = [(t, d) for t, d in self.cubo_lect if ahora - t < 1.5] + [(ahora, dist)]
        if self.estado == 'cubo_fino' or self.cubo_frenando or dist > CUBO_REACCION_M:
            return
        self.cubo_frenando = True
        self.imprimir('  CUBO %s a ~%.0f cm: dejo de explorar y me acerco despacio' % (color.upper(), 100 * dist))
        g = self.c.guiado
        if self.estado == 'avanzar_fin' and g is not None:           # esta avanzando: baja la velocidad y lo deja a CUBO_ANTES_M
            g['v'] = V_CUBO_MS
            g['stop_frente'] = CUBO_STOP_FRENTE_M                # el cubo adelante no es una pared: no frena a los 15 cm de las paredes
            g['dist'] = (self.c.odo.s - g['s0']) + max(0.01, dist - max(CUBO_ANTES_M, self.parada_cubo_m) - self.frenada_m)

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
        if self.estado in ('espera', 'fin', 'detenido', 'esperar_y', 'esperar_canasto', 'pausa_reentrar') or self._ocupado():
            return
        ahora = self.reloj()
        getattr(self, '_' + self.estado)(ahora)

    # ---- 1. medir y decidir
    def _medir(self, ahora):
        son = self.c.sonares()
        if son is None:
            self.detener('la ESP32 no informa los sonares')
            return
        if self.cubo_frenando:
            self._ir('cubo_fino')
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
        dire = self.mapa.decidir(self.celda, self.rumbo, destino=self.mapa.inicio) if self.volviendo else self.mapa.decidir(self.celda, self.rumbo)
        if dire is None and self.volviendo:
            if self.celda == self.mapa.inicio:
                self._ir('canasto')
            else:
                self.volviendo = False
                self._pausa_reentrar('PAUSA: no conozco un camino a la partida desde %s (se perdio el camino; el cubo %s NO lo di por entregado).' % (self.celda, (self.cubo_color or '?').upper()))
            return
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
    def _cerrar_tramo(self):
        """Cuenta giros: anota lo recto avanzado (odometria) desde el ultimo giro."""
        d = self.c.odo.s - self.s_tramo0
        if d > 0.02:
            self.log.append(('recto', d))
        self.s_tramo0 = self.c.odo.s

    def _girar(self, ahora):
        rel = (self.dire - self.rumbo) % 4
        self._cerrar_tramo()
        self.log.append(('media',) if rel == 2 else ('giro', 1 if rel == 1 else -1))
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
        self.s_tramo0 = self.c.odo.s                              # lo recto se cuenta desde que termino el giro
        if rel:
            self.camino.append(('giro', rel))
        self.rumbo = self.nuevo_rumbo
        self._revisar_fin()
        if self.estado != 'detenido':
            self._ir('medir', 'giro hecho: miro %s' % NOMBRES[self.rumbo])

    # ---- 3. avanzar una celda
    def _avanzar(self, ahora):
        dist = max(0.10, CELDA - self.pos - self.frenada_m)
        self._s_ini = self.c.odo.s
        if not self.c.enderezar_avanzando(dist, self.v, FRENTE_GIRO_M):
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
        if self.cubo_frenando:                                     # el avance lo interrumpio el cubo: se acerca y se para
            self.pos = hecho
            self._ir('cubo_fino')
            return
        if hecho >= CELDA - 0.19:                                  # llego (o casi: la pared de enfrente lo hizo frenar)
            self.mapa.recorrer(self.celda, self.rumbo)
            self.celda = vecina(self.celda, self.rumbo)
            self.pos = hecho - CELDA
            self.cuenta_celdas += 1
            self.camino.append(('celda', self.celda, self.rumbo))
            if self.volviendo and self.celda == self.mapa.inicio:
                self._ir('canasto')
                return
        else:
            self.mapa.bloquear(self.celda, self.rumbo)
            self.pos = hecho
            self.imprimir('  frene a %.0f cm: algo bloquea el paso, lo marco como pared' % (100 * hecho))
        self._ir('medir')

    # ---- cubo: se acerca, se para a ~4 cm, espera el OK, gira 180° y cuenta el camino
    def _cubo_fino(self, ahora):
        if ahora - self.t_estado < 0.8:
            return
        lect = [d for t, d in self.cubo_lect if ahora - t < 1.2]
        dist = sorted(lect)[len(lect) // 2] if lect else CUBO_ANTES_M          # sin lectura (muy cerca): lo que se dejo
        resto = dist - self.parada_cubo_m
        if resto > 0.015:
            self.imprimir('  cubo a ~%.0f cm: avanzo %.0f cm y paro a ~%.0f cm' % (100 * dist, 100 * resto, 100 * self.parada_cubo_m))
            self.pos += resto
            self.c.distancia(resto, V_CUBO_MS)
        self._ir('cubo_llego')

    def _cubo_llego(self, ahora):
        while self.pos > CELDA / 2:                            # el acercamiento al cubo puede haber cruzado celdas: se anotan
            self.mapa.recorrer(self.celda, self.rumbo)
            self.celda = vecina(self.celda, self.rumbo)
            self.mapa.visitadas.add(self.celda)
            self.pos -= CELDA
            self.cuenta_celdas += 1
            self.camino.append(('celda', self.celda, self.rumbo))
        self.imprimir('== CUBO %s a ~%.0f cm. ESPERANDO el OK (tecla Y) ==' % ((self.cubo_color or '?').upper(), 100 * self.parada_cubo_m))
        self._ir('esperar_y')

    def texto_camino(self):
        """Lineas del camino andado: de que celda a cual y que giros hizo."""
        out, ult = [], None
        for ev in self.camino:
            if ev[0] == 'celda':
                out.append('celda %s mirando %s' % (ev[1], NOMBRES[ev[2]]))
            else:
                out.append('   giro 90° a la %s' % ('izquierda' if ev[1] == 1 else 'derecha' if ev[1] == 3 else '180°'))
        return out

    def _media_cubo(self, ahora):
        self._cerrar_tramo()
        self.imprimir('== MAPA: asi llegue al cubo %s (%d celda(s) avanzadas) ==' % ((self.cubo_color or '?').upper(), self.cuenta_celdas))
        for t in self.texto_camino():
            self.imprimir('   ' + t)
        self.imprimir('   Ahora estoy en la celda %s mirando %s. Paredes que vi (# pared, . paso, ? desconocido, I inicio):' % (
            self.celda, NOMBRES[self.rumbo]))
        for fila in self.mapa.dibujo().split('\n'):
            self.imprimir('     ' + fila)
        self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)
        self._s_ini = self.c.odo.s
        self._ir('media_cubo_fin')

    def _media_cubo_fin(self, ahora):
        avance = self.c.odo.s - self._s_ini
        self.pos = -(self.pos + avance)                       # media vuelta: lo que estaba adelante queda atras
        self.rumbo = (self.rumbo + 2) % 4
        self.volviendo, self.cubo_frenando, self.cubo_lect = True, False, []
        self._revisar_fin()
        if self.estado != 'detenido' and self.cuenta_giros:
            self._empezar_retorno_giros()
            return
        if self.estado != 'detenido':
            self._ir('medir', 'media vuelta hecha: vuelvo a la partida %s por el camino mas corto que conozco' % (self.mapa.inicio,))

    # ---- cuenta giros: vuelve a ciegas repitiendo al reves lo anotado (sin mapa, sin camara, sin mirar bifurcaciones ni cubos)
    @staticmethod
    def texto_plan(plan):
        return ['recto %.0f cm' % (100 * e[1]) if e[0] == 'recto' else (('giro 90° IZQ' if e[1] > 0 else 'giro 90° DER') if e[0] == 'giro' else 'media vuelta')
                for e in plan]

    def _empezar_retorno_giros(self):
        camino = Seguidor.compactar(self.log)
        self.plan = [('giro', -e[1]) if e[0] == 'giro' else e for e in reversed(camino)]       # al reves: los giros se invierten
        self.imprimir('== VUELVO A CIEGAS repitiendo al reves los giros anotados (%d paso(s)): ==' % len(self.plan))
        for i, t in enumerate(self.texto_plan(self.plan), 1):
            self.imprimir('   %d. %s' % (i, t))
        self._ir('retorno')

    def _retorno(self, ahora):
        self._revisar_fin()
        if self.estado == 'detenido':
            return
        if not self.plan:                                           # termino lo anotado: esta de nuevo en la partida
            self.celda, self.rumbo, self.pos = self.mapa.inicio, (self.rumbo0 + 2) % 4, 0.0
            self._ir('canasto')
            return
        ev = self.plan.pop(0)
        self.imprimir('  [retorno] %s (%d paso(s) mas)' % (self.texto_plan([ev])[0], len(self.plan)))
        if ev[0] == 'recto':
            if not self.c.enderezar_avanzando(ev[1], self.v, FRENTE_GIRO_M):
                self.imprimir('  (pared de frente en el camino de vuelta: sigo con el paso que sigue)')
        elif ev[0] == 'giro':
            if not self.c.girar(ev[1] * (90.0 + self.corr90), self.vel_dps):
                self.detener('no pude empezar un giro de la vuelta')
        else:
            self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)

    def _canasto(self, ahora):
        self.imprimir('== LLEGUE A LA PARTIDA %s (el canasto). ESPERANDO el OK (tecla Y) para dar el cubo por entregado ==' % (self.mapa.inicio,))
        self._ir('esperar_canasto')

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
