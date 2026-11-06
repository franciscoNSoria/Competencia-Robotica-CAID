"""Seguidor de flechas REACTIVO con las piezas del panel de ruedas (sin mapa de celdas, sin ROS: se prueba con el simulador).

Recorrido, todo desde la PC (el firmware es el de siempre, robot_4ruedas/main.c):
  1. AVANZA derecho enderezandose con los sonares (Controlador.enderezar_avanzando, el de LINEA RECTA, sin cambios).
  2. La camara (vision_node) fija una flecha cuando la ve cerca (distancia_fija) y manda /vision/proximo_giro. Esa flecha queda PENDIENTE.
  3. Con una flecha pendiente, apenas el sonar LATERAL del lado de la flecha ve el pasillo libre, el robot sigue `avance_giro_m` (12 cm) mas,
     enderezandose, y DOBLA 90 grados (el Giro 90 del panel, con tu sumador).
  4. Despues del giro avanza a ritmo LENTO (`lento_m`) mientras se endereza y sigue su camino a velocidad normal.
  5. Frente a una pared sin flecha: dobla hacia el lado libre (o la flecha, si ese lado esta libre); sin salida, media vuelta (dos giros de 90).
  6. CUBO: cuando la camara ve el cubo del color que sigue se acerca despacio y se detiene a ~`parada_cubo_m` (4 cm) de el. ESPERA a que se
     apriete Y (confirmar()): simula la captura (no lo toma).
  7. Con la Y VUELVE por el mismo camino hasta la ENTRADA (se anotan los tramos y los giros y se repiten al reves; las idas y vueltas a un
     callejon se cancelan), 'deja el cubo' (simulado), da media vuelta y VUELVE A ENTRAR a buscar el siguiente cubo IGNORANDO las flechas
     (y los cubos) del color que habia elegido.
Si algo sale mal (un giro no arranca, un avance se cae) se DETIENE y lo dice.
"""
import math
import os
import sys
from collections import deque

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import ruedas_logica as rl  # noqa: E402

LATERAL_ABIERTO_M = 0.28          # un lateral que lee mas que esto (o sin eco) ve un pasillo libre (pared a ~12 cm, centrado)
T_ABIERTO_S = 0.20                # ... durante esto seguido (los laterales se actualizan cada ~240 ms)
AVANCE_GIRO_M = 0.12              # desde que el lateral ve el pasillo libre sigue esto y dobla
V_NORMAL_MS = rl.V_LENTA_MS       # 0,16 m/s
V_LENTO_MS = 0.10                 # despues de doblar
LENTO_M = 0.25                    # ... durante este tramo
PARADA_CUBO_M = 0.04              # se detiene a esto del cubo
V_CUBO_MS = 0.08
CUBO_REACCION_M = 0.60            # el cubo a esta distancia o menos: empieza a acercarse despacio
CUBO_ANTES_M = 0.10               # el avance guiado lo deja a esta distancia; el resto lo hace una orden de distancia
CUBO_LADO_M = 0.020
FX_CAMARA_PX = 950.0
VIGENCIA_FLECHA_S = 10.0          # una flecha fijada sin usar vale esto (hasta llegar al cruce)
SORDO_TRAS_GIRO_S = 2.5           # tras doblar la vision sigue mandando la flecha vieja un rato: se ignora
MIN_RECTO_M = 0.08                # tras un giro no se mira el lateral hasta haber avanzado esto


class Seguidor:
    def __init__(self, c, vel_dps=153.0, corr90=0.0, corr180=0.0, v=V_NORMAL_MS, v_lento=V_LENTO_MS, lento_m=LENTO_M,
                 avance_giro_m=AVANCE_GIRO_M, parada_cubo_m=PARADA_CUBO_M, imprimir=print, reloj=None):
        self.c, self.imprimir = c, imprimir
        self.reloj = reloj or c.reloj
        self.vel_dps, self.corr90, self.corr180 = vel_dps, corr90, corr180
        self.v, self.v_lento, self.lento_m = v, v_lento, lento_m
        self.avance_giro_m, self.parada_cubo_m = avance_giro_m, parada_cubo_m
        self.fx = FX_CAMARA_PX
        self.estado, self.motivo = 'espera', ''
        self.t_estado = 0.0
        self.busca = None                   # color de flecha / cubo que sigue (el primero que llega)
        self.ignora = set()                 # colores ya entregados
        self.al_cambiar_estado = None       # callback(busca, ignora): laberinto/seguidor_real avisa a la vision por /nav/estado
        self.flecha = None                  # {'giro', 'color', 'tamano', 't'}: flecha fijada todavia sin usar
        self.sordo_hasta = 0.0
        self._ult_vista = None
        self.lado_giro = None               # 'izquierda' / 'derecha': el giro que sigue
        self.motivo_giro = ''
        self._t_abierto = None
        self.lento = False
        self.s_run0 = 0.0
        self.tramo = 0.0                    # recto acumulado desde el ultimo giro (m)
        self.log = []                       # camino: ('recto', m), ('giro', +1 izq / -1 der), ('media',)
        self.plan = deque()
        self.cubo_lect = []                 # (t, distancia) de la camara del cubo que sigue
        self.cubo_color = None
        self.cubo_frenando = False
        self.n_cubos = 0
        c.ajustar_frenado = False

    # ------------------------------------------------------------------ control desde afuera
    def arrancar(self):
        self.imprimir('== ARRANCO: avanzo derecho y espero las flechas ==')
        self._iniciar_recto(lento=False)

    def confirmar(self):
        """Tecla Y: simula la captura del cubo y vuelve a la entrada."""
        if self.estado != 'esperar_y':
            self.imprimir('  (la Y solo vale cuando estoy esperando frente al cubo)')
            return False
        self.imprimir('  Y: cubo %s capturado (simulado). Vuelvo por el camino hacia la entrada.' % (self.cubo_color or '?'))
        self._ir('retorno_giro')
        return True

    def detener(self, motivo='detenido'):
        self.c.parar()
        self.motivo = motivo
        self.estado = 'detenido'
        self.imprimir('  ! DETENIDO: ' + motivo)

    def terminado(self):
        return self.estado in ('fin', 'detenido')

    def _ir(self, estado, texto=None):
        self.estado, self.t_estado = estado, self.reloj()
        if texto:
            self.imprimir('  ' + texto)

    def _ocupado(self):
        c = self.c
        return c.maniobra is not None or c.marcha is not None or c.guiado is not None or bool(c.cola) or bool(c.secuencia)

    def _avisar_estado(self):
        if self.al_cambiar_estado:
            self.al_cambiar_estado(self.busca, sorted(self.ignora))

    # ------------------------------------------------------------------ vision (las llama seguidor_real desde ROS)
    def recibir_flecha(self, datos):
        """/vision/proximo_giro: {'giro': 'derecha'|'izquierda', 'color', 'tamano'}: la flecha FIJA. Queda pendiente hasta el cruce."""
        ahora = self.reloj()
        giro, color, tam = datos.get('giro'), datos.get('color'), datos.get('tamano', '?')
        if giro not in ('derecha', 'izquierda') or ahora < self.sordo_hasta or color in self.ignora:
            return
        if self.busca is None and color:
            self.busca = color
            self.imprimir('  >> adopto el color %s (%s): de aca en mas sigo solo ese' % (color.upper(), tam))
            self._avisar_estado()
        if color != self.busca:
            return
        clave = (color, giro, tam)
        if clave != self._ult_vista:
            self._ult_vista = clave
            self.imprimir('  vision: flecha %s %s FIJA -> doblo a la %s' % (color.upper(), tam, giro.upper()))
        self.flecha = {'giro': giro, 'color': color, 'tamano': tam, 't': ahora}

    def recibir_cubo(self, color, area):
        """/vision/cubo/<color> (area en px2): el cubo del color que sigue (o cualquiera no entregado si todavia no hay color)."""
        if self.estado not in ('recto', 'cubo_fino') or area <= 0 or color in self.ignora:
            return
        if self.busca is not None and color != self.busca:
            return
        dist = self.fx * CUBO_LADO_M / math.sqrt(area)
        ahora = self.reloj()
        self.cubo_color = color
        self.cubo_lect = [(t, d) for t, d in self.cubo_lect if ahora - t < 1.5] + [(ahora, dist)]
        if self.estado == 'recto' and not self.cubo_frenando and dist <= CUBO_REACCION_M and self.c.guiado is not None:
            g = self.c.guiado
            hecho = self.c.odo.s - g['s0']
            g['v'] = V_CUBO_MS
            g['dist'] = hecho + max(0.01, dist - CUBO_ANTES_M)
            self.cubo_frenando = True
            self.imprimir('  CUBO %s a ~%.0f cm: me acerco despacio' % (color.upper(), 100 * dist))

    def _flecha_vigente(self, ahora):
        f = self.flecha
        return f if (f is not None and ahora - f['t'] <= VIGENCIA_FLECHA_S) else None

    # ------------------------------------------------------------------ bucle (llamar despues de Controlador.tick)
    def tick(self):
        if self.estado in ('espera', 'fin', 'detenido', 'esperar_y'):
            return
        if self.estado == 'recto':
            if self.c.guiado is None and self.c.maniobra is None and not self.c.cola:
                self._fin_recto()
            else:
                self._vigilar()
            return
        if self._ocupado():
            return
        getattr(self, '_' + self.estado)(self.reloj())

    # ------------------------------------------------------------------ avance derecho
    def _iniciar_recto(self, lento=False, distancia=None):
        self.lento = lento
        self._t_abierto = None
        self.s_run0 = self.c.odo.s
        v = self.v_lento if lento else self.v
        if not self.c.enderezar_avanzando(distancia, v):               # frente pegado a una pared: ya hay que decidir
            self._ir('pared')
            return
        self._ir('recto')

    def _vigilar(self):
        ahora, g, c = self.reloj(), self.c.guiado, self.c
        if g is None:
            return
        hecho = c.odo.s - g['s0']
        if self.lento and hecho >= self.lento_m:                        # termino el tramo lento: velocidad normal (sin parar)
            g['v'] = self.v
            self.lento = False
        f = self._flecha_vigente(ahora)
        son = c.sonares()
        if f is None or self.lado_giro is not None or son is None or hecho < MIN_RECTO_M or self.cubo_frenando:
            return
        d = son[1] if f['giro'] == 'izquierda' else son[2]
        if d is None or d > LATERAL_ABIERTO_M:                          # el lateral del lado de la flecha ve un pasillo libre
            if self._t_abierto is None:
                self._t_abierto = ahora
            elif ahora - self._t_abierto >= T_ABIERTO_S:
                self.lado_giro, self.motivo_giro = f['giro'], 'flecha %s' % f['color']
                g['dist'] = max(hecho + 0.01, hecho + self.avance_giro_m)
                self.imprimir('  pasillo libre a la %s: sigo %.0f cm y doblo' % (f['giro'], 100 * self.avance_giro_m))
        else:
            self._t_abierto = None

    def _fin_recto(self):
        texto = getattr(self.c, 'ultimo_fin', '') or ''
        self.c.ultimo_fin = ''
        self.tramo += self.c.odo.s - self.s_run0
        if texto.startswith(('ME DETENGO', 'ABORTADO', 'Se cortaron')):
            self.detener(texto)
        elif self.cubo_frenando:
            self._ir('cubo_fino')
        elif self.lado_giro is not None:
            self._ir('girar')
        elif texto.startswith('Frene: pared de frente'):
            self._ir('pared')
        else:                                                           # tope de tiempo u otro: sigue
            self._iniciar_recto(lento=self.lento)

    # ------------------------------------------------------------------ pared de frente: elegir hacia donde doblar
    def _pared(self, ahora):
        if ahora - self.t_estado < 0.8:
            return
        son = self.c.sonares()
        if son is None:
            self.detener('la ESP32 no informa los sonares')
            return
        izq_libre = son[1] is None or son[1] > LATERAL_ABIERTO_M
        der_libre = son[2] is None or son[2] > LATERAL_ABIERTO_M
        f = self._flecha_vigente(ahora)
        if f is not None and ((f['giro'] == 'izquierda' and izq_libre) or (f['giro'] == 'derecha' and der_libre)):
            self.lado_giro, self.motivo_giro = f['giro'], 'flecha %s' % f['color']
        elif izq_libre and der_libre:
            self.lado_giro, self.motivo_giro = 'derecha', 'pared de frente, los dos lados libres'
        elif der_libre or izq_libre:
            self.lado_giro, self.motivo_giro = ('derecha' if der_libre else 'izquierda'), 'pared de frente, un solo lado libre'
        else:
            self._ir('media', 'callejon sin salida: media vuelta')
            return
        self._ir('girar')

    # ------------------------------------------------------------------ giros
    def _cerrar_tramo(self):
        if self.tramo > 0.02:
            self.log.append(('recto', self.tramo))
        self.tramo = 0.0

    def _girar(self, ahora):
        lado = self.lado_giro
        self.imprimir('  giro 90° a la %s (%s)' % (lado, self.motivo_giro))
        signo = 1 if lado == 'izquierda' else -1
        if not self.c.girar(signo * (90.0 + self.corr90), self.vel_dps):
            self.detener('no pude empezar el giro')
            return
        self._cerrar_tramo()
        self.log.append(('giro', signo))
        self.flecha, self.sordo_hasta = None, ahora + SORDO_TRAS_GIRO_S
        self.lado_giro = None
        self._ir('tras_giro')

    def _media(self, ahora):
        self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)
        self._cerrar_tramo()
        self.log.append(('media',))
        self.flecha, self.sordo_hasta = None, ahora + SORDO_TRAS_GIRO_S
        self._ir('tras_giro')

    def _tras_giro(self, ahora):
        self._iniciar_recto(lento=True)                                  # despues de doblar: lento mientras se endereza

    # ------------------------------------------------------------------ cubo
    def _cubo_fino(self, ahora):
        if ahora - self.t_estado < 0.8:
            return
        lect = [d for t, d in self.cubo_lect if ahora - t < 1.2]
        dist = sorted(lect)[len(lect) // 2] if lect else CUBO_ANTES_M       # sin lectura (muy cerca): lo que se dejo
        resto = dist - self.parada_cubo_m
        if resto > 0.015:
            self.imprimir('  cubo a ~%.0f cm: avanzo %.0f cm y paro a ~%.0f cm' % (100 * dist, 100 * resto, 100 * self.parada_cubo_m))
            self.tramo += resto
            self.c.distancia(resto, V_CUBO_MS)
        self._ir('cubo_llego')

    def _cubo_llego(self, ahora):
        self.n_cubos += 1
        self.imprimir('== CUBO %s a ~%.0f cm. ESPERANDO la tecla Y para capturarlo (simulado) ==' % (
            (self.cubo_color or '?').upper(), 100 * self.parada_cubo_m))
        self._ir('esperar_y')

    # ------------------------------------------------------------------ vuelta a la entrada
    @staticmethod
    def compactar(log):
        """Une rectos seguidos y cancela las idas y vueltas a un callejon (recto a, media vuelta, recto b -> recto a - b)."""
        out = []
        for ev in log:
            if ev[0] == 'recto':
                if len(out) >= 2 and out[-1][0] == 'media' and out[-2][0] == 'recto':
                    a = out[-2][1]
                    out.pop()
                    out.pop()
                    ev = ('recto', a - ev[1])
                if ev[1] <= 0.05:
                    continue
                if out and out[-1][0] == 'recto':
                    out[-1] = ('recto', out[-1][1] + ev[1])
                    continue
            out.append(ev)
        return out

    def _retorno_giro(self, ahora):
        self._cerrar_tramo()
        camino = self.compactar(self.log)
        self.plan = deque(('giro', -ev[1]) if ev[0] == 'giro' else ev for ev in reversed(camino))
        self.imprimir('  vuelvo a la entrada: %d paso(s) (%s)' % (len(self.plan), ', '.join(
            '%.0f cm' % (100 * e[1]) if e[0] == 'recto' else ('izq' if e[1] > 0 else 'der') if e[0] == 'giro' else '180' for e in self.plan)))
        self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)
        self._ir('retorno')

    def _retorno(self, ahora):
        if not self.plan:
            self._ir('entrega')
            return
        ev = self.plan.popleft()
        if ev[0] == 'recto':
            if not self.c.enderezar_avanzando(ev[1], self.v):
                self.imprimir('  (pared de frente en el camino de vuelta: sigo con el paso que sigue)')
        elif ev[0] == 'giro':
            if not self.c.girar(ev[1] * (90.0 + self.corr90), self.vel_dps):
                self.detener('no pude empezar un giro de la vuelta')
        else:
            self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)

    def _entrega(self, ahora):
        self.imprimir('== DEJE EL CUBO %s en la entrada (simulado) ==' % (self.cubo_color or '?').upper())
        if self.cubo_color:
            self.ignora.add(self.cubo_color)
        self.busca = None
        self._avisar_estado()
        self._ir('reentrar')

    def _reentrar(self, ahora):
        self.imprimir('  vuelvo a entrar a buscar el siguiente cubo (ignoro: %s)' % ', '.join(sorted(self.ignora)))
        self.c.giro_180_hacia_la_pared(self.vel_dps, self.corr180, None, self.corr90)
        self.log, self.tramo, self.flecha, self._ult_vista = [], 0.0, None, None
        self.cubo_frenando, self.cubo_lect, self.cubo_color = False, [], None
        self.sordo_hasta = ahora + SORDO_TRAS_GIRO_S
        self._ir('reentrar_listo')

    def _reentrar_listo(self, ahora):
        self._iniciar_recto(lento=False)
