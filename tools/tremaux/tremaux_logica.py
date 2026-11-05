"""Trémaux sin cámara, para recorrer un laberinto de 40 cm por celda con el robot real sin chocar (sin ROS: la usan tremaux.py y
el simulador).

Parte de tools/torneo/torneo_logica.py (conducción por sonares laterales con Kalman, giros por la ESP32, guardia contra la pared
de enfrente) y de caid_navigation/mapa.py (el Trémaux del repo: marca lo visitado, sigue derecho si puede, vuelve por el camino más
corto a la frontera más cercana). Se le sacó la cámara y el cubo: explora TODO el laberinto y vuelve a la partida. Lo que cambia
respecto de torneo_logica, todo para no chocar:

  1. Antes de girar hacia un ramal el robot tiene que estar en el centro del cruce también a lo largo del pasillo (3 cm pasado,
     4 cm corto como máximo). Si no, se corre con una orden de avance de la ESP32. Antes giraba así nomás y entraba al ramal
     corrido.
  2. Después de un giro, el _off (cuánto está corrido a lo largo del rumbo) se convierte: lo que estaba corrido a lo largo del
     rumbo viejo pasa a ser lateral, y el lateral pasa a ser a lo largo del rumbo nuevo. Antes se arrastraba el valor viejo.
  3. Inclinación (ENCARAR). Un giro que patina deja el robot inclinado 10-25°, y con eso el sonar frontal (que mira en cono) lee
     la pared del costado como si fuera una pared de enfrente: el mapa se llena de callejones sin salida falsos y el robot
     avanza hacia donde no debe. Si el mapa dice que adelante hay paso y el frontal lee menos de `f_libre`, está inclinado. El
     signo no se puede sacar de los sonares laterales (acierta el 50 %), así que se prueba: gira un paso chico, mira si el
     frontal mejora y, si empeora, invierte. Si lee lo mismo antes y después, es una pared de verdad.
  4. Si no logra dejar el robot derecho y centrado, NO sigue: se detiene y avisa (`SEGURIDAD`). Es preferible a chocar.
  5. Al terminar de explorar vuelve a la partida (no hay cubo que buscar) y se detiene ahí.
"""
import math
import os
import sys
from dataclasses import dataclass

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, '..', 'torneo'))
import torneo_logica as tl  # noqa: E402
from torneo_logica import (ALINEAR, CORRER, DETENIDO, ESPERA, FIN, GIRAR, MEDIR, PARANDO, RETROCEDER,  # noqa: E402,F401
                           VERIFICAR, NOMBRE_DIR, ang, lateral, mediana)

CENTRAR, ENCARAR = 'CENTRAR', 'ENCARAR'


class Observador(tl.Observador):
    """Igual que el de torneo_logica, pero solo toma un lateral como pared si el mapa dice que ahí hay pared, o si el mapa no sabe
    y lee muy cerca. En un cruce con salidas al costado el sonar ve los rincones de las salidas a 15-24 cm: con d_pared = 0,20
    el filtro los tomaba como una pared lejana, 'corregía' hacia el otro lado y el robot se iba contra la pared que ya tenía al
    lado. `tope` lo fija el piloto en cada paso, lado por lado (0 = ignorar ese lateral)."""

    def __init__(self):
        super().__init__()
        self.tope = {'izq': 0.17, 'der': 0.17}

    def corregir(self, s, izq, der, p):
        f = lambda d, lado: d if d is not None and d <= self.tope[lado] else None
        super().corregir(s, f(izq, 'izq'), f(der, 'der'), p)


@dataclass
class Parametros(tl.Parametros):
    off_pasado: float = 0.03       # m: pasado del centro del cruce más que esto, no se gira (el frente barre 16,1 cm contra 20)
    off_corto_pared: float = 0.04   # m: ... y con una pared de enfrente medida, que la guardia no deja acercarse más
    off_corto: float = 0.04        # m: corto más que esto, no se gira hacia un ramal (entraría corrido)
    f_libre: float = 0.40          # m: con paso adelante, el frontal tiene que leer más que esto (si no, está inclinado)
    f_pared: float = 0.10          # m: menos que esto es una pared de verdad pegada, no una inclinación
    paso_encarar: float = 0.21     # rad (12°): cada prueba de enderezado
    pasos_encarar: int = 6
    df_encarar: float = 0.03       # m: cambio del frontal que cuenta como "mejoró" o "empeoró"
    d_kalman: float = 0.17         # m: un lateral a esto o menos, con pared en el mapa, vale para el filtro (ver Observador)
    gating_mapa: bool = True       # el filtro usa un lateral solo si el mapa dice pared (False: siempre que lea <= d_kalman_plano)
    d_kalman_plano: float = 0.165
    d_kalman_nuevo: float = 0.145  # m: ... y si el mapa no sabe, solo si lee esto o menos (una pared real, bien cerca)
    v_giro_flancos: float = 75.0   # flancos/s por rueda en los giros grandes (orden 7): el máximo del firmware, 0,40 m/s por lado
    giro_chico: float = 0.5        # rad: por debajo de esto el giro va a la velocidad por defecto del firmware (30 flancos/s)
    giro_escala: float = 1.0       # (90 + sumador del panel) / 90: lo que hay que pedirle a la ESP32 para que el robot gire lo pedido (giros grandes)
    compensar_max: float = 0.12    # rad: el rumbo del Kalman no es confiable más allá de esto para descontarlo del giro


class PilotoTremaux(tl.Piloto):
    """tl.Piloto sin cámara, con el recorrido completo y los cambios de arriba."""

    def __init__(self, params=None, **kw):
        super().__init__(params or Parametros(), **kw)
        self.obs = Observador()
        self._centrados = 0
        self._y_virtual = 0.0
        self._prueba = None
        self._encarado = None            # (celda, rumbo) donde se comprobó que el frente corto es una pared de verdad
        self.eventos = []                # (t, texto) de lo que merece revisarse después: sirve al simulador y al CSV
        self.n_probes = 0

    def _topes(self):
        """Qué tan lejos puede leer cada lateral para valer como pared: según lo que el mapa sabe de la celda que tiene al lado."""
        p = self.p
        if self.estado == CORRER:
            sens = self._pos + tl.L_X
            k = round(sens / tl.CELDA)
            entre = abs(sens - k * tl.CELDA) > 0.10                 # cerca del límite entre dos celdas: no se sabe cuál es
            celda = self._celda_k(max(0, k))
        else:
            entre, celda = False, self.celda
        out = {}
        if not p.gating_mapa:
            return {'izq': p.d_kalman_plano, 'der': p.d_kalman_plano}
        for lado, rel in (('izq', 1), ('der', 3)):
            paso = None if entre else self.mapa.abierto(celda, (self.rumbo + rel) % 4)
            out[lado] = 0.0 if paso is True else (p.d_kalman if paso is False else p.d_kalman_nuevo)
        return out

    def paso(self, t, tele, s, th):
        self.obs.tope = self._topes()
        if self.estado == ENCARAR:          # girando en el lugar: el rumbo del Kalman no se predice (se reinicia al terminar)
            self._t_ult, self._s_ult, self._th_ult = t, s, th
            if not tele['fresco']:
                return self._vw(0.0, 0.0)
            return self._con_velocidad_de_giro(self._encarar(t, tele, s))
        return self._con_velocidad_de_giro(super().paso(t, tele, s, th))

    def _con_velocidad_de_giro(self, res):
        """Los giros grandes (orden 7) llevan la velocidad v_giro_flancos; los chicos (correcciones) la del firmware. Se arma acá,
        sobre lo que sale, porque torneo_logica manda siempre vel = 0 (y la repite así si hay que reenviar la orden)."""
        v, w, orden = res
        if orden is not None and orden[0] == 7 and abs(orden[1]) >= self.p.giro_chico:
            orden = (orden[0], orden[1] * self.p.giro_escala, self.p.v_giro_flancos)
        return v, w, orden

    # ------------------------------------------------------------------ utilidades
    def _evento(self, t, texto):
        self.eventos.append((t, texto))
        self.imprimir('  ! ' + texto)

    def _seguridad(self, t, motivo):
        self._evento(t, 'SEGURIDAD: ' + motivo)
        self._cambiar(DETENIDO, t, 'SEGURIDAD: ' + motivo)
        return self._vw(0.0, 0.0)

    def flecha(self, *a, **k):          # sin cámara
        return

    def ver_cubo(self, *a, **k):
        return

    def _sospecha_inclinado(self, f):
        """Hay paso adelante según el mapa y el frontal lee una pared a media distancia: es la pared del costado."""
        p = self.p
        if f is None or f >= p.f_libre or f < p.f_pared:
            return False
        if self._encarado == (self.celda, self.rumbo):
            return False
        return self.mapa.abierto(self.celda, self.rumbo) is not False

    # ------------------------------------------------------------------ quieto en una celda: mide y decide
    def _medir(self, t, tele, s):
        p = self.p
        if t - self.t_estado > 0.3 and tele['quieto']:
            self._muestras.append((tele['frente'], tele['izq'], tele['der']))
        if t - self.t_estado < p.t_medir or len(self._muestras) < 6:
            return self._vw(0.0, 0.0)
        f, mi, md = self._medido()
        if self._sospecha_inclinado(f):
            return self._empezar_encarar(t, tele, f, MEDIR)
        izq, der = lateral(mi, p), lateral(md, p)
        frente = f is None or f > tl.MEDIA + 0.05
        if f is not None and f < 0.25:
            self._off = tl.F_CENTRO - f
        self.obs.reiniciar(mediana([d for d in mi if d is not None and d <= p.d_kalman_nuevo]),
                           mediana([d for d in md if d is not None and d <= p.d_kalman_nuevo]))
        self.mapa.registrar(self.celda, self.rumbo, frente, True if izq is None else izq, True if der is None else der)
        self.imprimir('  celda %s rumbo %s: frente %s izq %s der %s  (f=%s)' % (
            self.celda, NOMBRE_DIR[self.rumbo], 'paso' if frente else 'pared', 'paso' if izq else 'pared',
            'paso' if der else 'pared', '--' if f is None else '%.3f' % f))
        return self._decidir(t, tele, f, mi, md)

    # ------------------------------------------------------------------ decidir
    def _decidir(self, t, tele, f, mi, md):
        p = self.p
        d = self._direccion(self.celda, self.rumbo)
        if d is None and self.modo == 'EXPLORAR':
            self.modo = 'VOLVER'
            self.guia = None
            self.imprimir('  no queda nada por explorar: vuelvo a la partida')
            d = self._direccion(self.celda, self.rumbo)
        if d is None:
            self._cambiar(FIN, t, 'de vuelta en la partida' if self.celda == self.mapa.inicio else 'no hay camino a la partida')
            return self._vw(0.0, 0.0)
        rel = (d - self.rumbo) % 4
        if rel == 0:
            self._centrados = 0
            return self._empezar_correr(t, tele)

        # 1. a lo largo del pasillo: el centro del cruce. Contra una pared de enfrente la guardia de la ESP32 no deja acercarse
        #    más que f = 0,10 m con una orden de avance (4 cm corto): ahí vale hasta 5,5 cm.
        corto = p.off_corto_pared if (f is not None and f < 0.25) else p.off_corto
        if self._off > p.off_pasado or self._off < -corto:
            if self._centrados < 2:
                self._centrados += 1
                self._cambiar(CENTRAR, t, 'estoy %.0f cm %s del centro del cruce: me corro antes de girar' % (
                    abs(100 * self._off), 'pasado' if self._off > 0 else 'corto'))
                self._ajuste = -self._off
                if f is not None and f < 0.25 and self._ajuste > 0:      # hacia la pared de enfrente: no más allá de la guardia
                    self._ajuste = max(0.0, min(self._ajuste, f - (p.d_stop + 0.006)))
                return self._vw(0.0, 0.0, self._mover(t, 3, self._ajuste, tele))
            if self._off > p.off_pasado or self._off < -p.off_corto_pared:
                return self._seguridad(t, 'no logro centrarme en el cruce (%.0f cm corrido)' % (100 * self._off))
            self.imprimir('  no puedo acercarme más al centro (%.0f cm): sigo' % (100 * self._off))

        # 2. a lo ancho: si estima estar corrido (o un costado queda muy cerca) retrocede y vuelve a entrar centrándose. Para
        #    girar vale la distancia que miden los laterales (la pared más cerca del barrido de 16,1 cm), no la y estimada.
        libre = self._puede_girar(mi, md)
        if abs(self.obs.y) > p.descentrado_max or not libre:
            if self._acomodos < 2 and abs(self.obs.psi) < 0.15 and self.mapa.abierto(self.celda, (self.rumbo + 2) % 4):
                self._acomodos += 1
                self._cambiar(RETROCEDER, t, 'quedé corrido %.0f cm del centro: retrocedo %.0f cm y vuelvo a entrar' % (
                    100 * self.obs.y, 100 * p.acomodar_atras))
                self._acomodando = True
                return self._vw(0.0, 0.0, self._mover(t, 3, -p.acomodar_atras, tele))
            if not libre:
                return self._seguridad(t, 'una pared queda a menos de %.0f cm del barrido del giro y no puedo reacomodarme' % (
                    100 * p.lateral_min_giro))
        self._acomodos = 0
        self._centrados = 0
        self._final = None
        giro = {1: math.pi / 2, 3: -math.pi / 2}.get(rel)
        if giro is None:                           # media vuelta: dos de 90 verificados, hacia el lado con más lugar
            li = mediana([x if x is not None else 9.0 for x in mi]) or 9.0
            ld = mediana([x if x is not None else 9.0 for x in md]) or 9.0
            giro = math.pi / 2 if li >= ld else -math.pi / 2
            self._final = d
            d = (self.rumbo + (1 if giro > 0 else 3)) % 4
        return self._girar_hacia(t, tele, d, giro, primera=True)

    def _girar_hacia(self, t, tele, d, giro, primera=False):
        self._prueba = None
        # El giro es sobre el centro del robot: lo que estaba corrido a lo largo del pasillo (_off) pasa a ser lateral y lo
        # lateral (y) pasa a ser a lo largo del rumbo nuevo. torneo_logica arrastraba el _off viejo (hasta 15 cm de error).
        if primera:
            self._y_virtual = self.obs.y
        sg = 1 if giro > 0 else -1
        self._off, self._y_virtual = sg * self._y_virtual, -sg * self._off
        return super()._girar_hacia(t, tele, d, giro)

    def _puede_girar(self, mi, md):
        """Girar en el lugar barre 16,1 cm: los dos laterales (los que ven pared) tienen que leer al menos lateral_min_giro."""
        for ms in (mi, md):
            validos = [d for d in ms if d is not None and d <= self.p.d_pared]
            if validos and mediana(validos) < self.p.lateral_min_giro:
                return False
        return True

    # ------------------------------------------------------------------ correrse a lo largo del pasillo
    def _centrar(self, t, tele, s):
        r = self._mov_terminado(t, tele)
        if r == 'reenviar':
            return self._vw(0.0, 0.0, (self._mov['ident'], self._mov['valor'], 0.0))
        if not r:
            return self._vw(0.0, 0.0)
        self._off += self._ajuste
        self._cambiar(MEDIR, t, 'mido de nuevo en el centro del cruce')
        return self._vw(0.0, 0.0)

    # ------------------------------------------------------------------ después de un giro: ¿quedó derecho?
    def _verificar(self, t, tele, s):
        p = self.p
        if t - self.t_estado > 0.3 and tele['quieto']:
            self._muestras.append((tele['frente'], tele['izq'], tele['der']))
        if t - self.t_estado < p.t_medir or len(self._muestras) < 6:
            return self._vw(0.0, 0.0)
        f, mi, md = self._medido()
        if self._final is not None and self.rumbo != self._final:
            # primera mitad de una media vuelta: mira una pared de costado, no hay con qué verificar; sigue con la segunda
            d = self._final
            self._final = None
            return self._girar_hacia(t, tele, d, self._ultimo_giro)
        if self._sospecha_inclinado(f):
            return self._empezar_encarar(t, tele, f, VERIFICAR)
        if f is not None and p.f_pared * 0.5 <= f < 0.25:
            self._off = tl.F_CENTRO - f
        self.obs.reiniciar(mediana([x for x in mi if x is not None and x <= p.d_kalman_nuevo]),
                           mediana([x for x in md if x is not None and x <= p.d_kalman_nuevo]),
                           psi=0.0, sigma_psi=math.radians(8))
        return self._decidir(t, tele, f, mi, md)

    # ------------------------------------------------------------------ enderezarse mirando el frontal
    def _empezar_encarar(self, t, tele, f, volver_a):
        s0 = 1 if self._ultimo_giro >= 0 else -1          # lo más común: el giro quedó corto, hay que seguir en su sentido
        paso = self.p.paso_encarar
        # blancos acumulados respecto de donde quedó: zigzag +12, -12, +24, -24, +36, -36 (un giro pasado de largo o muy corto)
        blancos = [s0 * k * paso * (1 if i % 2 == 0 else -1) for i, k in enumerate((1, 1, 2, 2, 3, 3))]
        self._prueba = dict(f_ant=f, f0=f, pos=0.0, blancos=blancos, n=0, volver_a=volver_a, ultimo=0.0, sigue=None,
                            deshacer=False)
        self._evento(t, 'celda %s: el frente lee %.2f m con paso adelante: sospecho que estoy inclinado, pruebo' % (self.celda, f))
        return self._probar(t, tele)

    def _probar(self, t, tele, destino=None):
        p = self.p
        pr = self._prueba
        if pr['n'] >= p.pasos_encarar:
            return self._seguridad(t, 'no logro enderezar el robot (el frente sigue leyendo %.2f m, el mapa dice que hay paso)' % pr['f_ant'])
        if destino is None:
            destino = pr['blancos'][pr['n']]
        giro = destino - pr['pos']
        pr['n'] += 1
        pr['ultimo'] = giro
        pr['pos'] = destino
        self.n_probes += 1
        self._cambiar(ENCARAR, t, 'prueba %d: giro %+.0f° (acumulado %+.0f°) y miro el frente' % (
            pr['n'], math.degrees(giro), math.degrees(destino)))
        return self._vw(0.0, 0.0, self._mover(t, 7, giro, tele))

    def _encarar(self, t, tele, s):
        """Espera que termine la prueba de giro, mide el frontal y decide la próxima."""
        p = self.p
        pr = self._prueba
        if 'medir_desde' not in pr:
            r = self._mov_terminado(t, tele)
            if r == 'reenviar':
                return self._vw(0.0, 0.0, (self._mov['ident'], self._mov['valor'], 0.0))
            if not r:
                return self._vw(0.0, 0.0)
            if pr['deshacer']:
                return self._encarado_listo(t, 'era una pared de verdad: deshice la prueba')
            pr['medir_desde'] = t
            self._muestras = []
        if t - pr['medir_desde'] > 0.3 and tele['quieto']:
            self._muestras.append((tele['frente'], tele['izq'], tele['der']))
        if t - pr['medir_desde'] < p.t_medir or len(self._muestras) < 6:
            return self._vw(0.0, 0.0)
        del pr['medir_desde']
        f, mi, md = self._medido()
        if f is None or f >= p.f_libre:
            return self._encarado_listo(t, 'derecho (el frente lee %s)' % ('libre' if f is None else '%.2f m' % f))
        df = f - pr['f_ant']
        pr['f_ant'] = f
        # Con el mapa sin medir (None) y un frontal que no cambia en la primera prueba: es una pared de verdad.
        if pr['n'] == 1 and abs(df) <= p.df_encarar and self.mapa.abierto(self.celda, self.rumbo) is None and f >= p.f_pared:
            self._encarado = (self.celda, self.rumbo)
            self._evento(t, 'el frente no cambia al girar (%.2f m) y el mapa no sabía: es una pared de verdad' % f)
            pr['deshacer'] = True
            self._cambiar(ENCARAR, t, 'deshago la prueba (%+.0f°)' % math.degrees(-pr['ultimo']))
            return self._vw(0.0, 0.0, self._mover(t, 7, -pr['ultimo'], tele))
        if df > p.df_encarar and pr['ultimo'] != 0.0:      # mejoró: sigue en el mismo sentido
            return self._probar(t, tele, destino=pr['pos'] + math.copysign(p.paso_encarar, pr['ultimo']))
        if df < -p.df_encarar:
            self._evento(t, 'la prueba empeoró (frente %.2f -> %.2f m): pruebo hacia el otro lado' % (f - df, f))
        return self._probar(t, tele)                       # sin gradiente claro o empeoró: siguiente blanco del zigzag

    def _encarado_listo(self, t, motivo):
        pr = self._prueba
        self._prueba = None
        self.imprimir('  encarado: %s' % motivo)
        self.obs.x[1] = 0.0
        self.obs.P[1][1] = math.radians(8) ** 2
        self._cambiar(pr['volver_a'], t, 'mido de nuevo ya derecho')
        return self._vw(0.0, 0.0)

    def _empezar_correr(self, t, tele):
        self._centrados = 0
        return super()._empezar_correr(t, tele)
