"""Navegación del laberinto para el robot REAL (sin ROS: la usan torneo.py y simular_torneo.py).

Por qué no es el explorador de la simulación: el firmware de ruedas (firmware/banco/robot_4ruedas) no sostiene una rueda por
debajo de 25 flancos/s (~0,13 m/s) y redondea a 0 lo que pida por debajo de la mitad. Pedir velocidades o giros chicos para
"afinar" apaga una rueda y deja la otra a 0,13: el robot pivotea contra la pared. Y el rumbo de la odometría de un 4x4 que
patina no sirve para alinearse en un pasillo de 40 cm. Por eso, igual que tools/autonomo (que anda en el piso):

  - Avanza siempre a v_crucero con correcciones acotadas: las dos ruedas quedan siempre por encima del mínimo.
  - El rumbo y la posición lateral salen de los sonares laterales (un observador y, psi), no de la odometría. La odometría
    (promedio de los 4 encoders) solo mide cuánto avanzó, que sí es confiable (~1,5 %).
  - Los giros los hace la ESP32 (/robot_cmd 7, con sus encoders, sin el retardo del Wi-Fi). Después se verifica con los
    sonares que haya quedado alineado (1 de cada 10 giros patina y rinde 40-75 %, M 3/11) y se corrige.
  - Recorre los pasillos de corrido y frena solo donde tiene que decidir o girar (cada arranque desde quieto tuerce el robot:
    una rueda arranca 0,25-0,9 s tarde, M).
  - El freno del frente es doble: la PC frena por su cuenta y la guardia de la ESP32 (d_stop 0,05, el mínimo que acepta) es
    el último recurso, sin depender del Wi-Fi.

Mapa y decisión: caid_navigation/mapa.py (Trémaux con BFS), y seguir_guia para las flechas (el largo dice cuántos tramos
rectos siguen). Grilla de 0,40 m: el centro de la celda (i, j) está en (0,4 i, 0,4 j); rumbo 0 = +x, 1 = +y, 2 = -x, 3 = -y.

Convenciones: v en m/s, w en rad/s (+ izquierda). Distancias de los sonares en m, None = sin eco.
"""
import math
import os
import sys
from dataclasses import dataclass

_NAV = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'ros2_ws', 'src', 'caid_navigation')
if os.path.isdir(_NAV) and _NAV not in sys.path:
    sys.path.insert(0, _NAV)
from caid_navigation.mapa import Mapa, PASOS, seguir_guia, vecina  # noqa: E402

CELDA = 0.40
MEDIA = CELDA / 2
F_X = 0.1399                 # sonar frontal: al ras del frente (hardware/cad/dimensions.yaml)
L_X = 0.096                  # sonares laterales: 9,6 cm adelante del centro
L_Y = 0.0815                 # ... y a 8,15 cm del eje
F_CENTRO = MEDIA - F_X       # 0,06 m: lo que mide el frontal en el centro de una celda con pared adelante
NOMBRE_DIR = ('+x', '+y', '-x', '-y')

ESPERA, MEDIR, GIRAR, VERIFICAR, CORRER, PARANDO, ALINEAR, ACERCAR, AGARRE, RETROCEDER, FIN, DETENIDO = (
    'ESPERA', 'MEDIR', 'GIRAR', 'VERIFICAR', 'CORRER', 'PARANDO', 'ALINEAR', 'ACERCAR', 'AGARRE', 'RETROCEDER', 'FIN',
    'DETENIDO')


@dataclass
class Parametros:
    v_crucero: float = 0.20      # m/s
    v_lenta: float = 0.15        # m/s cerca de una pared de frente (la guardia igual topa a 0,13 desde d_lento)
    margen_rueda: float = 0.012  # m/s por encima del mínimo de rueda que se deja al corregir el rumbo
    v_curva: float = 0.27        # m/s: para corregir fuerte sube hasta esto (la rueda de adentro queda en el mínimo)
    w_tope: float = 0.80         # rad/s
    # Girar en el lugar barre un radio de 16,1 cm (robot de 27,8 x 16,3): en una celda de 40 cm quedan 3,9 cm de margen.
    descentrado_max: float = 0.025  # m: más corrido que esto del centro, no gira en el lugar: retrocede y vuelve a entrar
    acomodar_atras: float = 0.25
    lateral_min_giro: float = 0.085  # m: con un lateral contra pared más cerca que esto, no se gira en el lugar
    f_parar: float = 0.065       # m: respaldo de la PC; la que frena de verdad contra una pared es la guardia de la ESP32
    psi_parar: float = 0.30      # rad: tan torcido y con el frente a menos de 30 cm, frena y se endereza en el lugar
    psi_enderezar: float = 0.15  # rad: si frena más torcido que esto, se endereza antes de medir
    max_enderezar: float = 0.70
    # Laterales: una pared del pasillo se mide a ~12 cm (20 - 8,15) +- lo descentrado. En el centro de una celda con salida al
    # costado, el sonar (9,6 cm adelante del centro) ve de reojo la esquina del fondo de la salida a ~24 cm: eso es paso.
    d_pared: float = 0.20        # m: un lateral a esto o menos es pared
    d_abierto: float = 0.235     # m: un lateral a esto o más (o sin eco) es paso
    k_y: float = 2.5             # rumbo deseado = atan(-k_y * y)
    k_psi: float = 1.5           # w = k_psi * (rumbo deseado - rumbo estimado). Suaves: hay ~0,4 s de retardo (Wi-Fi + ruedas)
    compensar_max: float = 0.45  # rad: al girar se descuenta lo torcido que venía (hasta 25°)
    latencia: float = 0.30       # s: ida y vuelta sonar -> PC -> ESP32 (+ edad de la odometría): para frenar a tiempo en un cruce
    t_medir: float = 0.7         # s quieto midiendo en el centro de una celda
    giro_tol: float = 0.17       # rad: un giro que quedó más torcido que esto se corrige
    max_correcciones: int = 3    # 4 de 10 giros medidos el 3/11 quedaron muy cortos: hay que poder corregir varias veces
    parada_cubo: float = 0.06    # m: frena con el frente del robot a esto del cubo (la guardia no deja bajar de 0,05)
    cubo_lejos: float = 0.70     # m: un cubo verde visto más cerca que esto, enfrente, se va a buscar
    # Guardia de la ESP32: es la que frena contra una pared de enfrente, sin el retardo del Wi-Fi (~0,4 s ida y vuelta = 8 cm a
    # 0,20 m/s). Con d_stop 0,075 el robot queda con el frente a 6-7,5 cm = el centro de la celda (F_CENTRO = 6 cm).
    d_stop: float = 0.075
    d_lento: float = 0.13        # su zona lenta (0,13 m/s) solo en los últimos 5,5 cm
    d_stop_cubo: float = 0.05    # al acercarse al cubo (el mínimo que acepta el firmware)
    d_lento_cubo: float = 0.10
    color: str = 'verde'


def ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def mediana(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else None


def lateral(muestras, p):
    """Clasifica un lateral: True paso, False pared, None no se sabe. Un 'sin eco' suelto al lado de una pared es un eco
    perdido (30 % bajo 6 cm, M), no un paso: vale medio voto."""
    if not muestras:
        return None
    pared = sum(1 for d in muestras if d is not None and d <= p.d_pared)
    lejos = sum(1 for d in muestras if d is not None and d >= p.d_abierto)
    sin_eco = sum(1 for d in muestras if d is None)
    paso = lejos + 0.5 * sin_eco
    if pared >= 2 and lejos == 0 and pared >= paso:
        return False
    if (lejos >= 2 or sin_eco >= 0.6 * len(muestras)) and paso > 1.5 * pared:
        return True
    if len(muestras) <= 2 and pared and not lejos:
        return False
    return None


class Observador:
    """Posición lateral y (m, + izquierda del centro del pasillo) y rumbo psi (rad, + izquierda) respecto del pasillo: un filtro
    de Kalman de 2 estados. Predice con lo que midieron los encoders (avance ds y giro dth: confiables en tramos cortos) y
    corrige con cada lectura lateral nueva contra una pared. El rumbo no se mide directo: sale de cómo cambia y al avanzar."""

    R_LAT = 0.004 ** 2           # m², una lectura lateral (2 mm de ruido + inclinación + ancho del pasillo)

    def __init__(self):
        self.x = [0.0, 0.0]
        self.P = [[0.02 ** 2, 0.0], [0.0, math.radians(4) ** 2]]
        self.ancho = CELDA       # ancho libre del pasillo, se aprende cuando se ven las dos paredes
        self._ult = {'izq': None, 'der': None}

    @property
    def y(self):
        return self.x[0]

    @property
    def psi(self):
        return self.x[1]

    def reiniciar(self, izq, der, psi=None, sigma_psi=math.radians(4)):
        """Quieto en una celda: y sale de los laterales. psi se conserva salvo que se pase uno nuevo (después de un giro)."""
        if psi is not None:
            self.x[1] = psi
            self.P = [[self.P[0][0], 0.0], [0.0, sigma_psi ** 2]]
        self._ult = {'izq': izq, 'der': der}
        y = self._y_de(izq, der)
        self.x[0] = 0.0 if y is None else y
        self.P[0][0] = (0.006 if y is not None else 0.05) ** 2
        self.P[0][1] = self.P[1][0] = 0.0

    def _y_de(self, izq, der):
        c, sn = math.cos(self.psi), math.sin(self.psi)
        yi = None if izq is None else self.ancho / 2 - L_Y - izq * c - L_X * sn
        yd = None if der is None else der * c + L_Y - self.ancho / 2 - L_X * sn
        if yi is not None and yd is not None:
            return (yi + yd) / 2
        return yi if yi is not None else yd

    def predecir(self, ds, dth):
        y, psi = self.x
        self.x = [y + ds * math.sin(psi + dth / 2), psi + dth]
        f01 = ds * math.cos(psi)
        P = self.P
        a = P[0][0] + 2 * f01 * P[0][1] + f01 * f01 * P[1][1]
        b = P[0][1] + f01 * P[1][1]
        q_y = (0.002 * abs(ds) / 0.05) ** 2
        q_psi = (0.10 * abs(dth)) ** 2 + 0.02 ** 2 * abs(ds)        # patinaje al corregir + desvío propio (rad²/m)
        self.P = [[a + q_y, b], [b, P[1][1] + q_psi]]

    def corregir(self, s, izq, der, p):
        """izq/der: última medición (None = sin eco). Solo usa las nuevas, contra una pared (<= d_pared)."""
        nuevos = []
        for lado, d in (('izq', izq), ('der', der)):
            if d != self._ult[lado]:
                self._ult[lado] = d
                if d is not None and d <= p.d_pared:
                    nuevos.append((lado, d))
        if izq is not None and der is not None and izq <= p.d_pared and der <= p.d_pared and nuevos:
            w = (izq + der) * math.cos(self.psi) + 2 * L_Y
            if 0.30 < w < 0.55:
                self.ancho += 0.2 * (w - self.ancho)
        for lado, d in nuevos:
            y, psi = self.x
            c = math.cos(psi)
            z = (self.ancho / 2 - L_Y - d * c) if lado == 'izq' else (d * c + L_Y - self.ancho / 2)
            h0, h1 = 1.0, L_X * c
            innov = z - (y + L_X * math.sin(psi))
            P = self.P
            ph0 = P[0][0] * h0 + P[0][1] * h1
            ph1 = P[1][0] * h0 + P[1][1] * h1
            S = h0 * ph0 + h1 * ph1 + self.R_LAT
            if innov * innov > 16 * S:           # una lectura que no cuadra (un eco de costado, el canto de una pared)
                continue
            k0, k1 = ph0 / S, ph1 / S
            self.x = [y + k0 * innov, psi + k1 * innov]
            self.P = [[P[0][0] - k0 * ph0, P[0][1] - k0 * ph1], [P[1][0] - k1 * ph0, P[1][1] - k1 * ph1]]


class Piloto:
    """Máquina de estados. paso() devuelve (v, w, orden): orden es None o una tupla para /robot_cmd (se manda una vez)."""

    def __init__(self, params=None, trocha=0.291, diametro=0.0645, imprimir=print):
        self.p = params or Parametros()
        self.trocha = trocha
        self.v_min_rueda = 25.0 * math.pi * diametro / 40.0          # VEL_MIN_FLANCOS del firmware, en m/s
        self.imprimir = imprimir
        self.mapa = Mapa()
        self.mapa.cerrar((0, 0), 2)          # atrás de la partida está la salida: no se explora
        self.flechas = {}                    # celda -> (dir, largo) de una flecha del color buscado
        self.guia = None
        self.obs = Observador()
        self.estado = ESPERA
        self.modo = 'EXPLORAR'               # EXPLORAR -> (cubo) -> VOLVER
        self.motivo = 'esperando la orden de arrancar'
        self.celda = (0, 0)
        self.rumbo = 0
        self.t_estado = 0.0
        self.agarrado = False
        self.cubo = None                     # (distancia del frente del robot al cubo, t) de la cámara
        self._muestras = []
        self._mov = None
        self._remapeos = 0
        self._final = None
        self._realinear = False
        self._acomodando = False
        self._acomodos = 0
        self._s_ult = 0.0
        self._th_ult = 0.0
        self._c0 = (0, 0)
        self._s0 = None
        self._verif = 0
        self._ultimo_giro = 0.0
        self._t_ult = None
        self._orden_vw = (0.0, 0.0)
        self.choques_evitados = 0

    # ---------------------------------------------------------------- órdenes del operador / la cámara
    def iniciar(self, t):
        self._cambiar(MEDIR, t, 'arranca: mido la celda de partida')

    def detener(self, t, motivo='detenido por el operador'):
        self._cambiar(DETENIDO, t, motivo)

    def flecha(self, celda, direccion, largo, color):
        if color != self.p.color or self.modo != 'EXPLORAR':
            return
        if self.flechas.get(celda) != (direccion, largo):
            self.flechas[celda] = (direccion, largo)
            self.imprimir('  flecha %s en la celda %s: hacia %s, %s tramo(s)' % (
                color, celda, NOMBRE_DIR[direccion], 'tres o más' if largo >= 3 else largo))

    def ver_cubo(self, t, dist_frente):
        self.cubo = (dist_frente, t)

    # ---------------------------------------------------------------- interno
    def _cambiar(self, estado, t, motivo):
        self.estado = estado
        self.t_estado = t
        self.motivo = motivo
        self._muestras = []
        self.imprimir('  [%s] %s' % (estado, motivo))

    def guardia(self):
        """(d_stop, d_lento) que hay que pedirle a la ESP32 ahora."""
        if self.estado in (ACERCAR, AGARRE):
            return self.p.d_stop_cubo, self.p.d_lento_cubo
        return self.p.d_stop, self.p.d_lento

    def w_max(self, v):
        return max(0.0, (v - self.v_min_rueda - self.p.margen_rueda) / (self.trocha / 2))

    def pose(self):
        """Pose aproximada (x, y, yaw) en el marco del mapa: para ubicar las flechas de la cámara."""
        u, n = PASOS[self.rumbo], PASOS[(self.rumbo + 1) % 4]
        pos = self._pos if self.estado == CORRER else self._off
        x = self.celda[0] * CELDA + u[0] * pos + n[0] * self.obs.y
        y = self.celda[1] * CELDA + u[1] * pos + n[1] * self.obs.y
        return x, y, self.rumbo * math.pi / 2 + self.obs.psi

    _pos = 0.0
    _off = 0.0

    def _mover(self, t, ident, valor, tele):
        """Orden de movimiento a la ESP32 (7 girar, 3 avanzar). Devuelve la tupla a mandar."""
        self._mov = dict(ident=ident, valor=valor, seq=tele['mov_seq'], t=t, intentos=1, visto=False)
        return (ident, valor, 0.0)

    def _mov_terminado(self, t, tele):
        """True cuando la ESP32 terminó; 'reenviar' si la orden parece perdida; False si sigue."""
        m = self._mov
        if tele['mov_activo']:
            m['visto'] = True
        if tele['mov_seq'] != m['seq'] and not tele['mov_activo']:
            return True
        if not m['visto'] and tele['mov_seq'] == m['seq'] and t - m['t'] > 0.8 and m['intentos'] < 4:
            m['t'] = t
            m['intentos'] += 1
            return 'reenviar'
        if t - m['t'] > 12.0:
            return True
        return False

    def _medido(self):
        validos = [m[0] for m in self._muestras if m[0] is not None]
        f = mediana(validos)
        nf = len(self._muestras) - len(validos)
        cerca = [x for x in validos if x < 0.25]
        if cerca and len(cerca) >= 2:
            f = mediana(cerca)                    # una pared muy cerca pierde muchos ecos (32 % a 5,5 cm, M): vale la que se vio
        elif f is None or nf > len(self._muestras) / 2:
            f = None
        return f, [m[1] for m in self._muestras], [m[2] for m in self._muestras]

    # ---------------------------------------------------------------- un paso (cada ~50 ms)
    def paso(self, t, tele, s, th):
        """tele: dict(frente, izq, der [m o None], guardia, mov_activo, mov_seq, quieto, fresco).
        s, th: avance (m) y giro (rad, + izquierda) acumulados de los encoders."""
        if self._t_ult is not None and self.estado not in (GIRAR, ALINEAR):
            self.obs.predecir(s - self._s_ult, ang(th - self._th_ult))
        self._t_ult = t
        self._s_ult = s
        self._th_ult = th
        if self.estado in (ESPERA, FIN, DETENIDO) or not tele['fresco']:
            return self._vw(0.0, 0.0)
        fn = getattr(self, '_' + self.estado.lower())
        return fn(t, tele, s)

    def _vw(self, v, w, orden=None):
        self._orden_vw = (v, w)
        return v, w, orden

    # --- quieto en el centro de una celda: mide y decide
    def _medir(self, t, tele, s):
        if t - self.t_estado > 0.3 and tele['quieto']:
            self._muestras.append((tele['frente'], tele['izq'], tele['der']))
        if t - self.t_estado < self.p.t_medir or len(self._muestras) < 6:
            return self._vw(0.0, 0.0)
        f, mi, md = self._medido()
        izq, der = lateral(mi, self.p), lateral(md, self.p)
        frente = f is None or f > MEDIA + 0.05
        if f is not None and f < 0.25:
            self._off = F_CENTRO - f             # + = el robot quedó adelante del centro
        self.obs.reiniciar(mediana([d for d in mi if d is not None and d <= self.p.d_pared]),
                           mediana([d for d in md if d is not None and d <= self.p.d_pared]))
        self.mapa.registrar(self.celda, self.rumbo, frente, True if izq is None else izq, True if der is None else der)
        self.imprimir('  celda %s rumbo %s: frente %s izq %s der %s  (f=%s)' % (
            self.celda, NOMBRE_DIR[self.rumbo], 'paso' if frente else 'pared', 'paso' if izq else 'pared',
            'paso' if der else 'pared', '--' if f is None else '%.3f' % f))
        return self._decidir(t, tele, f, mi, md)

    def _puede_girar(self, mi, md):
        """Girar en el lugar barre un radio de 16,1 cm: con un costado a menos de ~8 cm de la pared, una esquina la toca."""
        for ms in (mi, md):
            validos = [d for d in ms if d is not None and d <= self.p.d_pared]
            if validos and mediana(validos) < self.p.lateral_min_giro:
                return False
        return abs(self.obs.y) <= 0.045

    def _direccion(self, celda, rumbo):
        """Hacia dónde ir desde `celda` mirando a `rumbo` (None = terminó)."""
        if self.modo == 'VOLVER':
            if celda == self.mapa.inicio:
                return None
            return self.mapa.decidir(celda, rumbo, destino=self.mapa.inicio)
        pista = self.flechas.get(celda)
        if pista is not None:
            self.guia = {'color': self.p.color, 'dir': pista[0], 'tramos': pista[1]}
        pref = None
        if self.guia is not None:
            pref, self.guia = seguir_guia(self.mapa, celda, self.guia)
        return self.mapa.decidir(celda, rumbo, preferida=pref)

    def _referencia(self):
        """Toma la celda actual como origen de la posición a lo largo del rumbo actual."""
        self._c0 = self.celda
        self._s0 = self._s_ult - self._off

    def _decidir(self, t, tele, f, mi, md):
        if self.modo == 'EXPLORAR' and self._cubo_cerca(t):
            self._referencia()
            self._cambiar(ACERCAR, t, 'cubo %s enfrente: me acerco' % self.p.color)
            return self._vw(0.0, 0.0)
        d = self._direccion(self.celda, self.rumbo)
        if d is None and self.modo == 'EXPLORAR' and self._remapeos < 2:
            # Sin cubo y sin nada por explorar: casi seguro un lateral mal medido cerró una parte del laberinto. Se borra lo
            # medido (no lo recorrido) y se vuelve a explorar desde acá.
            self._remapeos += 1
            self.mapa.reiniciar()
            self.flechas.clear()
            self.guia = None
            self.imprimir('  no encontré el cubo: borro el mapa y exploro de nuevo (%d)' % self._remapeos)
            self.mapa.registrar(self.celda, self.rumbo, f is None or f > MEDIA + 0.05,
                                lateral(mi, self.p) is not False, lateral(md, self.p) is not False)
            d = self._direccion(self.celda, self.rumbo)
        if d is None:
            self._cambiar(FIN, t, 'de vuelta en la partida' if self.modo == 'VOLVER' else 'no queda nada por explorar')
            return self._vw(0.0, 0.0)
        rel = (d - self.rumbo) % 4
        if rel == 0:
            return self._empezar_correr(t, tele)
        if (abs(self.obs.y) > self.p.descentrado_max or not self._puede_girar(mi, md)) and self._acomodos < 2 and \
                abs(self.obs.psi) < 0.15 and \
                self.mapa.abierto(self.celda, (self.rumbo + 2) % 4):
            self._acomodos += 1
            self._cambiar(RETROCEDER, t, 'quedé corrido %.0f cm del centro: retrocedo %.0f cm y vuelvo a entrar' % (
                100 * self.obs.y, 100 * self.p.acomodar_atras))
            self._acomodando = True
            return self._vw(0.0, 0.0, self._mover(t, 3, -self.p.acomodar_atras, tele))
        self._acomodos = 0
        self._final = None
        ang = {1: math.pi / 2, 3: -math.pi / 2}.get(rel)
        if ang is None:                           # media vuelta: dos de 90 verificados, hacia el lado con más lugar
            li = mediana([x if x is not None else 9.0 for x in mi]) or 9.0
            ld = mediana([x if x is not None else 9.0 for x in md]) or 9.0
            ang = math.pi / 2 if li >= ld else -math.pi / 2
            self._final = d
            d = (self.rumbo + (1 if ang > 0 else 3)) % 4
        return self._girar_hacia(t, tele, d, ang)

    def _girar_hacia(self, t, tele, d, ang):
        self._destino = d
        self._ultimo_giro = ang
        self._verif = 0
        torcido = max(-self.p.compensar_max, min(self.p.compensar_max, self.obs.psi))
        cmd = ang - torcido                       # si venía torcido, el giro lo endereza
        self._cambiar(GIRAR, t, 'giro %+.0f° hacia %s (venía torcido %+.0f°)' % (
            math.degrees(cmd), NOMBRE_DIR[d], math.degrees(torcido)))
        return self._vw(0.0, 0.0, self._mover(t, 7, cmd, tele))

    # --- giro en el lugar, lo hace la ESP32
    def _girar(self, t, tele, s):
        r = self._mov_terminado(t, tele)
        if r == 'reenviar':
            return self._vw(0.0, 0.0, (self._mov['ident'], self._mov['valor'], 0.0))
        if not r:
            return self._vw(0.0, 0.0)
        if self._verif == 0:
            self.rumbo = self._destino
        self._cambiar(VERIFICAR, t, 'verifico la alineación después del giro')
        return self._vw(0.0, 0.0)

    def _verificar(self, t, tele, s):
        if t - self.t_estado > 0.3 and tele['quieto']:
            self._muestras.append((tele['frente'], tele['izq'], tele['der']))
        if t - self.t_estado < self.p.t_medir or len(self._muestras) < 6:
            return self._vw(0.0, 0.0)
        f, mi, md = self._medido()
        error = self._error_alineacion(f, mi, md)
        if error is not None and self._verif < self.p.max_correcciones:
            self._verif += 1
            corr = math.copysign(min(error, 0.45), self._ultimo_giro)   # lo típico es que el giro quede corto (patina)
            self._cambiar(GIRAR, t, 'quedó torcido ~%.0f°: corrijo %+.0f°' % (math.degrees(error), math.degrees(corr)))
            return self._vw(0.0, 0.0, self._mover(t, 7, corr, tele))
        if self._final is not None and self.rumbo != self._final:
            d = self._final
            self._final = None
            return self._girar_hacia(t, tele, d, self._ultimo_giro)
        if f is not None and f < 0.25:
            self._off = F_CENTRO - f
        pared_i = [d for d in mi if d is not None and d <= self.p.d_pared]
        pared_d = [d for d in md if d is not None and d <= self.p.d_pared]
        self.obs.reiniciar(mediana(pared_i), mediana(pared_d), psi=0.0, sigma_psi=math.radians(8))
        return self._decidir(t, tele, f, mi, md)

    def _error_alineacion(self, f, mi, md):
        """Cuánto parece torcido (rad, sin signo) comparando lo medido con lo que el mapa dice que hay alrededor; None si está
        bien. Torcido e, un lateral contra una pared mide d0 / cos e (y sin eco pasados ~30°), y el frontal hacia un pasillo
        abierto ve la pared de costado a ~0,2 / sen e."""
        c = self.celda
        esperado = {'frente': self.mapa.abierto(c, self.rumbo), 'izq': self.mapa.abierto(c, (self.rumbo + 1) % 4),
                    'der': self.mapa.abierto(c, (self.rumbo + 3) % 4)}
        d0 = self.obs.ancho / 2 - L_Y
        lat = {}
        errores = []
        for lado, ms in (('izq', mi), ('der', md)):
            validos = [d for d in ms if d is not None]
            lat[lado] = mediana(validos) if len(validos) >= len(ms) / 2 else None
            if esperado[lado] is False:
                if lat[lado] is None or lat[lado] > 0.35:
                    errores.append(0.50)
                elif lat[lado] > self.p.d_pared + 0.01:
                    errores.append(math.acos(max(-1.0, min(1.0, d0 / lat[lado]))))
        if esperado['izq'] is False and esperado['der'] is False and lat['izq'] and lat['der'] and \
                lat['izq'] < 0.35 and lat['der'] < 0.35:
            suma = lat['izq'] + lat['der']
            errores.append(math.acos(max(-1.0, min(1.0, 2 * d0 / suma))))
        if esperado['frente'] is True and f is not None and f < 0.30:
            errores.append(math.asin(max(0.0, min(1.0, (MEDIA - L_Y) / (f + F_X)))))
        if esperado['frente'] is False and (f is None or f > 0.30):
            errores.append(0.50)
        if not errores:
            return None
        e = max(errores)
        return e if e >= self.p.giro_tol else None

    # --- avanzar por el pasillo
    def _empezar_correr(self, t, tele):
        self._c0 = self.celda
        self._s0 = None
        self._pos = self._off
        self._k = 1                                  # próxima celda a evaluar
        self._parar_en = None
        self._lat = {}                               # k -> [(izq, der)]
        self._frente_k = {}
        self._cambiar(CORRER, t, 'avanzo hacia %s' % NOMBRE_DIR[self.rumbo])
        return self._vw(self.p.v_crucero, 0.0)

    def _celda_k(self, k):
        u = PASOS[self.rumbo]
        return (self._c0[0] + k * u[0], self._c0[1] + k * u[1])

    def _correr(self, t, tele, s):
        p = self.p
        if self._s0 is None:
            self._s0 = s - self._off
        self._pos = s - self._s0
        pos = self._pos
        f, izq, der = tele['frente'], tele['izq'], tele['der']
        self.obs.corregir(s, izq, der, p)

        # laterales de la celda que tiene el sensor al lado (para el mapa y para decidir si frenar ahí)
        sens = pos + L_X
        k_sens = round(sens / CELDA)
        if k_sens >= self._k and -0.17 < sens - k_sens * CELDA < 0.15 and abs(self.obs.psi) < 0.15:
            self._lat.setdefault(k_sens, []).append((izq, der))
        if f is not None and abs(self.obs.psi) < 0.10:   # torcido, el frontal ve de reojo la pared de costado
            pared = pos + F_X + f                    # dónde está la pared de enfrente
            self._frente_k.setdefault(round(pos / CELDA), []).append(pared)

        # cubo enfrente (cámara)
        if self.modo == 'EXPLORAR' and self._cubo_cerca(t):
            self._fijar_celda(round(pos / CELDA), pos)
            self._cambiar(ACERCAR, t, 'cubo %s enfrente: me acerco' % p.color)
            return self._vw(p.v_lenta, 0.0)

        # Decidir la celda k: apenas un lateral se confirma abierto (un cruce: hay que saber ya si se frena, el sensor está solo
        # 9,6 cm adelante del centro), o cuando el sensor llega cerca de la mitad de la celda (pasillo: no se frena).
        v = p.v_crucero
        while self._parar_en is None:
            k = self._k
            muestras = self._lat.get(k, [])
            izq_k = lateral([m[0] for m in muestras], p)
            der_k = lateral([m[1] for m in muestras], p)
            limite = sens >= k * CELDA - 0.06
            if not limite and izq_k is not True and der_k is not True:
                break
            celda = self._celda_k(k)
            paredes = self._frente_k.get(k, []) + self._frente_k.get(k - 1, [])
            frente_k = not (len(paredes) >= 2 and mediana(paredes) < k * CELDA + MEDIA + 0.08)
            self.mapa.recorrer(self._celda_k(k - 1), self.rumbo)
            # Solo lo confirmado va al mapa: un lateral dudoso queda sin medir (si hace falta, la exploración vuelve a mirarlo).
            # Frenar a medir cada celda dudosa es peor: cada arranque desde quieto tuerce el robot.
            self.mapa.visitadas.add(celda)
            self.mapa.marcar_lado(celda, self.rumbo, frente_k)
            for rel, val in ((1, izq_k), (3, der_k)):
                if val is not None:
                    self.mapa.marcar_lado(celda, (self.rumbo + rel) % 4, val)
            seguir = frente_k and self._direccion(celda, self.rumbo) == self.rumbo
            if not seguir:
                self._parar_en = k
                break
            self._k += 1

        # frenar
        if self._parar_en is not None and pos >= self._parar_en * CELDA - v * p.latencia:
            self._fijar_celda(self._parar_en, pos)
            self._cambiar(PARANDO, t, 'freno en la celda %s' % (self.celda,))
            return self._vw(0.0, 0.0)
        if tele['guardia'] == 3 or (f is not None and f <= p.f_parar):
            k = round(pos / CELDA)
            self._fijar_celda(k, pos)
            self._cambiar(PARANDO, t, 'pared enfrente (%s m): freno en la celda %s' % (
                '--' if f is None else '%.3f' % f, self.celda))
            return self._vw(0.0, 0.0)
        if abs(self.obs.psi) > p.psi_parar and f is not None and f < 0.30:
            k = round(pos / CELDA)
            self._fijar_celda(k, pos)
            self._realinear = True
            self._cambiar(PARANDO, t, 'voy muy torcido (%+.0f°) hacia una pared: freno a enderezarme' % math.degrees(self.obs.psi))
            return self._vw(0.0, 0.0)

        return self._vw(*self._timon(v, izq, der))

    def _timon(self, v, izq, der):
        """(v, w): w para volver al centro del pasillo; si hace falta mucha, sube v para que la rueda de adentro no baje del
        mínimo del firmware (por debajo la redondea a 0,13 o la apaga, y el robot pivotea)."""
        p = self.p
        o = self.obs
        y = o.y
        cerca = [(d, s) for d, s in ((izq, 1.0), (der, -1.0)) if d is not None and d < 0.05]
        if cerca:                                    # casi tocando: alejarse sí o sí
            y = cerca[0][1] * 0.08
        psi_d = math.atan(-p.k_y * y)
        w = max(-p.w_tope, min(p.w_tope, p.k_psi * (psi_d - o.psi)))
        v = max(v, min(p.v_curva, self.v_min_rueda + p.margen_rueda + abs(w) * self.trocha / 2))
        wm = self.w_max(v)
        return v, max(-wm, min(wm, w))

    def _fijar_celda(self, k, pos):
        if k >= 1:
            self.mapa.recorrer(self._celda_k(k - 1), self.rumbo)
        self.celda = self._celda_k(k)
        self._off = pos - k * CELDA

    def _parando(self, t, tele, s):
        if self._s0 is not None:
            self._off = (s - self._s0) - (self._celda_k_inv())
        if not (tele['quieto'] or t - self.t_estado > 1.0):
            return self._vw(0.0, 0.0)
        if self._realinear or abs(self.obs.psi) > self.p.psi_enderezar:
            self._realinear = False
            corr = -max(-self.p.max_enderezar, min(self.p.max_enderezar, self.obs.psi))
            self._cambiar(ALINEAR, t, 'me enderezo %+.0f° y sigo hacia %s' % (math.degrees(corr), NOMBRE_DIR[self.rumbo]))
            return self._vw(0.0, 0.0, self._mover(t, 7, corr, tele))
        self._cambiar(MEDIR, t, 'quieto en la celda %s' % (self.celda,))
        return self._vw(0.0, 0.0)

    def _alinear(self, t, tele, s):
        r = self._mov_terminado(t, tele)
        if r == 'reenviar':
            return self._vw(0.0, 0.0, (self._mov['ident'], self._mov['valor'], 0.0))
        if not r:
            return self._vw(0.0, 0.0)
        self.obs.reiniciar(tele['izq'] if tele['izq'] is not None and tele['izq'] <= self.p.d_pared else None,
                           tele['der'] if tele['der'] is not None and tele['der'] <= self.p.d_pared else None,
                           psi=0.0, sigma_psi=math.radians(7))
        if abs(self._off) < 0.10:
            self._cambiar(MEDIR, t, 'enderezado en la celda %s' % (self.celda,))
            return self._vw(0.0, 0.0)
        return self._empezar_correr(t, tele)

    def _celda_k_inv(self):
        u = PASOS[self.rumbo]
        k = (self.celda[0] - self._c0[0]) * u[0] + (self.celda[1] - self._c0[1]) * u[1]
        return k * CELDA

    # --- el cubo
    def _cubo_cerca(self, t):
        return self.cubo is not None and t - self.cubo[1] < 0.6 and self.cubo[0] < self.p.cubo_lejos

    def _acercar(self, t, tele, s):
        p = self.p
        f = tele['frente']
        dist = [x for x in (f, self.cubo[0] if self.cubo and t - self.cubo[1] < 0.6 else None) if x is not None]
        d = min(dist) if dist else None
        if (d is not None and d <= p.parada_cubo + p.v_lenta * 0.12) or tele['guardia'] == 3 \
                or t - self.t_estado > 8.0:
            if self._s0 is not None:
                pos = s - self._s0
                k = round(pos / CELDA)
                self.celda = self._celda_k(k)
                self._off = pos - k * CELDA
            self._cambiar(AGARRE, t, "frenado frente al cubo (%s m). Agarralo con los sliders y apretá 'y'" % (
                '?' if d is None else '%.3f' % d))
            return self._vw(0.0, 0.0)
        if self._s0 is not None:
            self._pos = s - self._s0
        self.obs.corregir(s, tele['izq'], tele['der'], p)
        v, w = self._timon(p.v_lenta, tele['izq'], tele['der'])
        return self._vw(p.v_lenta, max(-self.w_max(p.v_lenta), min(self.w_max(p.v_lenta), w)))

    def _agarre(self, t, tele, s):
        if not self.agarrado:
            return self._vw(0.0, 0.0)
        self.modo = 'VOLVER'
        self.guia = None
        off = self._off
        atras = max(0.08, off) if off > 0.0 else 0.08         # al centro de la celda, y por lo menos 8 cm del cubo
        self._cambiar(RETROCEDER, t, 'agarrado: retrocedo %.0f cm para tener lugar y vuelvo a la partida' % (100 * atras))
        return self._vw(0.0, 0.0, self._mover(t, 3, -atras, tele))

    def _retroceder(self, t, tele, s):
        r = self._mov_terminado(t, tele)
        if r == 'reenviar':
            return self._vw(0.0, 0.0, (self._mov['ident'], self._mov['valor'], 0.0))
        if not r:
            return self._vw(0.0, 0.0)
        if self._acomodando:                          # vuelve a entrar a la misma celda, centrándose
            self._acomodando = False
            self._off -= self.p.acomodar_atras
            self.obs.reiniciar(tele['izq'] if tele['izq'] is not None and tele['izq'] <= self.p.d_pared else None,
                               tele['der'] if tele['der'] is not None and tele['der'] <= self.p.d_pared else None)
            self._empezar_correr(t, tele)
            self._parar_en = 0
            return self._vw(self.p.v_crucero, 0.0)
        # quedó en el centro de alguna celda (aprox.): la que corresponde a lo que retrocedió
        if self._s0 is not None:
            pos = s - self._s0
            k = round(pos / CELDA)
            self.celda = self._celda_k(k)
            self._off = pos - k * CELDA
        self._cambiar(MEDIR, t, 'mido antes de volver')
        return self._vw(0.0, 0.0)
