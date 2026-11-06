"""Logica del panel de las 4 ruedas (sin ventana y sin ROS: se prueba sola, ver test_ruedas_logica.py).

El firmware es firmware/banco/robot_4ruedas/main.c (el protocolo esta en su cabecera y en docs/guias/robot-4-ruedas-pid-y-comunicacion.md):
  /robot_cmd [1, d0..d3]   duty por rueda, lazo abierto (-1 a 1)       [2, v0..v3]  velocidad por rueda, m/s, con PI
             [3, dist_m, vel]  mover una distancia con rampas            [7, ang_rad, vel]  girar en el lugar (+ izquierda)
             [4]  parar todo                                            [5, diam, trocha, frenado, frenado_giro]  configuracion
  /cmd_vel   Twist (m/s y rad/s)
  /wheel_ticks  4 flancos acumulados CON SIGNO (FL, FR, RL, RR)         /wheel_state  26 numeros, 10 Hz (ver el firmware)
Orden de las ruedas en todo el programa: FL (delantera izquierda), FR, RL (trasera izquierda), RR.
"""
import csv
import datetime
import json
import math
import os
import time
from collections import deque

RUEDAS = ['FL', 'FR', 'RL', 'RR']
NOMBRES = ['Delantera izq. (FL)', 'Delantera der. (FR)', 'Trasera izq. (RL)', 'Trasera der. (RR)']
GIRO_FACTOR_DEL_DEFECTO = 1.15           # el del firmware (GIRO_FACTOR_DELANTERAS): si nunca se cambio con la orden 9
FLANCOS_POR_VUELTA = 40                # 20 ranuras x 2 flancos (M)
DIAMETRO_M, TROCHA_M, FRENADO_M, FRENADO_GIRO_M = 0.067, 0.131, 0.030, 0.005   # los del firmware; los reales salen de calibrar
DIAMETRO_RANGO = (0.031, 0.149)        # el firmware descarta un diametro fuera de (0,03 ; 0,15)
TROCHA_RANGO = (0.051, 0.499)          # ... y una trocha fuera de (0,05 ; 0,5)
FRENADO_TOPE_M, FRENADO_GIRO_TOPE_M = 0.09, 0.04
UMBRAL_FRENADO_M, UMBRAL_FRENADO_GIRO_M = 0.005, 0.003   # diferencia a partir de la cual se ajusta el adelanto de frenado
VEL_MAX_FLANCOS, VEL_MIN_FLANCOS = 75.0, 25.0   # por rueda: el firmware los impone
SALTO_TICKS_REINICIO = 400             # si los ticks saltan mas que esto de una muestra a otra, la ESP32 se reinicio
SILENCIO_S = 2.0                       # sin /wheel_state por mas de esto: se considera desconectada
CARPETA = os.path.expanduser('~/caid/config')


# ---------------------------------------------------------------------------------------------------------------------
# Conversiones
# ---------------------------------------------------------------------------------------------------------------------
def limitar(x, lo, hi):
    return max(lo, min(hi, x))


def flancos_por_m(diametro_m):
    return FLANCOS_POR_VUELTA / (math.pi * diametro_m)


def ms_de_flancos(flancos_s, diametro_m):
    return flancos_s / flancos_por_m(diametro_m)


def flancos_de_ms(ms, diametro_m):
    return ms * flancos_por_m(diametro_m)


def grados_de_arco(arco_m, trocha_m):
    """Un giro en el lugar: cada rueda recorre un arco de radio trocha/2."""
    return math.degrees(arco_m / (trocha_m / 2.0))


def hacia(actual, objetivo, paso):
    """Se acerca a `objetivo` como mucho `paso` (rampa)."""
    if abs(objetivo - actual) <= paso:
        return objetivo
    return actual + (paso if objetivo > actual else -paso)


# ---------------------------------------------------------------------------------------------------------------------
# Odometria con los ticks (lo que el robot "cree" que se movio; el resbalamiento no lo ve)
# ---------------------------------------------------------------------------------------------------------------------
class Odometria:
    def __init__(self):
        self.previos = None
        self.reiniciar()

    def reiniciar(self):
        """Pone x, y, angulo y distancias en cero (no toca los ticks de la ESP32)."""
        self.x = self.y = 0.0
        self.theta = 0.0                # rad, SIN envolver: un giro de 360 grados da 360, no 0
        self.s = 0.0                    # avance neto con signo, m
        self.recorrido = 0.0            # camino recorrido (suma de |avance|), m

    def rebase(self):
        """La proxima muestra de ticks se toma como punto de partida (la ESP32 se reconecto o se reinicio)."""
        self.previos = None

    def foto(self):
        return {'x': self.x, 'y': self.y, 'theta': self.theta, 's': self.s, 'recorrido': self.recorrido}

    def actualizar(self, ticks, diametro_m, trocha_m):
        """Integra una muestra de los 4 ticks acumulados con signo. Devuelve True si la uso."""
        ticks = [int(t) for t in ticks]
        if len(ticks) < 4:
            return False
        previos, self.previos = self.previos, ticks
        if previos is None:
            return False
        delta = [a - b for a, b in zip(ticks, previos)]
        if max(abs(d) for d in delta) > SALTO_TICKS_REINICIO:
            return False                # salto imposible: reinicio de la ESP32; esta muestra queda como base
        fpm = flancos_por_m(diametro_m)
        izq = (delta[0] + delta[2]) / 2.0 / fpm         # metros que recorrio el lado izquierdo (FL y RL)
        der = (delta[1] + delta[3]) / 2.0 / fpm         # ... y el derecho (FR y RR)
        ds = (izq + der) / 2.0
        dth = (der - izq) / trocha_m
        self.x += ds * math.cos(self.theta + dth / 2.0)
        self.y += ds * math.sin(self.theta + dth / 2.0)
        self.theta += dth
        self.s += ds
        self.recorrido += abs(ds)
        return True


# ---------------------------------------------------------------------------------------------------------------------
# Correcciones con lo medido en el piso
# ---------------------------------------------------------------------------------------------------------------------
def diametro_corregido(diametro_m, real_m, calculado_m):
    """El encoder cuenta flancos: si el robot avanzo `real` y el calculo dio `calculado`, el diametro efectivo es otro."""
    if abs(calculado_m) < 1e-6 or abs(real_m) < 1e-6:
        return diametro_m
    return limitar(diametro_m * abs(real_m) / abs(calculado_m), *DIAMETRO_RANGO)


def trocha_corregida(trocha_m, calculado_deg, real_deg):
    """Giro en el lugar: las ruedas recorrieron el arco que da `calculado_deg` con la trocha usada. Si el robot giro `real_deg`
    de verdad, la trocha efectiva es trocha x calculado / real (el robot patina: gira menos de lo que dicen los encoders)."""
    if abs(calculado_deg) < 1e-3 or abs(real_deg) < 1e-3:
        return trocha_m
    return limitar(trocha_m * abs(calculado_deg) / abs(real_deg), *TROCHA_RANGO)


def frenado_corregido(frenado_m, sobre_m, tope_m):
    """`sobre_m` positivo = se paso de la meta: hay que cortar antes (mas adelanto); negativo = se quedo corto."""
    return limitar(frenado_m + sobre_m, 0.0, tope_m)


# ---------------------------------------------------------------------------------------------------------------------
# Archivos: calibracion compartida con robot_teclado.py, y registro de maniobras
# ---------------------------------------------------------------------------------------------------------------------
def cargar_cal(ruta):
    try:
        with open(ruta) as f:
            cal = json.load(f)
        return cal if isinstance(cal, dict) else {}
    except (OSError, ValueError):
        return {}


def guardar_cal(cal, ruta):
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, 'w') as f:
        json.dump(cal, f, indent=2)


ENCABEZADO_MANIOBRAS = ['fecha', 'maniobra', 'tipo', 'pedido', 'calculado_esp32', 'odometria_pc', 'real', 'desplazamiento_cm',
                        'diametro_mm', 'trocha_mm', 'frenado_mm', 'frenado_giro_mm', 'ticks_FL', 'ticks_FR', 'ticks_RL',
                        'ticks_RR']
ENCABEZADO_GRABACION = (['t_s', 'modo', 'maniobra'] + ['pedido_' + r for r in RUEDAS] + ['objetivo_fps_' + r for r in RUEDAS] +
                        ['v_fps_' + r for r in RUEDAS] + ['duty_' + r for r in RUEDAS] + ['ticks_' + r for r in RUEDAS] +
                        ['odo_x_m', 'odo_y_m', 'odo_theta_deg', 'odo_s_m'])


def agregar_csv(ruta, encabezado, fila):
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    nuevo = not os.path.exists(ruta)
    with open(ruta, 'a', newline='') as f:
        w = csv.writer(f)
        if nuevo:
            w.writerow(encabezado)
        w.writerow(fila)


# ---------------------------------------------------------------------------------------------------------------------
# El controlador: une las barras, las maniobras, la odometria y los archivos. La ventana solo lo llama y muestra.
# ---------------------------------------------------------------------------------------------------------------------
PERIODO_ORDEN_S = 0.1
TOLERANCIA_RELOJ_S = 0.01              # el reloj de la ventana tiene jitter: sin esto, 0,0999 s cuenta como 'todavia no' y salen 6,7 por segundo


# ---------------------------------------------------------------------------------------------------------------------
# Guiado con los sonares laterales: posicion en el pasillo, inclinacion y control
# ---------------------------------------------------------------------------------------------------------------------
PASILLO_M = 0.40                       # ancho libre del pasillo del reglamento (400 mm)
SON_LX, SON_LY = 0.096, 0.0815         # sonares laterales: 9,6 cm adelante del centro y 8,15 cm del eje (hardware/cad/dimensions.yaml)
V_LENTA_MS, V_CURVA_MS = 0.16, 0.24    # avance lento y tope al que sube si hace falta corregir fuerte (la rueda de adentro >= minimo)
K_Y, K_PSI, W_TOPE = 2.5, 1.5, 0.8     # rumbo deseado = atan(-K_Y y); w = K_PSI (deseado - estimado) + sesgo; rad/s
K_I, SESGO_TOPE = 0.30, 0.45           # integral del error de rumbo (rad/s por rad y por s): compensa una rueda mas floja que la otra
MARGEN_RUEDA_MS = 0.012                # m/s sobre el minimo de rueda que se deja al corregir
STOP_FRENTE_M, STOP_LATERAL_M = 0.14, 0.03   # frena si el frente llega a 14 cm (el robot sigue unos cm: queda a ~10 cm o mas de la pared) o un costado a 3
ALERTA_LATERAL_M = 0.055               # un costado a menos de esto (o que lo va a estar en 10 cm): frena, piensa, se endereza y sigue
ANTICIPO_M = 0.15                      # ... se mira lo que va a medir el lateral dentro de este avance (el robot sigue ~8 cm despues de la orden de frenar)
F_RETROCESO_M = 0.10                   # antes de girar 180, si el frente esta a menos de esto retrocede (el giro en el lugar 'avanza un poco')
ATRAS_MAX_M = 0.06                     # lo maximo que retrocede de una vez (atras no hay sonar)
V_ATRAS_RECTO_MS = 0.14                # retroceso en recto: las ruedas por encima del minimo del firmware
W_PIVOTE_RAD_S = 1.2                   # pivote en el lugar para enderezarse (cada rueda a 0,18 m/s: por encima del minimo)
ADELANTO_PIVOTE_RAD = 0.08             # el robot sigue girando un poco despues de cortar
PIVOTE_FLANCOS = 40.0                  # velocidad del pivote (orden 7 del firmware: lo cierra la ESP32 con los encoders, sin retardo de red)
LAT_DISPARO_M = 0.075                  # un costado a menos de esto Y acercandose (o a menos de LAT_CRITICO_M): correccion brusca EN MARCHA
LAT_CRITICO_M = 0.055                  # ... a esta distancia corrige aunque no se vea que se acerca
BAJA_DISPARO_M = 0.004                 # 'se acerca': el lateral bajo esto (m) en los ultimos ~0,5 s
W_BRUSCO_RAD_S = 1.3                   # correccion brusca: giro hacia el lado contrario mientras sigue avanzando
ANG_BRUSCO_RAD = 0.32                  # ... hasta cambiar el rumbo ~23° (cada repeticion seguida suma BRUSCO_MAS_RAD)
BRUSCO_MAS_RAD = 0.15
ANG_BRUSCO_TOPE_RAD = 0.75
AVANCE_REPETIDO_M = 0.30               # si vuelve a necesitar corregir antes de avanzar esto, cuenta como 'sigue pasando lo mismo'
W_DESPEGUE_RAD_S = 1.2                 # separarse de una pared a ~3 cm: arco hacia adelante alejandose de ella
ANG_DESPEGUE_RAD = 0.35                # giro maximo del primer despegue (cada reintento suma 0,15)
SEPARADO_M = 0.075                     # el despegue termina cuando el lateral cercano vuelve a medir esto
RECTO_TRAS_BRUSCA_M = 0.09             # despues de la correccion brusca avanza derecho esto mientras se vuelve a medir el rumbo
ESPERA_BRUSCA_M = 0.05                 # avance minimo despues de una correccion brusca antes de juzgar de nuevo (el sonar tarda en reflejarlo)
SALTO_SONAR_M = 0.04                   # un lateral que cambia mas que esto entre dos lecturas no es una pared continua


class EstimadorPasillo:
    """Donde esta el robot en un pasillo recto de ancho conocido, con los dos sonares laterales y el avance de los encoders.
      y    desplazamiento del centro respecto del eje del pasillo (m, + a la izquierda), de cada pared que se ve a menos de d_pared
           (un lateral mas lejos o sin eco no es la pared del pasillo)
      psi  rumbo respecto del pasillo (rad, + a la izquierda): es la pendiente con que cambia y al avanzar (dy/ds = tan psi),
           ajustada por minimos cuadrados con lo medido en los ultimos `ventana_m` de avance. Hacen falta 4 cm de avance: antes
           de eso devuelve None. El sonar solo no da el sentido de la inclinacion (torcido a un lado o al otro mide igual)."""

    def __init__(self, ancho=PASILLO_M, ventana_m=0.12, d_pared=0.30):
        self.ancho, self.ventana_m, self.d_pared = ancho, ventana_m, d_pared
        self.serie = {'izq': [], 'der': []}

    def y_de(self, lado, d):
        if lado == 'izq':
            return self.ancho / 2.0 - SON_LY - d
        return d + SON_LY - self.ancho / 2.0

    def posicion(self, izq, der):
        """y con lo que midan los laterales ahora (None si ninguno ve una pared)."""
        ys = [self.y_de(l, d) for l, d in (('izq', izq), ('der', der)) if d is not None and d <= self.d_pared]
        return sum(ys) / len(ys) if ys else None

    def actualizar(self, s, izq, der):
        """s: avance acumulado (m, creciente). Devuelve (y, psi); cada uno es None si no se puede saber."""
        for lado, d in (('izq', izq), ('der', der)):
            serie = self.serie[lado]
            if d is None or d > self.d_pared:
                serie.clear()
                continue
            if serie and abs(self.y_de(lado, d) - serie[-1][1]) > SALTO_SONAR_M:     # salto: un hueco en la pared o un rincon, no una pendiente
                serie.clear()
            serie.append((s, self.y_de(lado, d)))
            while serie and abs(s - serie[0][0]) > self.ventana_m:
                serie.pop(0)
        pendientes = []
        for serie in self.serie.values():
            if len(serie) >= 3 and abs(serie[-1][0] - serie[0][0]) >= 0.04:
                n = len(serie)
                sm = sum(a for a, _ in serie) / n
                ym = sum(b for _, b in serie) / n
                den = sum((a - sm) ** 2 for a, _ in serie)
                if den > 1e-9:
                    pendientes.append(sum((a - sm) * (b - ym) for a, b in serie) / den)
        psi = math.atan(sum(pendientes) / len(pendientes)) if pendientes else None
        return self.posicion(izq, der), psi


def inclinacion_por_suma(izq, der, ancho=PASILLO_M):
    """|rumbo| (rad, SIN signo) respecto del pasillo, de lo que suman los dos laterales: con ancho W, los dos sonares (a 8,15 cm del
    eje) miden en total W / cos(psi) - 2 x 8,15 cm. Solo sirve para detectar un robot MUY torcido (con un pasillo mas ancho de lo
    que se supone da falsos avisos de unos 15°); None si falta algun lateral."""
    if izq is None or der is None or izq > 0.30 or der > 0.30:
        return None
    c = ancho / (izq + der + 2.0 * SON_LY)
    return math.acos(min(1.0, c))


def error_de_rumbo(y, psi):
    """Rumbo deseado (hacia el eje del pasillo) menos el estimado, rad; + = hay que girar a la izquierda."""
    return math.atan(-K_Y * (y or 0.0)) - (psi or 0.0)


def timon_pasillo(v, y, psi, trocha, diametro, sesgo=0.0, v_curva=V_CURVA_MS):
    """(v, w) para volver al centro del pasillo y quedar derecho: w = K_PSI x error de rumbo + sesgo (el termino integral, que el
    que llama va acumulando: compensa una rueda mas floja, que si no deja al robot corrido de forma permanente). Si hace falta
    corregir fuerte sube v hasta v_curva para que la rueda de adentro no baje del minimo del firmware (por debajo la apaga y el
    robot pivotea contra la pared)."""
    v_min = VEL_MIN_FLANCOS / flancos_por_m(diametro) + MARGEN_RUEDA_MS
    w = limitar(K_PSI * error_de_rumbo(y, psi) + sesgo, -W_TOPE, W_TOPE)
    v = max(v, min(v_curva, v_min + abs(w) * trocha / 2.0))
    w_max = max(0.0, (v - v_min) / (trocha / 2.0))
    return v, limitar(w, -w_max, w_max)


class Controlador:
    """`enlace` es lo que habla con la ESP32 (en la PC real, un nodo ROS 2; en las pruebas, un robot simulado):
       .estado .n_estado   ultimo /wheel_state y un contador que sube con cada mensaje
       .ticks  .n_ticks    ultimo /wheel_ticks y su contador
       .enviar_cmd(id, *args)   /robot_cmd        .enviar_vel(lineal, angular)   /cmd_vel
    Hay que llamar a tick() cada ~50 ms."""

    def __init__(self, enlace, carpeta=CARPETA, reloj=time.monotonic, imprimir=print):
        self.enlace = enlace
        self.carpeta = carpeta
        self.reloj = reloj
        self.imprimir = imprimir
        self.ruta_cal = os.path.join(carpeta, 'calibracion.json')
        self.ruta_maniobras = os.path.join(carpeta, 'ruedas_maniobras.csv')
        self.cal = cargar_cal(self.ruta_cal)
        self.odo = Odometria()
        self.estado = None
        self._n_estado = self._n_ticks = None
        self.t_estado = None
        self.conectado = False
        self.modo = 'duty'                  # 'duty' (lazo abierto) o 'vel' (velocidad con PI, m/s)
        self.armado = False
        self.aceleracion = 100.0            # % del fondo de escala por segundo
        self.objetivo = [0.0] * 4           # lo que marcan las barras: % de duty o m/s
        self.aplicado = [0.0] * 4           # lo que sale de verdad, despues de la rampa de aceleracion
        self.maniobra = None
        self.marcha = None
        self.ultimo_fin = ''
        self.guiado = None                  # avance que endereza o giro de 180 con retroceso (por sonares), mientras dura
        self.secuencia = deque()            # pasos pendientes de una secuencia: ('distancia', m) o ('giro', grados)
        self._paso_ok = True                # el ultimo paso de la secuencia termino bien (si no, se corta)
        self._v_sec = (0.21, 120.0)         # m/s y grados/s de la secuencia en curso
        self.resultado = None               # ultima maniobra terminada, a la espera de que cargues lo medido con cinta
        self.ajustar_frenado = True
        self.cola = deque()                 # ordenes de configuracion a mandar de a una por tick
        self.ticks_base = [0, 0, 0, 0]      # los ticks de la ESP32 en el momento del ultimo 'cero' de la PC
        self.grabacion = None
        self._t_tick = None
        self._t_orden = 0.0
        self._t_informe = 0.0

    # --- valores en uso: lo que informa la ESP32, y si todavia no hay datos, lo guardado o el del firmware
    def _valor(self, indice, escala, clave, defecto):
        if self.estado is not None and len(self.estado) > indice and self.estado[indice] > 0:
            return self.estado[indice] / escala
        return self.cal.get(clave, defecto)

    @property
    def diametro(self):
        return self._valor(20, 1000.0, 'diametro_m', DIAMETRO_M)

    @property
    def trocha(self):
        return self._valor(23, 1000.0, 'trocha_m', TROCHA_M)

    @property
    def frenado(self):
        if self.estado is not None:
            return self.estado[21] / 1000.0                  # 0 mm es un valor valido: no se cae al guardado
        return self.cal.get('frenado_m', FRENADO_M)

    @property
    def frenado_giro(self):
        if self.estado is not None and len(self.estado) > 25:
            return self.estado[25] / 1000.0
        return self.cal.get('frenado_giro_m', FRENADO_GIRO_M)

    def poner_cero(self):
        """Odometria y ticks de la PC en cero (la ESP32 sigue contando por su cuenta)."""
        self.odo.reiniciar()
        self.ticks_base = list(self.enlace.ticks or [0] * 4)

    def ticks_desde_cero(self):
        t = self.enlace.ticks or [0] * 4
        return [a - b for a, b in zip(t, self.ticks_base)]

    # --- ruedas: lo que mide la ESP32, en m/s
    def v_medida_ms(self):
        if self.estado is None:
            return [0.0] * 4
        return [ms_de_flancos(self.estado[i], self.diametro) for i in range(4)]

    def v_objetivo_ms(self):
        if self.estado is None:
            return [0.0] * 4
        return [ms_de_flancos(self.estado[4 + i], self.diametro) for i in range(4)]

    def se_mueve(self):
        return self.maniobra is not None or self.marcha is not None or self.guiado is not None or (
            self.estado is not None and any(abs(self.estado[i]) > 1.0 for i in range(4)))

    # --- envios
    def _cmd(self, ident, *args):
        self.enlace.enviar_cmd(ident, *args)

    def _mandar_config(self, diametro=None, trocha=None, frenado=None, frenado_giro=None, guardar=True):
        """Guarda y manda la configuracion (3 veces, por si el Wi-Fi pierde un paquete)."""
        d = diametro if diametro is not None else self.diametro
        t = trocha if trocha is not None else self.trocha
        f = frenado if frenado is not None else self.frenado
        g = frenado_giro if frenado_giro is not None else self.frenado_giro
        self.cal.update({'diametro_m': d, 'trocha_m': t, 'frenado_m': f, 'frenado_giro_m': g})
        if guardar:
            try:
                guardar_cal({**cargar_cal(self.ruta_cal), **self.cal}, self.ruta_cal)
            except OSError as e:
                self.imprimir(f'No se pudo guardar la calibracion: {e}')
        for _ in range(3):
            self.cola.append((5, d, t, f, g))

    def _mandar_guardado(self):
        """Al conectarse la ESP32 (que no guarda nada al apagarse), se le pasa lo ultimo calibrado."""
        if not any(k in self.cal for k in ('diametro_m', 'trocha_m', 'frenado_m', 'frenado_giro_m', 'curvas', 'giro_factor_del')):
            self.imprimir('(sin calibracion guardada: la ESP32 usa los valores del firmware)')
            return
        self.imprimir('Enviando la calibracion guardada a la ESP32...')
        for _ in range(3):
            self.cola.append((5, self.cal.get('diametro_m', DIAMETRO_M), self.cal.get('trocha_m', TROCHA_M),
                              self.cal.get('frenado_m', FRENADO_M), self.cal.get('frenado_giro_m', FRENADO_GIRO_M)))
        if 'giro_factor_del' in self.cal:
            for _ in range(3):
                self.cola.append((9, self.cal['giro_factor_del'], self.cal.get('kick_del', 0)))
        for i, c in enumerate(self.cal.get('curvas', [])[:4]):
            plano = [x for p in c['puntos'][:10] for x in p]
            for _ in range(3):
                self.cola.append((6, i, c['d_min'], len(c['puntos'][:10]), *plano))

    def config_delanteras(self, factor, nivel):
        """Giros en el lugar: las delanteras van `factor` x la velocidad de las traseras y su empujon de arranque es el `nivel`
        (0 a 3; las traseras usan 2). Se manda a la ESP32 (orden 9) y queda en calibracion.json para reenviarlo al reconectar."""
        if not 0.5 <= factor <= 2.0 or nivel not in (0, 1, 2, 3):
            self.imprimir('Delanteras: el factor va de 0,5 a 2 y el nivel de arranque es 0, 1, 2 o 3.')
            return False
        self.cal['giro_factor_del'] = factor
        self.cal['kick_del'] = nivel
        guardar_cal(self.cal, self.ruta_cal)
        if self.conectado:
            for _ in range(3):
                self.cola.append((9, factor, nivel))
        self.imprimir(f'Giros: delanteras x{factor:g}, empujon de arranque nivel {nivel}.')
        return True

    def config_pre_fr(self, duty_pct, ms):
        """Pre-arranque de FR en los giros de ~90 grados: FR recibe `duty_pct` % de duty durante `ms` ms ANTES de mandar el giro,
        asi ya viene rodando y no queda de ancla (0 = apagado). Solo software: no hace falta reflashear la ESP32."""
        if not 0 <= duty_pct <= 100 or not 0 <= ms <= 500:
            self.imprimir('Pre-arranque de FR: duty de 0 a 100 % y tiempo de 0 a 500 ms.')
            return False
        self.cal['fr_pre_duty_pct'] = duty_pct
        self.cal['fr_pre_ms'] = ms
        guardar_cal(self.cal, self.ruta_cal)
        self.imprimir(f'Pre-arranque de FR en los giros de 90: {duty_pct:g} % durante {ms:g} ms' + (' (apagado).' if not duty_pct or not ms else '.'))
        return True

    # --- barras
    def poner_modo(self, modo):
        if modo != self.modo:
            self.modo = modo
            self.objetivo = [0.0] * 4
            self.aplicado = [0.0] * 4

    def _fondo_escala(self):
        return 100.0 if self.modo == 'duty' else 0.40

    def parar(self):
        """Frena todo y desarma las barras. Se manda varias veces: es lo unico que no puede perderse."""
        self.objetivo = [0.0] * 4
        self.aplicado = [0.0] * 4
        self.armado = False
        for _ in range(3):
            self._cmd(4)
        self.enlace.enviar_vel(0.0, 0.0)
        if self.maniobra is not None or self.marcha is not None or self.secuencia or self.guiado is not None:
            self.imprimir('Maniobra cancelada.')
        self.maniobra = None
        self.marcha = None
        self.guiado = None
        self.secuencia.clear()

    # --- maniobras
    def _libre(self):
        if not self.conectado:
            self.imprimir('La ESP32 no esta conectada.')
            return False
        if self.maniobra is not None or self.marcha is not None or self.guiado is not None:
            self.imprimir('Ya hay una maniobra en curso (PARAR la cancela).')
            return False
        if self.cola:
            self.imprimir('Un momento: todavia se esta enviando la configuracion a la ESP32.')
            return False
        self.objetivo = [0.0] * 4
        self.aplicado = [0.0] * 4
        return True

    def _nueva_maniobra(self, tipo, nombre, pedido, orden, ahora):
        e = self.estado
        self.resultado = None
        self.maniobra = {'tipo': tipo, 'nombre': nombre, 'pedido': pedido, 'orden': orden, 't0': ahora, 'proximo_envio': ahora,
                         'activa': False, 'seq0': e[19], 'odo0': self.odo.foto(), 'ticks0': list(self.enlace.ticks or [0] * 4),
                         'diametro': self.diametro, 'trocha': self.trocha, 'frenado': self.frenado,
                         'frenado_giro': self.frenado_giro, 'recorrido': 0.0}

    def distancia(self, metros, vel_ms):
        """Marcha de `metros` (con signo) con rampas, medida con los encoders. vel_ms es la velocidad maxima de cada rueda."""
        if not self._libre() or metros == 0:
            return False
        vel = limitar(flancos_de_ms(abs(vel_ms), self.diametro), VEL_MIN_FLANCOS, VEL_MAX_FLANCOS)
        sentido = 'adelante' if metros > 0 else 'atras'
        self._nueva_maniobra('distancia', f'{abs(metros):g} m {sentido}', metros, (3, metros, vel), self.reloj())
        self.imprimir(f'== {self.maniobra["nombre"]} a {vel_ms:.2f} m/s ({vel:.0f} flancos/s) ==')
        return True

    def velocidad_giro_traseras(self, vel_flancos):
        """Velocidad base (traseras) de un giro en el lugar, en flancos/s. En los giros las delanteras (FL y FR) van `factor` x las traseras
        con tope de 75: si las traseras ya estan en 75 (153 grados/s, el maximo), las delanteras no pueden ir mas rapido y el factor no
        hace nada. Con 'giro_bajar_traseras' (por defecto si) se BAJAN las traseras a 75 / factor: las delanteras, que son las que
        se traban (FR), quedan en el maximo y vale la ventaja que se les da. Todo desde la PC: solo cambia la velocidad que se pide."""
        if not self.cal.get('giro_bajar_traseras', True):
            return vel_flancos
        factor = self.cal.get('giro_factor_del', GIRO_FACTOR_DEL_DEFECTO)
        return min(vel_flancos, VEL_MAX_FLANCOS / max(1.0, factor))

    def girar(self, grados, vel_dps):
        """Giro en el lugar de `grados` (+ izquierda, - derecha). vel_dps: velocidad angular maxima, grados/s."""
        if not self._libre() or grados == 0:
            return False
        arco_ms = math.radians(abs(vel_dps)) * self.trocha / 2.0                   # m/s de cada rueda
        vel = limitar(flancos_de_ms(arco_ms, self.diametro), VEL_MIN_FLANCOS, VEL_MAX_FLANCOS)
        vel = self.velocidad_giro_traseras(vel)
        sentido = 'izquierda' if grados > 0 else 'derecha'
        self._nueva_maniobra('giro', f'giro de {abs(grados):g} grados a la {sentido}', grados,
                             (7, math.radians(grados), vel), self.reloj())
        duty, ms = self.cal.get('fr_pre_duty_pct', 0.0), self.cal.get('fr_pre_ms', 0.0)
        if duty > 0 and ms > 0 and 60.0 <= abs(grados) <= 120.0:          # solo los giros de ~90
            self.maniobra['pre'] = {'duty': math.copysign(duty / 100.0, grados), 'ms': ms, 't_ini': None}
        front = min(VEL_MAX_FLANCOS, vel * self.cal.get('giro_factor_del', GIRO_FACTOR_DEL_DEFECTO))
        self.imprimir(f'== {self.maniobra["nombre"]} (traseras a {vel:.0f} flancos/s, delanteras a {front:.0f}; {vel_dps:.0f} grados/s pedidos) ==')
        return True

    # --- sonares (del /wheel_state del firmware: [26] frente, [27] izquierdo, [28] derecho, en m; -1 = sin eco)
    def sonares(self):
        """(frente, izquierda, derecha) en m, con None donde no hay eco; None si la ESP32 no informa sonares."""
        e = self.estado
        if e is None or len(e) < 29:
            return None
        return tuple(None if e[i] < 0 else e[i] for i in (26, 27, 28))

    @staticmethod
    def texto_sonares(s):
        cm = lambda m: 'sin eco' if m is None else f'{100 * m:.1f} cm'
        return f'pared izquierda {cm(s[1])} | pared derecha {cm(s[2])} | frente {cm(s[0])}'

    def girar_180_auto(self, vel_dps, empate_m=0.01):
        """Giro de 180 grados hacia el lado contrario de la pared lateral mas cercana (segun los sonares). Al terminar informa las
        distancias a las paredes laterales y a la frontal. Un lado sin eco cuenta como libre; sin eco en los dos, o con distancias
        iguales (menos de `empate_m`), no hay con que decidir y gira a la izquierda."""
        if not self.conectado:
            self.imprimir('La ESP32 no esta conectada.')
            return False
        son = self.sonares()
        if son is None:
            self.imprimir('La ESP32 no informa los sonares (hace falta el firmware con sonares: /wheel_state de 34 valores).')
            return False
        _, izq, der = son
        li, ld = (9.0 if izq is None else izq), (9.0 if der is None else der)
        if izq is None and der is None:
            self.imprimir('Sin eco en los dos laterales: no hay pared de referencia, giro a la izquierda.')
            grados = 180.0
        elif abs(li - ld) < empate_m:
            self.imprimir('Las dos paredes laterales estan a la misma distancia: giro a la izquierda.')
            grados = 180.0
        else:
            grados = -180.0 if li < ld else 180.0                  # pared izquierda mas cerca -> gira a la derecha, y al reves
            self.imprimir('Mas cerca la pared ' + ('izquierda' if li < ld else 'derecha') + ': giro a la ' +
                          ('derecha' if grados < 0 else 'izquierda') + '.')
        self.imprimir('  Antes del giro: ' + self.texto_sonares(son))
        if not self.girar(grados, vel_dps):
            return False
        self.maniobra['informar_sonares'] = True
        return True

    def secuenciar(self, pasos, vel_ms, vel_dps):
        """Encadena maniobras: pasos = [('distancia', m), ('giro', grados), ...] (m y grados con signo). Cada una arranca cuando la
        anterior termino (la ESP32 ya espero que el robot se asiente). Si una falla o se cancela, las demas no se ejecutan."""
        if not self._libre() or not pasos:
            return False
        self.secuencia = deque(pasos)
        self._v_sec = (vel_ms, vel_dps)
        self.imprimir('== secuencia: ' + ' -> '.join(f'{v:+g} m' if t == 'distancia' else f'giro {v:+g}°' for t, v in pasos) + ' ==')
        self._siguiente_paso()
        return True

    def escape_de_pared(self, atras_m, grados, vel_ms, vel_dps, lado_pared=0):
        """Se despega de una pared: retrocede UNA vez `atras_m` en recto y gira `grados` hacia el lado CONTRARIO de la pared que lo
        toco o hacia la que esta inclinado (la pared lateral mas cercana segun los sonares). `lado_pared` +1 / -1 fuerza que la pared
        esta a la izquierda / derecha (0 = lo decide con los sonares; sin sonares, a la izquierda)."""
        if not self._libre():
            return False
        if lado_pared == 0:
            son = self.sonares()
            izq, der = (son[1], son[2]) if son else (None, None)
            lado_pared = -1 if (der is not None and (izq is None or der < izq)) else 1
        giro = -lado_pared * abs(grados)                    # pared a la izquierda (+1): gira a la derecha (-)
        self.imprimir(f'== escape: pared a la {"izquierda" if lado_pared > 0 else "derecha"}: retrocedo {100 * atras_m:.0f} cm y '
                      f'giro {abs(grados):g}° a la {"derecha" if giro < 0 else "izquierda"} ==')
        return self.secuenciar([('distancia', -abs(atras_m)), ('giro', giro)], vel_ms, vel_dps)

    def _siguiente_paso(self):
        tipo, valor = self.secuencia.popleft()
        self._paso_ok = False
        self.resultado = None
        ok = self.distancia(valor, self._v_sec[0]) if tipo == 'distancia' else self.girar(valor, self._v_sec[1])
        if not ok:
            self.secuencia.clear()

    def _avanzar_secuencia(self):
        """Se llama sin maniobra en curso: lanza el paso que sigue, o corta si el anterior no termino."""
        if not self.secuencia:
            return
        if not self._paso_ok:
            self.secuencia.clear()
            self.imprimir('Secuencia cortada: un paso no termino.')
        elif not self.cola:
            self._siguiente_paso()

    # --- guiado por sonares: linea recta enderezando y giro de 180 hacia la pared. Los dos comparten el ciclo
    #     FRENAR -> PENSAR -> (retroceder) -> ENDEREZAR (pivote) -> SEGUIR cuando algo se acerca demasiado.
    def _parar_motores(self):
        for _ in range(3):
            self._cmd(4)
        self.enlace.enviar_vel(0.0, 0.0)

    def _vel_guiada(self, g, ahora, v, w):
        if ahora >= g['proximo_envio'] - TOLERANCIA_RELOJ_S:
            self.enlace.enviar_vel(v, w)
            g['proximo_envio'] = ahora + PERIODO_ORDEN_S

    def _fin_guiado(self, texto):
        self.ultimo_fin = texto
        self.guiado = None
        self._parar_motores()
        self.imprimir('  ' + texto)

    def _etapa(self, g, nombre, ahora):
        g['etapa'], g['t_etapa'] = nombre, ahora

    def enderezar_avanzando(self, distancia_m=None, v_ms=V_LENTA_MS):
        """Linea recta por el pasillo: avanza despacio y se mantiene alineado y centrado con los sonares laterales (EstimadorPasillo
        y timon_pasillo, con termino integral). `distancia_m` None o 0: sigue hasta que haya una pared de frente (STOP_FRENTE_M) o
        PARAR. Si un costado se acerca demasiado (o va a estarlo en 10 cm segun el rumbo estimado) NO sigue: frena, piensa,
        retrocede un poco, se endereza girando en el lugar y recien ahi sigue (las veces que haga falta)."""
        if not self._libre():
            return False
        son = self.sonares()
        if son is None:
            self.imprimir('La ESP32 no informa los sonares (hace falta el firmware con sonares: /wheel_state de 34 valores).')
            return False
        frente, izq, der = son
        if frente is not None and frente <= STOP_FRENTE_M:
            self.imprimir(f'No avanzo: hay una pared de frente a {100 * frente:.1f} cm (el minimo es {100 * STOP_FRENTE_M:.0f} cm). '
                          'Poné el robot mirando hacia el lado abierto del pasillo.')
            return False
        if not distancia_m or distancia_m <= 0:
            distancia_m = float('inf')
        ahora = self.reloj()
        self.guiado = {'tipo': 'enderezar', 'etapa': 'avanzar', 't_etapa': ahora, 's0': self.odo.s, 'dist': distancia_m, 'v': v_ms,
                       't0': ahora, 'proximo_envio': ahora, 'proximo_log': ahora, 'est': EstimadorPasillo(), 'despues': None,
                       'sesgo': 0.0, 't_ant': ahora}
        donde = 'hasta la pared del frente' if distancia_m == float('inf') else f'{100 * distancia_m:.0f} cm'
        self.imprimir(f'== linea recta enderezando: {donde} a {v_ms:.2f} m/s ==')
        self.imprimir('  Al arrancar: ' + self.texto_sonares(son))
        lat = min([d for d in (izq, der) if d is not None], default=None)
        if lat is not None and lat <= STOP_LATERAL_M + 0.01:        # pegado a una pared: quieto, primero se endereza
            self._frenar_y_pensar(self.guiado, ahora, f'una pared lateral a {100 * lat:.1f} cm')
        else:
            tilt = inclinacion_por_suma(izq, der)
            if tilt is not None and (tilt > math.radians(40) or (tilt > math.radians(15) and lat is not None and lat < 0.10)):
                # muy torcido (o torcido y ya cerca de una pared): no arranca a ciegas, primero mide y se endereza
                self._frenar_y_pensar(self.guiado, ahora, f'parece muy inclinado (~{math.degrees(tilt):.0f}° por la suma de los laterales)')
            # el resto (un costado cerca, algo inclinado) lo corrige en marcha _paso_enderezar con correcciones bruscas
        return True

    def giro_180_hacia_la_pared(self, vel_dps, adicional_deg=0.0, despues_m=None, adicional90_deg=0.0):
        """Giro de 180 grados = DOS giros de 90 seguidos (la orden 7 de la ESP32, la del boton Giro 90), hacia el lado de la pared
        lateral mas cercana. Cada uno es 90 + `adicional90_deg` (el sumador del Giro 90 del panel); el segundo suma ademas
        `adicional_deg` (el del 180). Solo retrocede un poco si el frente esta muy cerca (F_RETROCESO_M). Con `despues_m`, al terminar
        avanza esa distancia enderezandose."""
        if not self._libre():
            return False
        son = self.sonares()
        if son is None:
            self.imprimir('La ESP32 no informa los sonares (hace falta el firmware con sonares: /wheel_state de 34 valores).')
            return False
        frente, izq, der = son
        lado = 1.0 if (izq is not None and (der is None or izq <= der)) else (-1.0 if der is not None else 1.0)   # + izquierda
        ahora = self.reloj()
        g = {'tipo': 'giro2', 'etapa': 'girando', 't_etapa': ahora, 't0': ahora, 'proximo_envio': ahora, 'proximo_log': ahora,
             'theta0': self.odo.theta, 'lado': lado, 'vel_dps': vel_dps, 'adicional': adicional_deg, 'adicional90': adicional90_deg,
             'despues': despues_m}
        self.guiado = g
        self.imprimir(f'== giro de 180° = dos giros de 90° hacia la {"izquierda" if lado > 0 else "derecha"} (pared mas cercana) ==')
        self.imprimir('  Antes del giro: ' + self.texto_sonares(son))
        if frente is not None and frente < F_RETROCESO_M:
            g['atras'] = {'dist': min(ATRAS_MAX_M, F_RETROCESO_M - frente + 0.01), 's0': self.odo.s, 'siguiente': 'girando'}
            self._etapa(g, 'atras', ahora)
            self.imprimir(f'  Frente a {100 * frente:.1f} cm: retrocedo {100 * g["atras"]["dist"]:.0f} cm antes de girar.')
        return True

    # ---- frenar y pensar: quieto un instante, mide, y se separa de la pared (nunca se rinde)
    def _frenar_y_pensar(self, g, ahora, motivo):
        self._parar_motores()
        self.imprimir(f'  FRENO: {motivo}. Pienso antes de seguir...')
        self._etapa(g, 'pensar', ahora)

    def _pensar(self, g, ahora, frente, izq, der):
        """Quieto 0,6 s para tener medidas firmes; despues retrocede si el frente esta cerca y se separa de la pared."""
        if ahora - g['t_etapa'] < 0.6:
            return
        if frente is not None and frente < F_RETROCESO_M:
            d_atras = min(ATRAS_MAX_M, F_RETROCESO_M - frente + 0.01)
            g['atras'] = {'dist': d_atras, 's0': self.odo.s, 'siguiente': 'despegar'}
            self.imprimir(f'  Pienso: {self.texto_sonares((frente, izq, der))} -> retrocedo {100 * d_atras:.0f} cm (frente cerca) y me separo.')
            self._etapa(g, 'atras', ahora)
        else:
            self.imprimir(f'  Pienso: {self.texto_sonares((frente, izq, der))} -> me separo de la pared.')
            self._despegar_desde_sonares(g, ahora, izq, der)

    def _despegar_desde_sonares(self, g, ahora, izq, der):
        cerca = min([d for d in (izq, der) if d is not None], default=0.1)
        self._empezar_despegue(g, ahora, izq is not None and (der is None or izq <= der), cerca)

    def _atras(self, g, ahora, frente, izq, der):
        a = g['atras']
        recorrido = a['s0'] - self.odo.s
        lateral = min([d for d in (izq, der) if d is not None], default=None)
        if recorrido >= a['dist'] or ahora - g['t_etapa'] > 4.0 or (lateral is not None and lateral <= STOP_LATERAL_M):
            self._parar_motores()
            self._etapa(g, 'asentar_atras', ahora)
            return
        self._vel_guiada(g, ahora, -V_ATRAS_RECTO_MS, 0.0)

    def _asentar_atras(self, g, ahora, frente, izq, der):
        a = g['atras']
        if ahora - g['t_etapa'] < 0.5:
            return
        self.imprimir(f'  Retrocedi {100 * (a["s0"] - self.odo.s):.0f} cm: ' + self.texto_sonares((frente, izq, der)))
        if a['siguiente'] == 'girando':                          # solo retrocedia para despejar el frente antes de girar
            self._etapa(g, 'girando', ahora)
            g['theta0'] = self.odo.theta
        else:
            self._despegar_desde_sonares(g, ahora, izq, der)

    # ---- despegarse de una pared: arco hacia adelante alejandose de ella (no se rinde: reintenta cada vez mas fuerte)
    def _empezar_despegue(self, g, ahora, es_izq, d):
        n = g.get('n_despegues', 0)
        g['n_despegues'] = n + 1
        ang = min(1.0, ANG_DESPEGUE_RAD + 0.15 * n)
        g['despegue'] = {'signo': -1.0 if es_izq else 1.0, 'theta0': self.odo.theta, 't0': ahora, 'ang': ang, 'izq': es_izq}
        self.imprimir(f'  Pegado a la pared {"izquierda" if es_izq else "derecha"} ({100 * d:.1f} cm): me separo avanzando en arco '
                      f'hacia el otro lado (hasta {math.degrees(ang):.0f}°, intento {n + 1}).')
        self._etapa(g, 'despegar', ahora)

    def _paso_despegue(self, ahora, g, frente, izq, der):
        b = g['despegue']
        girado = (self.odo.theta - b['theta0']) * b['signo']
        cerca = izq if b['izq'] else der
        if frente is not None and frente <= STOP_FRENTE_M + 0.03:
            self._parar_motores()                                # no hay lugar adelante: retrocede recto y reintenta
            self.imprimir(f'  Frente a {100 * frente:.1f} cm: retrocedo antes de separarme.')
            g['atras'] = {'dist': ATRAS_MAX_M, 's0': self.odo.s, 'siguiente': 'despegar'}
            self._etapa(g, 'atras', ahora)
            return
        separado = cerca is not None and cerca >= SEPARADO_M and girado >= 0.12
        if separado or girado >= b['ang'] - W_DESPEGUE_RAD_S * 0.2 or ahora - b['t0'] > 2.5:
            self._parar_motores()
            self.imprimir(f'  Separado: giré {math.degrees(girado):.0f}° | ' + self.texto_sonares((frente, izq, der)))
            self._etapa(g, 'asentar_despegue', ahora)
            return
        v_min = VEL_MIN_FLANCOS / flancos_por_m(self.diametro) + MARGEN_RUEDA_MS
        w = b['signo'] * W_DESPEGUE_RAD_S
        self._vel_guiada(g, ahora, max(g['v'], min(0.30, v_min + abs(w) * self.trocha / 2.0)), w)

    def _asentar_despegue(self, ahora, g, frente, izq, der):
        if ahora - g['t_etapa'] < 0.5:
            return
        cerca = min([d for d in (izq, der) if d is not None], default=None)
        if cerca is not None and cerca <= ALERTA_LATERAL_M:
            self._frenar_y_pensar(g, ahora, f'sigo pegado a una pared ({100 * cerca:.1f} cm): reintento')
            return
        self.imprimir('  Libre de la pared: sigo avanzando y vigilo.')
        g['est'] = EstimadorPasillo()
        g['hist'] = []
        g['t_ant'] = ahora
        g['s_brusca'] = self.odo.s
        g['recto_hasta'] = self.odo.s
        self._etapa(g, 'avanzar', ahora)

    # ---- pivotar en el lugar (orden 7 del firmware: la ESP32 cuenta los encoders y corta sola; aca solo se espera que termine)
    def _iniciar_pivote(self, g, ahora, delta, etapa, clave='pivote', flancos=PIVOTE_FLANCOS):
        g[clave] = {'delta': delta, 'seq0': self.estado[19], 'activa': False, 'proximo': ahora, 't0': ahora, 'flancos': flancos}
        self._etapa(g, etapa, ahora)

    def _pivotar(self, g, ahora, frente, izq, der, clave='pivote', siguiente=None):
        pv = g[clave]
        if abs(pv['delta']) < math.radians(3):                   # nada que girar
            self._etapa(g, siguiente or 'asentar_pivote', ahora)
            return
        e = self.estado
        if not pv['activa'] and e[16] > 0.5:
            pv['activa'] = True
        if not pv['activa']:
            if ahora >= pv['proximo']:                           # se repite hasta que la ESP32 arranca: UDP puede perder una
                self._cmd(7, pv['delta'], pv.get('flancos', PIVOTE_FLANCOS))
                pv['proximo'] = ahora + 0.3
            if ahora - pv['t0'] > 4.0:
                self._fin_guiado('ME DETENGO: la ESP32 no empezo el pivote.')
            return
        if e[19] > pv['seq0'] and e[16] < 0.5:                   # termino (y ya espero a que el robot se asiente)
            self._etapa(g, siguiente or 'asentar_pivote', ahora)
        elif ahora - pv['t0'] > 12.0:
            self._fin_guiado('ME DETENGO: el pivote tardo mas de 12 s.')

    # ---- correccion brusca en marcha: giro rapido alejandose de la pared sin frenar
    def _empezar_brusca(self, g, ahora, d, hacia, psi):
        """`hacia` +1: la pared cercana esta a la izquierda (hay que girar a la derecha), -1: al reves."""
        if self.odo.s - g.get('s_ult_brusca', -9.0) > AVANCE_REPETIDO_M:
            g['n_bruscas'] = 0
        g['n_bruscas'] = g.get('n_bruscas', 0) + 1
        extra = BRUSCO_MAS_RAD * (g['n_bruscas'] - 1)
        if psi is not None:                                      # sabe hacia donde apunta: gira lo justo para quedar ~11° alejandose de la pared
            ang = max(0.12, psi * hacia + 0.14)
        else:
            ang = ANG_BRUSCO_RAD
        ang = min(ANG_BRUSCO_TOPE_RAD, ang + extra)
        lado = 'izquierda' if hacia > 0 else 'derecha'
        self.imprimir(f'  CORRIJO ({g["n_bruscas"]}): pared {lado} a {100 * d:.1f} cm y se acerca -> giro {math.degrees(ang):.0f}° '
                      f'hacia el otro lado sin frenar.')
        g['brusca'] = {'signo': -hacia, 'ang': ang, 'theta0': self.odo.theta, 't0': ahora, 'd0': d}
        g['s_ult_brusca'] = g['s_brusca'] = self.odo.s
        self._etapa(g, 'brusca', ahora)

    def _paso_brusca(self, ahora, g, frente, izq, der):
        b = g['brusca']
        girado = (self.odo.theta - b['theta0']) * b['signo']
        lateral = min([d for d in (izq, der) if d is not None], default=None)
        if frente is not None and frente <= STOP_FRENTE_M:
            self._fin_guiado(f'Frene: pared de frente a {100 * frente:.1f} cm (avance {100 * (self.odo.s - g["s0"]):.0f} cm).')
            return
        if lateral is not None and lateral <= STOP_LATERAL_M + 0.005:
            self._frenar_y_pensar(g, ahora, f'una pared lateral a {100 * lateral:.1f} cm')
            return
        if girado >= b['ang'] - W_BRUSCO_RAD_S * 0.22 or ahora - b['t0'] > 1.6:      # el robot sigue girando un poco al cortar
            self.imprimir(f'  Corregido: giré {math.degrees(girado):.0f}° | ' + self.texto_sonares((frente, izq, der)) + '. Sigo y vigilo.')
            g['est'] = EstimadorPasillo()
            g['hist'] = []
            g['t_ant'] = ahora
            g['s_brusca'] = self.odo.s
            g['recto_hasta'] = self.odo.s + RECTO_TRAS_BRUSCA_M
            self._etapa(g, 'avanzar', ahora)
            return
        v_min = VEL_MIN_FLANCOS / flancos_por_m(self.diametro) + MARGEN_RUEDA_MS
        w = b['signo'] * W_BRUSCO_RAD_S
        v = max(g['v'], min(0.30, v_min + abs(w) * self.trocha / 2.0))
        self._vel_guiada(g, ahora, v, w)

    # ---- el avance recto
    def _paso_enderezar(self, ahora, g, frente, izq, der):
        recorrido = self.odo.s - g['s0']
        lateral = min([d for d in (izq, der) if d is not None], default=None)
        motivo = None
        if recorrido >= g['dist']:
            motivo = f'Llegue: avance {100 * recorrido:.0f} cm.'
        elif frente is not None and frente <= STOP_FRENTE_M:
            motivo = f'Frene: pared de frente a {100 * frente:.1f} cm (avance {100 * recorrido:.0f} cm).'
        elif ahora - g['t0'] > 90.0:
            motivo = 'Frene: tarde mas de 90 s.'
        if motivo:
            self._fin_guiado(motivo)
            self.imprimir('  Al terminar: ' + self.texto_sonares(self.sonares() or (frente, izq, der)))
            return
        y, psi = g['est'].actualizar(self.odo.s, izq, der)
        # Muy pegado a una pared: ahora si, frena y piensa (lo normal es corregir en marcha, ver abajo)
        if lateral is not None and lateral <= STOP_LATERAL_M + 0.005:
            self._frenar_y_pensar(g, ahora, f'una pared lateral a {100 * lateral:.1f} cm')
            return
        # Correccion brusca en marcha: un costado que baja de ~7,5 cm y se sigue acercando (o ya esta a 5,5 cm) -> gira rapido hacia
        # el otro lado SIN frenar y sigue avanzando; despues vuelve al control normal, que vigila que no se siga acercando.
        h = g.setdefault('hist', [])
        h.append((ahora, izq, der))
        while h and ahora - h[0][0] > 0.5:
            h.pop(0)
        if self.odo.s - g.get('s_brusca', -9.0) >= ESPERA_BRUSCA_M:
            for i, (d, hacia) in enumerate(((izq, +1.0), (der, -1.0))):
                if d is None:
                    continue
                antes = [x[1 + i] for x in h if x[1 + i] is not None]
                baja = (max(antes) - d) if antes else 0.0
                sube = (d - min(antes)) if antes else 0.0
                if sube >= 0.003 and d > STOP_LATERAL_M + 0.01:      # ya se esta alejando de esa pared: no corrige de mas
                    continue
                se_acerca_por_rumbo = psi is not None and d <= 0.16 and d - ANTICIPO_M * max(0.0, math.tan(psi) * hacia) <= STOP_LATERAL_M + 0.005
                if (d <= LAT_DISPARO_M and baja >= BAJA_DISPARO_M) or d <= LAT_CRITICO_M or se_acerca_por_rumbo:
                    self._empezar_brusca(g, ahora, d, hacia, psi)
                    return
        dt, g['t_ant'] = max(0.0, min(0.2, ahora - g['t_ant'])), ahora
        if psi is not None and y is not None:               # el termino integral solo aprende cuando se sabe donde esta y hacia donde apunta
            g['sesgo'] = limitar(g['sesgo'] + K_I * error_de_rumbo(y, psi) * dt, -SESGO_TOPE, SESGO_TOPE)
        v, w = timon_pasillo(g['v'], y, psi, self.trocha, self.diametro, g['sesgo'])
        if self.odo.s < g.get('recto_hasta', -1.0):          # recien corregido: derecho unos cm hasta medir el rumbo nuevo
            v, w = g['v'], 0.0
        self._vel_guiada(g, ahora, v, w)
        if ahora >= g['proximo_log']:
            g['proximo_log'] = ahora + 0.5
            self.imprimir(f'  avance {100 * recorrido:4.0f} cm | ' + self.texto_sonares((frente, izq, der)) +
                          f' | centro {"--" if y is None else f"{100 * y:+.1f} cm"} rumbo '
                          f'{"?" if psi is None else f"{math.degrees(psi):+.0f}°"} | v {v:.2f} w {w:+.2f} (sesgo {g["sesgo"]:+.2f})')

    # ---- el giro de 180 en dos giros de 90 (el pivote de la orden 7 que ya anda bien)
    def _empezar_giro_d(self, g, ahora, n):
        if n == 1:
            g['theta0'] = self.odo.theta
        grados = 90.0 + g['adicional90'] + (g['adicional'] if n == 2 else 0.0)
        arco_ms = math.radians(abs(g['vel_dps'])) * self.trocha / 2.0
        fl = self.velocidad_giro_traseras(limitar(flancos_de_ms(arco_ms, self.diametro), VEL_MIN_FLANCOS, VEL_MAX_FLANCOS))
        self.imprimir(f'  Giro {n}/2: {grados:g}° a la {"izquierda" if g["lado"] > 0 else "derecha"} ({fl:.0f} flancos/s).')
        self._iniciar_pivote(g, ahora, g['lado'] * math.radians(grados), f'pivotar_d{n}', clave=f'pivote_d{n}', flancos=fl)

    def _procesar_guiado(self, ahora):
        g = self.guiado
        son = self.sonares()
        if son is None:
            self._fin_guiado('Se cortaron los datos de los sonares: me detengo.')
            return
        frente, izq, der = son
        e = g['etapa']
        if e == 'avanzar':
            self._paso_enderezar(ahora, g, frente, izq, der)
        elif e == 'despegar':
            self._paso_despegue(ahora, g, frente, izq, der)
        elif e == 'asentar_despegue':
            self._asentar_despegue(ahora, g, frente, izq, der)
        elif e == 'brusca':
            self._paso_brusca(ahora, g, frente, izq, der)
        elif e == 'girando':
            self._empezar_giro_d(g, ahora, 1)
        elif e == 'pivotar_d1':
            self._pivotar(g, ahora, frente, izq, der, clave='pivote_d1', siguiente='pausa_d')
        elif e == 'pausa_d':
            self._empezar_giro_d(g, ahora, 2)
        elif e == 'pivotar_d2':
            self._pivotar(g, ahora, frente, izq, der, clave='pivote_d2', siguiente='asentando')
        elif e == 'pensar':
            self._pensar(g, ahora, frente, izq, der)
        elif e == 'atras':
            self._atras(g, ahora, frente, izq, der)
        elif e == 'asentar_atras':
            self._asentar_atras(g, ahora, frente, izq, der)
        elif e == 'asentando':
            if ahora - g['t_etapa'] >= 0.6:
                g['girado'] = (self.odo.theta - g['theta0']) * g['lado']
                self.guiado = None
                self.imprimir(f'  Giro hecho: {math.degrees(g["girado"]):.1f}° segun los encoders (pedido {180 + 2 * g["adicional90"] + g["adicional"]:g}°).')
                self.imprimir('  Despues del giro: ' + self.texto_sonares((frente, izq, der)))
                if g['despues']:
                    self.enderezar_avanzando(g['despues'])

    def marchar(self, v, w, segundos):
        """Velocidad lineal v (m/s) y angular w (rad/s) con /cmd_vel durante `segundos`. Termina frenando."""
        if not self._libre() or segundos <= 0:
            return False
        ahora = self.reloj()
        self.resultado = None
        self.marcha = {'v': v, 'w': w, 't': segundos, 't0': ahora, 'odo0': self.odo.foto(),
                       'ticks0': list(self.enlace.ticks or [0] * 4), 'proximo_envio': ahora, 'vel_muestras': []}
        self.imprimir(f'== marchar a {v:+.2f} m/s, {w:+.2f} rad/s durante {segundos:g} s ==')
        return True

    def _procesar_maniobra(self, ahora):
        m, e = self.maniobra, self.estado
        m['recorrido'] = e[17]
        if not m['activa'] and e[16] > 0.5:
            m['activa'] = True
        pre = m.get('pre')
        if pre is not None and not m['activa']:                # FR arranca primero (lazo abierto) y despues va el giro
            if pre['t_ini'] is None:
                pre['t_ini'] = ahora
            if ahora - pre['t_ini'] < pre['ms'] / 1000.0:
                self._cmd(1, 0.0, pre['duty'], 0.0, 0.0)       # solo FR; + = adelante (grados + = izquierda: la derecha avanza)
                return
            m['pre'] = None
            m['proximo_envio'] = ahora
        if not m['activa']:
            if ahora >= m['proximo_envio']:                  # se repite hasta que la ESP32 arranca: UDP puede perder uno
                self._cmd(*m['orden'])
                m['proximo_envio'] = ahora + 0.3
            if ahora - m['t0'] > 4.0:
                self.imprimir('La ESP32 no empezo la maniobra (ya tenia una en curso o la orden no llego). Cancelada.')
                self.maniobra = None
            return
        if e[19] > m['seq0'] and e[16] < 0.5:
            self._terminar_maniobra(e)
        elif e[19] < m['seq0']:
            m['seq0'] = e[19]                                # la ESP32 se reinicio: el contador volvio a empezar
        elif ahora - m['t0'] > 45.0:
            self.imprimir('La maniobra tardo mas de 45 s: se corta.')
            self.parar()

    def _terminar_maniobra(self, e):
        m = self.maniobra
        self.maniobra = None
        odo1 = self.odo.foto()
        o0 = m['odo0']
        ticks = [a - b for a, b in zip(self.enlace.ticks or [0] * 4, m['ticks0'])]
        r = {'tipo': m['tipo'], 'nombre': m['nombre'], 'pedido': m['pedido'], 'ticks': ticks, 'diametro': m['diametro'],
             'trocha': m['trocha'], 'frenado': m['frenado'], 'frenado_giro': m['frenado_giro'], 'duracion': self.reloj() - m['t0']}
        if m['tipo'] == 'distancia':
            r['calculado'] = e[18]
            r['odometria'] = odo1['s'] - o0['s']
            sobre = (abs(e[18]) - abs(m['pedido']))                       # positivo: paso de la meta
            r['sobre_m'] = sobre
            self.imprimir(f'  pedido {m["pedido"]:+.3f} m | calculado por la ESP32 {e[18]:+.3f} m (pasada {sobre * 1000:+.0f} mm)'
                          f' | odometria de la PC {r["odometria"]:+.3f} m')
            umbral, tope, clave = UMBRAL_FRENADO_M, FRENADO_TOPE_M, 'frenado'
        else:
            r['calculado'] = grados_de_arco(e[18], m['trocha'])
            r['odometria'] = math.degrees(odo1['theta'] - o0['theta'])
            sobre = math.radians(abs(r['calculado']) - abs(m['pedido'])) * m['trocha'] / 2.0   # arco de mas, m
            r['sobre_m'] = sobre
            dx, dy = odo1['x'] - o0['x'], odo1['y'] - o0['y']
            r['desplazamiento_odo_cm'] = 100.0 * math.hypot(dx, dy)
            self.imprimir(f'  pedido {m["pedido"]:+.1f}° | calculado por la ESP32 {r["calculado"]:+.1f}° (pasada {sobre * 1000:+.1f} mm de arco)'
                          f' | odometria de la PC {r["odometria"]:+.1f}° | el centro se movio {r["desplazamiento_odo_cm"]:.1f} cm segun la odometria')
            umbral, tope, clave = UMBRAL_FRENADO_GIRO_M, FRENADO_GIRO_TOPE_M, 'frenado_giro'
        self.imprimir('  ticks de la maniobra: ' + '  '.join(f'{n} {t:+d}' for n, t in zip(RUEDAS, ticks)))
        if self.ajustar_frenado and abs(r['sobre_m']) > umbral:
            nuevo = frenado_corregido(m[clave], r['sobre_m'], tope)
            if abs(nuevo - m[clave]) > 1e-6:
                self.imprimir(f'  Ajuste automatico del adelanto de frenado: {m[clave] * 1000:.1f} mm -> {nuevo * 1000:.1f} mm')
                self._mandar_config(**{clave: nuevo})
        if m.get('informar_sonares'):
            son = self.sonares()
            self.imprimir('  Despues del giro: ' + ('sin datos de sonar' if son is None else self.texto_sonares(son)))
        self.resultado = r
        self._paso_ok = True
        if self.secuencia:
            self.imprimir(f'  Sigue la secuencia ({len(self.secuencia)} paso(s) mas).')
        else:
            self.imprimir('  Medi con cinta/transportador y carga el valor real en el panel (o REGISTRAR sin corregir).')

    def _procesar_marcha(self, ahora):
        k = self.marcha
        if ahora >= k['proximo_envio'] - TOLERANCIA_RELOJ_S:
            self.enlace.enviar_vel(k['v'], k['w'])
            k['proximo_envio'] = ahora + PERIODO_ORDEN_S
        if self.estado is not None:
            k['vel_muestras'].append(self.v_medida_ms())
        if ahora - k['t0'] >= k['t']:
            self.marcha = None
            for _ in range(3):
                self._cmd(4)
            self.enlace.enviar_vel(0.0, 0.0)
            odo1, o0 = self.odo.foto(), k['odo0']
            ticks = [a - b for a, b in zip(self.enlace.ticks or [0] * 4, k['ticks0'])]
            r = {'tipo': 'marcha', 'nombre': f'marcha {k["v"]:+.2f} m/s, {k["w"]:+.2f} rad/s, {k["t"]:g} s',
                 'pedido': k['v'] * k['t'], 'calculado': odo1['s'] - o0['s'], 'odometria': odo1['s'] - o0['s'],
                 'ticks': ticks, 'diametro': self.diametro, 'trocha': self.trocha, 'frenado': self.frenado,
                 'frenado_giro': self.frenado_giro, 'duracion': ahora - k['t0'],
                 'giro_odo_deg': math.degrees(odo1['theta'] - o0['theta'])}
            prom = [sum(abs(v[i]) for v in k['vel_muestras']) / max(1, len(k['vel_muestras'])) for i in range(4)]
            self.imprimir(f'  esperado v x t = {r["pedido"]:+.3f} m | odometria de la PC {r["odometria"]:+.3f} m y '
                          f'{r["giro_odo_deg"]:+.1f}° | velocidad media medida (m/s) ' +
                          '  '.join(f'{n} {p:.3f}' for n, p in zip(RUEDAS, prom)))
            self.resultado = r
            self.imprimir('  Mide el avance con cinta y cargalo en el panel (o REGISTRAR sin corregir).')

    def registrar(self, real=None, desplazamiento_cm=None, corregir=False):
        """Anota la ultima maniobra en ruedas_maniobras.csv con lo medido a mano. Si `corregir`, usa `real` para corregir el
        diametro efectivo (distancias y marchas) o la trocha efectiva (giros) y manda el valor nuevo a la ESP32."""
        r = self.resultado
        if r is None:
            self.imprimir('No hay una maniobra terminada para registrar.')
            return False
        agregar_csv(self.ruta_maniobras, ENCABEZADO_MANIOBRAS,
                    [datetime.datetime.now().isoformat(timespec='seconds'), r['nombre'], r['tipo'], f'{r["pedido"]:.4f}',
                     f'{r["calculado"]:.4f}', f'{r["odometria"]:.4f}', '' if real is None else real,
                     '' if desplazamiento_cm is None else desplazamiento_cm, f'{r["diametro"] * 1000:.2f}',
                     f'{r["trocha"] * 1000:.1f}', f'{r["frenado"] * 1000:.1f}', f'{r["frenado_giro"] * 1000:.1f}', *r['ticks']])
        self.imprimir(f'Registrado en {self.ruta_maniobras}')
        if corregir and real is not None:
            if r['tipo'] == 'giro':
                nueva = trocha_corregida(r['trocha'], r['calculado'], real)
                self.imprimir(f'Trocha efectiva: {r["trocha"] * 1000:.1f} mm -> {nueva * 1000:.1f} mm '
                              f'(la ESP32 calculo {r["calculado"]:+.1f}° y el robot giro {real:g}°)')
                self._mandar_config(trocha=nueva)
            else:
                nuevo = diametro_corregido(r['diametro'], real, r['calculado'])
                self.imprimir(f'Diametro efectivo: {r["diametro"] * 1000:.2f} mm -> {nuevo * 1000:.2f} mm '
                              f'(calculado {abs(r["calculado"]):.3f} m, real {abs(real):g} m)')
                self._mandar_config(diametro=nuevo)
        self.resultado = None
        return True

    # --- el reloj del programa
    def tick(self):
        ahora = self.reloj()
        dt = 0.05 if self._t_tick is None else max(0.0, ahora - self._t_tick)
        self._t_tick = ahora
        self._leer_enlace(ahora)
        if self.conectado and self.cola and (self.maniobra is None or self.maniobra['activa']):
            self._cmd(*self.cola.popleft())
        if self.conectado and self.maniobra is not None:
            self._procesar_maniobra(ahora)
        elif self.conectado and self.marcha is not None:
            self._procesar_marcha(ahora)
        elif self.conectado and self.guiado is not None:
            self._procesar_guiado(ahora)
        elif self.conectado and self.secuencia:
            self._avanzar_secuencia()
        elif self.conectado:
            self._ordenes_de_las_barras(ahora, dt)
        self._informar(ahora)

    def _leer_enlace(self, ahora):
        if self.enlace.n_estado != self._n_estado and self.enlace.estado is not None and len(self.enlace.estado) >= 24:
            self._n_estado = self.enlace.n_estado
            self.estado = list(self.enlace.estado)
            self.t_estado = ahora
            if not self.conectado:
                self.conectado = True
                self.odo.rebase()
                self.imprimir('[ESP32 conectada]')
                self._mandar_guardado()
            self._grabar(ahora)
        elif self.conectado and self.t_estado is not None and ahora - self.t_estado > SILENCIO_S:
            self.conectado = False
            self.imprimir(f'[ESP32 sin datos hace {SILENCIO_S:g} s]')
            if self.maniobra is not None or self.marcha is not None or self.guiado is not None:
                self.maniobra = None
                self.marcha = None
                self.guiado = None
                self.secuencia.clear()
                self.imprimir('Maniobra interrumpida por falta de datos.')
        if self.enlace.n_ticks != self._n_ticks and self.enlace.ticks is not None:
            self._n_ticks = self.enlace.n_ticks
            self.odo.actualizar(self.enlace.ticks, self.diametro, self.trocha)

    def _ordenes_de_las_barras(self, ahora, dt):
        if not self.armado:
            self.aplicado = [0.0] * 4
            return
        paso = self.aceleracion / 100.0 * self._fondo_escala() * dt         # lo que puede cambiar cada rueda en este tick
        self.aplicado = [hacia(a, o, paso) for a, o in zip(self.aplicado, self.objetivo)]
        if ahora - self._t_orden < PERIODO_ORDEN_S - TOLERANCIA_RELOJ_S:
            return
        self._t_orden = ahora
        if self.modo == 'duty':
            self._cmd(1, *[limitar(a / 100.0, -1.0, 1.0) for a in self.aplicado])
        else:
            self._cmd(2, *[limitar(a, -0.40, 0.40) for a in self.aplicado])

    # --- informes: terminal y archivo
    def _informar(self, ahora):
        if self.estado is None or not self.se_mueve() or ahora - self._t_informe < 1.0:
            return
        self._t_informe = ahora
        v = self.v_medida_ms()
        o = self.odo
        self.imprimir('  v medida (m/s) ' + ' '.join(f'{n} {x:+.3f}' for n, x in zip(RUEDAS, v)) +
                      f' | odometria x {o.x:+.3f} m  y {o.y:+.3f} m  giro {math.degrees(o.theta):+.1f}°  recorrido {o.recorrido:.3f} m')

    def grabar(self, activar):
        if activar and self.grabacion is None:
            os.makedirs(self.carpeta, exist_ok=True)
            ruta = os.path.join(self.carpeta, datetime.datetime.now().strftime('ruedas_%Y%m%d_%H%M%S.csv'))
            self.grabacion = open(ruta, 'w', newline='')
            self.grabacion_ruta = ruta
            self._escritor = csv.writer(self.grabacion)
            self._escritor.writerow(ENCABEZADO_GRABACION)
            self._t_grab0 = self.reloj()
            self.imprimir(f'Grabando cada 0,1 s en {ruta}')
        elif not activar and self.grabacion is not None:
            self.grabacion.close()
            self.grabacion = None
            self.imprimir(f'Grabacion cerrada: {self.grabacion_ruta}')

    def _grabar(self, ahora):
        if self.grabacion is None or self.estado is None:
            return
        e, o = self.estado, self.odo
        pedido = self.aplicado if self.armado else [0.0] * 4
        nombre = (self.maniobra or {}).get('nombre') or ('marcha' if self.marcha else '')
        ticks = list(self.enlace.ticks or [0] * 4)
        self._escritor.writerow([f'{ahora - self._t_grab0:.2f}', self.modo, nombre] + [f'{p:.4f}' for p in pedido] +
                                [f'{e[4 + i]:.2f}' for i in range(4)] + [f'{e[i]:.2f}' for i in range(4)] +
                                [f'{e[8 + i]:.3f}' for i in range(4)] + ticks +
                                [f'{o.x:.4f}', f'{o.y:.4f}', f'{math.degrees(o.theta):.2f}', f'{o.s:.4f}'])
        self.grabacion.flush()

    def cerrar(self):
        self.grabar(False)
