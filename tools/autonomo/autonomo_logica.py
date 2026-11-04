"""Comportamiento "vagabundo" del robot: avanza derecho y, al ver un obstaculo con el sonar, gira en el lugar hasta ver libre.

Sin ROS: lo usan el nodo (robot_autonomo.py) y el simulador (simular.py), y se prueba solo (test_autonomo_logica.py).

La PC decide; el freno por sonar vive en la ESP32 (firmware/pruebas/comun/guardia.h) y actua aunque el Wi-Fi se cuelgue.
Esta logica le deja ese freno como ultimo recurso: gira antes de que haga falta (d_evitar > d_stop de la guardia).

Convenciones: Twist de ROS (v en m/s hacia adelante, w en rad/s, + = izquierda, antihorario). Distancias en m, o None si el
sonar no recibio eco (nada a menos de 4 m, o una superficie que no devuelve el eco).

Estados:  ESPERA     quieto, esperando la orden de arrancar
          AVANZAR    derecho a v_crucero, esquivando suavemente lo que tenga muy cerca a los costados
          GIRAR      en el lugar, hacia el lado con mas espacio, hasta ver libre adelante (mas un poco al azar)
          DETENIDO   quieto por una falla o por orden del operador: hay que volver a armar a mano
"""
import random
from dataclasses import dataclass

ESPERA, AVANZAR, GIRAR, DETENIDO = 'ESPERA', 'AVANZAR', 'GIRAR', 'DETENIDO'

# Estados de la guardia de la ESP32 (/wheel_state[29])
G_APAGADA, G_LIBRE, G_LENTA, G_FRENADA, G_FALLA = 0, 1, 2, 3, 4
NOMBRES_GUARDIA = {G_APAGADA: 'apagada', G_LIBRE: 'libre', G_LENTA: 'lenta', G_FRENADA: 'FRENADA', G_FALLA: 'FALLA'}


@dataclass
class Parametros:
    v_crucero: float = 0.20          # m/s: las 4 ruedas lo sostienen con margen (el techo practico medido es ~0,28)
    w_giro: float = 1.2              # rad/s en los giros en el lugar (cada rueda a ~0,18 m/s con trocha 0,30)
    d_evitar: float = 0.45           # m: con algo mas cerca que esto adelante, deja de avanzar y gira (la cadena PC-Wi-Fi-ESP32 tarda ~0,5 s)
    d_libre: float = 0.70            # m: el frente tiene que ver mas que esto para terminar el giro
    lecturas_libres: int = 2         # mediciones nuevas seguidas por encima de d_libre
    giro_min_s: float = 0.6          # un giro dura al menos esto
    giro_extra_s: tuple = (0.0, 1.0)  # despues de ver libre sigue girando un tiempo al azar entre estos
    giro_max_s: float = 8.0          # ~550 grados a 1,2 rad/s: si no encontro salida, se rinde
    d_lateral: float = 0.22          # m: un lado mas cerca que esto empuja el rumbo hacia el otro
    w_lateral: float = 0.35          # rad/s de ese empujon
    # m: girar en el lugar barre 16 cm de radio (8 cm mas que el costado del robot) en AMBOS lados; con un lado mas cerca que
    # esto no se gira: queda DETENIDO (encajonado)
    d_barrido: float = 0.12
    sin_datos_s: float = 0.6         # sin telemetria por mas de esto: se frena (y se reanuda sola si vuelve pronto)
    sin_datos_rearmar_s: float = 3.0  # ... pero si dura mas que esto, hay que armar a mano
    sonar_mudo_s: float = 1.0        # el contador del frontal no avanza por mas de esto: el sonar no mide


class Vagabundo:
    def __init__(self, params=None, rng=None):
        self.p = params or Parametros()
        self.rng = rng or random.Random()
        self.estado = ESPERA
        self.motivo = 'esperando la orden de arrancar'
        self.t_estado = 0.0
        self.giros = 0
        self.detenciones = 0
        self.historial = []              # (t, motivo) de cada detencion
        self._dir = 1
        self._libres = 0
        self._listo_t = None
        self._extra = 0.0
        self._n_ant = None
        self._t_n = 0.0
        self._t_sin_datos = None

    # ------------------------------------------------------------------ ordenes del operador
    def iniciar(self, t):
        """El operador arma: arranca desde cero (tambien despues de una detencion)."""
        self._cambiar(AVANZAR, t, 'avanzando')
        self._n_ant = None
        self._t_sin_datos = None

    def detener(self, t, motivo='detenido por el operador'):
        self._detenido(t, motivo)

    # ------------------------------------------------------------------ interno
    def _cambiar(self, estado, t, motivo):
        self.estado = estado
        self.motivo = motivo
        self.t_estado = t

    def _detenido(self, t, motivo):
        if self.estado != DETENIDO:
            self.detenciones += 1
            self.historial.append((t, motivo))
        self._cambiar(DETENIDO, t, motivo)

    def _elegir_giro(self, izq, der):
        """+1 izquierda, -1 derecha, None si no hay lugar para girar. Un lado sin eco cuenta como espacio libre. Girar en el lugar
        barre las dos esquinas diagonales hacia afuera, asi que hace falta lugar a los DOS lados, y se gira hacia el lado mas libre."""
        p = self.p
        li = 9.0 if izq is None else izq
        ld = 9.0 if der is None else der
        if li < p.d_barrido or ld < p.d_barrido:
            return None
        if abs(li - ld) < 0.05:
            return self.rng.choice((1, -1))
        return 1 if li > ld else -1

    def _empezar_giro(self, t, izq, der, motivo):
        lado = self._elegir_giro(izq, der)
        if lado is None:
            self._detenido(t, 'encajonado: un lado a menos de %.0f cm, no hay lugar para girar en el lugar' % (100 * self.p.d_barrido))
            return
        self._dir = lado
        self._libres = 0
        self._listo_t = None
        self._extra = self.rng.uniform(*self.p.giro_extra_s)
        self.giros += 1
        self._cambiar(GIRAR, t, '%s: giro a la %s' % (motivo, 'izquierda' if lado > 0 else 'derecha'))

    # ------------------------------------------------------------------ un paso (cada ~100 ms)
    def paso(self, t, frente, izq, der, guardia, n_frente, datos_frescos=True):
        """Devuelve (v, w) para /cmd_vel. frente/izq/der en m o None; guardia = /wheel_state[29]; n_frente = /wheel_state[32]."""
        p = self.p
        if self.estado in (ESPERA, DETENIDO):
            return 0.0, 0.0

        # telemetria: si se corta, quieto; si vuelve pronto sigue, si tarda hay que armar de nuevo
        if not datos_frescos:
            if self._t_sin_datos is None:
                self._t_sin_datos = t
            if t - self._t_sin_datos > p.sin_datos_rearmar_s:
                self._detenido(t, 'sin telemetria de la ESP32 por mas de %.0f s' % p.sin_datos_rearmar_s)
            return 0.0, 0.0
        if self._t_sin_datos is not None and t - self._t_sin_datos > p.sin_datos_s:
            self._n_ant = None           # volvio: se vuelve a mirar si el sonar avanza
        self._t_sin_datos = None

        # el contador de mediciones del frontal tiene que avanzar: si no, el sonar esta muerto
        nueva = False
        if self._n_ant is None or n_frente != self._n_ant:
            nueva = self._n_ant is not None
            self._n_ant = n_frente
            self._t_n = t
        elif t - self._t_n > p.sonar_mudo_s:
            self._detenido(t, 'el sonar frontal no entrega mediciones')
            return 0.0, 0.0

        if self.estado == AVANZAR:
            cerca = frente is not None and frente <= p.d_evitar
            if cerca or guardia in (G_FRENADA, G_FALLA):
                self._empezar_giro(t, izq, der, 'obstaculo adelante' if cerca else 'la guardia no deja avanzar')
                if self.estado == GIRAR:
                    return 0.0, self._dir * p.w_giro
                return 0.0, 0.0
            w = 0.0
            if izq is not None and izq < p.d_lateral and (der is None or der > izq):
                w = -p.w_lateral         # algo cerca a la izquierda: empuja a la derecha
            elif der is not None and der < p.d_lateral and (izq is None or izq > der):
                w = p.w_lateral
            return p.v_crucero, w

        # GIRAR
        dur = t - self.t_estado
        # pasada media vuelta sin ver libre, se conforma con un poco mas que d_evitar (un cuarto chico o con muchos muebles)
        umbral = p.d_libre if dur < p.giro_max_s / 2 else p.d_evitar + 0.10
        if nueva:
            self._libres = self._libres + 1 if (frente is not None and frente >= umbral) else 0
        if self._listo_t is None and self._libres >= p.lecturas_libres and dur >= p.giro_min_s:
            self._listo_t = t
        if self._listo_t is not None and t - self._listo_t >= self._extra:
            self._cambiar(AVANZAR, t, 'camino libre')
            return p.v_crucero, 0.0
        if dur > p.giro_max_s:
            self._detenido(t, 'giro %.0f s sin encontrar salida' % p.giro_max_s)
            return 0.0, 0.0
        return 0.0, self._dir * p.w_giro
