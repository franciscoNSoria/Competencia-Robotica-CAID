"""
Los cuatro servos del brazo como los maneja la ESP32 (firmware/brazo_sliders), sin ROS.

Lo usa brazo_sim para que el brazo de Gazebo se comporte igual que el real: misma orden
(/brazo/cmd), mismo estado (/brazo/estado), misma rampa de velocidad, mismos límites y topes, y la
misma regla de la base: solo gira con el hombro y el codo en el reposo seguro (para no chocar con
la cámara). Los valores de arranque son los de firmware/brazo_sliders/main.c.
"""
SERVOS = ('base', 'hombro', 'codo', 'pinza')
TOPES = (180.0, 180.0, 200.0, 180.0)
LIMITES_ARRANQUE = ((0.0, 180.0), (10.0, 125.0), (80.0, 200.0), (100.0, 180.0))
INICIO = (95.0, 60.0, 180.0, 150.0)
POSE_SEGURA = (60.0, 180.0)            # hombro, codo
TOL_REPOSO = 1.0                       # grados
ESPERA_REPOSO = 1.0                    # s con el brazo en el reposo seguro antes de girar la base
BASE_UMBRAL = 0.5                      # grados: una base que pide moverse más activa el reposo
LIBRE, ESPERANDO, GIRANDO = 0, 1, 2
B, H, C, P = 0, 1, 2, 3


def _acotar(x, lo, hi):
    return max(lo, min(hi, x))


class ServosBrazo:

    def __init__(self):
        self.actual = list(INICIO)
        self.objetivo = list(INICIO)
        self.limites = [list(lim) for lim in LIMITES_ARRANQUE]
        self.activos = [True] * 4          # en la simulación el brazo arranca energizado
        self.velocidad = 90.0              # grados/s; 0 = salto directo
        self.pose_segura = list(POSE_SEGURA)
        self.fase = LIBRE
        self.t_reposo = 0.0
        self.contador = 0
        self.t_sin_orden = 0.0

    def orden(self, datos):
        """Una orden de /brazo/cmd (lista de floats). Devuelve False si se la descarta."""
        if len(datos) < 6 or any(d != d for d in datos):      # d != d: NaN
            return False
        if len(datos) >= 14:
            for i in range(4):
                lo = _acotar(datos[6 + 2 * i], 0.0, TOPES[i])
                hi = _acotar(datos[7 + 2 * i], 0.0, TOPES[i])
                if hi - lo >= 1.0:
                    self.limites[i] = [lo, hi]
        if len(datos) >= 16:
            self.pose_segura = [datos[14], datos[15]]
        mascara = int(round(datos[4]))
        self.activos = [bool((mascara >> i) & 1) for i in range(4)]
        self.velocidad = max(0.0, datos[5])
        for i in range(4):
            self.objetivo[i] = _acotar(datos[i], *self.limites[i])
        self.contador += 1
        self.t_sin_orden = 0.0
        return True

    def segura(self, i):
        return _acotar(self.pose_segura[i - 1], *self.limites[i])

    def paso(self, dt):
        """Avanza dt segundos: rampa de cada servo y fases de la base."""
        self.t_sin_orden += dt
        meta = list(self.objetivo)
        quiere_girar = self.activos[B] and abs(self.objetivo[B] - self.actual[B]) > BASE_UMBRAL
        if quiere_girar and self.fase == LIBRE:
            self.fase, self.t_reposo = ESPERANDO, 0.0
        if self.fase == ESPERANDO:
            meta[H], meta[C], meta[B] = self.segura(H), self.segura(C), self.actual[B]
            llegado = all(abs(self.actual[i] - meta[i]) <= TOL_REPOSO for i in (H, C))
            self.t_reposo = self.t_reposo + dt if llegado else 0.0
            if self.t_reposo >= ESPERA_REPOSO:
                self.fase = GIRANDO
        elif self.fase == GIRANDO:
            meta[H], meta[C] = self.segura(H), self.segura(C)
            if abs(self.objetivo[B] - self.actual[B]) <= BASE_UMBRAL:
                self.fase = LIBRE
        for i in range(4):
            if not self.activos[i]:
                continue
            if self.velocidad <= 0.0:
                self.actual[i] = meta[i]
            else:
                d = meta[i] - self.actual[i]
                maximo = self.velocidad * dt
                self.actual[i] += _acotar(d, -maximo, maximo)

    def estado(self):
        """Los 23 números de /brazo/estado, en el orden de la ESP32."""
        mascara = sum(1 << i for i, a in enumerate(self.activos) if a)
        datos = list(self.actual) + [float(mascara), self.velocidad, float(self.contador),
                                     self.t_sin_orden * 1000.0]
        for lo, hi in self.limites:
            datos += [lo, hi]
        datos += list(TOPES) + [float(self.fase)] + [self.segura(H), self.segura(C)]
        return [float(d) for d in datos]
