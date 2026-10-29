"""Robot del simulador 2D: cinemática diferencial, odometría con error, ultrasonidos y choques."""
import math
import random

from caid_navigation.geometria import (
    a_mundo, ang_dif, CONO_SONAR, esquinas_huella, poligono_toca_caja, rayo_caja, SONARES)


class RobotSim:
    """
    Errores (valores por defecto, P): el robot real gira un 1,5 % más de lo que dice la odometría
    (error que queda después de calibrar el factor de giro) y avanza un 1 % más; la odometría
    tiene ruido blanco chico. Sirven para que la corrección con las paredes tenga trabajo; con
    ruido=False todo es perfecto.
    """

    def __init__(self, mundo, ruido=True, semilla=1, err_giro=0.015, err_avance=0.01,
                 sigma_sonar=0.005):
        self.mundo = mundo
        self.rng = random.Random(semilla)
        self.ruido = ruido
        self.err_giro = err_giro if ruido else 0.0
        self.err_avance = err_avance if ruido else 0.0
        self.sigma_sonar = sigma_sonar if ruido else 0.0
        self.pose = list(mundo.inicio)
        self.v = self.w = 0.0
        self.odom = [0.0, 0.0, 0.0]
        self.pan = 0.0
        self.pan_cmd = 0.0
        self.choques = []
        self.t = 0.0
        self.recorrido = 0.0
        self.otras = [b for b in mundo.cajas if b.tipo != 'pared']

    # ---------- movimiento ----------

    def paso(self, dt, v_cmd, w_cmd, pan_cmd=None):
        v_cmd = max(-0.5, min(0.5, v_cmd))
        w_cmd = max(-3.0, min(3.0, w_cmd))
        dv = max(-1.0 * dt, min(1.0 * dt, v_cmd - self.v))
        dw = max(-5.0 * dt, min(5.0 * dt, w_cmd - self.w))
        self.v += dv
        self.w += dw
        v_real = self.v * (1 + self.err_avance)
        w_real = self.w * (1 + self.err_giro)
        x, y, th = self.pose
        nueva = [x + v_real * dt * math.cos(th + w_real * dt / 2),
                 y + v_real * dt * math.sin(th + w_real * dt / 2),
                 ang_dif(th + w_real * dt, 0.0)]
        choque = self.choca(nueva)
        if choque:
            if not self.choques or self.t - self.choques[-1][0] > 1.0:
                self.choques.append((self.t, tuple(nueva), choque))
            self.v = self.w = 0.0
        else:
            self.recorrido += abs(v_real) * dt
            self.pose = nueva
            ox, oy, oth = self.odom
            n_v = self.rng.gauss(0, 0.002) if self.ruido else 0.0
            n_w = self.rng.gauss(0, 0.004) if self.ruido else 0.0
            vo, wo = self.v + n_v * (self.v != 0), self.w + n_w * (self.w != 0)
            self.odom = [ox + vo * dt * math.cos(oth + wo * dt / 2),
                         oy + vo * dt * math.sin(oth + wo * dt / 2),
                         ang_dif(oth + wo * dt, 0.0)]
        if pan_cmd is not None:
            self.pan_cmd = max(-math.pi / 2, min(math.pi / 2, pan_cmd))
        self.pan += max(-3.0 * dt, min(3.0 * dt, self.pan_cmd - self.pan))
        self.t += dt

    def choca(self, pose):
        poli = esquinas_huella(pose)
        x, y = pose[0], pose[1]
        for b in self.mundo.cajas_cerca(x, y, 0.2):
            if b.z0 < 0.10 and poligono_toca_caja(poli, b.caja):
                return b.tipo
        return None

    # ---------- sensores ----------

    def rango(self, lado):
        sx, sy, syaw, sz = SONARES[lado]
        ox, oy = a_mundo(self.pose, sx, sy)
        mejor = math.inf
        for k in range(5):
            ang = self.pose[2] + syaw + CONO_SONAR * (k / 4 - 0.5)
            d = self.rayo(ox, oy, math.cos(ang), math.sin(ang), sz)
            mejor = min(mejor, d)
        if mejor == math.inf or mejor > 4.0:
            return math.inf
        mejor += self.rng.gauss(0, self.sigma_sonar) if self.sigma_sonar else 0.0
        return max(0.02, mejor)

    def rayo(self, ox, oy, dx, dy, z, alcance=4.0):
        """Distancia al primer obstáculo a la altura z (o inf). Paredes con DDA por la grilla."""
        mejor = math.inf
        for b in self.otras:
            if b.activa and b.z0 <= z + 0.01 and b.z1 >= z - 0.01:
                d = rayo_caja(ox, oy, dx, dy, b.caja)
                if d is not None and d < mejor:
                    mejor = d
        C = self.mundo.celda
        lab = self.mundo.lab
        cx, cy = math.floor(ox / C), math.floor(oy / C)
        paso_x = 1 if dx > 0 else -1
        paso_y = 1 if dy > 0 else -1
        t_max_x = ((cx + (dx > 0)) * C - ox) / dx if dx else math.inf
        t_max_y = ((cy + (dy > 0)) * C - oy) / dy if dy else math.inf
        t_dx = abs(C / dx) if dx else math.inf
        t_dy = abs(C / dy) if dy else math.inf
        t = 0.0
        while t < min(alcance, mejor):
            r, c = lab.nfilas - 1 - cy, cx
            if (r, c) in self.mundo.bloques:
                return min(mejor, t)
            if t_max_x < t_max_y:
                t = t_max_x
                t_max_x += t_dx
                cx += paso_x
            else:
                t = t_max_y
                t_max_y += t_dy
                cy += paso_y
        return mejor
