"""
Controladores de bajo nivel (núcleo puro, sin ROS): recta por un eje y giro en el lugar.

Recta: el robot va por el eje de un pasillo (rumbo h), hacia adelante o marcha atrás, hasta una
coordenada `a_fin` a lo largo del eje. La velocidad baja con la raíz de la distancia que falta
(desaceleración constante) y el rumbo de referencia lleva el error lateral a cero:

    sen(phi_ref) = -k_lat * e_lat * signo(v)      (e_lat = lateral - lateral_objetivo)
    w = k_phi * (phi_ref - phi)

El lateral objetivo puede cambiar a lo largo del recorrido (para pasar al costado de un obstáculo
o para llegar corrido a un giro).
"""
import math

from caid_navigation.geometria import PASOS, ang_dif, limitar, yaw_de


class Recta:

    def __init__(self, h, signo, a_fin, lateral, v_max=0.20, v_min=0.035, decel=0.35,
                 k_lat=6.0, k_phi=3.0, w_max=1.2):
        self.h = h
        self.signo = signo          # +1 adelante, -1 marcha atrás
        self.a_fin = a_fin
        self.lateral = lateral      # función a -> lateral objetivo (m), o número
        self.v_max = v_max
        self.v_min = v_min
        self.decel = decel
        self.k_lat = k_lat
        self.k_phi = k_phi
        self.w_max = w_max

    def ejes(self, pose):
        u, n = PASOS[self.h], PASOS[(self.h + 1) % 4]
        a = pose[0] * u[0] + pose[1] * u[1]
        lat = pose[0] * n[0] + pose[1] * n[1]
        return a, lat

    def falta(self, pose):
        a, _ = self.ejes(pose)
        return (self.a_fin - a) * self.signo

    def comando(self, pose, v_actual=None):
        """(v, w, terminado)."""
        a, lat = self.ejes(pose)
        falta = (self.a_fin - a) * self.signo
        if falta <= 0.004:
            return 0.0, 0.0, True
        v = min(self.v_max, math.sqrt(2 * self.decel * falta))
        v = max(v, self.v_min) * self.signo
        objetivo = self.lateral(a) if callable(self.lateral) else self.lateral
        e = lat - objetivo
        phi = ang_dif(pose[2], yaw_de(self.h))
        phi_ref = math.asin(limitar(-self.k_lat * e * self.signo, 0.35))
        w = limitar(self.k_phi * ang_dif(phi_ref, phi), self.w_max)
        return v, w, False


class Giro:

    def __init__(self, yaw_obj, w_max=1.2, w_min=0.22, k=2.5, tolerancia=0.025):
        self.yaw = yaw_obj
        self.w_max = w_max
        self.w_min = w_min
        self.k = k
        self.tol = tolerancia
        self.quieto = 0

    def comando(self, pose):
        err = ang_dif(self.yaw, pose[2])
        if abs(err) < self.tol:
            self.quieto += 1
            return 0.0, 0.0, self.quieto >= 3
        self.quieto = 0
        w = limitar(self.k * err, self.w_max)
        w = math.copysign(max(abs(w), self.w_min), w)
        return 0.0, w, False
