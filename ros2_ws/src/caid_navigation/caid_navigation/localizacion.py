"""
Pose del robot en el marco `map` (núcleo puro, sin ROS).

No hay IMU: la pose sale de la odometría de las ruedas y se corrige con las paredes conocidas.

- Odometría: se integran los incrementos (no la pose absoluta), así una corrección del rumbo
también
  corrige hacia dónde se acumula el avance que sigue.
- Lateral: cada ultrasonido de costado que ve una pared conocida da la distancia del robot a esa
  pared. Se descartan las lecturas que difieren más de 3 cm de lo esperado (un soporte o un bloque
  chico pegado a la pared acortan la lectura 4,5 u 8 cm).
- Longitudinal: el ultrasonido de frente contra una pared conocida, hasta 0,9 m, y los bordes de
  las aberturas de los costados: cuando un ultrasonido lateral pasa de "pared" a "abierto" (o al
  revés) el sensor está en el borde entre dos celdas, que se conoce.
- Rumbo: la pendiente de la posición lateral medida con las paredes mientras el robot avanza es la
  tangente del rumbo real respecto del pasillo; la diferencia con el rumbo estimado es el error.
- Factor de giro: un 4WD patina al girar y la odometría se equivoca siempre en la misma proporción.
  Cada corrección de rumbo después de un giro grande ajusta ese factor (entre 0,85 y 1,15).
"""
import math

from caid_navigation.geometria import (
    a_mundo, ang_dif, CELDA as CELDA_M, celda_de, centro, MEDIA, PASOS, rumbo_de, SONARES, yaw_de)


class Estimador:

    def __init__(self, ganancia_lateral=0.25, ganancia_frente=0.25, ganancia_rumbo=0.7,
                 tramo_rumbo=0.16):
        self.pose = [0.0, 0.0, 0.0]
        self.previa = None
        self.g_lat = ganancia_lateral
        self.g_frente = ganancia_frente
        self.g_rumbo = ganancia_rumbo
        self.tramo_rumbo = tramo_rumbo
        self.muestras = []
        self.eje_muestras = None
        self.correcciones = {'lateral': 0, 'frente': 0, 'rumbo': 0}
        self.factor_giro = 1.0
        self.giro_acumulado = 0.0
        self.lateral_previo = {}
        self.offset_borde = 0.20 * math.tan(math.radians(7.5))
        self.correcciones['borde'] = 0

    def odometria(self, ox, oy, oth):
        if self.previa is None:
            self.previa = (ox, oy, oth)
            return
        px, py, pth = self.previa
        c, s = math.cos(pth), math.sin(pth)
        dx, dy = c * (ox - px) + s * (oy - py), -s * (ox - px) + c * (oy - py)
        dth = ang_dif(oth, pth) * self.factor_giro
        self.giro_acumulado += dth
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        self.pose = [x + c * dx - s * dy, y + s * dx + c * dy, ang_dif(th + dth, 0.0)]
        self.previa = (ox, oy, oth)

    def corregir(self, con, rangos, v):
        """Aplica las correcciones con las paredes. v: velocidad lineal actual (m/s)."""
        x, y, th = self.pose
        h = rumbo_de(th)
        err = ang_dif(th, yaw_de(h))
        if abs(err) > 0.12:
            self.muestras = []
            return
        self.v = v
        u, n = PASOS[h], PASOS[(h + 1) % 4]
        laterales = []
        medidos = []
        for lado, signo in (('izquierda', 1), ('derecha', -1)):
            r = rangos.get(lado, math.inf)
            if not math.isfinite(r) or r > 0.5:
                continue
            sx, sy = a_mundo(self.pose, *SONARES[lado][:2])
            celda = celda_de(sx, sy)
            cx, cy = centro(celda)
            if abs((sx - cx) * u[0] + (sy - cy) * u[1]) > 0.11:
                continue        # cerca del borde de la celda: la pared puede terminar ahí
            d_lado = (h + 1) % 4 if signo > 0 else (h + 3) % 4
            # La pared puede estar en el borde de esta celda o, si ese lado está abierto, en el de
            # la vecina (zonas de 2 celdas de ancho, como la de partida).
            k = 0
            while con.abierto(celda, d_lado) is True and k < 1:
                celda = (celda[0] + PASOS[d_lado][0], celda[1] + PASOS[d_lado][1])
                k += 1
            if con.abierto(celda, d_lado) is not False or (celda, d_lado) in con.bloqueos:
                continue
            if r > 0.25 + k * CELDA_M:
                continue
            cx, cy = centro(celda)
            lat_pared = cx * n[0] + cy * n[1] + signo * MEDIA
            lat_sensor = sx * n[0] + sy * n[1]
            innov = (lat_pared - signo * r) - lat_sensor
            if abs(innov) < 0.03:
                laterales.append(innov)
                # Lateral del centro del robot medido con la pared (no depende de la odometría).
                medidos.append(lat_pared - signo * r
                               - (0.096 * math.sin(err) + signo * 0.0815 * math.cos(err)))
        if laterales:
            d = self.g_lat * sum(laterales) / len(laterales)
            self.pose[0] += d * n[0]
            self.pose[1] += d * n[1]
            self.correcciones['lateral'] += 1
        self._rumbo(h, u, n, medidos, v)
        self._bordes(con, rangos, h, u, v)
        self._frente(con, rangos.get('frente', math.inf), h, u)

    def _rumbo(self, h, u, n, medidos, v):
        x, y, th = self.pose
        a = x * u[0] + y * u[1]
        if abs(v) < 0.04 or self.eje_muestras != h or \
                (self.muestras and abs(a - self.muestras[-1][0]) > 0.15):
            # Quieto, cambió de eje o pasó un tramo largo sin pared: se empieza de nuevo.
            self.muestras = []
            self.eje_muestras = h
        if not medidos or abs(v) < 0.04:
            return
        self.muestras.append((a, sum(medidos) / len(medidos), ang_dif(th, yaw_de(h))))
        m = self.muestras
        if abs(m[-1][0] - m[0][0]) < self.tramo_rumbo or len(m) < 8:
            return
        k = len(m)
        am = sum(s[0] for s in m) / k
        lm = sum(s[1] for s in m) / k
        var = sum((s[0] - am) ** 2 for s in m)
        pend = sum((s[0] - am) * (s[1] - lm) for s in m) / var
        resid = math.sqrt(sum((s[1] - lm - pend * (s[0] - am)) ** 2 for s in m) / k)
        if abs(pend) < 0.2 and resid < 0.012:
            delta = sum(s[2] for s in m) / k - math.atan(pend)
            self.pose[2] = ang_dif(self.pose[2] - self.g_rumbo * delta, 0.0)
            self.correcciones['rumbo'] += 1
            giro = self.giro_acumulado
            if abs(giro) > 0.8 and abs(delta) < 0.15:
                razon = (giro - delta) / giro
                self.factor_giro = min(1.15, max(0.85, self.factor_giro * (0.75 + 0.25 * razon)))
            self.giro_acumulado = 0.0
        self.muestras = []

    def _bordes(self, con, rangos, h, u, v):
        """Corrección a lo largo del pasillo con el borde de una abertura lateral."""
        n = PASOS[(h + 1) % 4]
        lat = self.pose[0] * n[0] + self.pose[1] * n[1]
        for lado, d_rel in (('izquierda', 1), ('derecha', 3)):
            r = rangos.get(lado, math.inf)
            pared = math.isfinite(r) and r < 0.20
            previo = self.lateral_previo.get(lado)
            self.lateral_previo[lado] = (pared, h)
            if previo is None or previo[1] != h or previo[0] == pared or abs(v) < 0.03:
                continue
            sx, sy = a_mundo(self.pose, *SONARES[lado][:2])
            a_s = sx * u[0] + sy * u[1]
            borde = (round(a_s / CELDA_M - 0.5) + 0.5) * CELDA_M
            if abs(a_s - borde) > 0.07:
                continue
            signo = 1 if v > 0 else -1

            def celda_en(a):
                return celda_de(a * u[0] + lat * n[0], a * u[1] + lat * n[1])
            d = (h + d_rel) % 4
            e_atras = con.abierto(celda_en(borde - signo * 0.1), d)
            e_adel = con.abierto(celda_en(borde + signo * 0.1), d)
            if e_atras is None or e_adel is None or e_atras == e_adel:
                continue
            if e_atras == previo[0] or e_adel == pared:
                continue        # el cambio no coincide con el mapa
            # El cambio se ve con el rayo extremo del cono: la lectura pasa el umbral (0,20 m)
            # cuando
            # ese rayo pega en la cara del bloque que da a la abertura a más de 0,20 m, o sea
            # 0,20 * tan(7,5°) = 2,6 cm después (pared -> abierto) o antes (abierto -> pared) del
            # borde. Con el sensor real hay que medirlo (parámetro).
            esperado = borde + signo * (self.offset_borde if previo[0] else -self.offset_borde)
            innov = esperado - a_s
            self.pose[0] += 0.6 * innov * u[0]
            self.pose[1] += 0.6 * innov * u[1]
            self.correcciones['borde'] += 1

    def _frente(self, con, f, h, u):
        if not math.isfinite(f):
            return
        # Solo hasta donde el cono no puede tocar las paredes de los costados, aun con 3° de error
        # de rumbo (si no, una pared lateral se confunde con la de enfrente).
        n = PASOS[(h + 1) % 4]
        cx, cy = centro(celda_de(self.pose[0], self.pose[1]))
        lat = (self.pose[0] - cx) * n[0] + (self.pose[1] - cy) * n[1]
        err = abs(ang_dif(self.pose[2], yaw_de(h)))
        alcance = (MEDIA - abs(lat) - 0.015) / math.tan(math.radians(7.5) + err + 0.05)
        if f > min(1.05, alcance):
            return
        sx, sy = a_mundo(self.pose, *SONARES['frente'][:2])
        a_s = sx * u[0] + sy * u[1]
        celda = celda_de(sx, sy)
        for _ in range(4):
            estado = con.abierto(celda, h)
            if estado is None:
                return
            if estado is False:
                if (celda, h) in con.bloqueos:
                    return
                cx, cy = centro(celda)
                borde = cx * u[0] + cy * u[1] + MEDIA
                vecina = (celda[0] + u[0], celda[1] + u[1])
                canon = min((celda, h), (vecina, (h + 2) % 4))
                if con.votos.get(canon, 0) > -4 or con.cajas.get(vecina):
                    return      # pared poco confirmada o con algo adelante: no se usa
                innov = (borde - f) - a_s
                if abs(innov) < 0.025:
                    d = self.g_frente * innov
                    self.pose[0] += d * u[0]
                    self.pose[1] += d * u[1]
                    self.correcciones['frente'] += 1
                return
            celda = (celda[0] + u[0], celda[1] + u[1])
