"""Simulador 2D de torneo_logica.py en un laberinto de bloques de 40 cm, con el modelo del robot de tools/autonomo/simular.py.

Del firmware (firmware/banco/robot_4ruedas/main.c) modela: /cmd_vel con zona muerta (< 12,5 flanc/s = 0) y mínimo de 25
flanc/s por rueda, la guardia real (el mismo C de guardia.h por ctypes), las órdenes 3 (avanzar) y 7 (girar en el lugar)
con rampas y el asentado de 1,5 s, y el corte a 1 s sin órdenes. Del robot (M 3/11 o P, ver tools/autonomo/simular.py):
arranque tardío de cada lado 0,25-0,9 s desde quieto, -1,5 % de avance, desvío de rumbo, giros con ganancia 1 +- 7 % y 1 de
cada 10 que patina (40-75 %), sonar con haz gaussiano, pérdidas y eco débil de costado, y Wi-Fi con demoras y huecos.

La cámara es un oráculo: entrega la flecha (celda, dirección, largo) cuando se ve de frente entre 0,40 y 1,1 m, y la
distancia al cubo cuando está enfrente. Lo que se prueba es la conducción, no la visión.

  python3 tools/torneo/simular_torneo.py                 # todos los laberintos, 20 semillas
  python3 tools/torneo/simular_torneo.py --pesimista --semillas 40
  python3 tools/torneo/simular_torneo.py --laberinto casa --detalle --semillas 1
"""
import argparse
import heapq
import math
import os
import random
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', 'autonomo'))
import simular as sa  # noqa: E402
import torneo_logica as tl  # noqa: E402

CELDA = tl.CELDA
LARGO, ANCHO = 0.278, 0.163
SONARES = {'F': (tl.F_X, 0.0, 0.0), 'I': (tl.L_X, tl.L_Y, math.pi / 2), 'D': (tl.L_X, -tl.L_Y, -math.pi / 2)}
SECUENCIA = ['F', 'D', 'F', 'I']
CAM_X, CAM_HFOV = 0.101, math.radians(60)
DT = 0.010
VMIN, VMAX = 25.0, 75.0
ACEL = 100.0

# '#' bloque, '.' libre, 'S' partida (mira a +x), 'C' cubo verde (sobre un cilindro de 4 cm en el centro de la celda),
# 'c' cubo verde pegado a la pared del fondo. Fila 0 arriba: y = -fila.
LABERINTOS = {
    'casa': """
########
#S..####
###.####
###...C#
########""",
    'torneo': """
#############
#S....#.....#
#####.#.###.#
#.....#...#.#
#.#######.#.#
#.....#...#.#
#####.#.###.#
#..........c#
#############""",
    'ramas': """
###########
#S...#....#
###.##.##.#
#...#..#..#
#.###.##.##
#.....#...#
#.###.#.#C#
#...#...#.#
###########""",
}


def parsear(texto):
    filas = [f for f in texto.strip('\n').split('\n')]
    libres, bloques, cubo, partida = set(), set(), None, None
    for r, fila in enumerate(filas):
        for c, ch in enumerate(fila):
            celda = (c, -r)
            if ch == '#':
                bloques.add(celda)
            else:
                libres.add(celda)
                if ch == 'S':
                    partida = celda
                elif ch in 'Cc':
                    cubo = (celda, ch)
    ox, oy = partida
    mover = lambda cel: (cel[0] - ox, cel[1] - oy)
    libres = {mover(c) for c in libres}
    bloques = {mover(c) for c in bloques}
    cubo = (mover(cubo[0]), cubo[1]) if cubo else None
    return libres, bloques, cubo


def camino_bfs(libres, a, b):
    previo = {a: None}
    cola = [a]
    while cola:
        c = cola.pop(0)
        if c == b:
            break
        for d, (dx, dy) in enumerate(tl.PASOS):
            n = (c[0] + dx, c[1] + dy)
            if n in libres and n not in previo:
                previo[n] = (c, d)
                cola.append(n)
    pasos = []
    c = b
    while previo[c] is not None:
        c, d = previo[c]
        pasos.append((c, d))
    return pasos[::-1]


class Laberinto:
    def __init__(self, texto, señuelos=True):
        self.libres, self.bloques, cubo = parsear(texto)
        self.cubo_celda = cubo[0] if cubo else None
        self.segs = []
        for c in self.libres:
            for d, (dx, dy) in enumerate(tl.PASOS):
                if (c[0] + dx, c[1] + dy) in self.bloques:
                    self.segs.append((self.cara(c, d), False))
        self.cubo = None
        if cubo:
            cx, cy = cubo[0][0] * CELDA, cubo[0][1] * CELDA
            if cubo[1] == 'c':                       # pegado a la pared del fondo (la que cierra la celda)
                for d, (dx, dy) in enumerate(tl.PASOS):
                    if (cubo[0][0] + dx, cubo[0][1] + dy) in self.bloques and \
                            (cubo[0][0] - dx, cubo[0][1] - dy) in self.libres:
                        cx += dx * (CELDA / 2 - 0.03)
                        cy += dy * (CELDA / 2 - 0.03)
                        break
            self.cubo = (cx, cy)
            poli = sa.rect(cx, cy, 0.04, 0.04)
            self.segs += [((a, b), True) for a, b in sa.aristas(poli)]
        self.flechas = self.poner_flechas(señuelos)

    def cara(self, c, d):
        cx, cy = c[0] * CELDA, c[1] * CELDA
        dx, dy = tl.PASOS[d]
        m = CELDA / 2
        px, py = cx + dx * m, cy + dy * m
        return ((px - dy * m, py + dx * m), (px + dy * m, py - dx * m))

    def poner_flechas(self, señuelos):
        """Flechas verdes en los cruces del camino al cubo: en la pared de enfrente al llegar, apuntando hacia donde sigue el
        camino (o en una pared lateral apuntando adelante si el camino sigue derecho por un cruce). Y alguna roja de señuelo."""
        if self.cubo_celda is None:
            return []
        pasos = camino_bfs(self.libres, (0, 0), self.cubo_celda)
        out = []
        for i in range(1, len(pasos)):
            c, d_sig = pasos[i]
            d_ant = pasos[i - 1][1]
            salidas = [d for d, (dx, dy) in enumerate(tl.PASOS) if (c[0] + dx, c[1] + dy) in self.libres]
            if len(salidas) < 3:
                continue
            frente = (c[0] + tl.PASOS[d_ant][0], c[1] + tl.PASOS[d_ant][1])
            if d_sig != d_ant and frente in self.bloques:
                out.append(dict(celda=c, pared=d_ant, dir=d_sig, largo=1, color='verde'))
            else:
                for lado in ((d_ant + 1) % 4, (d_ant + 3) % 4):
                    prev = pasos[i - 1][0]
                    if (prev[0] + tl.PASOS[lado][0], prev[1] + tl.PASOS[lado][1]) in self.bloques:
                        out.append(dict(celda=prev, pared=lado, dir=d_sig, largo=1, color='verde'))
                        break
            if señuelos:
                otra = [d for d in salidas if d not in (d_sig, (d_ant + 2) % 4)]
                if otra and frente in self.bloques and d_sig != d_ant:
                    out.append(dict(celda=c, pared=d_ant, dir=otra[0], largo=1, color='rojo'))
        return out

    def choca(self, pose):
        poli = sa.rect(pose[0], pose[1], LARGO, ANCHO, pose[2])
        c0 = (round(pose[0] / CELDA), round(pose[1] / CELDA))
        for i in range(-1, 2):
            for j in range(-1, 2):
                b = (c0[0] + i, c0[1] + j)
                if b in self.bloques and sa.se_tocan(poli, sa.rect(b[0] * CELDA, b[1] * CELDA, CELDA, CELDA)):
                    return True
        return False

    def toca_cubo(self, pose):
        return self.cubo is not None and sa.se_tocan(sa.rect(pose[0], pose[1], LARGO, ANCHO, pose[2]),
                                                    sa.rect(self.cubo[0], self.cubo[1], 0.04, 0.04))

    def visible(self, o, p):
        """Línea de vista de o a p sin cruzar caras de bloque (salvo justo en p)."""
        dx, dy = p[0] - o[0], p[1] - o[1]
        dist = math.hypot(dx, dy)
        for (a, b), delgado in self.segs:
            if delgado:
                continue
            r = sa.rayo_segmento(o[0], o[1], dx / dist, dy / dist, a, b)
            if r is not None and r[0] < dist - 0.01:
                return False
        return True


class Sim:
    def __init__(self, g, lab, semilla, pesimista=False, t_max=420.0, detalle=False):
        self.g, self.lab = g, lab
        self.rng = random.Random(semilla)
        self.detalle = detalle
        self.pose = [self.rng.gauss(0, 0.01), self.rng.gauss(0, 0.01), self.rng.gauss(0, math.radians(3))]
        self.t_max = t_max
        self.pesimista = pesimista
        self.haz = 14.0 if pesimista else 18.0
        self.umbral = 1.0 / 16.0 * (2.0 if pesimista else 1.0)
        self.p_perdida = 0.08 if pesimista else 0.03
        self.p_hueco = 0.06 if pesimista else 0.03
        self.k_avance = self.rng.gauss(0.985, 0.01)
        self.sesgo = self.rng.gauss(0.0, 0.03)
        self.diam = 0.0645
        self.trocha_cfg = 0.291
        self.trocha_real = self.trocha_cfg * self.rng.gauss(1.0, 0.03)
        self.fpm = 40.0 / (math.pi * self.diam)
        g.g_init(0, 0.20, 0.60, 5.0)
        self.cmd, self.t_cmd = (0.0, 0.0), -10.0
        self.lado, self.espera, self.obj_ant = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
        self.gan, self.w_ant = 1.0, 0.0
        self.enc = [0.0, 0.0]
        self.mov, self.mov_seq = None, 0
        self.son = {k: dict(m=-1.0, ms=0, valido=0, n=0) for k in 'FDI'}
        self.busy, self.t_disp, self.turno = None, -1.0, 0
        self.estado_g = 0
        self.cola, self.hueco, self.n_ev = [], -1.0, 0
        self.tele, self.t_tele, self.s_pc = None, -10.0, (0.0, 0.0)
        self.log = []
        self.piloto = tl.Piloto(tl.Parametros(), trocha=self.trocha_cfg, diametro=self.diam,
                                imprimir=(lambda m: self.log.append('%6.1f %s' % (self.t, m))))
        self.t = 0.0
        self.choque = None
        self.toco_cubo = False
        self.min_lat = 9.0

    # ------------------------------------------------------------------ red
    def enviar(self, tipo, datos):
        if self.t < self.hueco:
            return
        heapq.heappush(self.cola, (self.t + self.rng.uniform(0.05, 0.25), self.n_ev, tipo, datos))
        self.n_ev += 1

    # ------------------------------------------------------------------ sonar (el modelo de tools/autonomo/simular.py)
    def medir(self, k):
        sx, sy, sa_ = SONARES[k]
        x, y, th = self.pose
        ox, oy = x + sx * math.cos(th) - sy * math.sin(th), y + sx * math.sin(th) + sy * math.cos(th)
        mejor = None
        for i in range(-5, 6):
            off = math.radians(6 * i)
            ang = th + sa_ + off
            dx, dy = math.cos(ang), math.sin(ang)
            haz2 = math.exp(-(off / math.radians(self.haz)) ** 2) ** 2
            for (a, b), delgado in self.lab.segs:
                r = sa.rayo_segmento(ox, oy, dx, dy, a, b)
                if r is None:
                    continue
                d, alfa = r
                fuerza = (0.3 if delgado else 1.0) * haz2 * math.cos(alfa) ** 2 / max(d, 0.05) ** 2
                if fuerza >= self.umbral and (mejor is None or d < mejor):
                    mejor = d
        if mejor is None or not (0.02 <= mejor <= 4.0):
            return -1.0
        if self.rng.random() < self.p_perdida + (0.30 if mejor < 0.06 else 0.0):
            return -1.0
        return max(0.02, mejor + self.rng.gauss(0, 0.002))

    def sonar_ciclo(self):
        ms = int(round(self.t * 1000))
        if self.busy and self.t >= self.busy[1] - 1e-9:
            k, _, d = self.busy
            s = self.son[k]
            s['m'], s['ms'], s['n'] = d, ms, s['n'] + 1
            if d >= 0:
                s['valido'] = ms
            self.busy = None
        if self.busy is None and self.t - self.t_disp >= 0.060 - 1e-9:
            k = SECUENCIA[self.turno]
            self.turno = (self.turno + 1) % 4
            d = self.medir(k)
            self.busy = (k, self.t + max((2 * d / 343.0) if d >= 0 else 0.040, 0.0005), d)
            self.t_disp = self.t

    # ------------------------------------------------------------------ ESP32
    def orden(self, datos):
        ident = int(datos[0])
        if ident == 8:
            self.g.g_init(int(datos[1]), datos[2], datos[3], datos[4] if len(datos) > 4 else 5.0)
        elif ident == 4:
            self.mov = None
        elif ident in (3, 7) and self.mov is None:
            giro = ident == 7
            dist = datos[1] * self.trocha_cfg / 2 if giro else datos[1]
            self.mov = dict(giro=giro, dir=1 if dist >= 0 else -1, obj=abs(dist) * self.fpm, base=list(self.enc),
                            fase=1, t=self.t, vel=30.0 if giro else 40.0, fren=(0.0015 if giro else 0.0223) * self.fpm)
            self.t_cmd = self.t

    def paso_esp(self):
        self.sonar_ciclo()
        ms = int(round(self.t * 1000))
        m = self.mov
        if m is not None:
            self.t_cmd = self.t
            if m['fase'] == 1:
                rec = sum(abs(self.enc[i] - m['base'][i]) for i in range(2)) / 2 * self.fpm
                resta = m['obj'] - m['fren'] - rec
                if resta <= 0 or self.t - m['t'] > 20.0:
                    m['fase'], m['t2'] = 2, self.t
                    f = [0.0, 0.0]
                else:
                    v = max(VMIN, min(m['vel'], VMIN + ACEL * (self.t - m['t']), math.sqrt(2 * ACEL * resta)))
                    f = [-m['dir'] * v, m['dir'] * v] if m['giro'] else [m['dir'] * v, m['dir'] * v]
            else:
                f = [0.0, 0.0]
                if self.t - m['t2'] >= 1.5:
                    self.mov = None
                    self.mov_seq += 1
        else:
            cv, cw = self.cmd
            f = []
            for v in (cv - cw * self.trocha_cfg / 2, cv + cw * self.trocha_cfg / 2):
                x = v * self.fpm
                f.append(0.0 if abs(x) < VMIN * 0.5 else max(-VMAX, min(VMAX, x)))
            if self.t - self.t_cmd > 1.0:
                f = [0.0, 0.0]
        import ctypes
        c = (ctypes.c_float * 4)(f[0], f[1], f[0], f[1])
        s = self.son['F']
        lim = self.g.g_paso(ms, s['m'], s['n'], s['ms'], s['valido'], VMIN)
        self.estado_g = self.g.g_estado()
        self.g.g_limitar(c, 4, lim, VMIN * 0.5)
        if m is not None and m['fase'] == 1 and not m['giro'] and m['dir'] > 0 and lim == 0.0:
            self.mov = None                        # el firmware aborta un avance frenado por la guardia
            self.mov_seq += 1
        obj = [0.0 if c[i] == 0 else math.copysign(max(abs(c[i]), VMIN), c[i]) / self.fpm for i in range(2)]
        for i in range(2):
            if obj[i] == 0.0:
                self.lado[i] *= math.exp(-DT / 0.03)
                self.espera[i] = 0.0
            else:
                if self.obj_ant[i] == 0.0 or (obj[i] > 0) != (self.obj_ant[i] > 0):
                    # Arranque tardío (M 3/11: 0,8 s en las delanteras, las dos). Nominal: un retardo común a los dos lados más
                    # una diferencia chica (el escalonado de 100 ms por rueda y la dispersión). Pesimista: cada lado por su lado.
                    if self.pesimista:
                        self.espera[i] = self.rng.uniform(0.25, 0.9)
                    else:
                        if self.t - getattr(self, '_t_arr', -9) > 0.05:
                            self._t_arr, self._comun = self.t, self.rng.uniform(0.25, 0.9)
                        self.espera[i] = max(0.05, self._comun + self.rng.gauss(0.0, 0.08) - (0.05 if i == 0 else 0.0))
                    self.lado[i] = 0.0
                if self.espera[i] > 0.0:
                    self.espera[i] -= DT
                else:
                    self.lado[i] += (obj[i] - self.lado[i]) * (1 - math.exp(-DT / 0.25))
            self.obj_ant[i] = obj[i]
            self.enc[i] += self.lado[i] * DT
        v = (self.lado[0] + self.lado[1]) / 2 * self.k_avance
        dw = self.lado[1] - self.lado[0]
        if abs(dw) > 0.02 and abs(self.w_ant) <= 0.02:
            self.gan = self.rng.uniform(0.4, 0.75) if self.rng.random() < 0.10 else self.rng.gauss(1.0, 0.07)
        self.w_ant = dw
        w = dw / self.trocha_real * self.gan
        x, y, th = self.pose
        paso = v * DT
        th += w * DT + self.sesgo * paso + self.rng.gauss(0, 0.03) * math.sqrt(abs(paso))
        self.pose = [x + paso * math.cos(th), y + paso * math.sin(th), th]

    # ------------------------------------------------------------------ cámara (oráculo)
    def camara(self):
        p = self.piloto
        if p.estado in (tl.GIRAR, tl.ALINEAR, tl.ESPERA, tl.FIN):
            return
        x, y, th = self.pose
        o = (x + CAM_X * math.cos(th), y + CAM_X * math.sin(th))
        for f in self.lab.flechas:
            (a, b) = self.lab.cara(f['celda'], f['pared'])
            q = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            dist = math.hypot(q[0] - o[0], q[1] - o[1])
            ang = sa_ang(math.atan2(q[1] - o[1], q[0] - o[0]) - th)
            normal = tl.PASOS[f['pared']]
            inc = abs((q[0] - o[0]) * normal[0] + (q[1] - o[1]) * normal[1]) / dist
            if 0.40 < dist < 1.1 and abs(ang) < CAM_HFOV / 2 - 0.05 and inc > 0.4 and self.lab.visible(o, q) \
                    and self.rng.random() < 0.5:
                p.flecha(f['celda'], f['dir'], f['largo'], f['color'])
        if self.lab.cubo is not None:
            cx, cy = self.lab.cubo
            dist = math.hypot(cx - o[0], cy - o[1])
            ang = sa_ang(math.atan2(cy - o[1], cx - o[0]) - th)
            if dist < 1.2 and abs(ang) < math.radians(12) and self.lab.visible(o, (cx, cy)) and self.rng.random() < 0.7:
                p.ver_cubo(self.t, (dist - (tl.F_X - CAM_X) - 0.02) * self.rng.gauss(1.0, 0.05))

    # ------------------------------------------------------------------ todo
    def correr(self):
        p = self.piloto
        prox_pc = prox_tele = prox_ticks = prox_cam = prox_cfg = 0.0
        t_agarre = None
        p.iniciar(0.0)
        while self.t < self.t_max:
            if self.t >= self.hueco and self.rng.random() < self.p_hueco * DT:
                self.hueco = self.t + self.rng.uniform(0.3, 1.2)
            while self.cola and self.cola[0][0] <= self.t:
                _, _, tipo, datos = heapq.heappop(self.cola)
                if tipo == 'vel':
                    if self.mov is None:
                        self.cmd, self.t_cmd = datos, self.t
                elif tipo == 'cmd':
                    self.orden(datos)
                elif tipo == 'tele':
                    self.tele, self.t_tele = datos, self.t
                else:
                    self.s_pc = datos
            self.paso_esp()
            if self.lab.choca(self.pose):
                self.choque = (round(self.t, 1), p.estado, tuple(round(v, 3) for v in self.pose))
                break
            if self.lab.toca_cubo(self.pose):
                self.toco_cubo = True
            if self.t >= prox_tele:
                prox_tele += 0.1
                s = self.son
                quieto = all(abs(v) * self.fpm < 3.0 for v in self.lado)
                self.enviar('tele', (s['F']['m'], s['I']['m'], s['D']['m'], self.estado_g, s['F']['n'],
                                     1 if self.mov is not None else 0, self.mov_seq, quieto))
            if self.t >= prox_ticks:
                prox_ticks += 0.05
                self.enviar('ticks', ((self.enc[0] + self.enc[1]) / 2, (self.enc[1] - self.enc[0]) / self.trocha_cfg))
            if self.t >= prox_cam:
                prox_cam += 0.2
                self.camara()
            if self.t >= prox_cfg:
                prox_cfg += 1.0
                self.enviar('cmd', (8, 1) + p.guardia() + (5.0,))
            if p.guardia() != getattr(self, '_g_ult', None):
                self._g_ult = p.guardia()
                prox_cfg = self.t
            if self.t >= prox_pc:
                prox_pc += 0.05
                if self.tele is not None:
                    fm, im, dm, ge, n, act, seq, quieto = self.tele
                    nn = lambda v: None if v < 0 else v
                    tele = dict(frente=nn(fm), izq=nn(im), der=nn(dm), guardia=ge, n_frente=n, mov_activo=act,
                                mov_seq=seq, quieto=quieto, fresco=self.t - self.t_tele < 0.5)
                    v, w, orden = p.paso(self.t, tele, *self.s_pc)
                    self.enviar('vel', (v, w))
                    if orden is not None:
                        self.enviar('cmd', orden)
                if p.estado == tl.AGARRE:
                    t_agarre = t_agarre or self.t
                    if self.t - t_agarre > 2.0:
                        self.dist_agarre = self.dist_al_cubo()
                        p.agarrado = True
                if p.estado == tl.FIN:
                    break
            self.t += DT
        return self

    def dist_al_cubo(self):
        if self.lab.cubo is None:
            return None
        x, y, th = self.pose
        fx, fy = x + tl.F_X * math.cos(th), y + tl.F_X * math.sin(th)
        cx, cy = self.lab.cubo
        adelante = (cx - fx) * math.cos(th) + (cy - fy) * math.sin(th) - 0.02
        costado = -(cx - x) * math.sin(th) + (cy - y) * math.cos(th)
        return round(adelante, 3), round(costado, 3)


def sa_ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--semillas', type=int, default=20)
    ap.add_argument('--laberinto', default=None, choices=sorted(LABERINTOS))
    ap.add_argument('--pesimista', action='store_true')
    ap.add_argument('--detalle', action='store_true')
    a = ap.parse_args()
    g = sa.cargar_guardia()
    for nombre in ([a.laberinto] if a.laberinto else sorted(LABERINTOS)):
        lab = Laberinto(LABERINTOS[nombre])
        res = []
        for semilla in range(1, a.semillas + 1):
            s = Sim(g, lab, semilla, pesimista=a.pesimista).correr()
            p = s.piloto
            agarre = getattr(s, 'dist_agarre', None)
            res.append(dict(semilla=semilla, choque=s.choque, cubo=agarre, volvio=p.estado == tl.FIN and p.modo == 'VOLVER', t=round(s.t, 1), toco=s.toco_cubo, fin=p.estado, motivo=p.motivo))
            if a.detalle:
                print('\n'.join(s.log))
                print('pose final', [round(v, 3) for v in s.pose])
        choques = [r for r in res if r['choque']]
        agarres = [r for r in res if r['cubo']]
        volvio = [r for r in res if r['volvio']]
        print('%-7s %s: %d corridas | choques %d | llegó al cubo %d | volvió a la partida %d | tocó el cubo %d' % (
            nombre, 'pesimista' if a.pesimista else 'nominal', len(res), len(choques), len(agarres), len(volvio),
            sum(r['toco'] for r in res)))
        if agarres:
            ad = [r['cubo'][0] for r in agarres]
            co = [abs(r['cubo'][1]) for r in agarres]
            print('        frente-cubo al agarrar: %.3f a %.3f m | costado máx %.3f m | tiempo medio %.0f s' % (
                min(ad), max(ad), max(co), sum(r['t'] for r in res) / len(res)))
        for r in res:
            if r['choque'] or not r['volvio']:
                print('        semilla %d: choque %s | cubo %s | fin %s: %s' % (r['semilla'], r['choque'], r['cubo'], r['fin'],
                                                                         r['motivo']))


if __name__ == '__main__':
    main()
