"""Simulador 2D del vagabundo con el freno por sonar de la ESP32. Sirve para probar la logica ANTES de largar el robot al piso.

Usa el mismo C de firmware/pruebas/comun/guardia.h (se compila con gcc y se llama por ctypes) y la misma logica de
autonomo_logica.py. Lo demas es un modelo del robot con los errores medidos el 3/11/2026 en el piso (M) o supuestos (P):

  - avance: -1,5 % y desvio lateral (sesgo de rumbo + ruido) de unos 5 cm por metro (M)
  - giros: ganancia 1 +- 7 %, y 1 de cada 10 giros solo rinde 40-75 % (patina) (M: 6 de 10 corridas a +-7 %, 4 muy cortas)
  - arranque tardio de 0,25 a 0,9 s tras estar quieto (M: 0,8 s en las delanteras); el PI sostiene minimo 25 flanc/s (0,13 m/s)
  - freno: la rueda se detiene en menos de 0,5 cm (M)
  - sonar: haz gaussiano de 18 grados, ruido de 2 mm, 3 % de mediciones perdidas y 30 % bajo 6 cm (M: 32 % a 5,5 cm); la
    fuerza del eco baja con el coseno de la inclinacion de la pared y con 1/d^2, y un objeto fino devuelve 0,3 (P: medir con
    la prueba del cono y de la pared inclinada)
  - Wi-Fi: 3 % por segundo de empezar un hueco de 0,3-1,2 s sin mensajes en ningun sentido; si no, 50-250 ms de demora (M: huecos
    de 200-400 ms)
  - ubicacion de los sonares: el frontal al ras del frente y los laterales a la mitad del robot, mirando de costado (P)

Uso:  python3 tools/autonomo/simular.py [--semillas 20] [--minutos 5] [--pesimista] [--detalle]
Requiere gcc (para compilar guardia.h)."""
import argparse
import ctypes
import heapq
import math
import os
import random
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autonomo_logica as al  # noqa: E402

RAIZ = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
CABECERA = os.path.join(RAIZ, 'firmware', 'pruebas', 'comun')

LARGO, ANCHO = 0.278, 0.163
D_CONFIG = 0.065                 # diametro de la calibracion cargada en la ESP32
FLANCOS_POR_M = 40.0 / (math.pi * D_CONFIG)
TROCHA_CFG = 0.300               # trocha cargada (la que usa el firmware para pasar Twist a ruedas)
TROCHA_REAL = 0.291              # trocha efectiva medida (media de 6 giros)
VEL_MIN_FLANCOS, VEL_MAX_FLANCOS = 25.0, 75.0
DT = 0.010                       # ciclo de control de la ESP32

C_PUENTE = r'''
#include "guardia.h"
static guardia_t G;
void g_init(int activa, float d_stop, float d_lento, float ciego_s) { guardia_init(&G); guardia_configurar(&G, activa, d_stop, d_lento, ciego_s); }
float g_paso(uint32_t ahora_ms, float f_m, uint32_t f_seq, uint32_t f_ms, uint32_t f_valido_ms, float vel_lenta)
{ return guardia_paso(&G, ahora_ms, f_m, f_seq, f_ms, f_valido_ms, vel_lenta); }
int g_estado(void) { return G.estado; }
void g_limitar(float *c, int n, float lim, float zona) { guardia_limitar(c, n, lim, zona); }
'''


def cargar_guardia():
    d = tempfile.mkdtemp(prefix='guardia_')
    fuente, lib = os.path.join(d, 'g.c'), os.path.join(d, 'g.so')
    with open(fuente, 'w') as f:
        f.write(C_PUENTE)
    subprocess.run(['gcc', '-O2', '-shared', '-fPIC', '-I', CABECERA, '-o', lib, fuente, '-lm'], check=True)
    g = ctypes.CDLL(lib)
    g.g_paso.restype = ctypes.c_float
    g.g_paso.argtypes = [ctypes.c_uint32, ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_float]
    g.g_init.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    g.g_limitar.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_float, ctypes.c_float]
    return g


# ---------------------------------------------------------------------------------------------------------------------
# Geometria
# ---------------------------------------------------------------------------------------------------------------------
def rect(cx, cy, lx, ly, ang=0.0):
    c, s = math.cos(ang), math.sin(ang)
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((lx / 2, ly / 2), (-lx / 2, ly / 2), (-lx / 2, -ly / 2), (lx / 2, -ly / 2))]


def aristas(poli):
    return [(poli[i], poli[(i + 1) % len(poli)]) for i in range(len(poli))]


def rayo_segmento(ox, oy, dx, dy, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    wx, wy = a[0] - ox, a[1] - oy
    t = (wx * ey - wy * ex) / den
    u = (wx * dy - wy * dx) / den
    if t >= 0.0 and 0.0 <= u <= 1.0:
        n = math.hypot(ex, ey)
        alfa = math.acos(min(1.0, abs(dx * -ey + dy * ex) / n))      # 0 = el rayo es perpendicular a la pared
        return t, alfa
    return None


def se_tocan(p, q):
    """Dos poligonos convexos (eje separador)."""
    for poli in (p, q):
        for (a, b) in aristas(poli):
            nx, ny = -(b[1] - a[1]), b[0] - a[0]
            pa = [nx * x + ny * y for x, y in p]
            pb = [nx * x + ny * y for x, y in q]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class Mundo:
    def __init__(self, rng, pesimista=False):
        self.W = rng.uniform(3.0, 5.0)
        self.H = rng.uniform(2.5, 4.0)
        self.obst = []                       # (poligono, delgado)
        for _ in range(rng.randint(3, 7)):
            self.obst.append((rect(rng.uniform(0.5, self.W - 0.5), rng.uniform(0.5, self.H - 0.5), rng.uniform(0.12, 0.6),
                                   rng.uniform(0.12, 0.6), rng.uniform(0, math.pi)), False))
        if rng.random() < 0.7:               # una pata de silla: 4 cm
            self.obst.append((rect(rng.uniform(0.5, self.W - 0.5), rng.uniform(0.5, self.H - 0.5), 0.04, 0.04), True))
        if rng.random() < (0.9 if pesimista else 0.5):   # una pared inclinada (mueble contra la pared, puerta entreabierta)
            self.obst.append((rect(rng.uniform(0.8, self.W - 0.8), rng.uniform(0.8, self.H - 0.8), 0.6, 0.04,
                                   rng.uniform(0.5, 1.2)), False))
        self.paredes = [((0, 0), (self.W, 0)), ((self.W, 0), (self.W, self.H)), ((self.W, self.H), (0, self.H)), ((0, self.H), (0, 0))]
        self.segs = [(a, b, False) for a, b in self.paredes]
        for poli, delgado in self.obst:
            self.segs += [(a, b, delgado) for a, b in aristas(poli)]

    def libre(self, x, y, margen=0.45):
        if not (margen < x < self.W - margen and margen < y < self.H - margen):
            return False
        return all(math.hypot(x - sum(p[0] for p in poli) / 4, y - sum(p[1] for p in poli) / 4) > margen + 0.25 for poli, _ in self.obst)

    def choca(self, pose):
        poli = rect(pose[0] + 0.0, pose[1], LARGO, ANCHO, pose[2])
        if any(not (0 < x < self.W and 0 < y < self.H) for x, y in poli):
            return True
        return any(se_tocan(poli, o) for o, _ in self.obst)


# ---------------------------------------------------------------------------------------------------------------------
# Robot + ESP32 + PC
# ---------------------------------------------------------------------------------------------------------------------
SONARES = {'F': (LARGO / 2, 0.0, 0.0), 'D': (0.0, -ANCHO / 2, -math.pi / 2), 'I': (0.0, ANCHO / 2, math.pi / 2)}
SECUENCIA = ['F', 'D', 'F', 'I']


class Simulacion:
    def __init__(self, g, semilla, minutos, pesimista=False, v_cruc=None, d_stop=0.20, d_lento=0.60, usar_guardia=True):
        self.g = g
        self.rng = random.Random(semilla)
        self.pesimista = pesimista
        self.mundo = Mundo(self.rng, pesimista)
        while True:
            x, y = self.rng.uniform(0.5, self.mundo.W - 0.5), self.rng.uniform(0.5, self.mundo.H - 0.5)
            if self.mundo.libre(x, y):
                break
        self.pose = [x, y, self.rng.uniform(-math.pi, math.pi)]
        self.t_fin = minutos * 60.0
        self.haz_grados = 14.0 if pesimista else 18.0          # ancho del haz del sonar (P: medir con la prueba de cono)
        self.umbral = 1.0 / (4.0 ** 2) * (2.0 if pesimista else 1.0)   # una pared perpendicular se ve hasta 4 m (hasta 2,8 m si pesimista)
        self.p_perdida = 0.08 if pesimista else 0.03
        self.k_avance = self.rng.gauss(0.985, 0.01)
        self.sesgo = self.rng.gauss(0.0, 0.03)
        self.p_hueco = 0.06 if pesimista else 0.03
        self.vag = al.Vagabundo(al.Parametros(**({'v_crucero': v_cruc} if v_cruc else {})), random.Random(semilla + 1000))
        self.d_stop, self.d_lento = d_stop, d_lento
        self.usar_guardia = usar_guardia
        g.g_init(1 if usar_guardia else 0, d_stop, d_lento, 5.0)
        # ESP32
        self.cmd = (0.0, 0.0)
        self.t_ult_cmd = -10.0
        self.lado = [0.0, 0.0]                # velocidad real de la izquierda y la derecha (m/s)
        self.espera = [0.0, 0.0]              # arranque tardio
        self.obj_ant = [0.0, 0.0]
        self.gan_giro = 1.0
        self.w_ant = 0.0
        self.son = {k: dict(m=-1.0, ms=0, valido=0, n=0) for k in 'FDI'}
        self.son_busy = None                  # (sensor, t_listo, resultado)
        self.t_disparo = -1.0
        self.turno = 0
        self.estado_g = 0
        # red
        self.cola = []
        self.hueco_hasta = -1.0
        self.ult_tele = None
        self.t_tele = -10.0
        self.eventos = 0
        # resultados
        self.t = 0.0
        self.choque = None
        self.recorrido = 0.0
        self.min_frente = 9.0

    # --- red
    def enviar(self, t, tipo, datos):
        if t < self.hueco_hasta:
            return
        heapq.heappush(self.cola, (t + self.rng.uniform(0.05, 0.25), self.eventos, tipo, datos))
        self.eventos += 1

    # --- sonar
    def medir(self, sensor):
        """Eco = el primero de los rayos cuya fuerza supera el umbral. Fuerza = rho * (haz)^2 * cos(incidencia)^2 / d^2: una pared
        perpendicular se ve hasta 4 m; un objeto fino (rho 0,3) o una pared de costado solo cuando esta cerca. (P)"""
        sx, sy, sa = SONARES[sensor]
        x, y, th = self.pose
        ox = x + sx * math.cos(th) - sy * math.sin(th)
        oy = y + sx * math.sin(th) + sy * math.cos(th)
        mejor = None
        for k in range(-5, 6):                                  # 11 rayos, de -30 a +30 grados
            off = math.radians(6 * k)
            ang = th + sa + off
            dx, dy = math.cos(ang), math.sin(ang)
            haz2 = math.exp(-(off / math.radians(self.haz_grados)) ** 2) ** 2
            for a, b, delgado in self.mundo.segs:
                r = rayo_segmento(ox, oy, dx, dy, a, b)
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

    def sonar_ciclo(self, t):
        ms = int(round(t * 1000))
        if self.son_busy and t >= self.son_busy[1] - 1e-9:
            k, _, d = self.son_busy
            s = self.son[k]
            s['m'], s['ms'], s['n'] = d, ms, s['n'] + 1
            if d >= 0:
                s['valido'] = ms
            self.son_busy = None
        if self.son_busy is None and t - self.t_disparo >= 0.060 - 1e-9:
            k = SECUENCIA[self.turno]
            self.turno = (self.turno + 1) % 4
            d = self.medir(k)
            demora = (2 * d / 343.0) if d >= 0 else 0.040
            self.son_busy = (k, t + max(demora, 0.0005), d)
            self.t_disparo = t

    # --- un ciclo de 10 ms de la ESP32 y la fisica
    def paso_esp(self, t):
        self.sonar_ciclo(t)
        ms = int(round(t * 1000))
        cv, cw = self.cmd
        vl, vr = cv - cw * TROCHA_CFG / 2, cv + cw * TROCHA_CFG / 2
        f = []
        for v in (vl, vr):
            x = v * FLANCOS_POR_M
            f.append(0.0 if abs(x) < VEL_MIN_FLANCOS * 0.5 else max(-VEL_MAX_FLANCOS, min(VEL_MAX_FLANCOS, x)))
        if t - self.t_ult_cmd > 1.0:
            f = [0.0, 0.0]
        c = (ctypes.c_float * 4)(f[0], f[1], f[0], f[1])
        s = self.son['F']
        lim = self.g.g_paso(ms, s['m'], s['n'], s['ms'], s['valido'], VEL_MIN_FLANCOS)
        self.estado_g = self.g.g_estado()
        self.g.g_limitar(c, 4, lim, VEL_MIN_FLANCOS * 0.5)
        obj = []
        for i in range(2):
            x = c[i]
            obj.append(0.0 if x == 0 else math.copysign(max(abs(x), VEL_MIN_FLANCOS), x) / FLANCOS_POR_M)
        for i in range(2):
            if obj[i] == 0.0:
                self.lado[i] *= math.exp(-DT / 0.03)                  # freno: se detiene casi sin rodar
                self.espera[i] = 0.0
            else:
                if self.obj_ant[i] == 0.0 or (obj[i] > 0) != (self.obj_ant[i] > 0):
                    self.espera[i] = self.rng.uniform(0.25, 0.9)      # arranque tardio
                    if i == 0:
                        self.lado[i] = 0.0
                if self.espera[i] > 0.0:
                    self.espera[i] -= DT
                else:
                    self.lado[i] += (obj[i] - self.lado[i]) * (1 - math.exp(-DT / 0.25))
            self.obj_ant[i] = obj[i]
        v = (self.lado[0] + self.lado[1]) / 2 * self.k_avance
        dw = (self.lado[1] - self.lado[0])
        if abs(dw) > 0.02 and abs(self.w_ant) <= 0.02:                # empieza un giro: cuanto rinde esta vez
            self.gan_giro = self.rng.uniform(0.4, 0.75) if self.rng.random() < 0.10 else self.rng.gauss(1.0, 0.07)
        self.w_ant = dw
        w = dw / TROCHA_REAL * self.gan_giro
        x, y, th = self.pose
        paso = v * DT
        th += w * DT + self.sesgo * paso + self.rng.gauss(0, 0.03) * math.sqrt(abs(paso))
        self.pose = [x + paso * math.cos(th), y + paso * math.sin(th), th]
        self.recorrido += abs(paso)

    def correr(self):
        t = 0.0
        proximo_pc = 0.0
        proxima_tele = 0.0
        self.vag.iniciar(0.0)
        while t < self.t_fin:
            if t >= self.hueco_hasta and self.rng.random() < self.p_hueco * DT:
                self.hueco_hasta = t + self.rng.uniform(0.3, 1.2)
            while self.cola and self.cola[0][0] <= t:
                _, _, tipo, datos = heapq.heappop(self.cola)
                if tipo == 'cmd':
                    self.cmd, self.t_ult_cmd = datos, t
                else:
                    self.ult_tele, self.t_tele = datos, t
            self.paso_esp(t)
            if self.mundo.choca(self.pose):
                self.choque = (t, tuple(self.pose))
                break
            if t >= proxima_tele:
                proxima_tele += 0.1
                s = self.son
                self.enviar(t, 'tele', (s['F']['m'], s['I']['m'], s['D']['m'], self.estado_g, s['F']['n']))
            if t >= proximo_pc:
                proximo_pc += 0.1
                if self.ult_tele is not None:
                    f, i, d, ge, n = self.ult_tele
                    nn = lambda m: None if m < 0 else m
                    cv, cw = self.vag.paso(t, nn(f), nn(i), nn(d), ge, n, datos_frescos=(t - self.t_tele) < 0.6)
                    self.enviar(t, 'cmd', (cv, cw))
            t += DT
        self.t = t
        return self


def resumen(g, semillas, minutos, **kw):
    runs = [Simulacion(g, s, minutos, **kw).correr() for s in semillas]
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--semillas', type=int, default=20)
    ap.add_argument('--minutos', type=float, default=5.0)
    ap.add_argument('--pesimista', action='store_true')
    ap.add_argument('--sin-guardia', action='store_true', help='apaga el freno de la ESP32 (solo la logica de la PC)')
    ap.add_argument('--v', type=float, default=None)
    ap.add_argument('--detalle', action='store_true')
    a = ap.parse_args()
    g = cargar_guardia()
    runs = resumen(g, range(1, a.semillas + 1), a.minutos, pesimista=a.pesimista, v_cruc=a.v, usar_guardia=not a.sin_guardia)
    ok = [r for r in runs if r.choque is None]
    print('%d corridas de %.1f min simulados%s%s' % (len(runs), a.minutos, ' (modelo pesimista)' if a.pesimista else '',
                                                    ' SIN guardia' if a.sin_guardia else ''))
    print('  sin choques: %d/%d' % (len(ok), len(runs)))
    print('  recorrido medio: %.1f m | giros por corrida: %.1f | detenciones: %d' % (
        sum(r.recorrido for r in runs) / len(runs), sum(r.vag.giros for r in runs) / len(runs), sum(r.vag.detenciones for r in runs)))
    for r in runs:
        if r.choque is not None or a.detalle:
            print('  semilla-%s t=%.0f s %s | recorrido %.1f m | giros %d | %s' % (
                runs.index(r) + 1, r.t, 'CHOQUE en %.1f s' % r.choque[0] if r.choque else 'sin choque', r.recorrido, r.vag.giros, r.vag.motivo))


if __name__ == '__main__':
    main()
