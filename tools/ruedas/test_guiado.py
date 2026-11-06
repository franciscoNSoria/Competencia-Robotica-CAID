"""Pruebas del guiado por sonares (enderezar_avanzando y giro_180_hacia_la_pared) con un robot simulado en un pasillo de 40 cm.

  python3 -m pytest tools/ruedas/test_guiado.py

El simulador (RobotPasillo) imita lo que importa para estos controles: cinematica diferencial con las ruedas de cada lado juntas,
el minimo de 25 flancos/s y la zona muerta del firmware, un retardo de las ruedas, giro con ganancia distinta de la de los encoders
(patinaje), y los tres sonares con el ciclo del firmware (frontal, derecho, frontal, izquierdo: cada lateral se actualiza cada
~240 ms) y 2 mm de ruido. OJO: prueba la logica del control contra un modelo, no contra el piso; lo que diga el robot real manda.
"""
import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ruedas_logica as rl  # noqa: E402

LARGO, ANCHO = 0.278, 0.163
SONARES = {'F': (0.1399, 0.0, 0.0), 'I': (rl.SON_LX, rl.SON_LY, math.pi / 2), 'D': (rl.SON_LX, -rl.SON_LY, -math.pi / 2)}


def rayo(ox, oy, ang, a, b):
    """Distancia de un rayo al segmento a-b (None si no lo cruza)."""
    dx, dy = math.cos(ang), math.sin(ang)
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((a[0] - ox) * ey - (a[1] - oy) * ex) / den
    u = ((a[0] - ox) * dy - (a[1] - oy) * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


class RobotPasillo:
    """Pasillo a lo largo de x, de ancho `ancho`, abierto atras y, si `pared_x` no es None, cerrado adelante en x = pared_x."""

    def __init__(self, y=0.0, th=0.0, x=0.0, ancho=0.40, pared_x=None, semilla=1, ganancia_giro=0.9, ruido=0.002, retardo=0.0,
                 factor_der=1.0, deriva=0.0):
        self.rng = random.Random(semilla)
        self.x, self.y, self.th = x, y, th
        self.ancho, self.pared_x, self.ruido, self.ganancia = ancho, pared_x, ruido, ganancia_giro
        self.diam, self.trocha_cfg = 0.065, 0.300
        self.fpm = 40.0 / (math.pi * self.diam)
        self.t = 0.0
        self.obj = [0.0, 0.0]                        # flancos/s pedidos por lado
        self.deriva = deriva                          # m que 'avanza' el centro por radian al pivotar en el lugar (3,5 a 4,3 cm en 180°, M)
        self.retardo, self.factor_der = retardo, factor_der   # tiempo muerto de las ordenes (arranque tardio) y lado derecho mas flojo
        self.cola_obj = []
        self.cola_cmd = []                           # ordenes 7 (giro en el lugar) en camino
        self.mov, self.seq = None, 0                 # giro en el lugar en curso y contador de movimientos terminados (como el firmware)
        self.lado = [0.0, 0.0]                       # flancos/s reales por lado (con retardo)
        self.acum = [0.0, 0.0]
        self.ticks = [0, 0, 0, 0]
        self.n_ticks = 0
        self.estado = [0.0] * 34
        self.n_estado = 0
        self.enviadas = []
        self.son = {'F': -1.0, 'I': -1.0, 'D': -1.0}
        self.turno, self.t_son = 0, 0.0
        self.t_est = self.t_tick = 0.0
        self.min_hueco = 9.0                         # cuanto se acerco el cuerpo a una pared (m, negativo = choco)
        self.choco = False
        self.segmentos = [((-3.0, ancho / 2), (12.0, ancho / 2)), ((-3.0, -ancho / 2), (12.0, -ancho / 2))]
        if pared_x is not None:
            self.segmentos.append(((pared_x, -ancho / 2), (pared_x, ancho / 2)))
        self._sync()

    # ---- lo que recibe de la PC
    def enviar_vel(self, v, w):
        self.enviadas.append(('vel', (v, w)))
        t = self.trocha_cfg
        self.cola_obj.append((self.t + self.retardo, [(v - w * t / 2) * self.fpm, (v + w * t / 2) * self.fpm]))

    def enviar_cmd(self, ident, *a):
        self.enviadas.append(('cmd', (ident,) + a))
        if ident == 4:
            self.cola_obj.append((self.t, [0.0, 0.0]))
            self.mov = None
        elif ident == 7 and self.mov is None:
            self.cola_cmd.append((self.t + self.retardo, a))

    def atender(self):
        pass

    # ---- fisica
    def _flancos(self, f):
        if abs(f) < 12.5:
            return 0.0
        return math.copysign(max(25.0, min(75.0, abs(f))), f)

    def avanzar(self, dt):
        for _ in range(int(round(dt / 0.01))):
            self._paso(0.01)

    def _paso(self, dt):
        self.t += dt
        while self.cola_cmd and self.cola_cmd[0][0] <= self.t:
            ang, vel = self.cola_cmd.pop(0)[1][:2]
            if self.mov is None and abs(ang) > 1e-6:
                self.mov = {'dir': 1.0 if ang > 0 else -1.0, 'vel': vel if vel > 0 else 30.0, 'rec': 0.0, 'fase': 1,
                            'obj': abs(ang) * self.trocha_cfg / 2 * self.fpm - 0.0015 * self.fpm}
        while self.cola_obj and self.cola_obj[0][0] <= self.t:
            if self.mov is None:                                                    # el movimiento de distancia manda sobre /cmd_vel
                self.obj = self.cola_obj.pop(0)[1]
            else:
                self.cola_obj.pop(0)
        m = self.mov
        if m is not None:
            if m['fase'] == 1:
                self.obj = [-m['dir'] * m['vel'], m['dir'] * m['vel']]
                if m['rec'] >= m['obj']:
                    m['fase'], m['t2'] = 2, self.t
                    self.obj = [0.0, 0.0]
            elif self.t - m['t2'] >= 1.5:                                           # asentado de 1,5 s y recien ahi cuenta como terminado
                self.mov = None
                self.seq += 1
        for i in range(2):
            self.lado[i] += (self._flancos(self.obj[i]) - self.lado[i]) * (1 - math.exp(-dt / 0.15))
            self.acum[i] += self.lado[i] * dt
        if self.mov is not None and self.mov['fase'] == 1:
            self.mov['rec'] += (abs(self.lado[0]) + abs(self.lado[1])) / 2 * dt
        real = [self.lado[0], self.lado[1] * self.factor_der]                       # lo que mueve el piso (los encoders cuentan lado[])
        v = (real[0] + real[1]) / 2 / self.fpm
        w = (real[1] - real[0]) / self.fpm / 0.30 * self.ganancia          # la trocha real da otro giro que la configurada
        self.th += w * dt
        if abs(v) < 0.03:
            v += self.deriva * abs(w)                                                # el pivote real no es sobre el centro: avanza un poco
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self._cuerpo()
        self._sonares()
        if self.t - self.t_tick >= 0.05 - 1e-9:
            self.t_tick = self.t
            self.ticks = [int(self.acum[0]), int(self.acum[1]), int(self.acum[0]), int(self.acum[1])]
            self.n_ticks += 1
        if self.t - self.t_est >= 0.1 - 1e-9:
            self.t_est = self.t
            self._sync()

    def esquinas(self):
        c, s = math.cos(self.th), math.sin(self.th)
        return [(self.x + a * c - b * s, self.y + a * s + b * c) for a in (-LARGO / 2, LARGO / 2) for b in (-ANCHO / 2, ANCHO / 2)]

    def _cuerpo(self):
        for ex, ey in self.esquinas():
            hueco = self.ancho / 2 - abs(ey)
            if self.pared_x is not None:
                hueco = min(hueco, self.pared_x - ex)
            self.min_hueco = min(self.min_hueco, hueco)
            if hueco < 0:
                self.choco = True

    def _sonares(self):
        if self.t - self.t_son < 0.060 - 1e-9:
            return
        self.t_son = self.t
        k = 'FDFI'[self.turno]
        self.turno = (self.turno + 1) % 4
        sx, sy, sa = SONARES[k]
        c, s = math.cos(self.th), math.sin(self.th)
        ox, oy = self.x + sx * c - sy * s, self.y + sx * s + sy * c
        mejor = None
        for a, b in self.segmentos:
            d = rayo(ox, oy, self.th + sa, a, b)
            if d is not None and (mejor is None or d < mejor):
                mejor = d
        self.son[k] = -1.0 if mejor is None or mejor > 4.0 or mejor < 0.02 else mejor + self.rng.gauss(0, self.ruido)

    def _sync(self):
        e = self.estado
        e[0], e[1], e[2], e[3] = self.lado[0], self.lado[1], self.lado[0], self.lado[1]
        e[16], e[19] = (1.0 if self.mov is not None else 0.0), float(self.seq)
        e[20], e[23] = self.diam * 1000, self.trocha_cfg * 1000
        e[26], e[27], e[28] = self.son['F'], self.son['I'], self.son['D']
        self.estado = list(e)
        self.n_estado += 1


class Banco(unittest.TestCase):
    def montar(self, **kw):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.robot = RobotPasillo(**kw)
        self.mensajes = []
        self.c = rl.Controlador(self.robot, carpeta=self.dir.name, reloj=lambda: 100.0 + self.robot.t, imprimir=self.mensajes.append)
        self.addCleanup(self.c.cerrar)
        for _ in range(8):
            self.paso()
        self.assertTrue(self.c.conectado)

    def paso(self):
        self.robot.avanzar(0.05)
        self.c.tick()

    def correr_hasta(self, cond, limite_s=60.0):
        for _ in range(int(limite_s / 0.05)):
            self.paso()
            if cond():
                return True
        return False

    def terminado(self):
        return self.c.guiado is None

    def rumbo(self):
        return (self.robot.th + math.pi) % (2 * math.pi) - math.pi


class Estimador(unittest.TestCase):
    def test_posicion_con_una_o_dos_paredes(self):
        e = rl.EstimadorPasillo()
        self.assertAlmostEqual(e.posicion(0.1185, 0.1185), 0.0, places=3)         # centrado: cada lado a 20 - 8,15 = 11,85 cm
        self.assertAlmostEqual(e.posicion(0.0785, 0.1585), 0.04, places=3)        # 4 cm a la izquierda
        self.assertAlmostEqual(e.posicion(0.0785, None), 0.04, places=3)           # con una sola pared tambien
        self.assertAlmostEqual(e.posicion(None, 0.1585), 0.04, places=3)
        self.assertIsNone(e.posicion(None, None))
        self.assertIsNone(e.posicion(0.60, 0.55))                                  # lejos: no es la pared del pasillo

    def test_la_pendiente_da_el_rumbo_con_signo(self):
        for psi in (math.radians(15), math.radians(-10), 0.0):
            e = rl.EstimadorPasillo()
            r = None
            for k in range(30):                                                    # avanza 1 cm por muestra torciendo psi
                s = 0.01 * k
                y = s * math.tan(psi)
                r = e.actualizar(s, 0.1185 - y, 0.1185 + y)
            self.assertAlmostEqual(r[1], psi, delta=0.01)
            self.assertAlmostEqual(r[0], 0.29 * math.tan(psi), delta=0.002)

    def test_sin_avance_no_hay_rumbo(self):
        e = rl.EstimadorPasillo()
        self.assertIsNone(e.actualizar(0.0, 0.10, 0.14)[1])
        self.assertIsNone(e.actualizar(0.01, 0.10, 0.14)[1])

    def test_el_timon_no_baja_la_rueda_de_adentro_del_minimo(self):
        v_min = 25.0 / rl.flancos_por_m(0.065)
        for y, psi in ((0.06, 0.4), (-0.06, -0.4), (0.0, 0.3), (0.05, None)):
            v, w = rl.timon_pasillo(0.16, y, psi, 0.30, 0.065)
            self.assertGreaterEqual(v - abs(w) * 0.15, v_min - 1e-9)
            self.assertLessEqual(v, rl.V_CURVA_MS + 1e-9)


class Enderezar(Banco):
    def probar(self, y, grados, cm=60, **kw):
        self.montar(y=y, th=math.radians(grados), **kw)
        self.assertTrue(self.c.enderezar_avanzando(cm / 100.0))
        self.assertTrue(self.correr_hasta(self.terminado, 40))
        self.correr_hasta(lambda: False, 1.0)

    def test_se_endereza_inclinado_hacia_la_pared_izquierda(self):
        self.probar(0.0, 20)                                                       # apuntando 20° a la izquierda: va contra esa pared
        self.assertFalse(self.robot.choco, self.mensajes[-5:])
        self.assertLess(abs(self.rumbo()), math.radians(6))
        self.assertLess(abs(self.robot.y), 0.03)

    def test_se_endereza_inclinado_hacia_la_pared_derecha(self):
        self.probar(0.0, -20)
        self.assertFalse(self.robot.choco, self.mensajes[-5:])
        self.assertLess(abs(self.rumbo()), math.radians(6))
        self.assertLess(abs(self.robot.y), 0.03)

    def test_se_centra_estando_corrido_y_torcido(self):
        for y, g in ((0.05, 12), (-0.05, -12), (0.04, -10), (-0.04, 10)):
            self.probar(y, g, semilla=int(100 * abs(y)) + g + 20)
            self.assertFalse(self.robot.choco, (y, g, self.mensajes[-4:]))
            self.assertLess(abs(self.rumbo()), math.radians(7), (y, g))
            self.assertLess(abs(self.robot.y), 0.035, (y, g))

    def test_muy_inclinado_cerca_de_la_pared_frena_piensa_se_endereza_y_sigue_sin_chocar(self):
        """Lo que pasaba: 25-35° hacia una pared a 5 cm, el control no llegaba a corregir y chocaba."""
        for y, g in ((0.05, 25), (-0.05, -25), (0.05, 20), (-0.05, -20), (-0.05, 25), (0.0, 25)):
            self.probar(y, g, cm=80, semilla=abs(g) + int(100 * abs(y)))
            self.assertFalse(self.robot.choco, (y, g, self.robot.min_hueco, self.mensajes[-5:]))
            self.assertLess(abs(self.rumbo()), math.radians(30), (y, g))   # (puede terminar en plena correccion: lo que importa es no chocar)
            self.assertLess(abs(self.robot.y), 0.07, (y, g))
            if y * g > 0:                                                         # (-0.05, 25) ya apunta lejos de su pared cercana
                self.assertTrue(any('FRENO' in m or 'CORRIJO' in m for m in self.mensajes), (y, g))   # no siguio nomas: corrigio o freno
            self.assertGreater(self.robot.x, 0.6, (y, g))                              # y despues siguio

    def test_correccion_brusca_en_marcha_cuando_un_costado_baja_de_7cm_y_sigue_avanzando(self):
        """Un costado a 8 cm que baja: corrige fuerte hacia el otro lado SIN frenar (no hay 'FRENO'), sigue y no choca."""
        corrigio = 0
        for y, g in ((0.06, 0), (-0.06, 0), (0.06, 8), (-0.06, -8)):
            self.probar(y, g, cm=100, semilla=11 + abs(g))
            self.assertFalse(self.robot.choco, (y, g, self.mensajes[-5:]))
            corrigio += any('CORRIJO' in m for m in self.mensajes)
            self.assertFalse(any('FRENO' in m for m in self.mensajes), (y, g, self.mensajes[:12]))
            self.assertGreater(self.robot.x, 0.9, (y, g))
        self.assertGreaterEqual(corrigio, 2)

    def test_pegado_a_una_pared_paralelo_se_separa_y_sigue_sin_detenerse(self):
        """Lo que pasaba: a 3 cm de la pared y paralelo, pivotar no cambia nada: se rendia y pedia reacomodar a mano."""
        for y, g in ((-0.085, 0), (0.085, 0), (-0.085, 5), (0.085, -5)):
            self.probar(y, g, cm=100, semilla=3 + abs(g))
            self.assertFalse(self.robot.choco, (y, g, self.mensajes[-5:]))
            self.assertFalse(any('ME DETENGO' in m for m in self.mensajes), (y, g))
            self.assertGreater(self.robot.x, 0.9, (y, g))
            self.assertLess(abs(self.robot.y), 0.04, (y, g))

    def test_el_retardo_de_red_y_una_rueda_floja_no_lo_hacen_chocar_inclinado(self):
        for y, g in ((0.05, 25), (-0.05, -25)):
            self.probar(y, g, cm=80, retardo=0.3, factor_der=0.85, semilla=7)
            self.assertFalse(self.robot.choco, (y, g, self.mensajes[-5:]))
            self.assertLess(abs(self.rumbo()), math.radians(35))

    def test_derecho_y_centrado_sigue_derecho(self):
        self.probar(0.0, 0, cm=60)
        self.assertLess(abs(self.rumbo()), math.radians(3))
        self.assertLess(abs(self.robot.y), 0.015)
        self.assertAlmostEqual(self.robot.x, 0.60, delta=0.06)                     # recorrio lo pedido (el robot sigue unos cm)

    def test_una_rueda_mas_floja_no_deja_al_robot_corrido(self):
        """Con la derecha al 75 % el robot deriva siempre al mismo lado: sin termino integral se quedaba 4 cm corrido."""
        self.probar(0.0, 0, cm=120, factor_der=0.75)
        self.assertFalse(self.robot.choco)
        self.assertLess(abs(self.robot.y), 0.025)                                   # sin integral: 4 cm y creciendo
        self.assertLess(abs(self.rumbo()), math.radians(6))

    def test_con_retardo_en_las_ordenes_no_oscila_de_mas(self):
        self.probar(0.04, 12, cm=120, retardo=0.4, semilla=3)
        self.assertFalse(self.robot.choco)
        self.assertLess(abs(self.robot.y), 0.04)
        self.assertLess(abs(self.rumbo()), math.radians(30))

    def test_sin_distancia_sigue_hasta_la_pared_de_frente(self):
        self.montar(pared_x=1.0)
        self.assertTrue(self.c.enderezar_avanzando(None))
        self.assertTrue(self.correr_hasta(self.terminado, 40))
        self.assertFalse(self.robot.choco)
        self.assertGreater(self.robot.x, 0.7)
        self.assertTrue(any('pared de frente' in m for m in self.mensajes))

    def test_no_arranca_pegado_a_una_pared_de_frente_y_lo_dice(self):
        self.montar(pared_x=0.30)                                                  # el frente del robot queda a ~16 cm... se acerca mas
        self.robot.x = 0.30 - 0.1399 - 0.06                                         # frente a 6 cm, como en el fondo del pasillo
        for _ in range(8):
            self.paso()
        self.assertFalse(self.c.enderezar_avanzando(1.0))
        self.assertTrue(any('No avanzo: hay una pared de frente' in m for m in self.mensajes))

    def test_frena_ante_una_pared_de_frente(self):
        self.montar(pared_x=0.45)
        self.c.enderezar_avanzando(1.0)
        self.assertTrue(self.correr_hasta(self.terminado, 30))
        self.assertFalse(self.robot.choco)
        self.assertTrue(any('pared de frente' in m for m in self.mensajes))

    def test_no_arranca_sin_sonares_ni_en_otra_maniobra(self):
        self.montar()
        self.c.estado = self.c.estado[:26]
        self.assertFalse(self.c.enderezar_avanzando(0.3))
        self.c.estado = list(self.robot.estado)
        self.assertTrue(self.c.enderezar_avanzando(0.3))
        self.assertFalse(self.c.enderezar_avanzando(0.3))                          # ya hay una

    def test_parar_cancela_y_frena(self):
        self.montar()
        self.c.enderezar_avanzando(1.0)
        for _ in range(10):
            self.paso()
        self.c.parar()
        self.assertIsNone(self.c.guiado)
        self.robot.avanzar(0.1)
        self.assertEqual(self.robot.obj, [0.0, 0.0])


class Escape(Banco):
    def test_retrocede_una_vez_y_gira_al_lado_contrario_de_la_pared_mas_cercana(self):
        """(El simulador no hace la orden 3 de distancia: se comprueba la orden que sale y el sentido del giro.)"""
        for y, texto in ((0.05, 'pared a la izquierda'), (-0.05, 'pared a la derecha')):
            self.montar(y=y, semilla=3)
            self.assertTrue(self.c.escape_de_pared(0.06, 45.0, 0.12, 60.0))
            self.correr_hasta(lambda: False, 0.5)
            ordenes = [a for t, a in self.robot.enviadas if t == 'cmd' and a[0] == 3]
            self.assertEqual(len(set(ordenes)), 1)                                 # UNA sola orden de retroceso (se repite igual hasta que arranca)
            self.assertAlmostEqual(ordenes[0][1], -0.06, places=3)
            self.assertTrue(any(texto in m for m in self.mensajes), texto)
            self.assertEqual(list(self.c.secuencia), [('giro', -45.0 if y > 0 else 45.0)])
            self.c.parar()

    def test_lado_forzado(self):
        self.montar(y=0.0)
        self.c.escape_de_pared(0.05, 30.0, 0.12, 60.0, lado_pared=-1)             # pared a la derecha: gira a la izquierda
        self.assertTrue(any('giro 30° a la izquierda' in m for m in self.mensajes))


class Giro180(Banco):
    """El 180 son dos giros de 90 seguidos (orden 7) hacia la pared lateral mas cercana; solo retrocede un poco si el frente esta muy cerca."""

    def probar(self, y, grados=0.0, adicional=0.0, despues=None, corr90=0.0, vel=40, **kw):
        kw.setdefault('ganancia_giro', 1.0)
        self.montar(y=y, th=math.radians(grados), **kw)
        self.assertTrue(self.c.giro_180_hacia_la_pared(vel, adicional, despues, corr90))
        self.assertTrue(self.correr_hasta(self.terminado, 60))
        self.correr_hasta(lambda: False, 1.0)

    def test_dos_giros_de_90_seguidos_hacia_la_pared_mas_cercana(self):
        for y, lado in ((0.03, 'izquierda'), (-0.03, 'derecha')):
            self.probar(y, semilla=int(1000 * abs(y)))
            self.assertFalse(self.robot.choco, (y, self.robot.min_hueco))
            self.assertTrue(any('Giro 1/2' in m for m in self.mensajes) and any('Giro 2/2' in m for m in self.mensajes))
            self.assertTrue(any('dos giros de 90° hacia la ' + lado in m for m in self.mensajes), (y, lado))
            self.assertLess(abs(abs(self.rumbo()) - math.pi), math.radians(20), (y, math.degrees(self.rumbo())))   # el simulador se pasa ~10° por pivote
            self.assertFalse([a for t, a in self.robot.enviadas if t == 'vel' and a[0] < 0])                       # no retrocedio

    def test_los_sumadores_se_aplican_a_cada_giro_y_el_del_180_al_segundo(self):
        self.probar(0.0, adicional=5.0, corr90=10.0)
        self.assertTrue(any('Giro 1/2: 100°' in m for m in self.mensajes))
        self.assertTrue(any('Giro 2/2: 105°' in m for m in self.mensajes))
        self.assertTrue(any('pedido 205' in m for m in self.mensajes))

    def test_informa_las_distancias_al_terminar(self):
        self.probar(0.05)
        self.assertTrue(any('Despues del giro' in m and 'pared izquierda' in m and 'frente' in m for m in self.mensajes))

    def test_despues_avanza_enderezandose(self):
        self.montar(y=0.02, ganancia_giro=1.0)
        self.c.giro_180_hacia_la_pared(40, 0.0, despues_m=0.25)
        self.assertTrue(self.correr_hasta(lambda: self.c.guiado is None and any('Llegue' in m or 'Frene' in m for m in self.mensajes), 80))
        self.assertTrue(any('linea recta enderezando: 25 cm' in m for m in self.mensajes))

    def test_solo_retrocede_un_poco_si_el_frente_esta_muy_cerca(self):
        self.montar(y=0.0, pared_x=0.1399 + 0.06, ganancia_giro=1.0)
        self.assertTrue(self.c.giro_180_hacia_la_pared(40))
        self.assertTrue(self.correr_hasta(self.terminado, 60))
        self.assertTrue(any('retrocedo' in m and 'antes de girar' in m for m in self.mensajes))
        self.assertLess(-min(0.0, self.robot.x), 0.10)                              # unos cm (la pared estaba a 6 cm del frente)

    def test_sin_sonares_no_gira(self):
        self.montar()
        self.c.estado = self.c.estado[:26]
        self.assertFalse(self.c.giro_180_hacia_la_pared(150))


if __name__ == '__main__':
    unittest.main()
