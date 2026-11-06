"""Pruebas de ruedas_logica.py con un robot simulado que imita el protocolo de firmware/banco/robot_4ruedas (sin ROS ni ESP32).

  python3 -m pytest tools/ruedas/test_ruedas_logica.py        (o:  python3 -m unittest tools/ruedas/test_ruedas_logica.py)

OJO: lo que se prueba aca es la logica de la PC y el protocolo. Que el robot real frene donde debe, o que gire lo pedido, solo lo
dice el piso: esas pruebas las haces vos con el panel.
"""
import csv
import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ruedas_logica as rl  # noqa: E402


class RobotFalso:
    """Imita la ESP32: recibe /robot_cmd y /cmd_vel y publica /wheel_state (10 Hz) y /wheel_ticks (20 Hz).
    Las maniobras de distancia y de giro siguen la logica del firmware: velocidad constante hasta el adelanto de frenado, 1,5 s
    asentando y despues se informa el recorrido con el contador. `sobre_m` es lo que el robot rueda de mas despues de cortar."""

    def __init__(self, sobre_m=0.0, relacion_giro=1.0):
        self.estado = [0.0] * 26
        self.n_estado = 0
        self.ticks = [0, 0, 0, 0]
        self.n_ticks = 0
        self.enviadas = []                       # (tipo, args) de todo lo que llega
        self.diam, self.trocha, self.fren, self.fren_giro = 0.067, 0.131, 0.030, 0.005
        self.sobre_m = sobre_m
        self.mov = None
        self.seq = 0
        self.t = 0.0
        self.v_ruedas = [0.0] * 4               # flancos/s con signo que giran ahora
        self._acum = [0.0] * 4
        self._t_est = self._t_tick = 0.0
        self._sincronizar()

    def _fpm(self):
        return rl.flancos_por_m(self.diam)

    def enviar_vel(self, lineal, angular):
        self.enviadas.append(('vel', (lineal, angular)))
        vl, vr = lineal - angular * self.trocha / 2, lineal + angular * self.trocha / 2
        self.v_ruedas = [vl * self._fpm(), vr * self._fpm(), vl * self._fpm(), vr * self._fpm()]

    def enviar_cmd(self, ident, *a):
        self.enviadas.append(('cmd', (ident,) + a))
        if ident == 4:
            self.mov = None
            self.v_ruedas = [0.0] * 4
        elif ident == 3 and self.mov is None:
            self._empezar('distancia', a[0], a[1] if len(a) > 1 and a[1] > 0 else 40.0)
        elif ident == 7 and self.mov is None:
            self._empezar('giro', a[0] * self.trocha / 2.0, a[1] if len(a) > 1 and a[1] > 0 else 30.0)
        elif ident == 5:
            self.diam, self.trocha, self.fren = a[0], a[1], a[2]
            if len(a) > 4:
                self.fren_giro = a[4]

    def _empezar(self, tipo, dist, vel):
        self.mov = {'tipo': tipo, 'dir': 1 if dist >= 0 else -1, 'obj': abs(dist) * self._fpm(), 'vel': vel, 'rec': 0.0,
                    'fase': 1, 't_asentar': 0.0, 'frenado': (self.fren if tipo == 'distancia' else self.fren_giro) * self._fpm()}
        self.estado[16] = 1.0

    def avanzar(self, dt):
        self.t += dt
        m = self.mov
        fpm = self._fpm()
        flancos = [0.0] * 4                                   # lo que gira cada rueda en este paso, con signo
        if m is not None:
            signos = [1, 1, 1, 1] if m['tipo'] == 'distancia' else [-1, 1, -1, 1]
            if m['fase'] == 1:
                avance = m['vel'] * dt
                m['rec'] += avance
                self.v_ruedas = [s * m['dir'] * m['vel'] for s in signos]
                if m['rec'] >= m['obj'] - m['frenado']:       # se corta el motor; sigue rodando el adelanto de frenado y `sobre_m` mas
                    rueda = m['frenado'] + self.sobre_m * fpm
                    m['rec'] += rueda
                    avance += rueda
                    m['fase'] = 2
                    m['t_asentar'] = self.t
                    self.v_ruedas = [0.0] * 4
                flancos = [s * m['dir'] * avance for s in signos]
            elif self.t - m['t_asentar'] >= 1.5:
                self.estado[18] = m['dir'] * m['rec'] / fpm
                self.estado[17] = self.estado[18]
                self.estado[16] = 0.0
                self.seq += 1
                self.mov = None
        else:
            flancos = [v * dt for v in self.v_ruedas]         # /cmd_vel o duty: lo que marquen las ruedas
        for i in range(4):
            self._acum[i] += flancos[i]
            self.ticks[i] = int(self._acum[i])
        if self.t - self._t_tick >= 0.05 - 1e-9:
            self._t_tick = self.t
            self.n_ticks += 1
        if self.t - self._t_est >= 0.1 - 1e-9:
            self._t_est = self.t
            self._sincronizar()

    def _sincronizar(self):
        e = self.estado
        for i in range(4):
            e[i] = self.v_ruedas[i]
        e[19] = float(self.seq)
        e[20], e[21], e[23], e[25] = self.diam * 1000, self.fren * 1000, self.trocha * 1000, self.fren_giro * 1000
        e[24] = 1.0 if self.mov and self.mov['tipo'] == 'giro' else 0.0
        self.estado = list(e)
        self.n_estado += 1


class Banco(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.robot = RobotFalso()
        self.t = [100.0]
        self.mensajes = []
        self.c = rl.Controlador(self.robot, carpeta=self.dir.name, reloj=lambda: self.t[0], imprimir=self.mensajes.append)

    def tearDown(self):
        self.c.cerrar()
        self.dir.cleanup()

    def paso(self, segundos=0.05, n=1):
        for _ in range(n):
            self.robot.avanzar(segundos)
            self.t[0] += segundos
            self.c.tick()

    def correr(self, segundos):
        self.paso(0.05, int(round(segundos / 0.05)))

    def correr_hasta_resultado(self, limite_s=60):
        for _ in range(int(limite_s / 0.05)):
            self.paso()
            if self.c.resultado is not None:
                return
        self.fail('la maniobra no termino')

    def conectar(self):
        self.correr(0.3)
        self.assertTrue(self.c.conectado)

    def ordenes(self, tipo='cmd'):
        return [a for t, a in self.robot.enviadas if t == tipo]


class Conversiones(unittest.TestCase):
    def test_equivalencias_de_la_guia(self):
        # docs/guias/robot-4-ruedas-pid-y-comunicacion.md: 190,04 flancos por metro y 40 flancos/s = 0,2105 m/s con rueda de 67 mm
        self.assertAlmostEqual(rl.flancos_por_m(0.067), 190.04, places=2)
        self.assertAlmostEqual(rl.ms_de_flancos(40.0, 0.067), 0.2105, places=4)
        # 180 grados con trocha 0,131: arco de pi x 0,0655 = 0,2058 m
        self.assertAlmostEqual(rl.grados_de_arco(math.pi * 0.131 / 2, 0.131), 180.0, places=6)

    def test_correcciones(self):
        self.assertAlmostEqual(rl.diametro_corregido(0.067, 0.98, 1.00), 0.06566, places=5)       # avanzo menos: rueda mas chica
        self.assertEqual(rl.diametro_corregido(0.067, 1.0, 0.0), 0.067)                           # sin dato, no cambia
        self.assertEqual(rl.diametro_corregido(0.067, 100.0, 1.0), rl.DIAMETRO_RANGO[1])          # se acota a lo que acepta la ESP32
        # giro: los encoders dicen 360 y el robot giro 330 (patina): la trocha efectiva es mayor
        self.assertAlmostEqual(rl.trocha_corregida(0.131, 360.0, 330.0), 0.131 * 360 / 330, places=6)
        self.assertAlmostEqual(rl.frenado_corregido(0.030, 0.012, 0.09), 0.042, places=9)
        self.assertEqual(rl.frenado_corregido(0.030, -0.5, 0.09), 0.0)
        self.assertEqual(rl.hacia(0.0, 1.0, 0.1), 0.1)
        self.assertEqual(rl.hacia(0.95, 1.0, 0.1), 1.0)
        self.assertEqual(rl.hacia(0.0, -1.0, 0.25), -0.25)


class OdometriaTest(unittest.TestCase):
    D, T = 0.067, 0.131

    def seguir(self, tramos):
        o = rl.Odometria()
        o.actualizar([0, 0, 0, 0], self.D, self.T)
        ticks = [0, 0, 0, 0]
        for izq, der, pasos in tramos:                     # flancos por muestra de cada lado
            for _ in range(pasos):
                ticks = [ticks[0] + izq, ticks[1] + der, ticks[2] + izq, ticks[3] + der]
                o.actualizar(ticks, self.D, self.T)
        return o

    def test_recta(self):
        o = self.seguir([(2, 2, 95)])                       # 190 flancos = 1 m
        self.assertAlmostEqual(o.s, 1.0, places=2)
        self.assertAlmostEqual(o.x, 1.0, places=2)
        self.assertAlmostEqual(o.y, 0.0, places=6)
        self.assertAlmostEqual(o.theta, 0.0, places=9)

    def test_atras(self):
        o = self.seguir([(-2, -2, 95)])
        self.assertAlmostEqual(o.s, -1.0, places=2)
        self.assertAlmostEqual(o.recorrido, 1.0, places=2)

    def test_giro_en_el_lugar_de_360_no_se_envuelve_y_no_mueve_el_centro(self):
        fpm = rl.flancos_por_m(self.D)
        arco = 2 * math.pi * self.T / 2                     # metros que recorre cada rueda en 360 grados
        n = int(round(arco * fpm))
        o = self.seguir([(-1, 1, n)])
        self.assertAlmostEqual(math.degrees(o.theta), 360.0, delta=3.0)
        self.assertLess(math.hypot(o.x, o.y), 0.01)
        self.assertAlmostEqual(o.s, 0.0, places=6)
        o2 = self.seguir([(1, -1, n)])                       # horario: negativo
        self.assertAlmostEqual(math.degrees(o2.theta), -360.0, delta=3.0)

    def test_arco(self):
        # la rueda derecha avanza el doble que la izquierda: gira a la izquierda describiendo un arco
        o = self.seguir([(1, 2, 200)])
        self.assertGreater(o.theta, 0)
        self.assertGreater(o.y, 0)

    def test_reinicio_de_la_esp32_no_cuenta_como_movimiento(self):
        o = rl.Odometria()
        o.actualizar([0, 0, 0, 0], self.D, self.T)
        o.actualizar([2000, 2000, 2000, 2000], self.D, self.T)        # salto de 2000 flancos entre dos muestras: imposible
        self.assertEqual(o.s, 0.0)
        o.actualizar([2002, 2002, 2002, 2002], self.D, self.T)        # sigue normalmente desde la nueva base
        self.assertAlmostEqual(o.s, 2 / rl.flancos_por_m(self.D), places=6)
        o.rebase()
        o.actualizar([0, 0, 0, 0], self.D, self.T)                    # rebase explicito: la primera muestra es solo la base
        self.assertAlmostEqual(o.s, 2 / rl.flancos_por_m(self.D), places=6)

    def test_reiniciar_pone_en_cero(self):
        o = self.seguir([(2, 2, 10)])
        o.reiniciar()
        self.assertEqual((o.x, o.y, o.theta, o.s, o.recorrido), (0, 0, 0, 0, 0))


class Barras(Banco):
    def test_conexion_y_valores_en_uso(self):
        self.assertFalse(self.c.conectado)
        self.conectar()
        self.assertTrue(any('conectada' in m for m in self.mensajes))
        self.assertAlmostEqual(self.c.diametro, 0.067)
        self.assertAlmostEqual(self.c.trocha, 0.131)

    def test_desarmado_no_manda_nada(self):
        self.conectar()
        self.c.objetivo = [50.0, 50.0, 50.0, 50.0]
        self.correr(1.0)
        self.assertEqual([a for a in self.ordenes() if a[0] in (1, 2)], [])
        self.assertEqual(self.c.aplicado, [0.0] * 4)

    def test_duty_con_rampa_de_aceleracion(self):
        self.conectar()
        self.c.armado = True
        self.c.aceleracion = 100.0                           # 100 % del fondo de escala por segundo
        self.c.objetivo = [50.0, 0.0, -50.0, 20.0]
        self.correr(0.2)
        self.assertAlmostEqual(self.c.aplicado[0], 20.0, delta=3.0)              # 0,2 s x 100 %/s = 20 %
        self.correr(1.0)
        self.assertEqual(self.c.aplicado, [50.0, 0.0, -50.0, 20.0])
        ultima = [a for a in self.ordenes() if a[0] == 1][-1]
        self.assertEqual(list(ultima[1:]), [0.5, 0.0, -0.5, 0.2])                  # id 1: duty de -1 a 1
        n = len([a for a in self.ordenes() if a[0] == 1])
        self.assertGreater(n, 10)                                                  # 10 por segundo, no 20

    def test_la_aceleracion_se_regula(self):
        self.conectar()
        self.c.armado = True
        self.c.aceleracion = 25.0
        self.c.objetivo = [100.0, 0.0, 0.0, 0.0]
        self.correr(1.0)
        self.assertAlmostEqual(self.c.aplicado[0], 25.0, delta=3.0)

    def test_modo_velocidad(self):
        self.conectar()
        self.c.poner_modo('vel')
        self.c.armado = True
        self.c.aceleracion = 400.0
        self.c.objetivo = [0.2, 0.2, 0.5, -0.2]                                   # 0,5 se acota a 0,40 m/s
        self.correr(1.0)
        ultima = [a for a in self.ordenes() if a[0] == 2][-1]
        self.assertEqual(list(ultima[1:]), [0.2, 0.2, 0.4, -0.2])

    def test_cambiar_de_modo_pone_las_barras_en_cero(self):
        self.conectar()
        self.c.objetivo = [50.0] * 4
        self.c.aplicado = [50.0] * 4
        self.c.poner_modo('vel')
        self.assertEqual((self.c.objetivo, self.c.aplicado), ([0.0] * 4, [0.0] * 4))

    def test_cero_pone_en_cero_la_odometria_y_los_ticks_de_la_pc(self):
        self.conectar()
        self.robot.enviar_vel(0.2, 0.0)
        self.correr(1.0)
        self.assertGreater(self.c.odo.s, 0.1)
        self.assertGreater(self.c.ticks_desde_cero()[0], 10)
        self.c.poner_cero()
        self.assertEqual(self.c.odo.s, 0.0)
        self.assertEqual(self.c.ticks_desde_cero(), [0, 0, 0, 0])
        self.correr(0.5)
        self.assertGreater(self.c.ticks_desde_cero()[0], 5)                          # y sigue contando desde ahi

    def test_parar(self):
        self.conectar()
        self.c.armado = True
        self.c.objetivo = [50.0] * 4
        self.correr(0.5)
        self.c.parar()
        self.assertFalse(self.c.armado)
        self.assertEqual(self.c.objetivo, [0.0] * 4)
        self.assertGreaterEqual(len([a for a in self.ordenes() if a[0] == 4]), 3)
        self.assertEqual(self.ordenes('vel')[-1], (0.0, 0.0))


class Maniobras(Banco):
    def test_sin_conexion_no_arranca(self):
        self.assertFalse(self.c.distancia(1.0, 0.21))
        self.assertTrue(any('no esta conectada' in m for m in self.mensajes))

    def test_un_metro_adelante(self):
        self.robot.sobre_m = 0.002
        self.conectar()
        self.assertTrue(self.c.distancia(1.0, 0.21))
        self.correr_hasta_resultado()
        r = self.c.resultado
        ordenes3 = [a for a in self.ordenes() if a[0] == 3]
        self.assertEqual(len(ordenes3), 1)                                       # arranco a la primera: no se repite
        self.assertEqual(ordenes3[0][1], 1.0)
        self.assertAlmostEqual(ordenes3[0][2], 0.21 * 190.04, delta=0.2)         # 0,21 m/s = 39,9 flancos/s
        self.assertEqual(r['tipo'], 'distancia')
        self.assertAlmostEqual(r['calculado'], 1.0 + 0.002, delta=0.015)        # la meta mas lo que rodo de mas (y hasta un paso de 10 mm)
        self.assertAlmostEqual(r['odometria'], r['calculado'], delta=0.01)       # la PC cuenta lo mismo que la ESP32 (a 1 flanco)
        self.assertTrue(all(t > 150 for t in r['ticks']))
        self.assertAlmostEqual(self.c.odo.s, 1.0, delta=0.03)

    def test_se_repite_la_orden_si_se_perdio_y_se_rinde(self):
        self.conectar()
        self.robot.mov = {'tipo': 'distancia', 'dir': 1, 'obj': 1e9, 'vel': 0, 'rec': 0, 'fase': 1, 't_asentar': 0, 'frenado': 0}
        self.robot.estado[16] = 0.0                                              # la ESP32 "no la oyo": nunca dice que esta activa
        self.assertTrue(self.c.distancia(1.0, 0.21))
        self.correr(5.0)
        self.assertGreater(len([a for a in self.ordenes() if a[0] == 3]), 5)      # reintentos cada 0,3 s
        self.assertIsNone(self.c.maniobra)
        self.assertTrue(any('no empezo' in m for m in self.mensajes))

    def test_pasarse_de_la_meta_ajusta_el_adelanto_de_frenado(self):
        self.robot.sobre_m = 0.030                                                # rueda 3 cm de mas despues de cortar
        self.conectar()
        self.c.distancia(1.0, 0.21)
        self.correr_hasta_resultado()
        self.assertGreater(self.c.resultado['sobre_m'], 0.02)
        self.correr(1.0)                                                          # se envian de a una por tick
        cfg = [a for a in self.ordenes() if a[0] == 5][-1]
        self.assertAlmostEqual(cfg[3], 0.030 + self.c.resultado['sobre_m'], places=3)   # corta antes: mas adelanto
        self.assertAlmostEqual(self.robot.fren, cfg[3], places=6)                  # y la ESP32 simulada lo recibio
        guardado = json.load(open(os.path.join(self.dir.name, 'calibracion.json')))
        self.assertAlmostEqual(guardado['frenado_m'], cfg[3], places=6)

    def test_sin_ajuste_automatico_no_toca_el_frenado(self):
        self.robot.sobre_m = 0.030
        self.c.ajustar_frenado = False
        self.conectar()
        self.c.distancia(1.0, 0.21)
        self.correr_hasta_resultado()
        self.correr(1.0)
        self.assertEqual([a for a in self.ordenes() if a[0] == 5], [])

    def test_registrar_y_corregir_el_diametro(self):
        self.robot.sobre_m = 0.0
        self.c.ajustar_frenado = False
        self.conectar()
        self.c.distancia(1.0, 0.21)
        self.correr_hasta_resultado()
        calc = self.c.resultado['calculado']
        self.assertTrue(self.c.registrar(real=0.97, desplazamiento_cm=None, corregir=True))
        self.assertIsNone(self.c.resultado)
        self.correr(1.0)
        esperado = 0.067 * 0.97 / abs(calc)
        self.assertAlmostEqual(self.robot.diam, esperado, places=6)
        filas = list(csv.reader(open(os.path.join(self.dir.name, 'ruedas_maniobras.csv'))))
        self.assertEqual(filas[0], rl.ENCABEZADO_MANIOBRAS)
        self.assertEqual(filas[1][2], 'distancia')
        self.assertEqual(filas[1][6], '0.97')
        guardado = json.load(open(os.path.join(self.dir.name, 'calibracion.json')))
        self.assertAlmostEqual(guardado['diametro_m'], esperado, places=6)

    def test_registrar_sin_corregir_no_cambia_nada(self):
        self.c.ajustar_frenado = False
        self.conectar()
        self.c.distancia(-1.0, 0.21)
        self.correr_hasta_resultado()
        self.assertLess(self.c.resultado['calculado'], 0)                          # atras: recorrido con signo
        self.c.registrar(real=1.01, corregir=False)
        self.correr(1.0)
        self.assertEqual([a for a in self.ordenes() if a[0] == 5], [])
        self.assertFalse(self.c.registrar())                                       # ya no queda nada para registrar

    def test_delanteras_en_los_giros_se_manda_se_guarda_y_se_reenvia(self):
        self.conectar()
        self.assertFalse(self.c.config_delanteras(3.0, 0))                          # fuera de rango
        self.assertFalse(self.c.config_delanteras(1.2, 5))
        self.assertTrue(self.c.config_delanteras(1.2, 2))
        self.correr(1.0)
        self.assertEqual([a for a in self.ordenes() if a[0] == 9], [(9, 1.2, 2)] * 3)
        with open(os.path.join(self.dir.name, 'calibracion.json')) as f:
            g = json.load(f)
        self.assertEqual((g['giro_factor_del'], g['kick_del']), (1.2, 2))
        self.robot.enviadas.clear()                                                  # la ESP32 se reinicia y se reconecta
        self.c.conectado = False
        self.c._n_estado = None
        self.correr(1.5)
        self.assertEqual([a for a in self.ordenes() if a[0] == 9], [(9, 1.2, 2)] * 3)

    def _con_sonares(self, f, i, d):
        self.robot.estado = self.robot.estado + [0.0] * (34 - len(self.robot.estado))
        self.robot.estado[26], self.robot.estado[27], self.robot.estado[28] = f, i, d
        self.correr(0.3)

    def _giro_180_auto(self, f, i, d):
        self.c.ajustar_frenado = False
        self.conectar()
        self._con_sonares(f, i, d)
        self.assertTrue(self.c.girar_180_auto(120))
        self.correr_hasta_resultado()
        return [a for a in self.ordenes() if a[0] == 7][-1][1]

    def test_180_auto_gira_hacia_el_lado_contrario_de_la_pared_mas_cercana(self):
        self.assertAlmostEqual(self._giro_180_auto(0.30, 0.08, 0.20), -math.pi, places=5)         # izquierda cerca: a la derecha
        self.assertTrue(any('Mas cerca la pared izquierda: giro a la derecha' in m for m in self.mensajes))

    def test_180_auto_con_la_pared_derecha_mas_cerca_gira_a_la_izquierda(self):
        self.assertAlmostEqual(self._giro_180_auto(0.30, 0.20, 0.08), math.pi, places=5)

    def test_180_auto_un_lado_sin_eco_cuenta_como_libre(self):
        self.assertAlmostEqual(self._giro_180_auto(0.30, -1.0, 0.12), math.pi, places=5)          # izq sin eco, der cerca: a la izq.

    def test_180_auto_informa_las_distancias_al_terminar(self):
        self._giro_180_auto(0.307, 0.123, 0.181)
        despues = [m for m in self.mensajes if 'Despues del giro' in m]
        self.assertEqual(len(despues), 1)
        self.assertIn('pared izquierda 12.3 cm', despues[0])
        self.assertIn('pared derecha 18.1 cm', despues[0])
        self.assertIn('frente 30.7 cm', despues[0])

    def test_180_auto_sin_sonares_o_sin_conexion_no_gira(self):
        self.assertFalse(self.c.girar_180_auto(120))                                              # sin conexion
        self.conectar()
        self.assertFalse(self.c.girar_180_auto(120))                                              # estado de 26 valores: sin sonares
        self.assertEqual([a for a in self.ordenes() if a[0] == 7], [])

    def test_secuencia_recto_giro_recto(self):
        self.c.ajustar_frenado = False
        self.conectar()
        self.assertTrue(self.c.secuenciar([('distancia', 0.40), ('giro', 90.0), ('distancia', 0.20)], 0.21, 120))
        for _ in range(int(120 / 0.05)):
            self.paso()
            if not self.c.secuencia and self.c.maniobra is None:
                break
        cmds = [a for a in self.ordenes() if a[0] in (3, 7)]
        self.assertEqual([a[0] for a in cmds], [3, 7, 3])                          # una a una, en orden, sin repetirse
        self.assertAlmostEqual(cmds[0][1], 0.40)
        self.assertAlmostEqual(cmds[1][1], math.pi / 2, places=5)
        self.assertAlmostEqual(cmds[2][1], 0.20)
        self.assertAlmostEqual(self.c.resultado['calculado'], 0.20, delta=0.02)    # el ultimo resultado es el del 2.o tramo

    def test_parar_corta_la_secuencia(self):
        self.conectar()
        self.c.secuenciar([('distancia', 0.40), ('giro', 90.0), ('distancia', 0.20)], 0.21, 120)
        self.correr(1.0)
        self.c.parar()
        self.correr(10.0)
        self.assertEqual(len(self.c.secuencia), 0)
        self.assertEqual([a[0] for a in self.ordenes() if a[0] in (3, 7)], [3])    # nunca llego al giro

    def test_si_un_paso_no_arranca_se_corta_la_secuencia(self):
        self.conectar()
        self.robot.enviar_cmd = lambda *a: None                                    # la orden se pierde siempre: nunca arranca
        self.c.secuenciar([('distancia', 0.40), ('giro', 90.0)], 0.21, 120)
        self.correr(6.0)
        self.assertEqual(len(self.c.secuencia), 0)
        self.assertTrue(any('Secuencia cortada' in m for m in self.mensajes))

    def test_giro_de_360_a_la_izquierda(self):
        self.c.ajustar_frenado = False
        self.conectar()
        self.assertTrue(self.c.girar(360, 120))
        self.correr_hasta_resultado()
        ordenes7 = [a for a in self.ordenes() if a[0] == 7]
        self.assertAlmostEqual(ordenes7[0][1], 2 * math.pi, places=5)
        self.assertEqual(self.c.resultado['tipo'], 'giro')
        self.assertAlmostEqual(self.c.resultado['calculado'], 360.0, delta=8.0)
        self.assertAlmostEqual(self.c.resultado['odometria'], 360.0, delta=8.0)    # unwrapped: 360, no 0
        self.assertLess(self.c.resultado['desplazamiento_odo_cm'], 1.0)
        t = self.c.resultado['ticks']
        self.assertTrue(t[0] < 0 and t[2] < 0 and t[1] > 0 and t[3] > 0)           # izquierda atras, derecha adelante

    def test_pre_arranque_de_fr_solo_en_el_giro_de_90_y_antes_de_la_orden_7(self):
        self.c.ajustar_frenado = False
        self.conectar()
        self.assertFalse(self.c.config_pre_fr(150, 100))
        self.assertTrue(self.c.config_pre_fr(60, 100))
        self.correr(1.0)
        self.c.girar(90, 120)
        self.correr_hasta_resultado()
        env = [a for a in self.ordenes() if a[0] in (1, 7)]
        pre = [a for a in env if a[0] == 1]
        self.assertTrue(pre and all(a[1:] == (0.0, 0.6, 0.0, 0.0) for a in pre))      # solo FR, hacia adelante (giro a la izquierda)
        self.assertEqual(env[0][0], 1)                                                  # el pre-arranque va primero
        self.assertEqual(env[-1][0], 7)
        self.robot.enviadas.clear()
        self.c.girar(-180, 120)                                                         # el de 180 no lleva pre-arranque
        self.correr_hasta_resultado()
        self.assertEqual([a for a in self.ordenes() if a[0] == 1], [])

    def test_giro_a_la_derecha_y_velocidad_en_flancos(self):
        self.c.ajustar_frenado = False
        self.conectar()
        self.c.girar(-180, 120)
        self.correr_hasta_resultado()
        o = [a for a in self.ordenes() if a[0] == 7][0]
        self.assertAlmostEqual(o[1], -math.pi, places=5)
        # 120 grados/s = 2,094 rad/s x 0,0655 m = 0,1372 m/s por rueda = 26,1 flancos/s (entre 25 y 75: sin acotar)
        self.assertAlmostEqual(o[2], 26.07, delta=0.1)
        self.assertLess(self.c.resultado['calculado'], 0)

    def test_giro_corrige_la_trocha(self):
        self.c.ajustar_frenado = False
        self.conectar()
        self.c.girar(360, 120)
        self.correr_hasta_resultado()
        calc = self.c.resultado['calculado']
        self.c.registrar(real=330.0, desplazamiento_cm=4.0, corregir=True)
        self.correr(1.0)
        self.assertAlmostEqual(self.robot.trocha, 0.131 * abs(calc) / 330.0, places=6)
        self.assertAlmostEqual(self.robot.diam, 0.067, places=9)                    # el diametro no se toca
        filas = list(csv.reader(open(os.path.join(self.dir.name, 'ruedas_maniobras.csv'))))
        self.assertEqual(filas[1][7], '4.0')

    def test_no_se_arranca_con_la_configuracion_a_medio_enviar(self):
        self.conectar()
        self.c._mandar_config(diametro=0.066)
        self.assertFalse(self.c.distancia(1.0, 0.2))
        self.assertTrue(any('todavia se esta enviando' in m for m in self.mensajes))
        self.correr(1.0)
        self.assertTrue(self.c.distancia(1.0, 0.2))

    def test_un_adelanto_de_frenado_de_cero_es_valido(self):
        self.conectar()
        self.c._mandar_config(frenado=0.0)
        self.correr(1.0)
        self.assertEqual(self.c.frenado, 0.0)

    def test_una_maniobra_a_la_vez(self):
        self.conectar()
        self.assertTrue(self.c.girar(90, 120))
        self.assertFalse(self.c.distancia(1.0, 0.2))
        self.assertTrue(any('Ya hay una maniobra' in m for m in self.mensajes))

    def test_parar_cancela_la_maniobra(self):
        self.conectar()
        self.c.distancia(1.0, 0.21)
        self.correr(1.0)
        self.assertIsNotNone(self.c.maniobra)
        self.c.parar()
        self.assertIsNone(self.c.maniobra)
        self.assertIsNone(self.robot.mov)
        self.correr(1.0)
        self.assertIsNone(self.c.resultado)

    def test_marcha_por_cmd_vel(self):
        self.conectar()
        self.assertTrue(self.c.marchar(0.2, 0.0, 3.0))
        self.correr_hasta_resultado()
        velocidades = [a for a in self.ordenes('vel') if a != (0.0, 0.0)]
        self.assertGreater(len(velocidades), 25)                                    # 10 por segundo durante 3 s
        self.assertEqual(velocidades[0], (0.2, 0.0))
        self.assertEqual(self.ordenes('vel')[-1], (0.0, 0.0))                       # y al final frena
        self.assertGreaterEqual(len([a for a in self.ordenes() if a[0] == 4]), 3)
        r = self.c.resultado
        self.assertEqual(r['tipo'], 'marcha')
        self.assertAlmostEqual(r['pedido'], 0.6, places=6)
        self.assertAlmostEqual(r['odometria'], 0.6, delta=0.05)

    def test_si_se_corta_la_conexion_se_interrumpe_la_maniobra(self):
        self.conectar()
        self.c.distancia(1.0, 0.21)
        self.correr(1.0)
        self.c.enlace = type('Mudo', (), {'estado': self.robot.estado, 'n_estado': self.robot.n_estado, 'ticks': self.robot.ticks,
                                          'n_ticks': self.robot.n_ticks, 'enviar_cmd': lambda *a: None,
                                          'enviar_vel': lambda *a: None})()
        for _ in range(60):                                                         # 3 s sin mensajes nuevos
            self.t[0] += 0.05
            self.c.tick()
        self.assertFalse(self.c.conectado)
        self.assertIsNone(self.c.maniobra)
        self.assertTrue(any('sin datos' in m for m in self.mensajes))


class Archivos(Banco):
    def test_se_manda_la_calibracion_guardada_al_conectar(self):
        rl.guardar_cal({'diametro_m': 0.0655, 'trocha_m': 0.18, 'frenado_m': 0.02,
                        'curvas': [{'rueda': 'FL', 'd_min': 0.36, 'puntos': [[0.0, 0.36], [30.0, 0.4]]}]},
                       os.path.join(self.dir.name, 'calibracion.json'))
        self.c = rl.Controlador(self.robot, carpeta=self.dir.name, reloj=lambda: self.t[0], imprimir=self.mensajes.append)
        self.correr(2.0)
        cfg = [a for a in self.ordenes() if a[0] == 5]
        self.assertEqual(len(cfg), 3)
        self.assertEqual(cfg[0][1:4], (0.0655, 0.18, 0.02))
        curvas = [a for a in self.ordenes() if a[0] == 6]
        self.assertEqual(len(curvas), 3)
        self.assertEqual(curvas[0], (6, 0, 0.36, 2, 0.0, 0.36, 30.0, 0.4))
        self.assertAlmostEqual(self.robot.trocha, 0.18)                             # la ESP32 simulada los aplico

    def test_guardar_config_no_borra_las_curvas_de_robot_teclado(self):
        ruta = os.path.join(self.dir.name, 'calibracion.json')
        rl.guardar_cal({'curvas': [{'rueda': 'FL', 'd_min': 0.36, 'puntos': [[0.0, 0.36]]}], 'v_comun': 40.0}, ruta)
        self.c = rl.Controlador(self.robot, carpeta=self.dir.name, reloj=lambda: self.t[0], imprimir=self.mensajes.append)
        self.c._mandar_config(diametro=0.066)
        guardado = json.load(open(ruta))
        self.assertIn('curvas', guardado)
        self.assertEqual(guardado['v_comun'], 40.0)
        self.assertEqual(guardado['diametro_m'], 0.066)

    def test_grabacion_csv(self):
        self.conectar()
        self.c.armado = True
        self.c.objetivo = [30.0] * 4
        self.c.grabar(True)
        self.correr(1.0)
        self.c.grabar(False)
        filas = list(csv.reader(open(self.c.grabacion_ruta)))
        self.assertEqual(filas[0], rl.ENCABEZADO_GRABACION)
        self.assertGreaterEqual(len(filas), 9)                                      # 10 Hz durante ~1 s
        self.assertEqual(len(filas[1]), len(rl.ENCABEZADO_GRABACION))
        self.assertTrue(float(filas[-1][3]) > 0)                                    # pedido_FL


if __name__ == '__main__':
    unittest.main()
