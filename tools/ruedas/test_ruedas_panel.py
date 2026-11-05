"""Pruebas de la ventana ruedas_panel.py con el robot simulado de test_ruedas_logica.py (sin ROS ni ESP32).

Necesitan tkinter y una pantalla:   xvfb-run -a python3 -m unittest tools/ruedas/test_ruedas_panel.py -v
Sin tkinter o sin pantalla se saltean solas.
"""
import json
import math
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ruedas_logica as rl  # noqa: E402
from test_ruedas_logica import RobotFalso  # noqa: E402

try:
    import tkinter as tk
    _RAIZ = tk.Tk()
    _RAIZ.destroy()
    import ruedas_panel as rp
    HAY_PANTALLA = True
except Exception:                                    # sin tkinter o sin pantalla
    tk = rp = None
    HAY_PANTALLA = False


class RobotEnTiempoDeVentana(RobotFalso):
    """El robot simulado avanza 50 ms cada vez que la ventana atiende al enlace: la ventana corre con periodo de 1 ms real y el
    robot vive 50 veces mas rapido que el tiempo real."""

    def atender(self):
        self.avanzar(0.05)


def bombear(raiz, ms=300):
    fin = time.monotonic() + ms / 1000.0
    while time.monotonic() < fin:
        raiz.update()
        time.sleep(0.002)


@unittest.skipUnless(HAY_PANTALLA, 'sin tkinter o sin pantalla (xvfb-run -a ...)')
class Ventana(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.raiz = tk.Tk()
        self.robot = RobotEnTiempoDeVentana()
        self.panel = rp.Panel(self.raiz, self.robot, carpeta=self.dir.name, reloj=lambda: 100.0 + self.robot.t, periodo_ms=1)

    def tearDown(self):
        self.panel.detener()
        self.panel.c.cerrar()
        self.raiz.destroy()
        self.dir.cleanup()

    def ordenes(self, ident=None):
        return [a for t, a in self.robot.enviadas if t == 'cmd' and (ident is None or a[0] == ident)]

    def esperar(self, condicion, limite_s=20.0):
        fin = time.monotonic() + limite_s
        while time.monotonic() < fin:
            self.raiz.update()
            if condicion():
                return
            time.sleep(0.001)
        self.fail('no se cumplio la condicion a tiempo')

    def conectar(self):
        self.esperar(lambda: self.panel.c.conectado)

    def test_sin_datos_dice_que_espera(self):
        self.panel.detener()
        mudo = type('Mudo', (), {'estado': None, 'n_estado': 0, 'ticks': None, 'n_ticks': 0, 'atender': lambda s: None,
                                 'enviar_cmd': lambda s, *a: None, 'enviar_vel': lambda s, *a: None})()
        panel2 = rp.Panel(tk.Toplevel(self.raiz), mudo, carpeta=self.dir.name, periodo_ms=1)
        bombear(self.raiz, 100)
        self.assertIn('Esperando a la ESP32', panel2.estado.cget('text'))
        panel2.detener()

    def test_conexion_y_tabla(self):
        self.conectar()
        bombear(self.raiz, 100)
        self.assertIn('ESP32 conectada', self.panel.estado.cget('text'))
        self.assertIn('67.00 mm', self.panel.estado.cget('text'))
        self.assertEqual(self.panel.celdas[0][0].cget('text'), 'Delantera izq. (FL)')

    def test_las_barras_solo_mueven_con_armado(self):
        self.conectar()
        self.panel.barra[0].set(40)
        self.panel._movio(0)
        bombear(self.raiz, 200)
        self.assertEqual(self.ordenes(1), [])                                       # sin ARMADO no sale nada
        self.panel.tanque.set(False)                                                 # cada barra por su lado
        self.panel.armado.set(True)
        self.panel._armo()
        self.panel.barra[0].set(40)
        self.panel.barra[3].set(-20)
        self.panel._movio(0)
        self.esperar(lambda: self.ordenes(1) and self.ordenes(1)[-1][1] == 0.4)
        self.assertEqual(list(self.ordenes(1)[-1][1:]), [0.4, 0.0, 0.0, -0.2])
        self.assertEqual(self.panel.valor_barra[0].cget('text'), '+40 %')
        self.panel.armado.set(False)                                                 # desarmar frena
        self.panel._armo()
        self.assertGreaterEqual(len(self.ordenes(4)), 3)
        self.assertEqual([b.get() for b in self.panel.barra], [0.0] * 4)

    def test_tanque_mueve_juntas_las_ruedas_de_cada_lado(self):
        self.conectar()
        self.assertTrue(self.panel.tanque.get())                                    # viene tildado
        self.panel.armado.set(True)
        self.panel._armo()
        self.panel.barra[2].set(30)                                                 # RL arrastra a FL
        self.panel._movio(2)
        self.panel.barra[1].set(-50)                                                # FR arrastra a RR
        self.panel._movio(1)
        self.assertEqual([b.get() for b in self.panel.barra], [30.0, -50.0, 30.0, -50.0])
        self.assertEqual(self.panel.c.objetivo, [30.0, -50.0, 30.0, -50.0])

    def test_giro_de_90_manda_la_orden_7_a_la_izquierda_o_derecha(self):
        self.conectar()
        self.panel.sentido.set(-1)
        self.panel._giro(90.0)
        self.esperar(lambda: self.ordenes(7))
        self.assertAlmostEqual(self.ordenes(7)[0][1], -math.pi / 2, places=5)

    def test_sumador_del_giro_de_90(self):
        self.conectar()
        self.assertEqual(self.panel.v_giro.get(), '153')                            # 0,40 m/s por lado
        for _ in range(3):
            self.panel._ajuste90(+1.0)
        self.assertEqual(self.panel.corr90.get(), '3')
        self.assertIn('93', self.panel.txt90.get())
        with open(os.path.join(self.dir.name, 'calibracion.json')) as f:
            self.assertEqual(json.load(f)['giro90_corr_deg'], 3.0)
        self.panel.sentido.set(1)
        self.panel._giro90()
        self.esperar(lambda: self.ordenes(7))
        self.assertAlmostEqual(self.ordenes(7)[0][1], math.radians(93.0), places=5)
        c = self.panel.c                                                            # 153 °/s con la trocha que informa la ESP32
        esperado = rl.limitar(rl.flancos_de_ms(math.radians(153) * c.trocha / 2, c.diametro), 25.0, 75.0)
        self.assertAlmostEqual(self.ordenes(7)[0][2], esperado, places=3)

    def test_botones_con_sonares(self):
        self.conectar()
        self.robot.estado = self.robot.estado + [0.0] * (34 - len(self.robot.estado))
        self.robot.estado[26], self.robot.estado[27], self.robot.estado[28] = 0.5, 0.1185, 0.1185
        self.esperar(lambda: len(self.panel.c.estado) >= 34)                          # el panel ya recibio el estado con sonares
        self.panel._ajuste180(+1.0)
        self.assertIn('+1', self.panel.txt180.get())
        with open(os.path.join(self.dir.name, 'calibracion.json')) as f:
            self.assertEqual(json.load(f)['giro180_corr_deg'], 1.0)
        self.panel._enderezar()
        self.esperar(lambda: self.panel.c.guiado is not None)
        self.assertEqual(self.panel.c.guiado['tipo'], 'enderezar')
        self.panel._parar()
        self.assertIsNone(self.panel.c.guiado)
        self.panel._giro180_pared()
        self.assertEqual(self.panel.c.guiado['tipo'], 'arco')
        g = self.panel.c.guiado
        self.assertAlmostEqual(g['objetivo'], math.radians(181) - abs(g['w']) * rl.TAU_ARCO_S, delta=0.002)
        self.assertAlmostEqual(self.panel.c.guiado['despues'], 0.20)                 # viene tildado: despues avanza 20 cm
        self.panel._parar()

    def test_boton_de_secuencia(self):
        self.conectar()
        self.panel.sentido.set(-1)                                                  # giro a la derecha
        self.panel._secuencia()
        self.esperar(lambda: not self.panel.c.secuencia and self.panel.c.maniobra is None and self.panel.c.resultado is not None,
                     limite_s=60)
        o = [a for a in self.ordenes() if a[0] in (3, 7)]
        self.assertEqual([a[0] for a in o], [3, 7, 3])
        self.assertAlmostEqual(o[0][1], 0.40)
        self.assertAlmostEqual(o[1][1], -math.pi / 2, places=5)
        self.assertAlmostEqual(o[2][1], 0.20)

    def test_cambiar_de_modo_cambia_el_rango_y_pone_en_cero(self):
        self.panel.barra[1].set(70)
        self.panel.modo.set('vel')
        self.panel._cambio_modo()
        self.assertEqual(self.panel.barra[1].get(), 0.0)
        self.assertAlmostEqual(float(self.panel.deslizador[1].cget('to')), 0.40)
        self.assertEqual(self.panel.c.modo, 'vel')
        self.panel.modo.set('duty')
        self.panel._cambio_modo()
        self.assertEqual(float(self.panel.deslizador[1].cget('to')), 100.0)

    def test_enlazar_mueve_las_cuatro(self):
        self.panel.enlazar.set(True)
        self.panel.deslizador[2].set(55)                  # como si se arrastrara la barra de RL
        self.raiz.update()
        self.assertEqual([b.get() for b in self.panel.barra], [55.0] * 4)

    def test_un_metro_adelante_y_registro_con_correccion(self):
        self.robot.sobre_m = 0.0
        self.conectar()
        self.panel.ajustar.set(False)
        self.panel._act_ajuste()
        self.panel._distancia(+1.0)
        self.assertIsNotNone(self.panel.c.maniobra)
        self.esperar(lambda: self.panel.c.resultado is not None)
        bombear(self.raiz, 100)
        self.assertIn('Terminada: 1 m adelante', self.panel.resultado.cget('text'))
        self.panel.real.set('0.97')
        self.panel._registrar(True)
        self.esperar(lambda: self.ordenes(5))
        bombear(self.raiz, 100)
        self.assertLess(self.robot.diam, 0.067)                                       # avanzo menos de lo calculado: rueda mas chica
        self.assertEqual(self.panel.real.get(), '')
        self.assertTrue(os.path.exists(os.path.join(self.dir.name, 'ruedas_maniobras.csv')))

    def test_giro_de_360_hacia_la_derecha(self):
        self.conectar()
        self.panel.sentido.set(-1)
        self.panel._giro(360.0)
        self.esperar(lambda: self.panel.c.resultado is not None)
        self.assertLess(self.panel.c.resultado['calculado'], -340)
        bombear(self.raiz, 100)
        self.assertIn('Pedido -360.0°', self.panel.resultado.cget('text'))
        self.assertIn('lo que giro de verdad', self.panel.unidad_real.cget('text'))

    def test_entradas_invalidas_no_arrancan_nada(self):
        self.conectar()
        self.panel.v_marcha.set('rapido')
        self.panel._distancia(1.0)
        self.panel.v_marcha.set('0.21')
        self.panel.v_giro.set('5000')
        self.panel._giro(90.0)
        self.panel.m_t.set('0')
        self.panel._marchar()
        self.assertIsNone(self.panel.c.maniobra)
        self.assertIsNone(self.panel.c.marcha)
        self.assertEqual(self.ordenes(3) + self.ordenes(7), [])
        texto = self.panel.texto_log.get('1.0', 'end')
        self.assertIn('no es un numero', texto)
        self.assertIn('tiene que estar entre', texto)

    def test_parar_con_escape_cancela_la_maniobra(self):
        self.conectar()
        self.panel._distancia(1.0)
        self.esperar(lambda: self.panel.c.maniobra is not None and self.panel.c.maniobra['activa'])
        self.raiz.focus_force()
        self.raiz.event_generate('<Escape>')
        self.raiz.update()
        self.assertIsNone(self.panel.c.maniobra)
        self.assertIsNone(self.robot.mov)

    def test_marcha_y_grabacion(self):
        self.conectar()
        self.panel.grabar.set(True)
        self.panel._grabar()
        self.panel.m_t.set('1.0')
        self.panel._marchar()
        self.esperar(lambda: self.panel.c.resultado is not None)
        self.panel.grabar.set(False)
        self.panel._grabar()
        self.assertTrue(os.path.exists(self.panel.c.grabacion_ruta))

    def test_odometria_en_cero(self):
        self.conectar()
        self.robot.enviar_vel(0.2, 0.0)
        bombear(self.raiz, 200)
        self.assertGreater(self.panel.c.odo.s, 0.01)
        self.panel._cero()
        self.assertEqual(self.panel.c.odo.s, 0.0)


if __name__ == '__main__':
    unittest.main()
