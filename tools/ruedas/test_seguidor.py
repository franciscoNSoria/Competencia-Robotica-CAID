"""Seguidor de flechas reactivo en el simulador de laberinto (sin ROS ni camara: las flechas y el cubo se inyectan a mano)."""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ruedas_logica as rl  # noqa: E402
import seguidor as sg  # noqa: E402
import sim_laberinto as sim  # noqa: E402

# S mira a +x en (0,0). Pasillo recto hasta (6,0) con pared enfrente (x = 6,5 celdas) y brazos arriba y abajo en la celda 6.
T = """\
#########
#######.#
#######.#
#S......#
#######.#
#######.#
#########
"""


def correr(texto, escena=None, t_max=240.0, corr90=10.0, ganancia_giro=0.74, hasta=None, **kw):
    robot = sim.RobotLaberinto(texto, semilla=3, ganancia_giro=ganancia_giro)
    log = []
    c = rl.Controlador(robot, carpeta=tempfile.mkdtemp(), reloj=lambda: 100.0 + robot.t, imprimir=lambda m: None)
    s = sg.Seguidor(c, corr90=corr90, imprimir=log.append, reloj=lambda: 100.0 + robot.t, **kw)
    estados = []
    for _ in range(8):
        robot.avanzar(0.05)
        c.tick()
    s.arrancar()
    while robot.t < t_max and not s.terminado() and not robot.choco and not (hasta and hasta(robot, s)):
        robot.avanzar(0.05)
        c.tick()
        s.tick()
        if escena:
            escena(robot, s)
        if not estados or estados[-1] != s.estado:
            estados.append(s.estado)
    c.cerrar()
    return robot, s, log, estados


def flecha(giro, color='verde', desde=1.2):
    def esc(robot, s):
        if robot.x > desde and s.estado == 'recto':
            s.recibir_flecha({'giro': giro, 'color': color, 'tamano': 'P3'})
    return esc


class TestSeguidor(unittest.TestCase):
    def test_la_flecha_a_la_izquierda_dobla_a_la_izquierda_cuando_el_lateral_ve_el_pasillo(self):
        robot, s, log, estados = correr(T, flecha('izquierda'), t_max=70.0, hasta=lambda r, s: r.y > 0.45)
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertIn('pasillo libre a la izquierda', texto)
        self.assertIn('giro 90° a la izquierda (flecha verde)', texto)
        self.assertGreater(robot.y, 0.3, (robot.x, robot.y))                 # termino metido en el brazo de arriba
        self.assertIn('tras_giro', estados)

    def test_la_flecha_a_la_derecha_dobla_a_la_derecha(self):
        robot, s, log, _ = correr(T, flecha('derecha'), t_max=70.0, hasta=lambda r, s: r.y < -0.45)
        self.assertFalse(robot.choco, '\n'.join(log))
        self.assertLess(robot.y, -0.3)

    def test_sin_flecha_llega_a_la_pared_y_dobla_hacia_el_lado_libre(self):
        robot, s, log, _ = correr(T, None, t_max=70.0, hasta=lambda r, s: abs(r.y) > 0.45)
        self.assertFalse(robot.choco, '\n'.join(log))
        self.assertTrue(any('pared de frente' in m and 'giro 90' in m for m in log), '\n'.join(log))

    def test_el_cubo_frena_a_unos_4cm_espera_la_y_vuelve_y_busca_otro_color(self):
        estado = {'cubo': False}

        def escena(robot, s):
            flecha('izquierda')(robot, s)
            if s.estado == 'recto' and robot.y > 0.45 and not estado['cubo']:
                s.recibir_cubo('verde', (950 * 0.02 / 0.35) ** 2)             # ~35 cm
            if s.estado == 'cubo_fino':
                s.recibir_cubo('verde', (950 * 0.02 / 0.14) ** 2)
            if s.estado == 'esperar_y' and not estado['cubo']:
                estado['cubo'] = True
                estado['y'] = (robot.x, robot.y)
                s.confirmar()
        robot, s, log, estados = correr(T, escena, t_max=400.0, volver=True, hasta=lambda r, s: s.estado in ('reentrar', 'reentrar_listo'))
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertIn('retorno', estados)
        self.assertIn('vuelvo a la entrada: 3 paso(s)', texto)
        self.assertIn('cubo a ~14 cm: avanzo 10 cm y paro a ~4 cm', texto)
        self.assertIn('ESPERANDO la tecla Y', texto)
        self.assertIn('DEJE EL CUBO VERDE', texto)
        self.assertEqual(s.ignora, {'verde'})
        self.assertLess(robot.x, 0.6, (robot.x, robot.y, s.estado))                # volvio a la entrada (x chica, el brazo y el pasillo desandados)
        self.assertLess(abs(robot.y), 0.3)

    def test_tras_el_ok_frente_al_cubo_gira_180_y_cuenta_el_mapa_sin_volver(self):
        estado = {'cubo': False}

        def escena(robot, s):
            flecha('izquierda')(robot, s)
            if s.estado == 'recto' and robot.y > 0.45 and not estado['cubo']:
                s.recibir_cubo('verde', (950 * 0.02 / 0.35) ** 2)
            if s.estado == 'cubo_fino':
                s.recibir_cubo('verde', (950 * 0.02 / 0.14) ** 2)
            if s.estado == 'esperar_y' and not estado['cubo']:
                estado['cubo'] = True
                estado['rumbo'] = robot.th
                s.confirmar()
        robot, s, log, estados = correr(T, escena, t_max=400.0)
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertEqual(s.estado, 'fin', texto)
        self.assertIn('MAPA: asi llegue al cubo VERDE', texto)
        self.assertIn('giro 90° IZQ', texto)
        self.assertIn('Posicion segun la odometria, respecto de la entrada', texto)
        self.assertNotIn('retorno', estados)                                    # no vuelve solo
        self.assertGreater(abs(math.remainder(robot.th - estado['rumbo'], 2 * math.pi)), math.radians(140))   # dio la media vuelta

    def test_ignora_las_flechas_del_color_entregado(self):
        s = sg.Seguidor.__new__(sg.Seguidor)
        robot = sim.RobotLaberinto(T)
        c = rl.Controlador(robot, carpeta=tempfile.mkdtemp(), reloj=lambda: 100.0, imprimir=lambda m: None)
        s.__init__(c, imprimir=lambda m: None, reloj=lambda: 100.0)
        s.ignora = {'verde'}
        s.recibir_flecha({'giro': 'izquierda', 'color': 'verde', 'tamano': 'P3'})
        self.assertIsNone(s.flecha)
        s.recibir_flecha({'giro': 'derecha', 'color': 'rojo', 'tamano': 'P3'})
        self.assertEqual(s.busca, 'rojo')
        self.assertEqual(s.flecha['giro'], 'derecha')

    def test_el_camino_de_vuelta_cancela_las_idas_y_vueltas_a_un_callejon(self):
        log = [('recto', 1.0), ('giro', 1), ('recto', 0.8), ('media',), ('recto', 0.8), ('giro', -1), ('recto', 0.5)]
        self.assertEqual(sg.Seguidor.compactar(log), [('recto', 1.0), ('giro', 1), ('giro', -1), ('recto', 0.5)])


if __name__ == '__main__':
    unittest.main()
