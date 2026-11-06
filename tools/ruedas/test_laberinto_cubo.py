"""Laberinto (Trémaux por celdas) con el cubo: frena cerca, espera el OK, gira 180 y cuenta el camino (el cubo se inyecta a mano)."""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import laberinto_ruedas as lr  # noqa: E402
import ruedas_logica as rl  # noqa: E402
import sim_laberinto as sim  # noqa: E402

T = """\
#########
#######.#
#######.#
#S......#
#########
"""


def area(dist_m):
    return (lr.FX_CAMARA_PX * lr.CUBO_LADO_M / dist_m) ** 2


class TestLaberintoCubo(unittest.TestCase):
    def armar(self):
        robot = sim.RobotLaberinto(T, semilla=3, ganancia_giro=0.74)
        log = []
        c = rl.Controlador(robot, carpeta=tempfile.mkdtemp(), reloj=lambda: 100.0 + robot.t, imprimir=lambda m: None)
        rec = lr.Recorrido(c, corr90=10.0, imprimir=log.append, reloj=lambda: 100.0 + robot.t)
        for _ in range(8):
            robot.avanzar(0.05)
            c.tick()
        self.addCleanup(c.cerrar)
        return robot, c, rec, log

    def _correr(self, rec, robot, c, h):
        while robot.t < 900.0 and not rec.terminado() and not robot.choco:
            robot.avanzar(0.05)
            c.tick()
            rec.tick()
            if robot.y > 0.2 and not h['cubo'] and rec.estado not in ('esperar_y',):     # el cubo pegado a la pared del fondo (y = 1,0 m)
                rec.recibir_cubo('verde', area(max(0.03, 1.0 - (robot.y + 0.14))))
            if rec.estado == 'esperar_y' and not h['ok1']:
                h['ok1'], h['xy'] = True, (robot.x, robot.y)
                h['cubo'] = True
                for _ in range(40):                                          # espera quieto: no se mueve solo
                    robot.avanzar(0.05)
                    c.tick()
                    rec.tick()
                self.assertEqual((round(robot.x, 3), round(robot.y, 3)), (round(h['xy'][0], 3), round(h['xy'][1], 3)))
                rec.confirmar()
            if rec.estado == 'esperar_canasto' and not h['ok2']:
                h['ok2'], h['canasto'] = True, (robot.x, robot.y)
                for _ in range(40):                                          # en la partida tambien espera quieto
                    robot.avanzar(0.05)
                    c.tick()
                    rec.tick()
                self.assertEqual(rec.estado, 'esperar_canasto')
                rec.confirmar()
            if rec.estado == 'pausa_reentrar' and not h['ok3']:
                h['ok3'], h['pausa'] = True, (robot.x, robot.y, robot.th)
                for _ in range(40):                                          # la pausa es quieta
                    robot.avanzar(0.05)
                    c.tick()
                    rec.tick()
                self.assertEqual(rec.estado, 'pausa_reentrar')
                rec.confirmar()
            if h['ok3'] and rec.estado == 'medir' and 'reentro' not in h:
                h['reentro'] = (robot.x, robot.y, robot.th)
            if 'reentro' in h and rec.estado not in ('fin', 'detenido'):                 # al volver a entrar, el cubo verde sigue ahi: se ignora
                rec.recibir_cubo('verde', area(0.2))

    def test_cubo_cerca_frena_espera_ok_gira_180_vuelve_a_la_partida_pausa_y_vuelve_a_entrar_ignorando_el_color(self):
        robot, c, rec, log = self.armar()
        avisos = []
        rec.al_cambiar_estado = avisos.append
        rec.arrancar()
        h = {'cubo': False, 'ok1': False, 'ok2': False, 'ok3': False}
        self._correr(rec, robot, c, h)
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertTrue(h['ok1'] and h['ok2'] and h['ok3'], texto)
        self.assertIn('MAPA: asi llegue al cubo VERDE', texto)
        self.assertIn('vuelvo a la partida (0, 0)', texto)
        self.assertIn('LLEGUE A LA PARTIDA (0, 0)', texto)
        self.assertIn('cubo VERDE entregado en el canasto', texto)
        self.assertIn('PAUSA: cubo VERDE entregado', texto)
        self.assertEqual(avisos, [['verde']])                                     # la vision se entera de que el verde ya salio
        self.assertEqual(rec.ignora, {'verde'})
        self.assertLess(abs(h['canasto'][1]), 0.15, h['canasto'])                 # el canasto esta en la celda de partida
        self.assertLess(h['canasto'][0], 0.30, h['canasto'])
        self.assertIn('vuelvo a entrar', texto)
        self.assertGreater(abs(math.remainder(h['reentro'][2] - h['pausa'][2], 2 * math.pi)), math.radians(140))   # dio la media vuelta
        self.assertEqual(texto.count('dejo de explorar'), 1, texto)                  # el verde no se persigue de nuevo (se ignora)
        self.assertEqual(rec.estado, 'fin', texto)

    def test_si_se_pierde_el_camino_a_la_partida_se_pausa_y_con_el_ok_vuelve_a_entrar_sin_dar_el_cubo_por_entregado(self):
        robot, c, rec, log = self.armar()
        rec.arrancar()
        rec.mapa.camino = lambda *a, **k: None                                    # el mapa no sabe volver
        h = {'cubo': False, 'ok1': False, 'ok2': True, 'ok3': False}
        self._correr(rec, robot, c, h)
        texto = '\n'.join(log)
        self.assertTrue(h['ok1'] and h['ok3'], texto)
        self.assertIn('se perdio el camino', texto)
        self.assertEqual(rec.ignora, set())                                       # no se entrego: no se ignora nada


T2 = """\
#########
#S..#####
###.#####
###.....#
#########
"""


class TestCuentaGiros(unittest.TestCase):
    def test_vuelve_a_ciegas_repitiendo_al_reves_los_giros_anotados_sin_usar_el_mapa(self):
        robot = sim.RobotLaberinto(T2, semilla=3, ganancia_giro=0.74)
        log = []
        c = rl.Controlador(robot, carpeta=tempfile.mkdtemp(), reloj=lambda: 100.0 + robot.t, imprimir=lambda m: None)
        rec = lr.Recorrido(c, corr90=10.0, imprimir=log.append, reloj=lambda: 100.0 + robot.t, cuenta_giros=True)
        for _ in range(8):
            robot.avanzar(0.05)
            c.tick()
        self.addCleanup(c.cerrar)
        rec.arrancar()
        h = {'ok1': False}
        decidir = rec.mapa.decidir

        def sin_mapa(*a, **k):
            if rec.volviendo:
                raise AssertionError('la vuelta no tiene que consultar el mapa')
            return decidir(*a, **k)
        rec.mapa.decidir = sin_mapa
        while robot.t < 900.0 and rec.estado != 'esperar_canasto' and not rec.terminado() and not robot.choco:
            robot.avanzar(0.05)
            c.tick()
            rec.tick()
            if robot.y < -0.7 and robot.x > 0.9 and not rec.volviendo and rec.estado not in ('esperar_y', 'media_cubo', 'media_cubo_fin'):
                rec.recibir_cubo('verde', area(max(0.03, 2.6 - (robot.x + 0.14))))
            if rec.estado == 'esperar_y' and not h['ok1']:
                h['ok1'] = True
                rec.confirmar()
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertEqual(rec.estado, 'esperar_canasto', texto)
        self.assertIn('VUELVO A CIEGAS repitiendo al reves', texto)
        self.assertIn('giro 90° DER', texto)                                       # ida: derecha y despues izquierda; vuelta: al reves e invertidos
        self.assertIn('giro 90° IZQ', texto)
        self.assertLess(abs(robot.x), 0.35, (robot.x, robot.y))                    # volvio a la partida
        self.assertLess(abs(robot.y), 0.2, (robot.x, robot.y))
        self.assertEqual(rec.celda, (0, 0))


if __name__ == '__main__':
    unittest.main()
