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

    def test_cubo_cerca_frena_espera_ok_gira_180_vuelve_a_la_partida_y_espera_otro_ok(self):
        robot, c, rec, log = self.armar()
        rec.arrancar()
        h = {'cubo': False, 'ok1': False, 'ok2': False}
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
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertTrue(h['ok1'] and h['ok2'], texto)
        self.assertEqual(rec.estado, 'fin', texto)
        self.assertIn('MAPA: asi llegue al cubo VERDE', texto)
        self.assertIn('vuelvo a la partida (0, 0)', texto)
        self.assertIn('LLEGUE A LA PARTIDA (0, 0)', texto)
        self.assertIn('cubo VERDE entregado en el canasto', texto)
        self.assertLess(abs(h['canasto'][1]), 0.15, h['canasto'])             # el canasto esta en la celda de partida
        self.assertLess(h['canasto'][0], 0.30, h['canasto'])


if __name__ == '__main__':
    unittest.main()
