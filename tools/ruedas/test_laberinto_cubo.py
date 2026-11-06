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

    def test_cubo_cerca_frena_espera_ok_gira_180_y_cuenta_el_camino(self):
        robot, c, rec, log = self.armar()
        rec.arrancar()
        hecho = {'ok': False}
        while robot.t < 300.0 and not rec.terminado() and not robot.choco:
            robot.avanzar(0.05)
            c.tick()
            rec.tick()
            if robot.x > 0.5 and not hecho['ok']:
                rec.recibir_cubo('verde', area(max(0.14, 1.2 - robot.x - 0.0)) if rec.estado != 'cubo_fino' else area(0.14))
            if rec.estado == 'esperar_y' and not hecho['ok']:
                hecho['ok'] = True
                hecho['th'] = robot.th
                for _ in range(40):                                          # espera quieto: no se mueve solo
                    robot.avanzar(0.05)
                    c.tick()
                    rec.tick()
                hecho['x'] = robot.x
                rec.confirmar()
        texto = '\n'.join(log)
        self.assertFalse(robot.choco, texto)
        self.assertTrue(hecho['ok'], texto)
        self.assertEqual(rec.estado, 'fin', texto)
        self.assertIn('ESPERANDO el OK', texto)
        self.assertIn('MAPA: asi llegue al cubo VERDE', texto)
        self.assertIn('celda (0, 0) mirando +x', texto)
        self.assertGreater(abs(math.remainder(robot.th - hecho['th'], 2 * math.pi)), math.radians(140))   # dio la media vuelta
        self.assertLess(abs(robot.x - hecho['x']), 0.12)                     # y no se fue a otro lado


if __name__ == '__main__':
    unittest.main()
