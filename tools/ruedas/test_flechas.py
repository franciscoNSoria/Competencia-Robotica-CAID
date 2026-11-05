"""Flechas y cubo en el simulador de laberinto (sin ROS): la flecha manda el proximo giro y frena al llegar al cubo."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_laberinto as sim  # noqa: E402

# T: S mira a +x, en (2,0) hay pared enfrente y paso arriba (+y, izquierda) y abajo (-y, derecha).
# Tremaux sin flecha prefiere la derecha (abajo); el cubo esta arriba.
T = """\
#####
###.#
#S..#
###.#
#####
"""


def camara(giro, color='verde', tam='P3', cubo_en=None, otras=()):
    """cada_tick: manda la flecha mientras el robot se acerca a la pared de (2,0); el cubo al estar en cubo_en."""
    def tick(robot, rec):
        celda = robot.celda()
        if celda in ((1, 0), (2, 0)) and rec.rumbo == 0:
            rec.recibir_flecha({'giro': giro, 'color': color, 'tamano': tam})   # la vision manda primero la P3
            for c in otras:                               # despues, flechas de otros colores a la vista
                rec.recibir_flecha({'giro': 'derecha', 'color': c, 'tamano': 'P1'})
        if cubo_en is not None and celda == cubo_en[0]:
            rec.recibir_cubo(cubo_en[1], 20000.0)         # ~13 cm: llegue
    return tick


def celdas(log):
    return [ln for ln in log if 'celda (' in ln]


class TestFlechas(unittest.TestCase):
    def test_sin_flecha_tremaux_va_a_la_derecha(self):
        _, rec, log = sim.correr(T, t_max=60.0, cada_tick=camara(None))
        self.assertTrue(any('celda (2, -1)' in ln for ln in celdas(log)), '\n'.join(log))

    def test_la_flecha_manda_el_giro(self):
        _, rec, log = sim.correr(T, t_max=60.0, cada_tick=camara('izquierda', cubo_en=((2, 1), 'verde')))
        texto = '\n'.join(log)
        self.assertIn('sigo la flecha verde: giro a la izquierda', texto)
        self.assertEqual(rec.estado, 'fin', texto)
        self.assertIn('LLEGUE AL CUBO VERDE', texto)
        self.assertFalse(any('celda (2, -1)' in ln for ln in celdas(log)), texto)

    def test_sigue_el_primer_color_e_ignora_las_otras(self):
        _, rec, log = sim.correr(T, t_max=60.0, cada_tick=camara('izquierda', color='azul', otras=('rojo', 'verde'),
                                                                cubo_en=((2, 1), 'verde')))
        self.assertNotIn('LLEGUE AL CUBO', '\n'.join(log))     # el verde no es el suyo
        self.assertEqual(rec.busca, 'azul')
        self.assertIn('sigo la flecha azul', '\n'.join(log))

    def test_cubo_de_otro_color_no_lo_frena(self):
        _, rec, log = sim.correr(T, t_max=40.0, cada_tick=camara('izquierda', cubo_en=((2, 1), 'rojo')))
        self.assertNotIn('LLEGUE AL CUBO', '\n'.join(log))

    def test_flecha_de_mas_adelante_no_se_gasta(self):
        # en (1,0) los costados son pared y el frente libre: la flecha es para (2,0), no se ignora
        _, rec, log = sim.correr(T, t_max=60.0, cada_tick=camara('izquierda', cubo_en=((2, 1), 'verde')))
        self.assertNotIn('ignorada', '\n'.join(log))


if __name__ == '__main__':
    unittest.main()
