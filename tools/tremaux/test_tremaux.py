"""Pruebas de tremaux_logica / tremaux (sin ROS ni robot): python3 -m pytest tools/tremaux/test_tremaux.py -q
(o python3 tools/tremaux/test_tremaux.py)."""
import math
import os
import sys
import tempfile
import unittest

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import banco  # noqa: E402
import laberintos  # noqa: E402
import torneo_logica as tl  # noqa: E402
import tremaux_logica as tx  # noqa: E402


def piloto():
    return tx.PilotoTremaux(None, imprimir=lambda m: None)


class Logica(unittest.TestCase):
    def test_el_giro_convierte_el_off_y_el_lateral(self):
        p = piloto()
        p._off, p.obs.x[0] = 0.05, 0.02            # 5 cm pasado del centro, 2 cm a la izquierda
        tele = dict(mov_seq=0)
        p._girar_hacia(0.0, tele, 1, math.pi / 2, primera=True)      # giro a la izquierda
        self.assertAlmostEqual(p._off, 0.02)        # lo que estaba a la izquierda ahora es a lo largo del rumbo
        self.assertAlmostEqual(p._y_virtual, -0.05)  # y lo que estaba adelante ahora queda a la derecha
        p = piloto()
        p._off, p.obs.x[0] = 0.05, 0.02
        p._girar_hacia(0.0, tele, 3, -math.pi / 2, primera=True)     # giro a la derecha
        self.assertAlmostEqual(p._off, -0.02)
        self.assertAlmostEqual(p._y_virtual, 0.05)

    def test_inclinado_solo_si_el_mapa_dice_que_hay_paso(self):
        p = piloto()
        p.mapa.marcar_lado(p.celda, p.rumbo, True)
        self.assertTrue(p._sospecha_inclinado(0.25))
        self.assertFalse(p._sospecha_inclinado(0.60))            # libre de verdad
        self.assertFalse(p._sospecha_inclinado(0.07))            # pared pegada
        self.assertFalse(p._sospecha_inclinado(None))
        p.mapa.bloquear(p.celda, p.rumbo)                        # el mapa dice pared: no hay nada que comprobar
        self.assertFalse(p._sospecha_inclinado(0.25))

    def test_los_giros_grandes_llevan_la_velocidad_y_los_chicos_no(self):
        p = piloto()
        self.assertEqual(p._con_velocidad_de_giro((0.0, 0.0, (7, math.pi / 2, 0.0))), (0.0, 0.0, (7, math.pi / 2, 75.0)))
        self.assertEqual(p._con_velocidad_de_giro((0.0, 0.0, (7, -math.pi / 2, 0.0)))[2][2], 75.0)
        self.assertEqual(p._con_velocidad_de_giro((0.0, 0.0, (7, 0.21, 0.0)))[2], (7, 0.21, 0.0))   # paso chico: la del firmware
        self.assertEqual(p._con_velocidad_de_giro((0.0, 0.0, (3, 0.2, 0.0)))[2], (3, 0.2, 0.0))     # avances: no se tocan
        self.assertEqual(p._con_velocidad_de_giro((0.2, 0.0, None)), (0.2, 0.0, None))

    def test_no_arranca_solo_ni_manda_orden(self):
        p = piloto()
        tele = dict(frente=0.5, izq=0.12, der=0.12, guardia=1, mov_activo=False, mov_seq=0, quieto=True, fresco=True)
        self.assertEqual(p.paso(0.0, tele, 0.0, 0.0), (0.0, 0.0, None))

    def test_los_topes_del_filtro_siguen_al_mapa(self):
        p = piloto()
        p.estado = tl.MEDIR
        self.assertEqual(p._topes(), {'izq': p.p.d_kalman_nuevo, 'der': p.p.d_kalman_nuevo})
        p.mapa.marcar_lado(p.celda, 1, True)                     # paso a la izquierda
        p.mapa.marcar_lado(p.celda, 3, False)                    # pared a la derecha
        self.assertEqual(p._topes(), {'izq': 0.0, 'der': p.p.d_kalman})


class Simulacion(unittest.TestCase):
    """El simulador de tools/torneo con el modelo del robot real (ver ahí qué se modela). Semillas fijas: es determinista."""

    def corrida(self, nombre, semilla):
        return banco.correr(laberintos.todos()[nombre], semilla, tx.PilotoTremaux)

    def test_laberinto_chico_recorre_todo_y_vuelve(self):
        s = self.corrida('casa', 1)
        self.assertIsNone(s.choque)
        self.assertEqual(s.piloto.estado, tl.FIN)
        self.assertEqual(s.piloto.celda, (0, 0))

    def test_nunca_pasa_de_largo_una_pared_sin_chocar_en_casa(self):
        for semilla in (2, 3, 4):
            s = self.corrida('casa', semilla)
            self.assertIsNone(s.choque, 'semilla %d: %s' % (semilla, s.choque))


if __name__ == '__main__':
    unittest.main()
