"""Pruebas de caid_vision.ruta con escenas dibujadas (piso gris, paredes claras)."""
import cv2
import numpy as np

from caid_vision.ruta import analizar_ruta

PISO = (110, 110, 110)
PARED = (200, 215, 225)


def _escena(poligono_piso, ruido=3.0, semilla=0):
    img = np.full((480, 640, 3), PARED, np.uint8)
    cv2.fillPoly(img, [np.array(poligono_piso, np.int32)], PISO)
    rng = np.random.default_rng(semilla)
    img = np.clip(img + rng.normal(0, ruido, img.shape), 0, 255).astype(np.uint8)
    return img


def test_pasillo_recto_rumbo_cercano_a_cero():
    r = analizar_ruta(_escena([(0, 480), (640, 480), (400, 150), (240, 150)]))
    assert r['estado'] == 'libre'
    assert abs(r['rumbo_deg']) < 8


def test_pasillo_que_dobla_a_la_derecha():
    poligono = [(0, 480), (640, 480), (640, 120), (520, 120), (330, 300), (200, 300)]
    r = analizar_ruta(_escena(poligono))
    assert r['estado'] == 'libre'
    assert r['rumbo_deg'] > 10


def test_pasillo_que_dobla_a_la_izquierda():
    r = analizar_ruta(_escena([(0, 480), (640, 480), (440, 300), (310, 300), (120, 120), (0, 120)]))
    assert r['estado'] == 'libre'
    assert r['rumbo_deg'] < -10


def test_pared_de_frente_esta_bloqueado():
    r = analizar_ruta(_escena([(0, 480), (640, 480), (640, 420), (0, 420)]))
    assert r['estado'] == 'bloqueado'
    assert r['rumbo_deg'] is None


def test_imagen_negra_sin_piso_no_rompe():
    r = analizar_ruta(np.zeros((480, 640, 3), np.uint8))
    assert r['estado'] in ('sin_piso', 'libre', 'bloqueado')
