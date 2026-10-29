"""
Pruebas del detector con imágenes dibujadas por el simulador 2D de caid_navigation.

Se compara con las detecciones "perfectas" (el oráculo proyecta la geometría real del mundo).
Si caid_navigation no está disponible, se saltean.
"""
import math

import pytest

sim = pytest.importorskip('caid_navigation.simulacion.render')

from caid_navigation.camara import Camara  # noqa: E402
from caid_navigation.simulacion import oraculo  # noqa: E402
from caid_navigation.simulacion.mundo import Mundo  # noqa: E402
from caid_navigation.simulacion.robot import RobotSim  # noqa: E402
from caid_vision.detector import detectar  # noqa: E402


def _escena(nombre, dx=0.0, dy=0.0, pan=0.0, dyaw=0.0):
    m = Mundo(nombre)
    cam = Camara()
    x, y, th = m.inicio
    pose = (x + dx, y + dy, th + dyaw)
    img = sim.Render(m, cam, ruido=3.0).imagen(pose, pan)
    return detectar(img), oraculo.detecciones(m, RobotSim(m, ruido=False), cam, pose, pan)


def test_flechas_de_la_T_de_e2():
    det, verdad = _escena('e2_bifurcaciones', dx=0.96)
    assert {f['color'] for f in det['flechas']} == {f['color'] for f in verdad['flechas']}
    for f in det['flechas']:
        v = next(g for g in verdad['flechas'] if g['color'] == f['color'])
        for clave in ('punta', 'cuello', 'cola'):
            assert math.dist(f[clave], v[clave]) < 6, (f['color'], clave)


def test_canastos_en_la_partida():
    # Mirando hacia atrás desde la partida (robot girado 90° y cámara 90°) se ven los canastos.
    det, verdad = _escena('e2_bifurcaciones', pan=1.5, dyaw=math.pi / 2)
    assert {c['color'] for c in verdad['canastos']} <= {c['color'] for c in det['canastos']}
    assert not det['cubos']


def test_sin_zonas_falsas_por_flechas_rojas():
    det, _ = _escena('e2_bifurcaciones', dx=0.32)
    assert len(det['zonas']) < 20
