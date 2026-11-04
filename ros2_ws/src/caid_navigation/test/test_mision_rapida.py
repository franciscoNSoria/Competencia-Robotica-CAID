"""
La misión completa en el simulador rápido (tools/sim/mision_rapida.py), sin Gazebo.

Corre en otro proceso porque el simulador reemplaza los módulos de ROS por unos de mentira.
Si no encuentra el repo (paquete instalado suelto), se saltea.
"""
import os
import subprocess
import sys

import pytest

RAIZ = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                    '..', '..', '..', '..'))
SIM = os.path.join(RAIZ, 'tools', 'sim', 'mision_rapida.py')
MAZES = os.path.join(RAIZ, 'ros2_ws', 'src', 'caid_gazebo', 'mazes')

pytestmark = pytest.mark.skipif(not os.path.exists(SIM), reason='no está el repo completo')


def _correr(yaml, *extra):
    r = subprocess.run([sys.executable, SIM, os.path.join(MAZES, yaml), *extra],
                       capture_output=True, text=True, timeout=600, cwd=RAIZ)
    return r.returncode, r.stdout + r.stderr


@pytest.mark.parametrize('yaml', ['e2_bifurcaciones_simple.yaml',
                                  'e2_bifurcaciones_simple_abierta.yaml',
                                  'e4_isla_simple_abierta.yaml'])
def test_entrega_los_tres_cubos(yaml):
    codigo, salida = _correr(yaml)
    assert codigo == 0, salida[-2000:]
    assert 'Primera bifurcación' in salida          # hay punto de reingreso
    assert salida.count('En el punto de reingreso') == 2   # reingresó por ahí dos veces


def test_un_canasto_falso_adentro_no_rompe_la_mision():
    # Lo que pasó en Gazebo con E2: algo verde con forma de canasto adentro del laberinto.
    codigo, salida = _correr('e2_bifurcaciones_simple_abierta.yaml', '--canasto-falso')
    assert codigo == 0, salida[-2000:]
    assert 'relocalic' not in salida
