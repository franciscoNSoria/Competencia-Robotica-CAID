"""Pruebas del modelo del robot: que el xacro se expanda y que el árbol de piezas sea el esperado."""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

XACRO = Path(get_package_share_directory('caid_description')) / 'urdf' / 'caid_bot.urdf.xacro'

ENLACES_BASE = {
    'base_footprint', 'base_link',
    'wheel_fl_link', 'wheel_fr_link', 'wheel_rl_link', 'wheel_rr_link',
    'us_front', 'us_left', 'us_right',
    'camera_pan_link', 'camera_link', 'camera_optical_frame',
}
ENLACES_BRAZO = {
    'arm_base_link', 'arm_turret_link', 'arm_upper_link', 'arm_forearm_link',
    'arm_wrist_link', 'arm_claw_a_link', 'arm_claw_b_link', 'gripper_tcp',
}



def expandir(*argumentos):
    """Corre xacro sobre el modelo y devuelve el URDF (texto)."""
    r = subprocess.run(['xacro', str(XACRO), *argumentos], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    return r.stdout


def leer(urdf):
    """Devuelve (raíz del XML, nombres de los links, {hijo: padre})."""
    raiz = ET.fromstring(urdf.encode())
    enlaces = {l.get('name') for l in raiz.findall('link')}
    padres = {}
    for j in raiz.findall('joint'):
        hijo = j.find('child').get('link')
        assert hijo not in padres, f'{hijo} tiene dos padres'
        padres[hijo] = j.find('parent').get('link')
    return raiz, enlaces, padres



def test_arbol_completo():
    _, enlaces, padres = leer(expandir())
    assert enlaces == ENLACES_BASE | ENLACES_BRAZO
    assert enlaces - set(padres) == {'base_footprint'}   # una sola raíz


def test_sin_brazo():
    _, enlaces, _ = leer(expandir('arm:=none'))
    assert enlaces == ENLACES_BASE


def test_brazo_fijo_no_tiene_articulaciones_moviles():
    raiz, _, _ = leer(expandir('arm:=fixed'))
    for j in raiz.findall('joint'):
        if j.find('child').get('link') in ENLACES_BRAZO:
            assert j.get('type') == 'fixed', j.get('name')


def test_articulaciones_moviles():
    raiz, _, _ = leer(expandir())
    tipos = {j.get('name'): j.get('type') for j in raiz.findall('joint')}
    for rueda in ('wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint'):
        assert tipos[rueda] == 'continuous'
    for nombre in ('camera_pan_joint', 'arm_yaw_joint', 'arm_shoulder_joint', 'arm_elbow_joint',
                   'arm_wrist_joint', 'arm_claw_a_joint', 'arm_claw_b_joint'):
        assert tipos[nombre] == 'revolute', nombre
    garra_b = [j for j in raiz.findall('joint') if j.get('name') == 'arm_claw_b_joint'][0]
    assert garra_b.find('mimic').get('joint') == 'arm_claw_a_joint'


def test_check_urdf(tmp_path):
    archivo = tmp_path / 'caid_bot.urdf'
    archivo.write_text(expandir())
    r = subprocess.run(['check_urdf', str(archivo)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr