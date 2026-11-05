"""Laberintos para probar: los del repo (ros2_ws/src/caid_gazebo/mazes/*.yaml, 0,40 m por celda) y los dibujos en texto.

Formato (el de tools/torneo/simular_torneo.py): '#' bloque de pared, '.' libre, 'S' partida mirando a +x. Fila 0 arriba.
De los .yaml se toma 'mapa:' con inicio_dir E; los números (canastos) se tratan como pared y el resto de las letras
(R G B Z) como celda libre. Hay que poder correr esto sin ROS ni yaml instalado, por eso el parser es a mano.
"""
import os

AQUI = os.path.dirname(os.path.abspath(__file__))
MAZES = os.path.join(AQUI, '..', '..', 'ros2_ws', 'src', 'caid_gazebo', 'mazes')
CHICO = os.path.join(AQUI, '..', 'sim', 'laberintos', 'laberinto_chico.txt')


def _de_yaml(ruta):
    with open(ruta) as f:
        lineas = f.read().split('\n')
    i = next(k for k, l in enumerate(lineas) if l.startswith('mapa:'))
    filas = []
    for l in lineas[i + 1:]:
        if not l.startswith('  '):
            break
        filas.append(l.strip())
    return filas


def _a_texto(filas):
    out = []
    for fila in filas:
        out.append(''.join('S' if ch == 'S' else ('#' if ch in '#123~' else '.') for ch in fila))
    return '\n'.join(out)


def _chico():
    """laberinto_chico.txt: la entrada E está a la derecha mirando a -x. Se espeja para que mire a +x (como la S)."""
    with open(CHICO) as f:
        filas = [l.rstrip('\n') for l in f if l.strip()]
    return '\n'.join(fila[::-1].replace('E', 'S') for fila in filas)


def todos():
    labs = {
        'casa': """
########
#S..####
###.####
###....#
########""",
        'torneo': """
#############
#S....#.....#
#####.#.###.#
#.....#...#.#
#.#######.#.#
#.....#...#.#
#####.#.###.#
#...........#
#############""",
        'ramas': """
###########
#S...#....#
###.##.##.#
#...#..#..#
#.###.##.##
#.....#...#
#.###.#.#.#
#...#...#.#
###########""",
    }
    try:
        labs['chico'] = _chico()
    except OSError:
        pass
    for n in ('e1_serpentina', 'e2_bifurcaciones', 'e3_obstaculos', 'e4_isla', 'e5_competencia'):
        ruta = os.path.join(MAZES, n + '.yaml')
        if os.path.exists(ruta):
            labs[n.split('_')[0]] = _a_texto(_de_yaml(ruta))
    return labs
