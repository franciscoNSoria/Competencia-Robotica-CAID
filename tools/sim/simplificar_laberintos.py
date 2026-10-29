#!/usr/bin/env python3
"""
Copias simples de los escenarios de DESC-04 para probar la navegación por Trémaux + flechas.

Uso: python3 tools/sim/simplificar_laberintos.py ros2_ws/src/caid_gazebo/mazes/e4_isla.yaml ...

Por cada YAML escribe <nombre>_simple.yaml al lado, sin tocar el original:
- Saca las flechas que el robot pasa de costado: se queda solo con las de la pared hacia la que
  llega de frente por el camino más corto desde la partida (curvas y T).
- Cambia las zonas prohibidas (Z) por bloques de pared: igual no se podían pisar, así los caminos
  más cortos no cambian y las flechas que quedan siguen siendo correctas para el validador.
- Agranda la celda a 0,40 m: con 0,32 m el robot (diagonal de 322 mm) no puede girar en el lugar.
- Pone los cubos en el centro de su celda, a la altura de la cámara (centro a 138,5 mm), sobre un
  pilar cilíndrico de 80 mm, para que la cámara los vea de frente.
- Si la partida es un área abierta (~, como E6), la cambia por una zona cerrada como la de E1 a
  E5: 2 x 3 celdas pegadas a la entrada, con los canastos contra la pared oeste y la S al lado, y
  saca los distractores y bloques: quedan solo los tres cubos.
- Saca las flechas que el validador de gen_maze.py rechaza después de estos cambios (al sacar un
  bloque puede cambiar el camino más corto).
Después hay que generar el mundo con caid_gazebo/scripts/gen_maze.py.
"""
from collections import deque
import importlib.util
import os
import re
import sys

import yaml

GEN_MAZE = os.path.join(os.path.dirname(__file__), '..', '..', 'ros2_ws', 'src', 'caid_gazebo',
                        'scripts', 'gen_maze.py')
PASO = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'O': (0, -1)}
OPUESTA = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}
CAMARA_ALTURA_MM = 138.5   # hardware/cad/dimensions.yaml
CUBO_LADO_MM = 20.0
PILAR_DIAMETRO_MM = 80.0
CELDA_M = 0.40


class Bloque(str):
    pass


yaml.add_representer(Bloque, lambda d, s: d.represent_scalar('tag:yaml.org,2002:str', s, style='|'))


def libre(filas, r, c):
    return 0 <= r < len(filas) and 0 <= c < len(filas[r]) and filas[r][c] != '#'


def llegadas(filas):
    """Dirección con la que el robot entra a cada celda por un camino más corto desde la S."""
    inicio = next((r, f.index('S')) for r, f in enumerate(filas) if 'S' in f)
    llegada = {inicio: None}
    cola = deque([inicio])
    while cola:
        r, c = cola.popleft()
        for d, (dr, dc) in PASO.items():
            n = (r + dr, c + dc)
            if libre(filas, *n) and n not in llegada:
                llegada[n] = d
                cola.append(n)
    return llegada


def zona_cerrada(filas):
    """
    Cambia el área abierta (~) por una zona de partida cerrada de 2 x 3, como la de E1 a E5.

    La entrada es la primera celda de pasillo junto al área abierta; la zona queda justo al oeste:
    la columna de los canastos (1, 2, 3 de arriba a abajo) y la de la S, en la fila de la entrada.
    """
    r, c = next((r, c) for r, f in enumerate(filas) for c, ch in enumerate(f)
                if ch == '.' and c > 0 and f[c - 1] == '~')
    filas = [list(f.replace('~', '#').translate(str.maketrans('S123', '####'))) for f in filas]
    for i, canasto in zip((r - 1, r, r + 1), '123'):
        filas[i][c - 2] = canasto
        filas[i][c - 1] = '.'
    filas[r][c - 1] = 'S'
    return [''.join(f) for f in filas]


def simplificar(ruta):
    lab = yaml.safe_load(open(ruta))
    filas = [f.replace('Z', '#') for f in lab['mapa'].rstrip('\n').split('\n')]
    if any('~' in f for f in filas):
        filas = zona_cerrada(filas)
        lab['canastos_pared'] = 'O'
        lab['objetos'] = []
    llegada = llegadas(filas)
    quedan, sacadas = [], 0
    for f in lab.get('flechas', []):
        if llegada.get(tuple(f['celda'])) == f['pared']:
            quedan.append(f)
        else:
            sacadas += 1
    lab['mapa'] = Bloque('\n'.join(filas) + '\n')
    lab['flechas'] = quedan
    lab['altura_apoyo_cubo_mm'] = CAMARA_ALTURA_MM - CUBO_LADO_MM / 2
    lab['pedestal_forma'] = 'cilindro'
    lab['pedestal_lado_mm'] = PILAR_DIAMETRO_MM
    lab['celda_m'] = CELDA_M
    # Los tres al centro de su celda, para verlos de frente. El pilar corta el pasillo, pero la carga
    # simulada lo saca junto con el cubo; para el validador no bloquea.
    lab['cubos'] = {s: {'dx_mm': 0, 'dy_mm': 0, 'bloquea': False} for s in 'RGB'}
    lab['nombre'] = lab.get('nombre', '') + ' (simple: sin flechas laterales ni zonas)'
    salida = ruta.replace('.yaml', '_simple.yaml')
    invalidas = 0
    while True:
        escribir(lab, ruta, salida)
        malas = flechas_invalidas(salida)
        if not malas:
            break
        lab['flechas'] = [f for i, f in enumerate(lab['flechas']) if i not in malas]
        invalidas += len(malas)
    print(f'{salida}: {len(lab["flechas"])} flechas quedan, {sacadas} laterales y '
          f'{invalidas} inválidas sacadas')


def escribir(lab, ruta, salida):
    with open(salida, 'w') as out:
        out.write('# Generado con tools/sim/simplificar_laberintos.py a partir de '
                  f'{os.path.basename(ruta)}. No editar a mano.\n')
        yaml.dump(lab, out, allow_unicode=True, sort_keys=False, width=120)


def flechas_invalidas(ruta):
    """Índices de las flechas que el validador de gen_maze.py marca con ERROR."""
    spec = importlib.util.spec_from_file_location('gen_maze', GEN_MAZE)
    gm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gm)
    mensajes, _ = gm.validar(gm.cargar(ruta))
    return {int(m.group(1)) - 1 for nivel, texto in mensajes if nivel == 'ERROR'
            for m in [re.match(r'flecha (\d+) ', texto)] if m}


if __name__ == '__main__':
    for archivo in sys.argv[1:]:
        simplificar(archivo)
