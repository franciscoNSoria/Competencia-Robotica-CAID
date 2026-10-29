"""Mundo del simulador 2D: lo arma con gen_maze.py, igual que el mundo de Gazebo."""
import importlib.util
import math
import os
from pathlib import Path

CANASTO_EXT = 0.044
CANASTO_ALTO = 0.020
CANASTO_PARED = 0.002
CANASTO_SOPORTE = 0.036
COLORES = {'R': 'rojo', 'G': 'verde', 'B': 'azul'}
DEPOSITO = {'1': 'R', '2': 'G', '3': 'B'}


def ruta_gen_maze():
    candidatos = []
    if os.environ.get('CAID_GEN_MAZE'):
        candidatos.append(Path(os.environ['CAID_GEN_MAZE']))
    aqui = Path(__file__).resolve()
    for padre in aqui.parents:
        candidatos.append(padre / 'caid_gazebo' / 'scripts' / 'gen_maze.py')
        candidatos.append(padre / 'ros2_ws' / 'src' / 'caid_gazebo' / 'scripts' / 'gen_maze.py')
    try:
        from ament_index_python.packages import get_package_share_directory
        candidatos.append(Path(get_package_share_directory('caid_gazebo')) / 'scripts' / 'gen_maze.py')
    except Exception:
        pass
    for c in candidatos:
        if c.is_file():
            return c
    raise FileNotFoundError('No encuentro gen_maze.py (definí CAID_GEN_MAZE)')


def cargar_gen_maze():
    spec = importlib.util.spec_from_file_location('gen_maze', ruta_gen_maze())
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    return modulo


def ruta_laberinto(nombre):
    """Acepta una ruta a un YAML o el nombre de un escenario (e1_serpentina, e3_obstaculos, ...)."""
    if os.path.isfile(nombre):
        return nombre
    carpeta = ruta_gen_maze().parent.parent / 'mazes'
    for c in sorted(carpeta.glob('*.yaml')):
        if c.stem == nombre or c.stem.startswith(nombre + '_'):
            return str(c)
    raise FileNotFoundError(nombre)


class Caja:
    """Prisma alineado: (x0, y0, x1, y1) y alturas (z0, z1)."""

    __slots__ = ('caja', 'z0', 'z1', 'color', 'tipo', 'nombre', 'activa')

    def __init__(self, caja, z0, z1, color, tipo, nombre=''):
        self.caja = caja
        self.z0, self.z1 = z0, z1
        self.color = color
        self.tipo = tipo
        self.nombre = nombre
        self.activa = True


def caja_centrada(x, y, lado_x, lado_y=None):
    lado_y = lado_x if lado_y is None else lado_y
    return (x - lado_x / 2, y - lado_y / 2, x + lado_x / 2, y + lado_y / 2)


class Mundo:

    def __init__(self, nombre):
        gm = cargar_gen_maze()
        self.gm = gm
        self.lab = lab = gm.cargar(ruta_laberinto(nombre))
        mensajes, info = gm.validar(lab)
        errores = [t for n, t in mensajes if n == 'ERROR']
        if errores:
            raise ValueError('Laberinto inválido: ' + '; '.join(errores))
        self.info = info
        C = self.celda = lab.celda
        self.alto_pared = lab.alto
        self.bloques = set()
        self.cajas = []
        for r in range(lab.nfilas):
            for c in range(lab.ncols):
                if lab.car(r, c) == '#':
                    self.bloques.add((r, c))
                    x, y = lab.centro(r, c)
                    self.cajas.append(Caja(caja_centrada(x, y, C), 0.0, lab.alto,
                                           gm.COLOR_PARED, 'pared'))
        for ob in gm.objetos_con_posicion(lab):
            tipo = 'soporte' if ob['soporte'] else ob['tipo']
            self.cajas.append(Caja(caja_centrada(ob['x'], ob['y'], ob['lado']), ob['z_base'],
                                   ob['z_base'] + ob['alto'], ob['color'], tipo))
        self.cubos = {}
        for simb, (r, c) in info['cubos'].items():
            x, y, _ = gm.posicion_cubo(lab, simb, (r, c))
            elevado = lab.cubos_cfg.get(simb, {}).get('elevado', True)
            z0 = lab.altura_apoyo if elevado else 0.0
            caja = Caja(caja_centrada(x, y, gm.CUBO_M), z0, z0 + gm.CUBO_M,
                        gm.COLORES_CUBO[simb][1], 'cubo', COLORES[simb])
            self.cajas.append(caja)
            self.cubos[COLORES[simb]] = caja
        self.canastos = {}
        for k, (r, c) in info['deposito'].items():
            x, y = gm.posicion_canasto(lab, (r, c))
            color = COLORES[DEPOSITO[k]]
            rgb = gm.COLORES_CUBO[DEPOSITO[k]][1]
            base = max(lab.altura_borde_canasto - CANASTO_ALTO, 0.0)
            if base > 0.002:
                self.cajas.append(Caja(caja_centrada(x, y, CANASTO_SOPORTE), 0.0, base,
                                       gm.COLOR_SOPORTE, 'soporte'))
            E, T = CANASTO_EXT, CANASTO_PARED
            partes = [caja_centrada(x, y, E, E)]
            for parte in partes:
                self.cajas.append(Caja(parte, base, base + 0.001, rgb, 'canasto', color))
            for caja in (caja_centrada(x, y + E / 2 - T / 2, E, T),
                         caja_centrada(x, y - E / 2 + T / 2, E, T),
                         caja_centrada(x + E / 2 - T / 2, y, T, E - 2 * T),
                         caja_centrada(x - E / 2 + T / 2, y, T, E - 2 * T)):
                self.cajas.append(Caja(caja, base, base + CANASTO_ALTO, rgb, 'canasto', color))
            self.canastos[color] = (x, y, base)
        self.zonas = []           # barras rojas sobre el piso: (x0, y0, x1, y1)
        for (r, c, h, w) in gm.rectangulos_de_zonas(lab):
            x0 = c * C + gm.ZONA_INSET_M
            x1 = (c + w) * C - gm.ZONA_INSET_M
            y1 = (lab.nfilas - r) * C - gm.ZONA_INSET_M
            y0 = (lab.nfilas - r - h) * C + gm.ZONA_INSET_M
            lw = gm.ZONA_LINEA_M
            self.zonas += [(x0, y1 - lw, x1, y1), (x0, y0, x1, y0 + lw),
                           (x1 - lw, y0 + lw, x1, y1 - lw), (x0, y0 + lw, x0 + lw, y1 - lw)]
        self.celdas_zona = set(lab.buscar('Z'))
        self.flechas = []
        for fl in lab.flechas:
            centro3, (u, p, normal) = gm.posicion_flecha(lab, fl)
            self.flechas.append({'centro': centro3, 'u': u, 'p': p, 'normal': normal,
                                 'largo': int(fl['largo']), 'color': fl['color'],
                                 'rgb': gm.COLOR_FLECHA[fl['color']], 'celda': tuple(fl['celda']),
                                 'pared': fl['pared'], 'dir': fl['dir']})
        (r, c), = lab.buscar('S')
        self.inicio_celda = (r, c)
        x, y = lab.centro(r, c)
        self.inicio = (x, y, gm.YAW[lab.inicio_dir])
        self.significado = {k: [COLORES[s] for s in v] for k, v in lab.significado_colores.items()}

    def celda_rc(self, x, y):
        """(fila, columna) de un punto del mundo."""
        return (self.lab.nfilas - 1 - int(math.floor(y / self.celda)), int(math.floor(x / self.celda)))

    def cajas_cerca(self, x, y, radio):
        return [b for b in self.cajas if b.activa and b.caja[0] - radio < x < b.caja[2] + radio
                and b.caja[1] - radio < y < b.caja[3] + radio]
