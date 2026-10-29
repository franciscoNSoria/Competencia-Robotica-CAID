#!/usr/bin/env python3
"""Generador de laberintos para Gazebo (DESC-04): YAML con grilla ASCII -> mundo SDF + vista SVG.

Uso:
    python3 gen_maze.py ../mazes/e1_serpentina.yaml                 # un laberinto
    python3 gen_maze.py ../mazes/*.yaml --svg ../preview            # todos, con vista en SVG
    python3 gen_maze.py ../mazes/e2_bifurcaciones.yaml --rutas      # la vista dibuja las rutas a cada cubo

Escribe el mundo en ../worlds/<nombre>.world, valida el laberinto (ver `validar`) e imprime el comando
para lanzar la simulacion en el punto de inicio. Sale con codigo 1 si hay errores.
El formato del YAML esta descrito en docs/arquitectura/laberintos.md.
"""
import argparse
import math
import os
import sys
from collections import deque

import yaml

# ---------------------------------------------------------------------------------------------
# Medidas (metros). Fuente entre parentesis.
# ---------------------------------------------------------------------------------------------
CELDA_M = 0.32               # ancho del pasillo y lado del bloque de pared (oficial, informado por el equipo)
ALTO_PARED_M = 0.17          # alto del bloque de pared (oficial, informado por el equipo)
ROBOT_ANCHO_M = 0.163        # hardware/cad/dimensions.yaml: paredes.ancho
ROBOT_LARGO_M = 0.2781       # dimensions.yaml: 139,9 + 138,2 (de paragolpes a paragolpes, sin el brazo)
MARGEN_M = 0.015             # holgura lateral exigida a cada lado para considerar "pasable"

CUBO_M = 0.020               # reglamento del desafio, anexo
CUBO_MASA_KG = 0.0099        # dimensions.yaml: cubo.masa_g (PLA macizo, provisorio)

FLECHA_PUNTA_ANCHO_M = 0.070  # reglamento, anexo (70 +-1 mm), medido perpendicular al eje
FLECHA_PUNTA_ALTO_M = 0.050   # reglamento, anexo (50 +-1 mm), medido a lo largo del eje
FLECHA_CUERPO_ANCHO_M = 0.030
FLECHA_CUERPO_LARGO_M = {1: 0.030, 2: 0.060, 3: 0.100}   # "cerca", "normal", "lejos"
FLECHA_GROSOR_M = 0.002       # espesor de las flechas del STL del equipo
FLECHA_ALTURA_CENTRO_M = 0.115  # centro de la flecha sobre el piso (STL del equipo: z = 80..150 mm)
FLECHA_TILT_MAX_GRADOS = 30.0   # reglamento: inclinacion maxima +-30 grados
FLECHA_TIRAS_PUNTA = 25         # la punta triangular se arma con 25 tiras de 2 mm (SDF no tiene prismas)

CANASTO_EXT_M = 0.044         # Canasto_Ficticio.stl: 44 x 44 x 20 mm
CANASTO_PARED_M = 0.002
CANASTO_FONDO_M = 0.001
CANASTO_ALTO_M = 0.020
CANASTO_SEPARACION_M = 0.010  # separacion entre el canasto y la pared contra la que se apoya
CANASTO_SOPORTE_M = 0.036     # P: columna que sostiene el canasto a la altura del borde pedido
ALTURA_APOYO_CUBO_M = 0.1085  # el cubo apoya a la altura de la base del brazo = cara superior del 2.o piso (dimensions.yaml: 108,5)
ALTURA_BORDE_CANASTO_M = 0.1085  # P: el borde de la pared trasera del canasto queda a esa altura
PEDESTAL_LADO_M = 0.040       # P: lado de la columna que sostiene cada cubo
CUBO_HOLGURA_PARED_M = 0.005  # luz entre el soporte del cubo y el bloque contra el que se apoya

TIPOS_OBJETO = {
    # tipo: (lado m, alto m, bloquea el paso, dinamico, color rgb)
    'bloque_8cm': (0.08, 0.08, True, False, (0.45, 0.45, 0.45)),
    'bloque_20cm': (0.20, 0.20, True, False, (0.45, 0.45, 0.45)),
    'distractor_amarillo': (CUBO_M, CUBO_M, False, True, (1.0, 0.85, 0.0)),
    'distractor_naranja': (CUBO_M, CUBO_M, False, True, (1.0, 0.45, 0.0)),
    'distractor_violeta': (CUBO_M, CUBO_M, False, True, (0.55, 0.15, 0.75)),
}

COLORES_CUBO = {'R': ('rojo', (1.0, 0.0, 0.0)), 'G': ('verde', (0.0, 0.7, 0.1)), 'B': ('azul', (0.0, 0.2, 1.0))}
COLOR_FLECHA = {'gris': (0.22, 0.22, 0.22), 'rojo': (1.0, 0.0, 0.0), 'verde': (0.0, 0.7, 0.1), 'azul': (0.0, 0.2, 1.0)}
# Que cubo(s) indica cada color de flecha. Es PARAMETRIZABLE (`significado_colores` en el YAML): el reglamento solo dice que en las
# bifurcaciones el color indica el camino al objetivo. Una lista vacia = la flecha no significa nada y hay que ignorarla.
SIGNIFICADO_COLOR_DEFECTO = {'rojo': ['R'], 'verde': ['G'], 'azul': ['B'], 'gris': []}
COLOR_PARED = (0.90, 0.90, 0.86)
COLOR_PISO = (0.55, 0.55, 0.55)
COLOR_SOPORTE = (0.70, 0.70, 0.66)
COLOR_ZONA_ROJA = (0.85, 0.0, 0.0)
ZONA_LINEA_M = 0.020
ZONA_INSET_M = 0.005

DIRS = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'O': (0, -1)}   # (d fila, d columna); la fila 0 es el norte
OPUESTO = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}
YAW = {'E': 0.0, 'N': math.pi / 2, 'O': math.pi, 'S': -math.pi / 2}
SIMBOLOS = set('#.~SRGB123Z')
EXTERIOR = set('~S123Z')        # simbolos que pueden estar sobre el borde de la grilla (area abierta sin paredes)


# ---------------------------------------------------------------------------------------------
# Carga
# ---------------------------------------------------------------------------------------------
class Laberinto:
    def __init__(self, datos, ruta):
        self.ruta = ruta
        self.id = os.path.splitext(os.path.basename(ruta))[0]
        self.nombre = datos.get('nombre', self.id)
        self.descripcion = datos.get('descripcion', '')
        self.celda = float(datos.get('celda_m', CELDA_M))
        self.alto = float(datos.get('alto_pared_m', ALTO_PARED_M))
        self.inicio_dir = str(datos.get('inicio_dir', 'E'))
        self.canastos_pared = str(datos.get('canastos_pared', ''))
        self.altura_flechas = float(datos.get('altura_flechas_mm', FLECHA_ALTURA_CENTRO_M * 1000.0)) / 1000.0
        self.altura_apoyo = float(datos.get('altura_apoyo_cubo_mm', ALTURA_APOYO_CUBO_M * 1000.0)) / 1000.0
        self.altura_borde_canasto = float(datos.get('altura_borde_canasto_mm', ALTURA_BORDE_CANASTO_M * 1000.0)) / 1000.0
        self.pedestal_lado = float(datos.get('pedestal_lado_mm', PEDESTAL_LADO_M * 1000.0)) / 1000.0
        self.cubos_cfg = {str(k): dict(v) for k, v in (datos.get('cubos') or {}).items()}
        significado = dict(SIGNIFICADO_COLOR_DEFECTO)
        significado.update({str(k): list(v or []) for k, v in (datos.get('significado_colores') or {}).items()})
        self.significado_colores = significado
        self.filas = [f for f in str(datos['mapa']).split('\n') if f.strip() != '']
        self.flechas = [dict(f) for f in datos.get('flechas', [])]
        self.objetos = [dict(o) for o in datos.get('objetos', [])]
        self.nfilas = len(self.filas)
        self.ncols = max(len(f) for f in self.filas) if self.filas else 0

    def car(self, r, c):
        if 0 <= r < self.nfilas and 0 <= c < len(self.filas[r]):
            return self.filas[r][c]
        return '#'

    def centro(self, r, c):
        """Centro de la celda en el mundo: origen en la esquina suroeste de la grilla, x al este, y al norte."""
        return ((c + 0.5) * self.celda, (self.nfilas - r - 0.5) * self.celda)

    def buscar(self, simbolo):
        return [(r, c) for r in range(self.nfilas) for c in range(len(self.filas[r])) if self.filas[r][c] == simbolo]


def cargar(ruta):
    with open(ruta, encoding='utf-8') as f:
        datos = yaml.safe_load(f)
    return Laberinto(datos, ruta)


# ---------------------------------------------------------------------------------------------
# Objetos: posicion, bloqueo y pasabilidad
# ---------------------------------------------------------------------------------------------
def paredes_cerradas(lab, celda):
    return [d for d, (dr, dc) in DIRS.items() if lab.car(celda[0] + dr, celda[1] + dc) == '#']


def lado_automatico(lab, celda):
    """Pared contra la que se apoya un cubo en esa celda (None si la celda no tiene paredes: queda al centro).
    Prefiere una pared lateral (perpendicular al pasillo) y evita las paredes que ya tienen una flecha en esa celda."""
    cerradas = paredes_cerradas(lab, celda)
    if not cerradas:
        return None
    abiertas = [d for d in DIRS if d not in cerradas]
    eje_h = any(d in 'EO' for d in abiertas)
    eje_v = any(d in 'NS' for d in abiertas)
    con_flecha = {f.get('pared') for f in lab.flechas if tuple(f.get('celda', ())) == tuple(celda)}

    def lateral(d):
        return (d in 'NS' and eje_h and not eje_v) or (d in 'EO' and eje_v and not eje_h)

    orden = sorted(cerradas, key=lambda d: (d in con_flecha, not lateral(d), 'NESO'.index(d)))
    return orden[0]


def posicion_apoyada(lab, celda, lado, lado_soporte):
    """Centro (x, y) de un soporte de `lado_soporte` apoyado contra la pared `lado` de la celda (centro si lado es None)."""
    cx, cy = lab.centro(*celda)
    if lado is None:
        return cx, cy
    v = _vec_mundo(lado)
    desp = lab.celda / 2.0 - CUBO_HOLGURA_PARED_M - lado_soporte / 2.0
    return cx + v[0] * desp, cy + v[1] * desp


def posicion_cubo(lab, simb, celda):
    """(x, y, pared) del cubo objetivo `simb` en su celda: por defecto sobre un soporte pegado a una pared lateral."""
    cfg = lab.cubos_cfg.get(simb, {})
    cx, cy = lab.centro(*celda)
    if 'dx_mm' in cfg or 'dy_mm' in cfg:
        return cx + cfg.get('dx_mm', 0) / 1000.0, cy + cfg.get('dy_mm', 0) / 1000.0, None
    lado = cfg.get('lado') or lado_automatico(lab, celda)
    x, y = posicion_apoyada(lab, celda, lado, lab.pedestal_lado)
    return x, y, lado


def posicion_canasto(lab, celda):
    """(x, y) del canasto: apoyado contra `canastos_pared` si se indica, o al centro de la celda (area abierta)."""
    cx, cy = lab.centro(*celda)
    if lab.canastos_pared in DIRS:
        v = _vec_mundo(lab.canastos_pared)
        desp = lab.celda / 2.0 - CANASTO_SEPARACION_M - CANASTO_EXT_M / 2.0
        return cx + v[0] * desp, cy + v[1] * desp
    return cx, cy


def objetos_con_posicion(lab):
    """Todos los objetos del mundo (menos paredes, flechas y canastos), en coordenadas de mundo.

    Campos: tipo, celda, x, y, lado, alto, bloquea, dinamico, color, z_base y soporte (True si es una columna).
    Cada cubo objetivo lleva debajo una columna (tipo 'soporte') que sube hasta la altura de la base del brazo; los
    distractores tambien, salvo que se pida `elevado: false`."""
    salida = []
    for simb in 'RGB':
        pos = lab.buscar(simb)
        if len(pos) != 1:
            continue
        cfg = lab.cubos_cfg.get(simb, {})
        if cfg.get('elevado', True):
            x, y, _ = posicion_cubo(lab, simb, pos[0])
            salida.append({'tipo': 'soporte', 'origen': 'cubo_' + COLORES_CUBO[simb][0], 'celda': pos[0], 'x': x, 'y': y,
                           'lado': lab.pedestal_lado, 'alto': lab.altura_apoyo, 'bloquea': True, 'dinamico': False,
                           'color': COLOR_SOPORTE, 'z_base': 0.0, 'soporte': True})
    for i, o in enumerate(lab.objetos):
        lado, alto, bloquea, dinamico, color = TIPOS_OBJETO[o['tipo']]
        r, c = o['celda']
        cx, cy = lab.centro(r, c)
        pared = o.get('lado')
        if pared == 'auto':
            pared = lado_automatico(lab, (r, c))
        if pared in DIRS:
            # apoyado contra esa pared: el soporte (o el objeto mismo si no se eleva) queda pegado al bloque
            huella = lab.pedestal_lado if (dinamico and o.get('elevado', True)) else lado
            x, y = posicion_apoyada(lab, (r, c), pared, huella)
        else:
            x = cx + o.get('dx_mm', 0) / 1000.0
            y = cy + o.get('dy_mm', 0) / 1000.0
        elevado = dinamico and o.get('elevado', True)
        if elevado:
            salida.append({'tipo': 'soporte', 'origen': 'distractor_%d' % i, 'celda': (r, c), 'x': x, 'y': y,
                           'lado': lab.pedestal_lado, 'alto': lab.altura_apoyo, 'bloquea': True, 'dinamico': False,
                           'color': COLOR_SOPORTE, 'z_base': 0.0, 'soporte': True})
        salida.append({'tipo': o['tipo'], 'celda': (r, c), 'x': x, 'y': y, 'lado': lado, 'alto': alto,
                       'bloquea': bloquea, 'dinamico': dinamico, 'color': color,
                       'z_base': lab.altura_apoyo if elevado else 0.0, 'soporte': False, 'indice': i})
    return salida


def hueco_lateral(lab, objs_celda, eje):
    """Mayor ancho libre (m) de la celda para cruzarla por `eje` ('h' = este-oeste, 'v' = norte-sur)."""
    mitad = lab.celda / 2.0
    intervalos = []
    for ob in objs_celda:
        if not ob['bloquea']:
            continue
        centro = ob['y'] if eje == 'h' else ob['x']
        # la posicion del objeto es absoluta; se pasa a la coordenada relativa al centro de la celda
        rel = centro - (lab.centro(*ob['celda'])[1] if eje == 'h' else lab.centro(*ob['celda'])[0])
        intervalos.append((rel - ob['lado'] / 2.0, rel + ob['lado'] / 2.0))
    intervalos.sort()
    mejor = 0.0
    cursor = -mitad
    for a, b in intervalos:
        mejor = max(mejor, a - cursor)
        cursor = max(cursor, b)
    return max(mejor, mitad - cursor)


def construir_grafo(lab):
    """Devuelve (libres, pasable) donde pasable[(r,c)] = {'h': bool, 'v': bool}."""
    objs = objetos_con_posicion(lab)
    por_celda = {}
    for ob in objs:
        por_celda.setdefault(ob['celda'], []).append(ob)
    libres = set()
    pasable = {}
    necesario = ROBOT_ANCHO_M + 2 * MARGEN_M
    for r in range(lab.nfilas):
        for c in range(len(lab.filas[r])):
            if lab.car(r, c) in '#Z':
                continue
            libres.add((r, c))
            oc = por_celda.get((r, c), [])
            pasable[(r, c)] = {'h': hueco_lateral(lab, oc, 'h') >= necesario,
                               'v': hueco_lateral(lab, oc, 'v') >= necesario}
    return libres, pasable


def vecinos(lab, libres, pasable, celda):
    r, c = celda
    for d, (dr, dc) in DIRS.items():
        v = (r + dr, c + dc)
        if v not in libres:
            continue
        eje = 'h' if d in 'EO' else 'v'
        if pasable[celda][eje] and pasable[v][eje]:
            yield d, v


def distancias(lab, libres, pasable, origen):
    dist = {origen: 0}
    cola = deque([origen])
    while cola:
        u = cola.popleft()
        for _, v in vecinos(lab, libres, pasable, u):
            if v not in dist:
                dist[v] = dist[u] + 1
                cola.append(v)
    return dist


def ruta_corta(lab, libres, pasable, a, b):
    """Una ruta mas corta de a a b (lista de celdas) o None."""
    dist_b = distancias(lab, libres, pasable, b)
    if a not in dist_b:
        return None
    ruta = [a]
    while ruta[-1] != b:
        u = ruta[-1]
        for _, v in vecinos(lab, libres, pasable, u):
            if dist_b.get(v, 1e9) == dist_b[u] - 1:
                ruta.append(v)
                break
    return ruta


# ---------------------------------------------------------------------------------------------
# Flechas: semantica (direccion y largo) respecto del laberinto
# ---------------------------------------------------------------------------------------------
def largo_de_flecha(lab, libres, pasable, dist, celda, d):
    """Cuantos pasos seguidos en la direccion `d` da una ruta mas corta desde `celda` (tope 3).
    None si `d` no es el primer paso de una ruta mas corta hacia el objetivo de `dist`."""
    dr, dc = DIRS[d]
    actual = celda
    pasos = 0
    while True:
        sig = (actual[0] + dr, actual[1] + dc)
        if actual not in dist or sig not in dist or dist[sig] != dist[actual] - 1:
            break
        if sig not in dict(vecinos(lab, libres, pasable, actual)).values():
            break
        pasos += 1
        actual = sig
    return pasos if pasos > 0 else None


# ---------------------------------------------------------------------------------------------
# Validacion
# ---------------------------------------------------------------------------------------------
def en_zona_abierta(lab, u):
    """True si la celda pertenece a algun bloque de 2 x 2 celdas libres (area abierta, no pasillo)."""
    r, c = u
    for dr in (-1, 0):
        for dc in (-1, 0):
            if all(lab.car(r + dr + i, c + dc + j) != '#' for i in (0, 1) for j in (0, 1)):
                return True
    return False


def region_exterior(lab):
    """Celdas del area abierta exterior: las `~` y las S, canastos y zonas rojas conectadas a ellas sin cruzar paredes."""
    inicio = lab.buscar('~')
    ext = set(inicio)
    cola = deque(inicio)
    while cola:
        r, c = cola.popleft()
        for dr, dc in DIRS.values():
            v = (r + dr, c + dc)
            if v not in ext and lab.car(*v) in EXTERIOR:
                ext.add(v)
                cola.append(v)
    return ext


def validar(lab):
    """Devuelve (mensajes, info). mensajes = lista de (nivel, texto) con nivel 'ERROR', 'AVISO' o 'INFO'."""
    msg = []

    def err(t):
        msg.append(('ERROR', t))

    def aviso(t):
        msg.append(('AVISO', t))

    def info(t):
        msg.append(('INFO', t))

    infoval = {'rutas': {}, 'flechas': []}
    if lab.nfilas == 0:
        err('el mapa esta vacio')
        return msg, infoval
    for r, fila in enumerate(lab.filas):
        if len(fila) != lab.ncols:
            err('la fila %d tiene %d columnas y se esperaban %d' % (r, len(fila), lab.ncols))
        for c, ch in enumerate(fila):
            if ch not in SIMBOLOS:
                err('simbolo desconocido %r en fila %d, columna %d' % (ch, r, c))
    if any(m[0] == 'ERROR' for m in msg):
        return msg, infoval

    # Borde: solo puede haber piso abierto (`~`, la zona de inicio, canastos o zonas rojas); un pasillo no puede llegar al borde.
    for r in range(lab.nfilas):
        for c in range(lab.ncols):
            if (r in (0, lab.nfilas - 1) or c in (0, lab.ncols - 1)) and lab.car(r, c) != '#' and lab.car(r, c) not in EXTERIOR:
                err('hay una abertura en el borde (fila %d, columna %d): el laberinto debe estar cerrado' % (r, c))

    s = lab.buscar('S')
    if len(s) != 1:
        err('tiene que haber exactamente una S (hay %d)' % len(s))
        return msg, infoval
    s = s[0]
    if lab.inicio_dir not in DIRS:
        err('inicio_dir debe ser N, E, S u O')
        return msg, infoval
    dr, dc = DIRS[lab.inicio_dir]
    if lab.car(s[0] + dr, s[1] + dc) == '#':
        err('el robot mira a la pared: la celda delante de S (direccion %s) es un bloque' % lab.inicio_dir)

    for color, objetivos in lab.significado_colores.items():
        if color not in COLOR_FLECHA:
            err('significado_colores: el color %r no existe (gris, rojo, verde, azul)' % color)
        elif any(o not in 'RGB' for o in objetivos):
            err('significado_colores: %r tiene que ser una lista con R, G y/o B (o vacia: la flecha no significa nada)' % color)
    cubos = {}
    for simb in 'RGB':
        pos = lab.buscar(simb)
        if len(pos) != 1:
            err('tiene que haber exactamente un cubo %s (hay %d)' % (COLORES_CUBO[simb][0], len(pos)))
        else:
            cubos[simb] = pos[0]
    deposito = {}
    for k in '123':
        pos = lab.buscar(k)
        if len(pos) != 1:
            err('tiene que haber exactamente un deposito %s (hay %d)' % (k, len(pos)))
        else:
            deposito[k] = pos[0]
    if len(cubos) < 3 or len(deposito) < 3:
        return msg, infoval

    if lab.canastos_pared:
        if lab.canastos_pared not in DIRS:
            err('canastos_pared debe ser N, E, S u O (o no ponerlo: los canastos quedan al centro de su celda)')
        else:
            dpr, dpc = DIRS[lab.canastos_pared]
            for k, (r, c) in deposito.items():
                if lab.car(r + dpr, c + dpc) != '#':
                    err('el deposito %s (fila %d, columna %d) no tiene pared al %s: el canasto no tiene donde apoyarse'
                        % (k, r, c, lab.canastos_pared))

    ext = region_exterior(lab)
    if ext:
        puertas = []
        for (r, c) in ext:
            for d, (dr, dc) in DIRS.items():
                v = (r + dr, c + dc)
                if v not in ext and lab.car(*v) != '#':
                    puertas.append(((r, c), d, v))
        if len(puertas) != 1:
            err('el laberinto tiene que tener una unica entrada desde el area abierta y tiene %d (celdas: %s)'
                % (len(puertas), sorted(set(p[2] for p in puertas))))
        else:
            _, d, v = puertas[0]
            laterales = ('N', 'S') if d in 'EO' else ('E', 'O')
            for lat in laterales:
                if lab.car(v[0] + DIRS[lat][0], v[1] + DIRS[lat][1]) != '#':
                    err('la entrada %s no esta limitada por paredes: falta el bloque al %s' % (v, lat))
            info('entrada del laberinto en %s (se entra hacia %s), entre paredes; el area abierta tiene %d celdas' % (v, d, len(ext)))
        for nombre, pos in [('la S', [s])] + [('el canasto ' + k, [p]) for k, p in deposito.items()]:
            if pos[0] not in ext:
                aviso('%s no esta en el area abierta exterior' % nombre)

    for simb, cfg in lab.cubos_cfg.items():
        if simb not in cubos:
            err('cubos: %r no es R, G ni B' % simb)
            continue
        if cfg.get('lado') is not None:
            r, c = cubos[simb]
            if cfg['lado'] not in DIRS or lab.car(r + DIRS[cfg['lado']][0], c + DIRS[cfg['lado']][1]) != '#':
                err('cubo %s: no hay pared al %s para apoyar el soporte' % (COLORES_CUBO[simb][0], cfg['lado']))

    for ob in lab.objetos:
        if ob.get('tipo') not in TIPOS_OBJETO:
            err('tipo de objeto desconocido: %r (validos: %s)' % (ob.get('tipo'), ', '.join(sorted(TIPOS_OBJETO))))
            return msg, infoval
        r, c = ob['celda']
        if lab.car(r, c) in '#Z':
            err('el objeto %s esta en la celda (%d,%d), que es pared o zona prohibida' % (ob['tipo'], r, c))
        elif ob.get('lado') not in (None, 'auto') and (ob['lado'] not in DIRS or
                                                       lab.car(r + DIRS[ob['lado']][0], c + DIRS[ob['lado']][1]) != '#'):
            err('el objeto %s de la celda (%d,%d): no hay pared al %s para apoyarlo' % (ob['tipo'], r, c, ob['lado']))
    for o in objetos_con_posicion(lab):
        mitad = lab.celda / 2.0
        cx, cy = lab.centro(*o['celda'])
        if abs(o['x'] - cx) + o['lado'] / 2.0 > mitad + 1e-9 or abs(o['y'] - cy) + o['lado'] / 2.0 > mitad + 1e-9:
            err('el objeto %s de la celda %s se sale de la celda' % (o['tipo'], o['celda']))
    for simb, (r, c) in cubos.items():
        if lab.car(r, c) == 'Z':
            err('el cubo %s esta dentro de una zona prohibida' % COLORES_CUBO[simb][0])

    libres, pasable = construir_grafo(lab)
    reducidas = {c: [e for e, ok in p.items() if not ok] for c, p in pasable.items() if not (p['h'] and p['v'])}
    if reducidas:
        info('celdas donde un objeto no deja pasar al robot (%.0f mm + holgura) segun el eje (h = este-oeste, v = norte-sur): %s'
             % (ROBOT_ANCHO_M * 1000, ', '.join('%s %s' % (c, '+'.join(e)) for c, e in sorted(reducidas.items()))))

    # Alcanzabilidad y rutas
    dist_cubo = {}
    for simb, pos in cubos.items():
        dist_cubo[simb] = distancias(lab, libres, pasable, pos)
        if s not in dist_cubo[simb]:
            err('el cubo %s no es alcanzable desde la zona de inicio' % COLORES_CUBO[simb][0])
            continue
        ruta = ruta_corta(lab, libres, pasable, s, pos)
        infoval['rutas'][simb] = ruta
        info('cubo %s: %d celdas desde S (%.2f m)' % (COLORES_CUBO[simb][0], len(ruta) - 1, (len(ruta) - 1) * lab.celda))
    for k, pos in deposito.items():
        if s not in distancias(lab, libres, pasable, pos):
            err('el deposito %s no es alcanzable desde la zona de inicio' % k)

    # Flechas
    vistas = set()
    for i, fl in enumerate(lab.flechas):
        etiqueta = 'flecha %d %s' % (i + 1, fl.get('celda'))
        try:
            r, c = fl['celda']
            pared = fl['pared']
            d = fl['dir']
            largo = int(fl['largo'])
            color = fl['color']
            tilt = float(fl.get('inclinacion', 0.0))
        except (KeyError, TypeError, ValueError) as e:
            err('%s: falta o es invalido un campo (%s)' % (etiqueta, e))
            continue
        if pared not in DIRS or d not in DIRS:
            err('%s: pared y dir deben ser N, E, S u O' % etiqueta)
            continue
        if color not in COLOR_FLECHA:
            err('%s: color %r desconocido (gris, rojo, verde, azul)' % (etiqueta, color))
            continue
        if largo not in FLECHA_CUERPO_LARGO_M:
            err('%s: largo debe ser 1, 2 o 3' % etiqueta)
            continue
        if abs(tilt) > FLECHA_TILT_MAX_GRADOS + 1e-9:
            err('%s: inclinacion %.1f supera el maximo reglamentario de +-%.0f grados'
                % (etiqueta, tilt, FLECHA_TILT_MAX_GRADOS))
        if (r, c) not in libres:
            err('%s: la celda no es libre' % etiqueta)
            continue
        dpr, dpc = DIRS[pared]
        if lab.car(r + dpr, c + dpc) != '#':
            err('%s: no hay pared al %s de esa celda' % (etiqueta, pared))
            continue
        if (d in 'NS') == (pared in 'NS'):
            err('%s: la flecha apunta %s y la pared es %s; tiene que apuntar a lo largo de la pared' % (etiqueta, d, pared))
            continue
        mh, Mh, mz, Mz = extension_flecha(lab, fl)
        if mh < -lab.celda / 2 + 0.002 or Mh > lab.celda / 2 - 0.002:
            err('%s: la flecha se sale de la cara del bloque a lo largo de la pared (de %.0f a %.0f mm; la cara va de +-%.0f)'
                % (etiqueta, mh * 1000, Mh * 1000, lab.celda / 2 * 1000))
        if mz < 0.005 or Mz > lab.alto - 0.005:
            err('%s: la flecha se sale de la cara del bloque en altura (de %.0f a %.0f mm; el bloque mide %.0f)'
                % (etiqueta, mz * 1000, Mz * 1000, lab.alto * 1000))
        pares = [g for g in lab.flechas[:i] if tuple(g.get('celda', ())) == (r, c) and g.get('pared') == pared]
        for g in pares:
            mh2, Mh2, mz2, Mz2 = extension_flecha(lab, g)
            if not (Mh <= mh2 or Mh2 <= mh or Mz <= mz2 or Mz2 <= mz):
                err('%s: se superpone con otra flecha de la misma pared' % etiqueta)
        vistas.add((r, c, pared))

        # la flecha no puede quedar tapada por el soporte de un cubo pegado a su misma pared
        for so in objetos_con_posicion(lab):
            if not so['soporte'] or so['celda'] != (r, c):
                continue
            cx, cy = lab.centro(r, c)
            vp = _vec_mundo(pared)
            tg = tangente_pared(pared)
            hacia_pared = (so['x'] - cx) * vp[0] + (so['y'] - cy) * vp[1]
            luz = lab.celda / 2.0 - (hacia_pared + so['lado'] / 2.0)
            t0 = (so['x'] - cx) * tg[0] + (so['y'] - cy) * tg[1]
            if luz < 0.02 and not (Mh <= t0 - so['lado'] / 2.0 or mh >= t0 + so['lado'] / 2.0) and mz < so['alto'] + CUBO_M:
                err('%s: queda tapada por el soporte de %s (esta pegado a esa misma pared)' % (etiqueta, so['origen']))

        # semantica segun el significado del color: la direccion es el primer paso de una ruta mas corta y el largo son
        # los pasos seguidos. Un color sin significado (por defecto, el gris) es decorativo: no se verifica ni se cuenta.
        objetivos = [o for o in lab.significado_colores.get(color, []) if o in cubos]
        if not objetivos:
            infoval['decorativas'] = infoval.get('decorativas', 0) + 1
            continue
        largos = []
        for simb in objetivos:
            largos.append(largo_de_flecha(lab, libres, pasable, dist_cubo[simb], (r, c), d))
        if any(L is None for L in largos):
            nombres = ', '.join(COLORES_CUBO[o][0] for o in objetivos)
            err('%s: %s no es el primer paso de una ruta mas corta al cubo %s (el color %s significa: %s)'
                % (etiqueta, d, nombres, color, nombres))
            continue
        if len(set(min(3, L) for L in largos)) > 1:
            err('%s: el color %s significa varios cubos y dan largos distintos %s' % (etiqueta, color, [min(3, L) for L in largos]))
            continue
        real = min(3, largos[0])
        if real != largo:
            err('%s: declara largo %d pero a partir de ahi hay %d paso(s) recto(s) antes de la curva (el largo 3 significa 3 o mas)'
                % (etiqueta, largo, real))
            continue
        infoval['flechas'].append((r, c, pared, d, largo, color))

    # Cobertura: en cada bifurcacion de la ruta a un cubo tiene que haber una flecha de su color (o gris)
    for simb, ruta in infoval['rutas'].items():
        nombre = COLORES_CUBO[simb][0]
        for k in range(1, len(ruta) - 1):      # k = 0 es la zona de inicio
            u = ruta[k]
            if en_zona_abierta(lab, u):
                continue                        # en un area abierta no hay pasillos ni bifurcaciones
            salidas = [d for d, v in vecinos(lab, libres, pasable, u)]
            if len(salidas) >= 3:
                cerca = set([u] + ([ruta[k - 1]] if k > 0 else []))
                d_in = [d for d, (dr, dc) in DIRS.items() if (ruta[k - 1][0] + dr, ruta[k - 1][1] + dc) == u][0]
                d_out = [d for d, (dr, dc) in DIRS.items() if (u[0] + dr, u[1] + dc) == ruta[k + 1]][0]
                pared_frente = lab.car(u[0] + DIRS[d_in][0], u[1] + DIRS[d_in][1])
                if d_out != d_in and pared_frente != '#':
                    aviso('bifurcacion pasante en %s: la ruta al cubo %s dobla hacia %s y ninguna flecha puede indicarlo '
                          '(va a lo largo de la pared); se resuelve explorando' % (u, nombre, d_out))
                elif not any((f[0], f[1]) in cerca and simb in lab.significado_colores.get(f[5], []) for f in infoval['flechas']):
                    aviso('bifurcacion en %s sin una flecha que indique el cubo %s en esa celda ni en la anterior' % (u, nombre))

    # Cubos y canastos a la altura de la base del brazo
    for simb, (r, c) in cubos.items():
        x, y, lado = posicion_cubo(lab, simb, (r, c))
        donde = ('pegado a la pared %s' % lado) if lado else 'al centro de la celda'
        info('cubo %s: apoyado a %.1f mm del piso, %s' % (COLORES_CUBO[simb][0], lab.altura_apoyo * 1000, donde))
    info('canastos: borde de la pared trasera a %.1f mm del piso (base a %.1f mm)'
         % (lab.altura_borde_canasto * 1000, (lab.altura_borde_canasto - CANASTO_ALTO_M) * 1000))

    # Medidas del robot contra el pasillo
    diag = math.hypot(ROBOT_LARGO_M, ROBOT_ANCHO_M)
    if diag > lab.celda:
        info('giro en el lugar en un pasillo recto: la diagonal del robot (%.1f mm, sin brazo) supera el pasillo '
             '(%.0f mm) por %.1f mm' % (diag * 1000, lab.celda * 1000, (diag - lab.celda) * 1000))
    info('pasillo %.0f mm, robot %.0f mm de ancho: holgura %.1f mm por lado'
         % (lab.celda * 1000, ROBOT_ANCHO_M * 1000, (lab.celda - ROBOT_ANCHO_M) / 2 * 1000))
    clases = sorted(set(f[4] for f in infoval['flechas']))
    info('flechas con significado: %d (largos presentes: %s), decorativas (se ignoran): %d; zonas prohibidas: %d celdas; obstaculos: %d'
         % (len(infoval['flechas']), clases, infoval.get('decorativas', 0), len(lab.buscar('Z')), len(lab.objetos)))
    infoval['s'] = s
    infoval['cubos'] = cubos
    infoval['deposito'] = deposito
    return msg, infoval


# ---------------------------------------------------------------------------------------------
# Geometria 3D de las flechas
# ---------------------------------------------------------------------------------------------
def _norm(v):
    n = math.sqrt(sum(a * a for a in v))
    return tuple(a / n for a in v)


def _cruz(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _vec_mundo(d):
    """Vector unitario horizontal del mundo para N, E, S, O (norte = +y, este = +x)."""
    return {'N': (0.0, 1.0, 0.0), 'S': (0.0, -1.0, 0.0), 'E': (1.0, 0.0, 0.0), 'O': (-1.0, 0.0, 0.0)}[d]


def marco_flecha(pared, d, tilt_grados):
    """Marco local de una flecha: (u, arriba, normal_al_espectador).

    u = eje de la flecha (hacia donde apunta la punta), ya inclinado; arriba = perpendicular a u dentro de la
    pared, con z positiva. La inclinacion es positiva en sentido antihorario visto desde el pasillo.
    """
    n_pared = _vec_mundo(pared)                         # del centro de la celda hacia la pared
    mirada_a_la_pared = n_pared                         # el espectador mira hacia la pared
    z = (0.0, 0.0, 1.0)
    derecha = _cruz(mirada_a_la_pared, z)               # mirando a la pared, la derecha del espectador
    D = _vec_mundo(d)
    sentido = 1.0 if sum(a * b for a, b in zip(D, derecha)) > 0 else -1.0   # +1 apunta a la derecha del espectador
    t = math.radians(tilt_grados)
    # rotacion antihoraria de D en el plano (derecha, arriba)
    if sentido > 0:
        u = tuple(math.cos(t) * a + math.sin(t) * b for a, b in zip(derecha, z))
    else:
        izq = tuple(-a for a in derecha)
        u = tuple(math.cos(t) * a - math.sin(t) * b for a, b in zip(izq, z))
    u = _norm(u)
    # perpendicular a u dentro del plano de la pared, con z > 0
    p = _cruz(n_pared, u)
    if p[2] < 0:
        p = tuple(-a for a in p)
    p = _norm(p)
    normal = tuple(-a for a in n_pared)
    return u, p, normal


def matriz_a_rpy(cx, cy, cz):
    """Matriz con columnas (cx, cy, cz) -> (roll, pitch, yaw) de SDF (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    r00, r10, r20 = cx
    r21, r22 = cy[2], cz[2]
    pitch = -math.asin(max(-1.0, min(1.0, r20)))
    if abs(math.cos(pitch)) < 1e-9:
        roll, yaw = 0.0, math.atan2(-cy[0], cy[1])
    else:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    return roll, pitch, yaw


def rpy_a_matriz(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def piezas_flecha(largo):
    """Cajas (centro_u, ancho_u, alto) en el marco local de la flecha, con el origen en el centro de su longitud."""
    Lc = FLECHA_CUERPO_LARGO_M[largo]
    total = Lc + FLECHA_PUNTA_ALTO_M
    u0 = -total / 2.0
    piezas = [(u0 + Lc / 2.0, Lc, FLECHA_CUERPO_ANCHO_M)]
    paso = FLECHA_PUNTA_ALTO_M / FLECHA_TIRAS_PUNTA
    for i in range(FLECHA_TIRAS_PUNTA):
        um = u0 + Lc + (i + 0.5) * paso
        alto = FLECHA_PUNTA_ANCHO_M * (1.0 - i / FLECHA_TIRAS_PUNTA)    # la primera tira mide los 70 mm de la punta
        piezas.append((um, paso, alto))
    return piezas


# ---------------------------------------------------------------------------------------------
# SDF
# ---------------------------------------------------------------------------------------------
def f(x):
    return ('%.5f' % x).rstrip('0').rstrip('.') if abs(x) > 1e-12 else '0'


def material(rgb, indent):
    s = ' ' * indent
    return ('%s<material>\n%s  <ambient>%s %s %s 1</ambient>\n%s  <diffuse>%s %s %s 1</diffuse>\n%s</material>\n'
            % (s, s, f(rgb[0]), f(rgb[1]), f(rgb[2]), s, f(rgb[0]), f(rgb[1]), f(rgb[2]), s))


def caja_visual(nombre, pos, size, rgb, rpy=(0, 0, 0), indent=8):
    s = ' ' * indent
    return ('%s<visual name="%s">\n%s  <pose>%s %s %s %s %s %s</pose>\n%s  <geometry><box><size>%s %s %s</size></box></geometry>\n%s%s'
            % (s, nombre, s, f(pos[0]), f(pos[1]), f(pos[2]), f(rpy[0]), f(rpy[1]), f(rpy[2]), s,
               f(size[0]), f(size[1]), f(size[2]), material(rgb, indent + 2), s + '</visual>\n'))


def caja_colision(nombre, pos, size, indent=8, friccion=None):
    s = ' ' * indent
    surf = ''
    if friccion:
        surf = ('%s  <surface><friction><ode><mu>%s</mu><mu2>%s</mu2></ode></friction></surface>\n'
                % (s, f(friccion), f(friccion)))
    return ('%s<collision name="%s">\n%s  <pose>%s %s %s 0 0 0</pose>\n%s  <geometry><box><size>%s %s %s</size></box></geometry>\n%s%s</collision>\n'
            % (s, nombre, s, f(pos[0]), f(pos[1]), f(pos[2]), s, f(size[0]), f(size[1]), f(size[2]), surf, s))


def rectangulos_de_pared(lab):
    """Une bloques contiguos en rectangulos (menos entidades en Gazebo). Devuelve (r, c, alto_celdas, ancho_celdas)."""
    usado = [[False] * lab.ncols for _ in range(lab.nfilas)]
    res = []
    for r in range(lab.nfilas):
        for c in range(lab.ncols):
            if lab.car(r, c) != '#' or usado[r][c]:
                continue
            w = 1
            while c + w < lab.ncols and lab.car(r, c + w) == '#' and not usado[r][c + w]:
                w += 1
            h = 1
            while r + h < lab.nfilas and all(lab.car(r + h, c + k) == '#' and not usado[r + h][c + k] for k in range(w)):
                h += 1
            for dr in range(h):
                for dc in range(w):
                    usado[r + dr][c + dc] = True
            res.append((r, c, h, w))
    return res


def rectangulos_de_zonas(lab):
    """Une celdas Z contiguas en rectangulos: (r, c, h, w)."""
    usado = [[False] * lab.ncols for _ in range(lab.nfilas)]
    res = []
    for r in range(lab.nfilas):
        for c in range(lab.ncols):
            if lab.car(r, c) != 'Z' or usado[r][c]:
                continue
            w = 1
            while c + w < lab.ncols and lab.car(r, c + w) == 'Z' and not usado[r][c + w]:
                w += 1
            h = 1
            while r + h < lab.nfilas and all(lab.car(r + h, c + k) == 'Z' and not usado[r + h][c + k] for k in range(w)):
                h += 1
            for dr in range(h):
                for dc in range(w):
                    usado[r + dr][c + dc] = True
            res.append((r, c, h, w))
    return res


def tangente_pared(pared):
    """Vector horizontal paralelo a la pared: este para N/S, norte para E/O (el desplazamiento positivo va hacia el)."""
    return (1.0, 0.0, 0.0) if pared in 'NS' else (0.0, 1.0, 0.0)


def posicion_flecha(lab, fl):
    """Centro de la flecha en el mundo y marco local (u, p, normal).

    desplaza_mm: corrimiento a lo largo de la pared (positivo hacia el este en paredes N/S, hacia el norte en E/O).
    altura_mm: altura del centro sobre el piso (por defecto `altura_flechas_mm` del laberinto, 115 mm).
    """
    r, c, pared = fl['celda'][0], fl['celda'][1], fl['pared']
    cx, cy = lab.centro(r, c)
    n = _vec_mundo(pared)
    t = tangente_pared(pared)
    desp = float(fl.get('desplaza_mm', 0.0)) / 1000.0
    alt = float(fl.get('altura_mm', lab.altura_flechas * 1000.0)) / 1000.0
    prof = lab.celda / 2.0 - 0.0005 - FLECHA_GROSOR_M / 2.0     # 0,5 mm de luz hasta la cara del bloque
    centro = (cx + n[0] * prof + t[0] * desp, cy + n[1] * prof + t[1] * desp, alt)
    u, p, normal = marco_flecha(pared, fl['dir'], float(fl.get('inclinacion', 0.0)))
    return centro, (u, p, normal)


def extension_flecha(lab, fl):
    """(min_h, max_h, min_z, max_z) de la flecha respecto del centro de la celda: h a lo largo de la pared (m)."""
    centro, (u, p, _) = posicion_flecha(lab, fl)
    cx, cy = lab.centro(fl['celda'][0], fl['celda'][1])
    t = tangente_pared(fl['pared'])
    Lc = FLECHA_CUERPO_LARGO_M[int(fl['largo'])]
    total = Lc + FLECHA_PUNTA_ALTO_M
    a0 = -total / 2.0
    pts = [(a0, FLECHA_CUERPO_ANCHO_M / 2), (a0, -FLECHA_CUERPO_ANCHO_M / 2),
           (a0 + Lc, FLECHA_PUNTA_ANCHO_M / 2), (a0 + Lc, -FLECHA_PUNTA_ANCHO_M / 2), (total / 2, 0.0)]
    hs, zs = [], []
    for a, b in pts:
        v = tuple(a * ui + b * pi for ui, pi in zip(u, p))
        hs.append((centro[0] - cx + v[0]) * t[0] + (centro[1] - cy + v[1]) * t[1])
        zs.append(centro[2] + v[2])
    return min(hs), max(hs), min(zs), max(zs)


def generar_sdf(lab, infoval):
    C = lab.celda
    H = lab.alto
    s_r, s_c = infoval['s']
    sx, sy = lab.centro(s_r, s_c)
    out = []
    w = out.append
    w('<?xml version="1.0"?>\n')
    w('<!-- Generado por gen_maze.py a partir de mazes/%s.yaml. No editar a mano: editar el YAML y regenerar. -->\n' % lab.id)
    w('<!-- %s -->\n' % lab.nombre)
    w('<!-- Inicio del robot: x=%s y=%s yaw=%s -->\n' % (f(sx), f(sy), f(YAW[lab.inicio_dir])))
    w('<sdf version="1.9">\n  <world name="ros2_world">\n\n')
    w('''    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>

    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

''')
    w('''    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>30 30</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>30 30</size>
            </plane>
          </geometry>
%s        </visual>
      </link>
    </model>

''' % material(COLOR_PISO, 10))

    # Paredes
    w('    <model name="paredes">\n      <static>true</static>\n')
    for i, (r, c, h, ancho) in enumerate(rectangulos_de_pared(lab)):
        x = (c + ancho / 2.0) * C
        y = (lab.nfilas - r - h / 2.0) * C
        w('      <link name="bloque_%d">\n        <pose>%s %s %s 0 0 0</pose>\n' % (i, f(x), f(y), f(H / 2.0)))
        w(caja_colision('colision', (0, 0, 0), (ancho * C, h * C, H)))
        w(caja_visual('visual', (0, 0, 0), (ancho * C, h * C, H), COLOR_PARED))
        w('      </link>\n')
    w('    </model>\n\n')

    # Zonas prohibidas (solo visual, sobre el piso)
    zonas = rectangulos_de_zonas(lab)
    if zonas:
        w('    <model name="zonas_prohibidas">\n      <static>true</static>\n')
        for i, (r, c, h, ancho) in enumerate(zonas):
            x0 = c * C + ZONA_INSET_M
            x1 = (c + ancho) * C - ZONA_INSET_M
            y1 = (lab.nfilas - r) * C - ZONA_INSET_M
            y0 = (lab.nfilas - r - h) * C + ZONA_INSET_M
            lw = ZONA_LINEA_M
            barras = [('norte', ((x0 + x1) / 2, y1 - lw / 2), (x1 - x0, lw)),
                      ('sur', ((x0 + x1) / 2, y0 + lw / 2), (x1 - x0, lw)),
                      ('este', (x1 - lw / 2, (y0 + y1) / 2), (lw, y1 - y0 - 2 * lw)),
                      ('oeste', (x0 + lw / 2, (y0 + y1) / 2), (lw, y1 - y0 - 2 * lw))]
            w('      <link name="zona_%d">\n' % i)
            for nom, (px, py), (sx_, sy_) in barras:
                w(caja_visual(nom, (px, py, 0.0005), (sx_, sy_, 0.001), COLOR_ZONA_ROJA, indent=8))
            w('      </link>\n')
        w('    </model>\n\n')

    # Flechas: un modelo estatico con un link por flecha, solo visual
    if lab.flechas:
        w('    <model name="flechas">\n      <static>true</static>\n')
        for i, fl in enumerate(lab.flechas):
            centro, (u, p, normal) = posicion_flecha(lab, fl)
            # marco del link: X = eje de la flecha, Z = arriba en la pared, Y = Z x X (normal a la pared)
            y = _cruz(p, u)
            roll, pitch, yaw = matriz_a_rpy(u, y, p)
            color = COLOR_FLECHA[fl['color']]
            w('      <link name="flecha_%d">\n        <pose>%s %s %s %s %s %s</pose>\n'
              % (i, f(centro[0]), f(centro[1]), f(centro[2]), f(roll), f(pitch), f(yaw)))
            for j, (uc, ancho_u, alto) in enumerate(piezas_flecha(int(fl['largo']))):
                w(caja_visual('p%d' % j, (uc, 0, 0), (ancho_u, FLECHA_GROSOR_M, alto), color, indent=8))
            w('      </link>\n')
        w('    </model>\n\n')

    # Canastos: sobre una columna, con el borde de la pared trasera a la altura pedida (por defecto, la base del brazo)
    for k, (r, c) in sorted(infoval['deposito'].items()):
        px, py = posicion_canasto(lab, (r, c))
        simb = {'1': 'R', '2': 'G', '3': 'B'}[k]
        nombre, rgb = COLORES_CUBO[simb]
        base = lab.altura_borde_canasto - CANASTO_ALTO_M
        if base > 0.002:
            w('    <model name="soporte_canasto_%s">\n      <static>true</static>\n      <pose>%s %s %s 0 0 0</pose>\n      <link name="link">\n'
              % (nombre, f(px), f(py), f(base / 2.0)))
            size = (CANASTO_SOPORTE_M, CANASTO_SOPORTE_M, base)
            w(caja_colision('colision', (0, 0, 0), size))
            w(caja_visual('visual', (0, 0, 0), size, COLOR_SOPORTE))
            w('      </link>\n    </model>\n\n')
        w('    <model name="canasto_%s">\n      <static>true</static>\n      <pose>%s %s %s 0 0 0</pose>\n      <link name="link">\n'
          % (nombre, f(px), f(py), f(max(base, 0.0))))
        E, T, F, A = CANASTO_EXT_M, CANASTO_PARED_M, CANASTO_FONDO_M, CANASTO_ALTO_M
        partes = [('fondo', (0, 0, F / 2), (E, E, F)),
                  ('pared_n', (0, E / 2 - T / 2, A / 2), (E, T, A)),
                  ('pared_s', (0, -E / 2 + T / 2, A / 2), (E, T, A)),
                  ('pared_e', (E / 2 - T / 2, 0, A / 2), (T, E - 2 * T, A)),
                  ('pared_o', (-E / 2 + T / 2, 0, A / 2), (T, E - 2 * T, A))]
        for nom, pos, size in partes:
            w(caja_colision('c_' + nom, pos, size, friccion=1.0))
            w(caja_visual('v_' + nom, pos, size, rgb))
        w('      </link>\n    </model>\n\n')

    # Cubos objetivo: sobre su soporte, a la altura de la base del brazo
    for simb, (r, c) in sorted(infoval['cubos'].items()):
        nombre, rgb = COLORES_CUBO[simb]
        x, y, _ = posicion_cubo(lab, simb, (r, c))
        z_base = lab.altura_apoyo if lab.cubos_cfg.get(simb, {}).get('elevado', True) else 0.0
        w(cubo_sdf('cubo_' + nombre, x, y, rgb, z_base))

    # Soportes, obstaculos y distractores
    for ob in objetos_con_posicion(lab):
        if ob['soporte']:
            w('    <model name="soporte_%s">\n      <static>true</static>\n      <pose>%s %s %s 0 0 0</pose>\n      <link name="link">\n'
              % (ob['origen'], f(ob['x']), f(ob['y']), f(ob['alto'] / 2.0)))
            size = (ob['lado'], ob['lado'], ob['alto'])
            w(caja_colision('colision', (0, 0, 0), size))
            w(caja_visual('visual', (0, 0, 0), size, ob['color']))
            w('      </link>\n    </model>\n\n')
        elif ob['dinamico']:
            w(cubo_sdf('%s_%d' % (ob['tipo'], ob['indice']), ob['x'], ob['y'], ob['color'], ob['z_base']))
        else:
            w('    <model name="%s_%d">\n      <static>true</static>\n      <pose>%s %s %s 0 0 0</pose>\n      <link name="link">\n'
              % (ob['tipo'], ob['indice'], f(ob['x']), f(ob['y']), f(ob['alto'] / 2.0)))
            size = (ob['lado'], ob['lado'], ob['alto'])
            w(caja_colision('colision', (0, 0, 0), size))
            w(caja_visual('visual', (0, 0, 0), size, ob['color']))
            w('      </link>\n    </model>\n\n')

    w('  </world>\n</sdf>\n')
    return ''.join(out)


def cubo_sdf(nombre, x, y, rgb, z_base=0.0):
    a = CUBO_M
    m = CUBO_MASA_KG
    i = m * a * a / 6.0
    s = ('    <model name="%s">\n      <pose>%s %s %s 0 0 0</pose>\n      <link name="link">\n'
         '        <inertial>\n          <mass>%s</mass>\n          <inertia>\n'
         '            <ixx>%.4e</ixx><iyy>%.4e</iyy><izz>%.4e</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>\n'
         '          </inertia>\n        </inertial>\n'
         % (nombre, f(x), f(y), f(z_base + a / 2.0 + 0.0002), f(m), i, i, i))
    s += caja_colision('colision', (0, 0, 0), (a, a, a), friccion=1.0)
    s += caja_visual('visual', (0, 0, 0), (a, a, a), rgb)
    s += '      </link>\n    </model>\n\n'
    return s


# ---------------------------------------------------------------------------------------------
# SVG (vista desde arriba, para revisar el laberinto sin abrir Gazebo)
# ---------------------------------------------------------------------------------------------
def generar_svg(lab, infoval, con_rutas=False):
    px = 40
    ancho = lab.ncols * px
    alto = lab.nfilas * px
    margen = 30
    o = []
    w = o.append
    w('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" font-family="sans-serif">\n'
      % (ancho + 2 * margen, alto + 2 * margen + 34, ancho + 2 * margen, alto + 2 * margen + 34))
    w('<rect width="100%" height="100%" fill="#ffffff"/>\n')
    w('<text x="%d" y="20" font-size="14" font-weight="bold">%s</text>\n' % (margen, lab.nombre))
    w('<g transform="translate(%d,%d)">\n' % (margen, margen))

    def xy(r, c):
        return (c + 0.5) * px, (r + 0.5) * px

    for r in range(lab.nfilas):
        for c in range(lab.ncols):
            ch = lab.car(r, c)
            x, y = c * px, r * px
            if ch == '#':
                w('<rect x="%d" y="%d" width="%d" height="%d" fill="#555"/>\n' % (x, y, px, px))
            elif ch == '~':
                w('<rect x="%d" y="%d" width="%d" height="%d" fill="#e6f1e2" stroke="#cfe3c8" stroke-width="0.5"/>\n' % (x, y, px, px))
            else:
                w('<rect x="%d" y="%d" width="%d" height="%d" fill="#f4f4f4" stroke="#ddd" stroke-width="0.5"/>\n' % (x, y, px, px))
    for (r, c, h, ancho) in rectangulos_de_zonas(lab):
        w('<rect x="%d" y="%d" width="%d" height="%d" fill="rgba(220,0,0,0.12)" stroke="#d00" stroke-width="2"/>\n'
          % (c * px + 3, r * px + 3, ancho * px - 6, h * px - 6))
    colores_rutas = {'R': '#e00', 'G': '#0a0', 'B': '#03f'}
    if con_rutas:
        for simb, ruta in infoval['rutas'].items():
            pts = ' '.join('%.1f,%.1f' % xy(r, c) for r, c in ruta)
            w('<polyline points="%s" fill="none" stroke="%s" stroke-width="3" stroke-dasharray="6,4" opacity="0.7"/>\n'
              % (pts, colores_rutas[simb]))
    # deposito
    mapa_dep = {'1': '#e00', '2': '#0a0', '3': '#03f'}

    def a_pixel(x_m, y_m):
        return x_m / lab.celda * px, (lab.nfilas * lab.celda - y_m) / lab.celda * px

    for k, (r, c) in infoval['deposito'].items():
        bx, by = a_pixel(*posicion_canasto(lab, (r, c)))
        w('<rect x="%.1f" y="%.1f" width="12" height="12" fill="none" stroke="%s" stroke-width="3"/>\n' % (bx - 6, by - 6, mapa_dep[k]))
        w('<text x="%.1f" y="%.1f" font-size="9" fill="%s" text-anchor="middle">%s</text>\n' % (bx, by + 22, mapa_dep[k], k))
    # inicio
    s_r, s_c = infoval['s']
    x, y = xy(s_r, s_c)
    dr, dc = DIRS[lab.inicio_dir]
    w('<circle cx="%.1f" cy="%.1f" r="12" fill="#fc0" stroke="#850" stroke-width="2"/>\n' % (x, y))
    w('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#850" stroke-width="3"/>\n' % (x, y, x + dc * 16, y + dr * 16))
    w('<text x="%.1f" y="%.1f" font-size="10" text-anchor="middle" font-weight="bold">S</text>\n' % (x, y + 3))
    # objetos: primero los soportes, despues lo que va encima
    objs = objetos_con_posicion(lab)
    for ob in sorted(objs, key=lambda o: not o['soporte']):
        ox, oy = a_pixel(ob['x'], ob['y'])
        lado = max(ob['lado'] / lab.celda * px, 5)
        rgb = '#%02x%02x%02x' % tuple(int(255 * v) for v in ob['color'])
        w('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" stroke="#000" stroke-width="1"/>\n'
          % (ox - lado / 2, oy - lado / 2, lado, lado, rgb))
    # cubos objetivo
    for simb, (r, c) in infoval['cubos'].items():
        x_m, y_m, _ = posicion_cubo(lab, simb, (r, c))
        ox, oy = a_pixel(x_m, y_m)
        w('<rect x="%.1f" y="%.1f" width="8" height="8" fill="%s" stroke="#000" stroke-width="1"/>\n' % (ox - 4, oy - 4, colores_rutas[simb]))
    # flechas: se dibujan pegadas a su pared, con el largo real a escala y la inclinacion
    for fl in lab.flechas:
        r, c = fl['celda']
        pared = fl['pared']
        d = fl['dir']
        x, y = xy(r, c)
        pr, pc = DIRS[pared]
        ax, ay = x + pc * (px / 2 - 4), y + pr * (px / 2 - 4)
        dd = DIRS[d]
        Lc = FLECHA_CUERPO_LARGO_M[int(fl['largo'])] / lab.celda * px
        Lh = FLECHA_PUNTA_ALTO_M / lab.celda * px
        total = Lc + Lh
        ux, uy = dd[1], dd[0]
        nx, ny = -uy, ux
        wb = 3.0
        wh = FLECHA_PUNTA_ANCHO_M / lab.celda * px / 2
        pts = [(-total / 2, wb / 2), (-total / 2 + Lc, wb / 2), (-total / 2 + Lc, wh), (total / 2, 0),
               (-total / 2 + Lc, -wh), (-total / 2 + Lc, -wb / 2), (-total / 2, -wb / 2)]
        # en la vista desde arriba la flecha se ve de canto: se achica el ancho para que no tape el pasillo
        pts = [(a_, b_ * 0.25) for a_, b_ in pts]
        poli = ' '.join('%.1f,%.1f' % (ax + a_ * ux + b_ * nx, ay + a_ * uy + b_ * ny) for a_, b_ in pts)
        col = '#%02x%02x%02x' % tuple(int(255 * v) for v in COLOR_FLECHA[fl['color']])
        w('<polygon points="%s" fill="%s" stroke="#000" stroke-width="0.4"/>\n' % (poli, col))
        w('<text x="%.1f" y="%.1f" font-size="8" fill="#000" text-anchor="middle">%d</text>\n'
          % (ax - pc * 8, ay - pr * 8 + 3, int(fl['largo'])))
    w('</g>\n')
    # leyenda
    ly = alto + 2 * margen + 6
    w('<text x="%d" y="%d" font-size="10" fill="#333">%d filas x %d columnas, celda %.0f mm. S = inicio (la linea apunta adonde mira).</text>\n'
      % (margen, ly, lab.nfilas, lab.ncols, lab.celda * 1000))
    w('<text x="%d" y="%d" font-size="10" fill="#333">Cuadrados de color = cubos; marcos de color = canastos; '
      'cintas pegadas a la pared = flechas (de canto, con su largo).</text>\n' % (margen, ly + 13))
    w('</svg>\n')
    return ''.join(o)


# ---------------------------------------------------------------------------------------------
# Sugerencia de flechas (ayuda para escribir un laberinto nuevo; las flechas finales se editan a mano en el YAML)
# ---------------------------------------------------------------------------------------------
RANURAS = {1: [(0, 0)], 2: [(-80, 0), (80, 0)], 3: [(-80, 0), (80, 10), (80, -70)],
           4: [(-80, 10), (-80, -70), (80, 10), (80, -70)]}      # (desplazamiento mm, altura respecto de la base mm)


def sugerir_flechas(lab, inclinaciones=None):
    """Propone flechas con la convencion del proyecto:
    - una en la pared frontal de cada curva de la ruta, apuntando al lado en que gira (el largo es lo que sigue recto);
    - una en la primera celda del pasillo y en cada bifurcacion pasante si la ruta sigue recto (en una pared lateral);
    - una flecha por cubo, del color que `significado_colores` asigna a ese cubo (no se usan las grises, que no significan nada).
    Las coloca en ranuras de la pared (hasta 4) y las deja sin inclinar cuando comparten pared."""
    libres, pasable = construir_grafo(lab)
    s = lab.buscar('S')[0]
    cubos = {simb: lab.buscar(simb)[0] for simb in 'RGB'}
    dist = {simb: distancias(lab, libres, pasable, pos) for simb, pos in cubos.items()}
    rutas = {simb: ruta_corta(lab, libres, pasable, s, pos) for simb, pos in cubos.items()}

    def dir_de(a, b):
        for d, (dr, dc) in DIRS.items():
            if (a[0] + dr, a[1] + dc) == b:
                return d

    def cerrada(u, d):
        return lab.car(u[0] + DIRS[d][0], u[1] + DIRS[d][1]) == '#'

    cand = {}     # (celda, pared, dir) -> cubos cuya ruta usa esa flecha
    for simb, ruta in rutas.items():
        primero = next((k for k in range(1, len(ruta) - 1) if not en_zona_abierta(lab, ruta[k])), None)
        for k in range(1, len(ruta) - 1):
            u, v = ruta[k], ruta[k + 1]
            if en_zona_abierta(lab, u):
                continue
            d_out = dir_de(u, v)
            d_in = dir_de(ruta[k - 1], u)
            if d_out != d_in:
                if cerrada(u, d_in):
                    cand.setdefault((u, d_in, d_out), set()).add(simb)
            else:
                pasante = len(list(vecinos(lab, libres, pasable, u))) >= 3
                if k == primero or pasante:
                    for lado in (('N', 'S') if d_out in 'EO' else ('E', 'O')):
                        if cerrada(u, lado):
                            cand.setdefault((u, lado, d_out), set()).add(simb)
                            break
    color_de = {}
    for color in ('rojo', 'verde', 'azul'):
        objetivos = lab.significado_colores.get(color, [])
        if len(objetivos) == 1 and objetivos[0] not in color_de:
            color_de[objetivos[0]] = color
    por_pared = {}
    for (u, pared, d), simbs in sorted(cand.items()):
        for simb in sorted(simbs):
            largo = largo_de_flecha(lab, libres, pasable, dist[simb], u, d)
            if largo is not None and simb in color_de:
                por_pared.setdefault((u, pared), []).append({'dir': d, 'largo': min(3, largo), 'color': color_de[simb]})
    salida = []
    k = 0
    for (u, pared), lista in sorted(por_pared.items()):
        lista = lista[:4]
        ranuras = RANURAS[len(lista)]
        for i, a in enumerate(lista):
            desp, dalt = ranuras[i]
            tilt = inclinaciones[k % len(inclinaciones)] if inclinaciones and len(lista) == 1 else 0.0
            k += 1
            e = {'celda': [u[0], u[1]], 'pared': pared, 'dir': a['dir'], 'largo': a['largo'], 'color': a['color']}
            if tilt:
                e['inclinacion'] = tilt
            if desp:
                e['desplaza_mm'] = desp
            if dalt:
                e['altura_mm'] = round(lab.altura_flechas * 1000.0 + dalt, 1)
            salida.append(e)
    return salida


# ---------------------------------------------------------------------------------------------
# Programa
# ---------------------------------------------------------------------------------------------
def main(argv=None):
    aqui = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('archivos', nargs='+', help='uno o mas YAML de laberinto')
    ap.add_argument('--salida', default=os.path.join(aqui, '..', 'worlds'), help='carpeta de los .world (por defecto ../worlds)')
    ap.add_argument('--svg', default=None, help='si se da, carpeta donde escribir la vista SVG de cada laberinto')
    ap.add_argument('--rutas', action='store_true', help='dibujar en el SVG la ruta mas corta a cada cubo')
    ap.add_argument('--solo-validar', action='store_true', help='validar sin escribir archivos')
    ap.add_argument('--sugerir-flechas', action='store_true', help='imprimir flechas sugeridas (YAML) para pegar en el archivo')
    args = ap.parse_args(argv)

    hay_error = False
    for ruta in args.archivos:
        lab = cargar(ruta)
        msgs, infoval = validar(lab)
        print('=== %s (%s)' % (lab.id, lab.nombre))
        for nivel, texto in msgs:
            print('  %-5s %s' % (nivel, texto))
        errores = [m for m in msgs if m[0] == 'ERROR']
        if errores:
            hay_error = True
            print('  -> NO se genera el mundo (%d errores)' % len(errores))
            continue
        if args.sugerir_flechas:
            sugeridas = sugerir_flechas(lab, [0, 12, -9, 18, -15, 6, -21])
            print(yaml.safe_dump({'flechas': sugeridas}, allow_unicode=True, default_flow_style=None, sort_keys=False))
            continue
        if args.solo_validar:
            continue
        os.makedirs(args.salida, exist_ok=True)
        destino = os.path.join(args.salida, lab.id + '.world')
        with open(destino, 'w', encoding='utf-8') as fh:
            fh.write(generar_sdf(lab, infoval))
        sx, sy = lab.centro(*infoval['s'])
        print('  mundo: %s' % os.path.normpath(destino))
        print('  lanzar: ros2 launch caid_bringup sim.launch.py world:=%s x:=%s y:=%s yaw:=%s'
              % (os.path.abspath(destino), f(sx), f(sy), f(YAW[lab.inicio_dir])))
        if args.svg:
            os.makedirs(args.svg, exist_ok=True)
            with open(os.path.join(args.svg, lab.id + '.svg'), 'w', encoding='utf-8') as fh:
                fh.write(generar_svg(lab, infoval, args.rutas))
    return 1 if hay_error else 0


if __name__ == '__main__':
    sys.exit(main())
