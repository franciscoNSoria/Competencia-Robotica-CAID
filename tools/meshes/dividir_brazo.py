#!/usr/bin/env python3
"""Divide el brazo del ensamblado de Fusion en una malla por pieza móvil.

Uso:  python3 tools/meshes/dividir_brazo.py ENSAMBLADO.stl CARPETA_DE_SALIDA

Cada malla queda en el "marco del brazo": las coordenadas del ensamblado, pero con el origen en el
centro de la base del brazo y z medido desde la cara superior de la placa de arriba. No se gira ni
se espeja nada: en este marco el brazo apunta hacia +x, y en el robot eso es hacia atrás.
"""
import struct
import sys

import numpy as np

Z_PLACA_SUPERIOR = 64.0   # mm: cara superior de la placa de arriba en el ensamblado de Fusion

# Qué pieza va con qué eslabón. La clave es la cantidad de triángulos de la pieza en el ensamblado.
# Si volvés a exportar el ensamblado con otra tolerancia, esos números cambian y hay que revisar
# esta tabla (el programa avisa si encuentra una pieza que no reconoce).
ESLABON_POR_TRIANGULOS = {
    6035: 'base',       # 0-1-10 Base
    600: 'base',        # 0-1-150 Arandela de separación
    1388: 'turret',     # 0-1-130 Plato circular
    24678: 'turret',    # 0-1-120 Soporte principal
    2116: 'turret',     # 0-1-80 Soporte de servo (hay dos)
    2250: 'turret',     # 0-1-70 Brazo para servo
    2638: 'upper',      # 0-1-140 Brazo principal
    9180: 'upper',      # 0-1-90 Soporte triangular
    18120: 'forearm',   # 0-1-100 Brazo superior
    4622: 'wrist',      # 0-1-30 Soporte de garras
    1302: 'claw_a',     # 0-1-60 Garra
    1288: 'claw_a',     # 0-1-20 Engranaje conductor
    1124: 'claw_b',     # 0-1-50 Garra
    1274: 'claw_b',     # 0-1-40 Engranaje conducido
}
# Las tres barras de unión (0-1-110 Articulación) tienen los mismos triángulos y se distinguen por posición.

def leer_stl(ruta):
    datos = open(ruta, 'rb').read()
    n = struct.unpack('<I', datos[80:84])[0]
    reg = np.frombuffer(datos[84:84 + 50 * n],
                        dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return reg['v'].reshape(-1, 3, 3).astype(float)


def escribir_stl(ruta, tri):
    n = len(tri)
    normales = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    largo = np.linalg.norm(normales, axis=1, keepdims=True)
    normales = np.divide(normales, largo, out=np.zeros_like(normales), where=largo > 0)
    reg = np.zeros(n, dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    reg['n'] = normales
    reg['v'] = tri.reshape(n, 9)
    with open(ruta, 'wb') as f:
        f.write(b'pieza del brazo, marco del brazo'.ljust(80, b' '))
        f.write(struct.pack('<I', n))
        f.write(reg.tobytes())

def separar_piezas(tri):
    """Devuelve, para cada triángulo, el número de pieza a la que pertenece."""
    vert = np.round(tri.reshape(-1, 3), 3)
    _, ids = np.unique(vert, axis=0, return_inverse=True)
    ids = ids.reshape(-1, 3)
    padre = list(range(ids.max() + 1))

    def raiz(x):
        while padre[x] != x:
            padre[x] = padre[padre[x]]
            x = padre[x]
        return x

    for a, b, c in ids:
        for p, q in ((a, b), (b, c)):
            rp, rq = raiz(p), raiz(q)
            if rp != rq:
                padre[rp] = rq
    return np.array([raiz(x) for x in ids[:, 0]])

def eslabon_de(cantidad, centro):
    """Devuelve el eslabón de una pieza, o None si no la reconoce."""
    if cantidad == 6348:                      # barra de unión: se distingue por su posición
        if centro[0] > 30:
            return 'wrist'                    # la barra que llega a la pinza
        return 'turret' if centro[1] < 0 else 'forearm'
    return ESLABON_POR_TRIANGULOS.get(cantidad)

def main(entrada, carpeta):
    tri = leer_stl(entrada)
    pieza = separar_piezas(tri)
    grupos = {}
    base = None
    for p in np.unique(pieza):
        t = tri[pieza == p]
        v = t.reshape(-1, 3)
        mn, mx = v.min(0), v.max(0)
        centro = (mn + mx) / 2
        # Es del brazo si está sobre la placa de arriba, tiene volumen y no es de la cámara
        if not (mn[2] >= Z_PLACA_SUPERIOR - 0.5 and (mx[2] - mn[2]) > 0.5 and centro[0] > -50):
            continue
        eslabon = eslabon_de(len(t), centro)
        print(f'  {len(t):6d} triángulos  tamaño {(mx - mn).round(1)}  ->  {eslabon or "SIN ESLABON"}')
        if eslabon is None:
            sys.exit('Hay una pieza que no sé a qué eslabón pertenece: revisá la tabla.')
        grupos.setdefault(eslabon, []).append(t)
        if base is None or mn[2] < base[0]:
            base = (mn[2], centro[0], centro[1])     # la pieza más baja es la base: su centro es el origen
    _, cx, cy = base
    print(f'origen del marco del brazo en el ensamblado: x = {cx:.1f}, y = {cy:.1f} mm')
    for eslabon, lista in grupos.items():
        malla = np.concatenate(lista)
        malla[:, :, 0] -= cx
        malla[:, :, 1] -= cy
        malla[:, :, 2] -= Z_PLACA_SUPERIOR
        ruta = f'{carpeta}/arm_{eslabon}.stl'
        escribir_stl(ruta, malla)
        print(f'{ruta}: {len(lista)} piezas, {len(malla)} triángulos')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])