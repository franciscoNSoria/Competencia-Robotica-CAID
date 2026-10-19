#!/usr/bin/env python3
"""Extrae el brazo del ensamblado de Fusion y lo guarda como una sola malla en el marco de su base.

Uso:  python3 tools/meshes/extraer_brazo.py ENSAMBLADO.stl SALIDA.stl

Pasos:
  1. Lee el STL binario del ensamblado (todas las piezas juntas, en las coordenadas de Fusion).
  2. Separa las piezas: dos triángulos son de la misma pieza si comparten un vértice.
  3. Se queda con las del brazo (regla abajo).
  4. Pasa las coordenadas de Fusion a las de ROS (giro de 180 grados sobre z) y las expresa en el
     marco de la base del brazo: origen en el centro de la base, sobre la cara superior de la
     placa de arriba (en Fusion está a z = 64).
  5. Escribe un STL binario.
"""
import struct
import sys

import numpy as np

Z_PLACA_SUPERIOR = 64.0   # mm, cara superior de la placa de arriba en el ensamblado


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
        f.write(b'brazo extraido del ensamblado'.ljust(80, b' '))
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


def main(entrada, salida):
    tri = leer_stl(entrada)
    pieza = separar_piezas(tri)
    elegidos = []
    base = None   # la pieza más baja del brazo es su base: de ahí sale el origen
    for p in np.unique(pieza):
        t = tri[pieza == p]
        v = t.reshape(-1, 3)
        mn, mx = v.min(0), v.max(0)
        centro_x = (mn[0] + mx[0]) / 2
        # Regla del brazo: arriba de la placa de arriba, con volumen y del lado donde está el brazo
        if mn[2] >= Z_PLACA_SUPERIOR - 0.5 and (mx[2] - mn[2]) > 0.5 and centro_x > -50:
            elegidos.append(t)
            if base is None or mn[2] < base[0]:
                base = (mn[2], (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2)
            print(f'  pieza del brazo: {len(t):6d} triángulos, tamaño',
                  (mx - mn).round(1), 'z desde', round(mn[2], 1))
    _, cx, cy = base
    brazo = np.concatenate(elegidos)
    brazo[:, :, 0] = -(brazo[:, :, 0] - cx)   # giro de 180 grados sobre z: (x, y) -> (-x, -y),
    brazo[:, :, 1] = -(brazo[:, :, 1] - cy)   # con el origen en el centro de la base
    brazo[:, :, 2] -= Z_PLACA_SUPERIOR        # z desde la cara superior de la placa de arriba
    print(f'base del brazo en el marco base_link (mm): x = {-cx:.1f}, y = {-cy:.1f}')
    escribir_stl(salida, brazo)
    v = brazo.reshape(-1, 3)
    print(f'{len(elegidos)} piezas, {len(brazo)} triángulos -> {salida}')
    print('caja del brazo en el marco de su base (mm): min', v.min(0).round(1), 'max', v.max(0).round(1))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
