"""
Corre varios laberintos y semillas en paralelo y resume.

    python3 -m caid_navigation.simulacion.lote e1 e2 e3 e4 e5 e6 --semillas 1 2 3 [--vision render]
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import math
import os

from caid_navigation.simulacion.correr import correr


def _uno(args):
    nombre, semilla, vision, t_max = args
    r = correr(nombre, t_max, vision=vision, semilla=semilla)
    ultimos = [t for _, t in r.mensajes[-6:]]
    return (nombre, semilla, r.completa, r.tiempo, list(r.entregados), list(r.errores),
            len(r.choques), r.zonas, r.error_max, r.atascado, ultimos,
            [(round(c[0], 1), c[2]) for c in r.choques[:2]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('laberintos', nargs='+')
    ap.add_argument('--semillas', nargs='+', type=int, default=[1])
    ap.add_argument('--vision', default='oraculo')
    ap.add_argument('--t-max', type=float, default=1200.0)
    ap.add_argument('--detalle', action='store_true')
    a = ap.parse_args()
    trabajos = [(n, s, a.vision, a.t_max) for n in a.laberintos for s in a.semillas]
    ok = 0
    with ProcessPoolExecutor(max_workers=min(len(trabajos), os.cpu_count() or 2)) as ex:
        for (n, s, completa, t, ent, err, ch, zo, emax, atas, ult, chq) in ex.map(_uno, trabajos):
            ok += completa
            estado = 'OK ' if completa else 'MAL'
            print(f'{n:22s} semilla {s}: {estado} {t:5.0f} s  entregados {ent}'
                  f'  choques {ch}{" (atascado)" if atas else ""}  zonas {zo}  errores {err}'
                  f'  pose máx {emax[0] * 100:.1f} cm/{math.degrees(emax[1]):.1f}°')
            if a.detalle and not completa:
                print('      ', chq, ' | '.join(ult[-4:]))
    print(f'completas: {ok}/{len(trabajos)}')


if __name__ == '__main__':
    main()
