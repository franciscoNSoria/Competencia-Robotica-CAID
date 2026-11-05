#!/usr/bin/env python3
"""Evalúa el piloto de Trémaux sin cámara en el simulador, en todos los laberintos, con varias semillas (en paralelo).

  python3 tools/tremaux/evaluar.py                       # todos los laberintos, 12 semillas, modelo nominal
  python3 tools/tremaux/evaluar.py --pesimista -n 20     # el modelo pesimista de simular_torneo.py
  python3 tools/tremaux/evaluar.py -l e5,chico -n 30 --detalle   # el log de las corridas que fallan
  python3 tools/tremaux/evaluar.py --base                # el piloto de torneo_logica.py tal cual (para comparar)
"""
import argparse
import collections
import os
import sys
from multiprocessing import Pool

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import banco  # noqa: E402
import laberintos  # noqa: E402
import tremaux_logica as tx  # noqa: E402
import torneo_logica as tl  # noqa: E402


class Base(tl.Piloto):
    """El piloto de torneo_logica sin cámara: al terminar de explorar vuelve a la partida (solo para comparar)."""

    def _decidir(self, t, tele, f, mi, md):
        d = self._direccion(self.celda, self.rumbo)
        if d is None and self.modo == 'EXPLORAR':
            self.modo = 'VOLVER'
            d = self._direccion(self.celda, self.rumbo)
        if d is None:
            self._cambiar(tl.FIN, t, 'fin')
            return self._vw(0.0, 0.0)
        return super()._decidir(t, tele, f, mi, md)


AJUSTES = {}


class ConAjustes(tx.PilotoTremaux):
    def __init__(self, params=None, **kw):
        p = tx.Parametros()
        for k, v in AJUSTES.items():
            setattr(p, k, (v not in ('0', 'False', 'false')) if isinstance(getattr(p, k), bool) else type(getattr(p, k))(v))
        super().__init__(p, **kw)


def una(args):
    nombre, semilla, pesimista, base, ajustes = args
    AJUSTES.update(ajustes)
    texto = laberintos.todos()[nombre]
    s = banco.correr(texto, semilla, Base if base else ConAjustes, pesimista=pesimista)
    p = s.piloto
    alcanzables = len(s.lab.libres)
    return dict(lab=nombre, semilla=semilla, choque=s.choque, estado=p.estado, modo=p.modo, motivo=p.motivo, t=round(s.t),
                cobertura=len(p.mapa.visitadas & s.lab.libres_centros) / alcanzables if hasattr(s.lab, 'libres_centros')
                else len(p.mapa.visitadas) / alcanzables,
                eventos=list(getattr(p, 'eventos', [])), log=s.log)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-l', '--laberintos', default=None)
    ap.add_argument('-n', '--semillas', type=int, default=12)
    ap.add_argument('--pesimista', action='store_true')
    ap.add_argument('--base', action='store_true')
    ap.add_argument('--detalle', action='store_true')
    ap.add_argument('-j', '--procesos', type=int, default=os.cpu_count())
    ap.add_argument('--set', action='append', default=[], metavar='parametro=valor', help='pisa un parámetro de Parametros')
    a = ap.parse_args()
    ajustes = dict(x.split('=') for x in a.set)
    labs = a.laberintos.split(',') if a.laberintos else list(laberintos.todos())
    tareas = [(l, s, a.pesimista, a.base, ajustes) for l in labs for s in range(1, a.semillas + 1)]
    with Pool(a.procesos) as pool:
        res = pool.map(una, tareas, chunksize=1)
    print('%s, modelo %s, %d semillas' % ('BASE (torneo_logica)' if a.base else 'TREMAUX', 'pesimista' if a.pesimista else 'nominal',
                                          a.semillas))
    tot = collections.Counter()
    for l in labs:
        rs = [r for r in res if r['lab'] == l]
        choques = [r for r in rs if r['choque']]
        paradas = [r for r in rs if not r['choque'] and r['estado'] == tl.DETENIDO]
        ok = [r for r in rs if not r['choque'] and r['estado'] == tl.FIN and r['modo'] == 'VOLVER']
        por_estado = collections.Counter(r['choque'][1] for r in choques)
        print('%-6s choques %2d/%d %-28s | paró por seguridad %2d | recorrió todo y volvió %2d | cobertura %.2f | t medio %4.0f s' % (
            l, len(choques), len(rs), dict(por_estado), len(paradas), len(ok), sum(r['cobertura'] for r in rs) / len(rs),
            sum(r['t'] for r in rs) / len(rs)))
        tot.update(choques=len(choques), n=len(rs), paradas=len(paradas), ok=len(ok))
        if a.detalle:
            for r in rs:
                if r['choque'] or r['estado'] == tl.DETENIDO:
                    print('  -- semilla %d: choque %s | %s' % (r['semilla'], r['choque'], r['motivo']))
                    print('\n'.join('     ' + x for x in r['log'][-14:]))
    print('TOTAL choques %d/%d (%.1f %%) | paró por seguridad %d | completó %d' % (
        tot['choques'], tot['n'], 100.0 * tot['choques'] / max(1, tot['n']), tot['paradas'], tot['ok']))


if __name__ == '__main__':
    main()
