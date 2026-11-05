"""
Validación de flechas entre cuadros (sin ROS, para poder probarla).

De lejos solo se avisa que hay una flecha: la medida no es confiable. Cuando la flecha está cerca
(a `distancia_decision` o menos, 50 cm por defecto):

  - cada flecha detectada se asocia a la flecha seguida del mismo color más cercana en la imagen
    (si está a menos de `radio` veces su tamaño); si no hay ninguna, empieza una nueva. Así dos
    flechas del mismo color a la vista no se mezclan;
  - PRESENCIA: con la primera lectura cercana la flecha queda anotada y no se borra nunca;
  - DIRECCIÓN y TAMAÑO: cada lectura cercana vota, y vale la mayoría de las últimas `ventana`;
  - FIJA: al llegar a `distancia_fija` (30 cm) la decisión se congela con esa mayoría y no se
    cambia más (fijar());
  - una flecha que todavía no tuvo ninguna lectura cercana se olvida si no se ve en `olvido_s`.

El tamaño sale de una proporción (cuerpo / base de la punta) y no depende de la calibración de
la cámara; la distancia sí (fx), y solo decide desde dónde se empieza a votar.
"""
from collections import Counter, deque
from itertools import count
import math

# Flechas del reglamento (nov. 2026): punta de 29 mm de base; cuerpo de 20 / 50 / 85 mm
# (largo total 30 / 60 / 95 mm: P3 / P2 / P1).
PUNTA_ANCHO_M = 0.029
CORTES_CUERPO_MM = (35.0, 67.5)     # entre 20 y 50, y entre 50 y 85


def medir_flecha(f, fx):
    """
    Distancia (m), largo del cuerpo (mm) y tamaño ('P3', 'P2', 'P1') de una flecha del detector.

    La distancia sale del ancho de la base de la punta (29 mm) visto en píxeles: hace falta fx.
    El cuerpo sale de comparar su largo con esa misma base: es una proporción, no depende de la
    distancia ni de la cámara. Devuelve (None, None, None) si la flecha no trae hombros.
    """
    hombros = f.get('hombros')
    if not hombros or len(hombros) < 2:
        return None, None, None
    ancho_px = math.dist(*hombros)
    if ancho_px < 1:
        return None, None, None
    distancia = fx * PUNTA_ANCHO_M / ancho_px
    cuerpo_mm = math.dist(f['cuello'], f['cola']) / ancho_px * PUNTA_ANCHO_M * 1000
    tamano = 'P3' if cuerpo_mm < CORTES_CUERPO_MM[0] else (
        'P2' if cuerpo_mm < CORTES_CUERPO_MM[1] else 'P1')
    return distancia, cuerpo_mm, tamano


def mayoria(lecturas):
    """(valor más votado, votos, total) de una lista de lecturas, o (None, 0, 0)."""
    if not lecturas:
        return None, 0, 0
    valor, n = Counter(lecturas).most_common(1)[0]
    return valor, n, len(lecturas)


def _centro(f):
    return ((f['punta'][0] + f['cola'][0]) / 2, (f['punta'][1] + f['cola'][1]) / 2)


def _largo(f):
    return max(math.dist(f['punta'], f['cola']), 10.0)


class ValidadorFlechas:

    def __init__(self, ventana=10, olvido_s=1.5, radio=1.0):
        self.ventana = ventana
        self.olvido_s = olvido_s
        self.radio = radio
        self.seguidas = {}       # id -> dict(color, centro, largo, dirs, tams, visto, ...)
        self._ids = count()

    def actualizar(self, t, flechas):
        """
        t: tiempo en s. flechas: [(dirección, tamaño, cerca, flecha del detector)] del cuadro.

        cerca: True si la flecha está a distancia de decidir. Devuelve, para cada flecha de
        entrada, el id de la flecha seguida que le corresponde.
        """
        libres = set(self.seguidas)
        orden = sorted(range(len(flechas)), key=lambda i: -flechas[i][3].get('area', 0))
        asignado = {}
        for i in orden:
            direccion, tamano, cerca, f = flechas[i]
            c, largo = _centro(f), _largo(f)
            mejor, mejor_d = None, None
            for k in libres:
                s = self.seguidas[k]
                if s['color'] != f['color']:
                    continue
                d = math.dist(c, s['centro'])
                if d <= self.radio * max(largo, s['largo']) and (mejor is None or d < mejor_d):
                    mejor, mejor_d = k, d
            if mejor is None:
                mejor = next(self._ids)
                self.seguidas[mejor] = {'color': f['color'], 'dirs': deque(maxlen=self.ventana),
                                        'tams': deque(maxlen=self.ventana)}
            else:
                libres.discard(mejor)
            s = self.seguidas[mejor]
            s.update(centro=c, largo=largo, visto=t, flecha=f, cerca=cerca)
            if cerca and s.get('fija') is None:
                if direccion is not None:
                    s['dirs'].append(direccion)
                if tamano is not None:
                    s['tams'].append(tamano)
            asignado[i] = mejor
        for k in [k for k, s in self.seguidas.items()
                  if not s['dirs'] and t - s['visto'] > self.olvido_s]:
            del self.seguidas[k]        # las que ya tuvieron una lectura cercana quedan siempre
        return [asignado[i] for i in range(len(flechas))]

    def fijar(self, k):
        """Congela la decisión actual de la flecha k (a distancia_fija): ya no se cambia."""
        s = self.seguidas.get(k)
        if s and s.get('fija') is None and s['dirs']:
            s['fija'] = mayoria(list(s['dirs'])) + mayoria(list(s['tams']))
            return True
        return False

    def es_fija(self, k):
        s = self.seguidas.get(k)
        return bool(s and s.get('fija'))

    def decision(self, k):
        """(dirección, votos, total, tamaño, votos, total) de una flecha (la fija si ya está)."""
        s = self.seguidas.get(k)
        if not s or not s['dirs']:
            return None
        if s.get('fija'):
            return s['fija']
        return mayoria(list(s['dirs'])) + mayoria(list(s['tams']))

    def presentes(self):
        """[(id, color, dirección, tamaño, última flecha vista)] de las flechas anotadas."""
        out = []
        for k, s in sorted(self.seguidas.items()):
            d = self.decision(k)
            if d:
                out.append((k, s['color'], d[0], d[3], s['flecha']))
        return out

    def visto(self, k):
        s = self.seguidas.get(k)
        return s['visto'] if s else None

    def cerca(self, k):
        s = self.seguidas.get(k)
        return bool(s and s['cerca'])
