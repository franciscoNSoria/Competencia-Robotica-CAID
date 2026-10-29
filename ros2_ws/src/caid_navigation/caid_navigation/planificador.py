"""
Planificador sobre estados (celda, rumbo) (núcleo puro, sin ROS).

Acciones del robot, con su costo en segundos:
- `avanzar`: a la celda de enfrente, si el paso está abierto y no es zona prohibida.
- `retroceder`: a la celda de atrás, solo si ya se visitó (atrás no hay sensor).
- `girar`: 90° o 180° en el lugar, solo si giros.py encuentra un corrimiento que no toca nada.

Con eso el robot nunca intenta un giro imposible: en un callejón sale marcha atrás hasta una celda
donde sí puede girar. Dijkstra sobre todos los estados es barato (un laberinto de 13 x 24 celdas
tiene menos de 1300 estados).

`costo_extra(celda, d)` permite encarecer pasos: así las flechas empujan la exploración hacia el
lado que indican (ver mision.py).
"""
import heapq
import math

from caid_navigation.geometria import CELDA, vecina


class Costos:
    def __init__(self, v_avance=0.20, v_retro=0.12, w_giro=1.2, t_giro=1.0, t_retro=0.6):
        self.avanzar = CELDA / v_avance
        self.retroceder = CELDA / v_retro + t_retro
        self.giro90 = (math.pi / 2) / w_giro + t_giro
        self.giro180 = math.pi / w_giro + t_giro


def vecinos(con, estado, costos, costo_extra=None):
    celda, h = estado
    if con.transitable(celda, h):
        extra = costo_extra(celda, h) if costo_extra else 0.0
        yield ('avanzar', vecina(celda, h)), (vecina(celda, h), h), costos.avanzar + extra
    atras = (h + 2) % 4
    n = vecina(celda, atras)
    if con.transitable(celda, atras) and con.visitada(n):
        extra = costo_extra(celda, atras) if costo_extra else 0.0
        yield ('retroceder', n), (n, h), costos.retroceder + extra
    for giro, nuevo, costo in ((math.pi / 2, (h + 1) % 4, costos.giro90),
                               (-math.pi / 2, (h + 3) % 4, costos.giro90),
                               (math.pi, (h + 2) % 4, costos.giro180)):
        if (celda, h, nuevo) in getattr(con, 'giros_imposibles', ()):
            continue
        corr = con.giro(celda, h, giro)
        if corr is not None:
            extra = 3.0 * (abs(corr[0]) + abs(corr[1]))
            yield ('girar', nuevo, corr), (celda, nuevo), costo + extra


def dijkstra(con, inicio, costos, costo_extra=None, limite=None):
    """Costo mínimo y predecesor de cada estado alcanzable desde `inicio` = (celda, rumbo)."""
    dist = {inicio: 0.0}
    previo = {inicio: None}
    cola = [(0.0, 0, inicio)]
    k = 0
    while cola:
        c, _, e = heapq.heappop(cola)
        if c > dist[e]:
            continue
        if limite is not None and c > limite:
            break
        for accion, sig, costo in vecinos(con, e, costos, costo_extra):
            nc = c + costo
            if nc < dist.get(sig, math.inf) - 1e-9:
                dist[sig] = nc
                previo[sig] = (e, accion)
                k += 1
                heapq.heappush(cola, (nc, k, sig))
    return dist, previo


def reconstruir(previo, estado):
    acciones = []
    while previo[estado] is not None:
        estado, accion = previo[estado]
        acciones.append(accion)
    return acciones[::-1]


def planificar(con, inicio, es_objetivo, costos, costo_extra=None, bonus=None):
    """
    Plan al mejor estado objetivo: (acciones, estado final, costo) o None.

    es_objetivo(celda, rumbo) -> bool. bonus(celda, rumbo) -> segundos a restar (opcional): sirve
    para preferir un objetivo aunque esté un poco más lejos.
    """
    dist, previo = dijkstra(con, inicio, costos, costo_extra)
    mejor = None
    for estado, c in dist.items():
        if not es_objetivo(*estado):
            continue
        puntaje = c - (bonus(*estado) if bonus else 0.0)
        if mejor is None or puntaje < mejor[0] - 1e-9:
            mejor = (puntaje, estado, c)
    if mejor is None:
        return None
    return reconstruir(previo, mejor[1]), mejor[1], mejor[2]
