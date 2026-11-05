"""Dibujo del mapa en RViz (MarkerArray en el marco map)."""
from caid_navigation.mapa import PASOS
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def _marcador(tipo, ident, stamp, escala, color):
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = stamp
    m.ns = 'mapa'
    m.id = ident
    m.type = tipo
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = escala
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def marcadores(mapa, tam, rastro, cubo, stamp):
    """Arma los marcadores: piso visitado, paredes, inicio, cubo y recorrido del robot."""
    alto = 0.15
    piso = _marcador(Marker.CUBE_LIST, 0, stamp, (tam * 0.92, tam * 0.92, 0.005),
                     (0.75, 0.75, 0.75, 0.6))
    piso.points = [Point(x=c[0] * tam, y=c[1] * tam) for c in mapa.visitadas]

    # Una lista por orientación: las paredes de los lados ±x son finas en x y las de ±y, en y.
    paredes_x = _marcador(Marker.CUBE_LIST, 1, stamp, (0.03, tam, alto), (0.8, 0.3, 0.1, 0.9))
    paredes_y = _marcador(Marker.CUBE_LIST, 2, stamp, (tam, 0.03, alto), (0.8, 0.3, 0.1, 0.9))
    vistas = set()
    for (celda, d), hay_paso in mapa.lados.items():
        borde = (celda[0] * 2 + PASOS[d][0], celda[1] * 2 + PASOS[d][1])
        if hay_paso or borde in vistas:
            continue
        vistas.add(borde)
        p = Point(x=borde[0] * tam / 2, y=borde[1] * tam / 2, z=alto / 2)
        (paredes_x if d in (0, 2) else paredes_y).points.append(p)

    inicio = _marcador(Marker.CYLINDER, 3, stamp, (tam * 0.5, tam * 0.5, 0.01),
                       (0.1, 0.3, 0.9, 0.9))
    inicio.pose.position.x = mapa.inicio[0] * tam
    inicio.pose.position.y = mapa.inicio[1] * tam

    marca_cubo = _marcador(Marker.CUBE, 4, stamp, (0.06, 0.06, 0.06), (0.1, 0.85, 0.2, 1.0))
    if cubo is None:
        marca_cubo.action = Marker.DELETE
    else:
        marca_cubo.pose.position.x = cubo[0] * tam
        marca_cubo.pose.position.y = cubo[1] * tam
        marca_cubo.pose.position.z = 0.03

    recorrido = _marcador(Marker.LINE_STRIP, 5, stamp, (0.015, 0.0, 0.0), (0.95, 0.85, 0.1, 1.0))
    recorrido.points = [Point(x=x, y=y, z=0.01) for x, y in rastro]
    if len(recorrido.points) < 2:
        recorrido.action = Marker.DELETE

    return MarkerArray(markers=[piso, paredes_x, paredes_y, inicio, marca_cubo, recorrido])
