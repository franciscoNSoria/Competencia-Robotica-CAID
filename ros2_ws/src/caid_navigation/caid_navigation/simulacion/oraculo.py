"""
Detecciones "perfectas" para el simulador: proyecta los objetos reales del mundo a la imagen.

Entrega lo mismo que publica el nodo de visión (/vision/detecciones), así se puede probar la
navegación sin la parte de imagen. Una pared tapa todo lo que está detrás (las paredes miden 170 mm
y la cámara está a 138,5 mm).
"""
import math

COLORES_FLECHA = ('rojo', 'verde', 'azul')


def _visible(mundo, robot, o, p):
    """Sin pared entre la cámara o y el punto p (en planta)."""
    dx, dy = p[0] - o[0], p[1] - o[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return True
    # A 10 m de altura no hay cajas: el rayo solo choca con las paredes.
    return robot.rayo(o[0], o[1], dx / dist, dy / dist, 10.0, alcance=dist) >= dist - 0.004


def _en_imagen(cam, pix, margen=1):
    return pix is not None and margen <= pix[0] < cam.ancho - margen and \
        margen <= pix[1] < cam.alto - margen


def detecciones(mundo, robot, cam, pose, pan):
    o = cam.origen(pose, pan)
    det = {'flechas': [], 'cubos': [], 'canastos': [], 'zonas': []}
    for f in mundo.flechas:
        if f['color'] not in COLORES_FLECHA:
            continue
        cx, cy, cz = f['centro']
        hacia = (o[0] - cx, o[1] - cy, o[2] - cz)
        dist = math.sqrt(sum(a * a for a in hacia))
        cos_v = sum(a * b for a, b in zip(hacia, f['normal'])) / dist
        if cos_v < 0.3 or dist > 2.0:
            continue
        L = {1: 0.030, 2: 0.060, 3: 0.100}[f['largo']]
        total = L + 0.050
        u = f['u']

        def punto(s):
            return (cx + u[0] * s, cy + u[1] * s, cz + u[2] * s)
        cola, cuello, punta = punto(-total / 2), punto(-total / 2 + L), punto(total / 2)
        pix = [cam.proyectar(pose, pan, p) for p in (punta, cuello, cola)]
        if not all(_en_imagen(cam, p, 4) for p in pix):
            continue
        if not _visible(mundo, robot, o, cuello):
            continue
        if math.dist(pix[0], pix[1]) < 9:      # punta de menos de 9 px: no se reconoce
            continue
        det['flechas'].append({'color': f['color'], 'punta': pix[0], 'cuello': pix[1],
                               'cola': pix[2]})
    for b in mundo.cajas:
        if not b.activa or b.tipo not in ('cubo', 'canasto'):
            continue
        if b.tipo == 'canasto' and b.z1 - b.z0 < 0.005:
            continue
        x0, y0, x1, y1 = b.caja
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        if b.tipo == 'canasto':
            cx, cy, _ = mundo.canastos[b.nombre]
            x0, y0, x1, y1 = cx - 0.022, cy - 0.022, cx + 0.022, cy + 0.022
        esquinas = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (b.z0, b.z1)]
        pix = [cam.proyectar(pose, pan, p) for p in esquinas]
        if any(p is None for p in pix):
            continue
        us = [p[0] for p in pix]
        vs = [p[1] for p in pix]
        if max(us) < 0 or min(us) >= cam.ancho or max(vs) < 0 or min(vs) >= cam.alto:
            continue
        if min(us) < 2 or max(us) > cam.ancho - 2:
            continue
        if not _visible(mundo, robot, o, (cx, cy)):
            continue
        caja = (min(us), min(vs), max(us) - min(us), max(vs) - min(vs))
        if caja[2] < 6:
            continue
        clave = 'cubos' if b.tipo == 'cubo' else 'canastos'
        if clave == 'canastos' and any(d['color'] == b.nombre for d in det['canastos']):
            continue
        det[clave].append({'color': b.nombre, 'caja': caja})
    for x0, y0, x1, y1 in mundo.zonas:
        largo = max(x1 - x0, y1 - y0)
        k = max(2, int(largo / 0.01))
        for i in range(k + 1):
            t = i / k
            if x1 - x0 > y1 - y0:
                p = (x0 + t * (x1 - x0), (y0 + y1) / 2, 0.0)
            else:
                p = ((x0 + x1) / 2, y0 + t * (y1 - y0), 0.0)
            pix = cam.proyectar(pose, pan, p)
            if not _en_imagen(cam, pix) or pix[1] <= cam.cy:
                continue
            if math.hypot(p[0] - o[0], p[1] - o[1]) > 1.6:
                continue
            if not _visible(mundo, robot, o, p):
                continue
            det['zonas'].append(pix)
    return det
