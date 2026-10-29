"""
Imagen de la cámara del simulador 2D (640 x 480), para probar el detector de caid_vision.

Rasterizador chico con z-buffer: piso, caras de los bloques, soportes, obstáculos, cubos,
canastos, flechas (cuerpo + punta) y líneas rojas de las zonas. Iluminación difusa con la misma
dirección de sol que los mundos de Gazebo. No busca parecerse a Gazebo pixel a pixel: sirve para
verificar la geometría (de qué celda y pared es una flecha, a qué distancia está un cubo).
"""
import math

import cv2
import numpy as np

SOL = np.array([-0.5, 0.1, -0.9]) / np.linalg.norm([-0.5, 0.1, -0.9])
CIELO = (0.75, 0.80, 0.85)


def _luz(normal):
    return 0.45 + 0.6 * max(0.0, -float(np.dot(normal, SOL)))


class Render:

    def __init__(self, mundo, cam, ruido=3.0, semilla=1):
        self.mundo = mundo
        self.cam = cam
        self.ruido = ruido
        self.rng = np.random.default_rng(semilla)
        self.caras = self._caras_fijas()

    # ---------- geometría del mundo ----------

    def _caras_fijas(self):
        """Caras verticales de los bloques que dan a piso libre: (centro, normal, puntos, rgb)."""
        m = self.mundo
        C = m.celda
        H = m.alto_pared
        caras = []
        for (r, c) in m.bloques:
            x, y = m.lab.centro(r, c)
            for dr, dc, nx, ny in ((-1, 0, 0, 1), (1, 0, 0, -1), (0, 1, 1, 0), (0, -1, -1, 0)):
                if (r + dr, c + dc) in m.bloques:
                    continue
                fx, fy = x + nx * C / 2, y + ny * C / 2
                tx, ty = -ny * C / 2, nx * C / 2
                pts = [(fx - tx, fy - ty, 0.0), (fx + tx, fy + ty, 0.0),
                       (fx + tx, fy + ty, H), (fx - tx, fy - ty, H)]
                caras.append(((fx, fy), np.array([nx, ny, 0.0]), pts, m.gm.COLOR_PARED))
        return caras

    def _caras_caja(self, b):
        x0, y0, x1, y1 = b.caja
        z0, z1 = b.z0, b.z1
        return [
            (np.array([1.0, 0, 0]), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
            (np.array([-1.0, 0, 0]), [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)]),
            (np.array([0, 1.0, 0]), [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)]),
            (np.array([0, -1.0, 0]), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
            (np.array([0, 0, 1.0]), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]),
        ]

    def _flecha(self, f):
        cx, cy, cz = f['centro']
        u = np.array(f['u'])
        p = np.array(f['p'])
        n = np.array(f['normal'])
        L = {1: 0.030, 2: 0.060, 3: 0.100}[f['largo']]
        total = L + 0.050
        c = np.array([cx, cy, cz]) + n * 0.0012
        a0 = -total / 2
        cuerpo = [c + u * a0 - p * 0.015, c + u * (a0 + L) - p * 0.015,
                  c + u * (a0 + L) + p * 0.015, c + u * a0 + p * 0.015]
        punta = [c + u * (a0 + L) - p * 0.035, c + u * (a0 + total), c + u * (a0 + L) + p * 0.035]
        return n, [cuerpo, punta]

    # ---------- dibujo ----------

    def imagen(self, pose, pan):
        cam = self.cam
        W, Hh = cam.ancho, cam.alto
        o = np.array(cam.origen(pose, pan))
        psi = pose[2] + pan
        f = np.array([math.cos(psi), math.sin(psi), 0.0])
        der = np.array([math.sin(psi), -math.cos(psi), 0.0])
        img = np.empty((Hh, W, 3), np.float32)
        img[:] = CIELO[::-1]
        z = np.full((Hh, W), np.inf, np.float32)
        # Piso
        vs = np.arange(Hh, dtype=np.float32)
        filas = vs > cam.cy + 0.5
        prof = np.where(filas, cam.altura * cam.fy / np.maximum(vs - cam.cy, 1e-3), np.inf)
        z[:] = prof[:, None]
        piso = np.array(self.mundo.gm.COLOR_PISO) * _luz(np.array([0, 0, 1.0]))
        img[filas] = piso[::-1]

        def poligono(pts, rgb, normal, sesgo=0.0):
            P = np.array(pts, dtype=float) - o
            zc = P @ f
            xc = P @ der
            yc = -P[:, 2]
            poli = list(zip(xc, yc, zc))
            poli = _recortar(poli, 0.03)
            if len(poli) < 3:
                return
            q = np.array([(cam.cx + cam.fx * x / zz, cam.cy + cam.fy * y / zz, 1.0 / zz)
                          for x, y, zz in poli])
            umin = int(max(0, math.floor(q[:, 0].min())))
            umax = int(min(W - 1, math.ceil(q[:, 0].max())))
            vmin = int(max(0, math.floor(q[:, 1].min())))
            vmax = int(min(Hh - 1, math.ceil(q[:, 1].max())))
            if umin > umax or vmin > vmax:
                return
            mascara = np.zeros((vmax - vmin + 1, umax - umin + 1), np.uint8)
            pts2 = np.round((q[:, :2] - [umin, vmin]) * 16).astype(np.int32)
            cv2.fillPoly(mascara, [pts2], 1, lineType=cv2.LINE_8, shift=4)
            if not mascara.any():
                return
            # 1/z es lineal en (u, v) sobre un plano.
            A = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
            coef, *_ = np.linalg.lstsq(A, q[:, 2], rcond=None)
            uu, vv = np.meshgrid(np.arange(umin, umax + 1), np.arange(vmin, vmax + 1))
            inv = coef[0] * uu + coef[1] * vv + coef[2]
            zz = np.where(inv > 1e-6, 1.0 / np.maximum(inv, 1e-6), np.inf) - sesgo
            zona = z[vmin:vmax + 1, umin:umax + 1]
            dib = (mascara > 0) & (zz < zona)
            zona[dib] = zz[dib]
            color = np.clip(np.array(rgb) * _luz(normal), 0, 1)[::-1]
            img[vmin:vmax + 1, umin:umax + 1][dib] = color

        x, y = pose[0], pose[1]
        for (cxy, normal, pts, rgb) in self.caras:
            dx, dy = cxy[0] - o[0], cxy[1] - o[1]
            if dx * normal[0] + dy * normal[1] > 0 or math.hypot(dx, dy) > 3.5:
                continue        # cara de espaldas o lejos
            if dx * f[0] + dy * f[1] < -0.3:
                continue
            poligono(pts, rgb, normal)
        for x0, y0, x1, y1 in self.mundo.zonas:
            if math.hypot((x0 + x1) / 2 - x, (y0 + y1) / 2 - y) < 3.0:
                poligono([(x0, y0, 0.0005), (x1, y0, 0.0005), (x1, y1, 0.0005), (x0, y1, 0.0005)],
                         self.mundo.gm.COLOR_ZONA_ROJA, np.array([0, 0, 1.0]), sesgo=0.003)
        for b in self.mundo.cajas:
            if not b.activa or b.tipo == 'pared':
                continue
            bx, by = (b.caja[0] + b.caja[2]) / 2, (b.caja[1] + b.caja[3]) / 2
            if math.hypot(bx - x, by - y) > 3.0:
                continue
            for normal, pts in self._caras_caja(b):
                c = np.mean(pts, axis=0)
                if np.dot(c - o, normal) > 0:
                    continue
                poligono(pts, b.color, normal)
        for fl in self.mundo.flechas:
            if math.hypot(fl['centro'][0] - x, fl['centro'][1] - y) > 3.0:
                continue
            normal, partes = self._flecha(fl)
            if np.dot(np.array(fl['centro']) - o, normal) > 0:
                continue
            for pts in partes:
                poligono(pts, fl['rgb'], normal, sesgo=0.002)
        salida = img * 255.0
        if self.ruido:
            salida += self.rng.normal(0, self.ruido, salida.shape)
        salida = cv2.GaussianBlur(np.clip(salida, 0, 255).astype(np.uint8), (3, 3), 0)
        return salida


def _recortar(poli, z_min):
    """Recorta el polígono (x, y, z) contra el plano z = z_min (Sutherland-Hodgman)."""
    salida = []
    n = len(poli)
    for i in range(n):
        a, b = poli[i], poli[(i + 1) % n]
        a_in, b_in = a[2] >= z_min, b[2] >= z_min
        if a_in:
            salida.append(a)
        if a_in != b_in:
            t = (z_min - a[2]) / (b[2] - a[2])
            salida.append(tuple(a[k] + t * (b[k] - a[k]) for k in range(3)))
    return salida
