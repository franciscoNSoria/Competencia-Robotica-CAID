"""
Panel web de la cámara: video en vivo con detecciones, fps, ajustes de la ESP32-CAM y rumbo.

    ros2 run caid_vision panel --camara http://192.168.1.42
    # y abrir http://localhost:8080 en el navegador

Qué hace:
  - Lee el stream MJPEG de la ESP32-CAM (puerto 81) y dibuja encima cubos, flechas, canastos,
    el piso libre y el rumbo (caid_vision.detector y caid_vision.ruta).
  - Muestra fps, tiempo de proceso y los datos de /status de la cámara.
  - Cambia en vivo los ajustes de la cámara (espejo, volteo, brillo, ...) llamando a /control.
  - "Guardar" escribe esos ajustes en la sección `camara:` de colores.yaml. camara_node se los manda
    a la ESP32-CAM cada vez que se conecta, así que quedan para el robot.

No necesita ROS corriendo: solo OpenCV y numpy. No toca el robot.
"""
import argparse
import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from caid_vision import detector
from caid_vision.panel_html import PAGINA
from caid_vision.ruta import analizar_ruta, dibujar_ruta

# Ajustes de la cámara que se pueden cambiar y guardar: nombre -> (mínimo, máximo)
AJUSTES = {'hmirror': (0, 1), 'vflip': (0, 1), 'brightness': (-2, 2), 'contrast': (-2, 2),
           'saturation': (-2, 2), 'quality': (4, 63), 'framesize': (0, 13), 'fps_max': (0, 60)}
COLORES_BGR = {'rojo': (60, 60, 255), 'verde': (60, 220, 60), 'azul': (255, 130, 40)}


def clasificar_direccion(ang):
    """Como vision_node: solo derecha o izquierda (0 = derecha, 90 = arriba, 180 = izquierda)."""
    return 'derecha' if math.cos(math.radians(ang)) >= 0 else 'izquierda'


def buscar_colores_yaml():
    """colores.yaml de src (el que va a git): sube desde este archivo hasta encontrarlo."""
    carpeta = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        candidato = os.path.join(carpeta, 'config', 'colores.yaml')
        if os.path.exists(candidato):
            return candidato
        carpeta = os.path.dirname(carpeta)
    return None


class Fuente(threading.Thread):
    """Lee el stream MJPEG y se queda solo con el último cuadro. Reconecta solo."""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.cond = threading.Condition()
        self.frame = None
        self.num = 0
        self.fps = 0.0
        self.conectado = False
        self.error = ''

    def run(self):
        while True:
            try:
                self._leer()
            except (OSError, ValueError) as e:
                self.error = str(e)
            self.conectado = False
            self.fps = 0.0
            time.sleep(1.0)

    def _leer(self):
        buf = b''
        t0, n = time.time(), 0
        with urllib.request.urlopen(self.url, timeout=5) as r:
            self.conectado, self.error = True, ''
            while True:
                datos = r.read(4096)
                if not datos:
                    raise OSError('stream cerrado')
                buf += datos
                if len(buf) > 2_000_000:
                    buf = b''
                while True:
                    a = buf.find(b'\xff\xd8')
                    b = buf.find(b'\xff\xd9', a + 2) if a >= 0 else -1
                    if a < 0 or b < 0:
                        break
                    jpg, buf = buf[a:b + 2], buf[b + 2:]
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if img is None:
                        continue
                    n += 1
                    if time.time() - t0 >= 1.0:
                        self.fps, t0, n = n / (time.time() - t0), time.time(), 0
                    with self.cond:
                        self.frame, self.num = img, self.num + 1
                        self.cond.notify_all()


class Panel:
    def __init__(self, base, colores, fotos=None):
        self.base = base.rstrip('/')
        self.fotos = fotos or os.path.expanduser('~/fotos_camara')
        self.rotacion = 0
        self.ultimo_raw = None
        self.fuente = Fuente(self.base + ':81/stream')
        self.colores = colores
        self.mtime_colores = None
        self.capas = {'cubos': True, 'flechas': True, 'ruta': True, 'piso': True}
        self.lock = threading.Lock()
        self.jpeg = b''
        self.jpeg_num = 0
        self.cond = threading.Condition()
        self.estado = {'detecciones': {}, 'ruta': {}, 'ms_proceso': 0.0, 'fps_proceso': 0.0}
        self.esp = {}
        self.mensaje = ''

    # ---------- cámara ----------
    def esp_get(self, ruta, timeout=3):
        with urllib.request.urlopen(self.base + ruta, timeout=timeout) as r:
            return r.read()

    def sondear_esp(self):
        while True:
            try:
                self.esp = json.loads(self.esp_get('/status'))
            except (OSError, ValueError):
                self.esp = {}
            time.sleep(1.0)

    def ajustar(self, var, val):
        if var not in AJUSTES:
            raise ValueError(f'ajuste desconocido: {var}')
        lo, hi = AJUSTES[var]
        val = int(max(lo, min(hi, int(val))))
        self.esp_get(f'/control?var={var}&val={val}')
        self.esp[var] = val
        return val

    def guardar(self):
        """Escribe la sección `camara:` de colores.yaml con los valores actuales de la cámara."""
        if not self.colores:
            raise OSError('no encuentro config/colores.yaml')
        actuales = json.loads(self.esp_get('/status'))
        bloque = {k: int(actuales[k]) for k in AJUSTES if k in actuales}
        with open(self.colores) as fh:
            texto = fh.read()
        # Se saca el bloque `camara:` viejo (la línea y lo que cuelgue de ella) y se agrega el nuevo
        texto = re.sub(r'(?m)^camara:.*(?:\n[ \t]+.*)*\n?', '', texto).rstrip('\n') + '\n'
        linea = 'camara: {' + ', '.join(f'{k}: {v}' for k, v in bloque.items()) + '}'
        with open(self.colores, 'w') as fh:
            fh.write(texto + f'# ajustes de la ESP32-CAM, guardados desde el panel el '
                     f'{time.strftime("%Y-%m-%d %H:%M")}\n{linea}\n')
        self.mensaje = 'Guardado en ' + os.path.relpath(self.colores)
        return bloque

    # ---------- rotación y fotos ----------
    def rotar(self, img):
        """Gira la imagen (solo en el panel; la cámara y ROS no se enteran)."""
        codigos = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                   270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        return cv2.rotate(img, codigos[self.rotacion]) if self.rotacion in codigos else img

    def poner_rotacion(self, grados):
        if int(grados) not in (0, 90, 180, 270):
            raise ValueError('rotación: 0, 90, 180 o 270')
        self.rotacion = int(grados)

    def guardar_foto(self):
        """Guarda el cuadro actual sin dibujos y con dibujos, más un .json con lo detectado."""
        if self.ultimo_raw is None:
            raise OSError('todavía no llegó ningún cuadro')
        os.makedirs(self.fotos, exist_ok=True)
        base = os.path.join(self.fotos, time.strftime('%Y%m%d_%H%M%S'))
        cv2.imwrite(base + '_cruda.jpg', self.ultimo_raw, [cv2.IMWRITE_JPEG_QUALITY, 95])
        with open(base + '_anotada.jpg', 'wb') as fh:
            fh.write(self.jpeg)
        with open(base + '.json', 'w') as fh:
            json.dump({'detecciones': self.estado['detecciones'], 'ruta': self.estado['ruta'],
                       'camara': {k: self.esp.get(k) for k in AJUSTES},
                       'rotacion': self.rotacion}, fh, indent=1)
        self.mensaje = 'Foto guardada en ' + base
        return base

    # ---------- detección ----------
    def cargar_colores(self):
        if not self.colores:
            return
        try:
            mtime = os.path.getmtime(self.colores)
        except OSError:
            return
        if mtime == self.mtime_colores:
            return
        self.mtime_colores = mtime
        import yaml
        with open(self.colores) as fh:
            d = yaml.safe_load(fh) or {}
        if d.get('rangos'):
            detector.RANGOS = {c: [(tuple(int(x) for x in a), tuple(int(x) for x in b))
                                   for a, b in tramos] for c, tramos in d['rangos'].items()}
            detector.V_MIN = detector.V_MIN_AZUL = detector.S_MIN = 0

    def procesar(self, img):
        capas = dict(self.capas)
        det = detector.detectar(img)
        ruta = analizar_ruta(img)
        out = img.copy()
        if capas['ruta']:
            dibujar_ruta(out, ruta) if capas['piso'] else dibujar_ruta(
                out, dict(ruta, mascara=None))
        if capas['cubos']:
            for clave in ('cubos', 'canastos'):
                for c in det[clave]:
                    x, y, w, h = c['caja']
                    col = COLORES_BGR.get(c['color'], (255, 255, 255))
                    cv2.rectangle(out, (x, y), (x + w, y + h), col, 2)
                    cv2.putText(out, f"{clave[:-1]} {c['color']}", (x, max(y - 5, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        if capas['flechas']:
            for f in det['flechas']:
                col = COLORES_BGR.get(f['color'], (255, 255, 255))
                cola = tuple(int(v) for v in f['cola'])
                punta = tuple(int(v) for v in f['punta'])
                cv2.arrowedLine(out, cola, punta, (0, 0, 0), 6, tipLength=0.3)
                cv2.arrowedLine(out, cola, punta, col, 3, tipLength=0.3)
                texto = clasificar_direccion(f['angulo']).upper()
                cv2.putText(out, texto, (cola[0], max(cola[1] - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        return out, det, ruta

    def bucle_proceso(self):
        ultimo, t0, n = 0, time.time(), 0
        while True:
            with self.fuente.cond:
                self.fuente.cond.wait_for(lambda: self.fuente.num != ultimo, timeout=2.0)
                if self.fuente.num == ultimo:
                    continue
                img, ultimo = self.fuente.frame, self.fuente.num
            self.cargar_colores()
            img = self.rotar(img)
            self.ultimo_raw = img
            t = time.time()
            out, det, ruta = self.procesar(img)
            ms = (time.time() - t) * 1000
            ok, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 80])
            n += 1
            if time.time() - t0 >= 1.0:
                self.estado['fps_proceso'], t0, n = n / (time.time() - t0), time.time(), 0
            self.estado['ms_proceso'] = ms
            self.estado['detecciones'] = {
                'cubos': [{'color': c['color'], 'area': int(c['area'])} for c in det['cubos']],
                'flechas': [{'color': f['color'], 'direccion': clasificar_direccion(f['angulo'])}
                            for f in det['flechas']],
                'canastos': [{'color': c['color']} for c in det['canastos']]}
            claves = ('rumbo_deg', 'estado', 'confianza', 'libre', 'profundidad')
            self.estado['ruta'] = {k: ruta[k] for k in claves}
            if ok:
                with self.cond:
                    self.jpeg, self.jpeg_num = jpg.tobytes(), self.jpeg_num + 1
                    self.cond.notify_all()

    def json_estado(self):
        e = self.esp
        return {'conectado': self.fuente.conectado, 'error': self.fuente.error,
                'fps_recibidos': round(self.fuente.fps, 1),
                'fps_proceso': round(self.estado['fps_proceso'], 1),
                'ms_proceso': round(self.estado['ms_proceso'], 1),
                'detecciones': self.estado['detecciones'], 'ruta': self.estado['ruta'],
                'capas': self.capas, 'mensaje': self.mensaje, 'rotacion': self.rotacion,
                'esp': {k: e.get(k) for k in list(AJUSTES) + ['fps', 'rssi_dbm', 'uptime_s']}
                if e else {}}


def hacer_handler(panel):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, datos, codigo=200):
            cuerpo = json.dumps(datos).encode()
            self.send_response(codigo)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(cuerpo)))
            self.end_headers()
            self.wfile.write(cuerpo)

        def _cuerpo(self):
            n = int(self.headers.get('Content-Length') or 0)
            return json.loads(self.rfile.read(n) or b'{}')

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                cuerpo = PAGINA.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(cuerpo)))
                self.end_headers()
                self.wfile.write(cuerpo)
            elif self.path.startswith('/api/estado'):
                self._json(panel.json_estado())
            elif self.path.startswith('/video'):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=cuadro')
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                visto = 0
                try:
                    while True:
                        with panel.cond:
                            panel.cond.wait_for(lambda: panel.jpeg_num != visto, timeout=2.0)
                            jpg, visto = panel.jpeg, panel.jpeg_num
                        if not jpg:
                            continue
                        self.wfile.write(b'--cuadro\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                except OSError:
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            try:
                datos = self._cuerpo()
                if self.path == '/api/camara':
                    self._json({'ok': True, 'valor': panel.ajustar(datos['var'], datos['val'])})
                elif self.path == '/api/capas':
                    for k in panel.capas:
                        if k in datos:
                            panel.capas[k] = bool(datos[k])
                    self._json({'ok': True})
                elif self.path == '/api/rotacion':
                    panel.poner_rotacion(datos['grados'])
                    self._json({'ok': True})
                elif self.path == '/api/foto':
                    self._json({'ok': True, 'ruta': panel.guardar_foto()})
                elif self.path == '/api/guardar':
                    self._json({'ok': True, 'guardado': panel.guardar()})
                else:
                    self.send_error(404)
            except (OSError, ValueError, KeyError) as e:
                self._json({'ok': False, 'error': str(e)}, 400)
    return H


def main():
    ap = argparse.ArgumentParser(description='Panel web de la ESP32-CAM')
    ap.add_argument('--camara', default='http://192.168.1.42', help='http://IP de la ESP32-CAM')
    ap.add_argument('--puerto', type=int, default=8080)
    ap.add_argument('--fotos', default=os.path.expanduser('~/fotos_camara'),
                    help='carpeta donde se guardan las fotos del botón "Guardar foto"')
    ap.add_argument('--colores', default=buscar_colores_yaml(),
                    help='colores.yaml a leer y escribir')
    args = ap.parse_args()

    panel = Panel(args.camara, args.colores, args.fotos)
    panel.fuente.start()
    threading.Thread(target=panel.sondear_esp, daemon=True).start()
    threading.Thread(target=panel.bucle_proceso, daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', args.puerto), hacer_handler(panel))
    srv.daemon_threads = True
    print(f'Panel en http://localhost:{args.puerto}   cámara: {args.camara}')
    print(f'colores: {args.colores}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
