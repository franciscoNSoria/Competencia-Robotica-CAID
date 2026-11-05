#!/usr/bin/env python3
"""Robot en el laberinto, SIN cámara: recorre todo con Trémaux usando solo los sonares y vuelve a la partida, frenando antes de
chocar. La navegación está en tremaux_logica.py; esto es el enlace con la ESP32 (igual que tools/torneo/torneo.py, que se reutiliza).

    python3 tools/tremaux/tremaux.py          # g arranca, ESPACIO frena, q sale

Habla directo con la ESP32 (firmware/banco/robot_4ruedas): lee /wheel_state y /wheel_ticks, manda /cmd_vel y /robot_cmd. No hace
falta caid_bringup ni la cámara. Requisitos: ROS 2 Jazzy cargado, el agente micro-ROS corriendo y la ESP32 conectada. CERRAR
cualquier otro programa que mande /cmd_vel (torneo.py, robot_autonomo.py, el panel de ruedas): dos programas se pisan.
Guarda un CSV de cada corrida en ~/caid/config/tremaux_AAAAMMDD_HHMMSS.csv."""
import argparse
import csv
import datetime
import json
import math
import os
import sys
import termios
import time
import tty

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', 'torneo'))
import torneo as to  # noqa: E402
import torneo_logica as tl  # noqa: E402
import tremaux_logica as tx  # noqa: E402

AYUDA = "  g arrancar    ESPACIO detener    q salir    ? ayuda"


class Tremaux(to.Torneo):
    """to.Torneo sin cámara y con el piloto de Trémaux."""

    def __init__(self, enlace, calib, params, imprimir=print, carpeta=to.CARPETA):
        super().__init__(enlace, calib, params, usar_camara=False, imprimir=imprimir, carpeta=carpeta)
        self.giro_del = None
        try:
            with open(os.path.join(carpeta, 'calibracion.json')) as f:
                c = json.load(f)
            if 'giro_factor_del' in c:
                self.giro_del = (float(c['giro_factor_del']), int(c.get('kick_del', 0)))
        except (OSError, ValueError):
            pass
        self._f.close()
        os.remove(self.ruta_csv)
        self.piloto = tx.PilotoTremaux(params, trocha=calib['trocha_m'], diametro=calib['diametro_m'], imprimir=imprimir)
        self.ruta_csv = os.path.join(carpeta, 'tremaux_%s.csv' % datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        self._f = open(self.ruta_csv, 'w', newline='')
        self.csv = csv.writer(self._f)
        self.csv.writerow(['t', 'estado', 'modo', 'celda', 'rumbo', 'v', 'w', 'frente', 'izq', 'der', 'guardia', 'y_lat', 'psi_deg',
                           's', 'off_cm', 'motivo'])

    def paso(self, ahora):
        # igual que Torneo.paso pero con la columna off_cm; se arma la fila acá para no tocar el archivo de torneo
        e = self.enlace
        tele = self.tele(ahora)
        if tele is not None and not self.calib_aplicada(tele) and ahora - self._t_calib >= 1.0:
            self._t_calib = ahora
            c = self.calib
            e.enviar_cmd(5, c['diametro_m'], c['trocha_m'], c['frenado_m'], c['frenado_giro_m'])
        g = self.piloto.guardia()
        if ahora - self._t_cfg >= 1.0 or g != self._g_ult:
            self._t_cfg, self._g_ult = ahora, g
            e.enviar_cmd(8, 1, g[0], g[1], 5.0)
            if self.giro_del is not None:                # lo que se puso en el panel de ruedas (orden 9)
                e.enviar_cmd(9, *self.giro_del)
        odo = self.odometria()
        if tele is None or odo is None:
            self.v = self.w = 0.0
            e.enviar_vel(0.0, 0.0)
            return
        v, w, orden = self.piloto.paso(ahora, tele, odo[0], odo[1])
        self.v, self.w = v, w
        e.enviar_vel(v, w)
        if orden is not None:
            e.enviar_cmd(*orden)
        p = self.piloto
        fm = lambda m: '' if m is None else '%.3f' % m
        self.csv.writerow(['%.2f' % (ahora - self.t0), p.estado, p.modo, p.celda, p.rumbo, '%.3f' % v, '%.3f' % w,
                           fm(tele['frente']), fm(tele['izq']), fm(tele['der']), tele['guardia'], '%.3f' % p.obs.y,
                           '%.1f' % math.degrees(p.obs.psi), '%.3f' % odo[0], '%.1f' % (100 * p._off), p.motivo])
        self._f.flush()

    def tecla(self, k, ahora):
        if k == 'y':
            return
        super().tecla(k, ahora)
        if k == '?':
            self.imprimir(AYUDA)


def main():
    d = tx.Parametros()
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--v', type=float, default=d.v_crucero,
                    help='velocidad de crucero, m/s (%.2f; por debajo de ~0,17 las ruedas no la sostienen)' % d.v_crucero)
    a = ap.parse_args()
    params = tx.Parametros(v_crucero=a.v)

    calib, hay = to.cargar_calibracion()
    print('Calibración %s: diámetro %.1f mm, trocha %.0f mm' % (
        'de calibracion.json' if hay else 'POR DEFECTO (no hay ~/caid/config/calibracion.json)',
        1000 * calib['diametro_m'], 1000 * calib['trocha_m']))
    enlace = to.crear_enlace_ros(False)
    interactivo = sys.stdin.isatty()

    def imprimir(m):
        print('\r\x1b[K' + m, flush=True)

    tor = Tremaux(enlace, calib, params, imprimir=imprimir)
    term = termios.tcgetattr(sys.stdin) if interactivo else None
    if interactivo:
        tty.setcbreak(sys.stdin.fileno())
    print('Trémaux sin cámara: recorre todo y vuelve a la partida. Teclas: g arrancar, ESPACIO detener, q salir.')
    print('Registro: %s' % tor.ruta_csv)
    if tor.giro_del is not None:
        print('Giros: delanteras x%g, empujon de arranque nivel %d (del panel de ruedas).' % tor.giro_del)
    print('El robot va en el CENTRO de la celda de entrada, derecho, mirando hacia adentro. Esperá a que diga "guardia 1" y apretá g.')
    t_paso = t_linea = 0.0
    try:
        while True:
            enlace.atender()
            ahora = time.time()
            k = to.tecla() if interactivo else None
            if k == 'q':
                break
            if k is not None:
                tor.tecla(k, ahora)
            if ahora - t_paso >= 0.05:
                t_paso = ahora
                tor.paso(ahora)
            if ahora - t_linea >= 0.25:
                t_linea = ahora
                print('\r\x1b[K' + tor.linea(ahora), end='', flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        tor.cerrar()
        if interactivo:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term)
        time.sleep(0.2)
        try:
            enlace.atender()
            enlace.cerrar()
        except Exception:
            pass
        print('\nListo. El robot quedó frenado y la guardia apagada.')
        if tor.piloto.eventos:
            print('Eventos para revisar:')
            for t, txt in tor.piloto.eventos:
                print('  %s' % txt)


if __name__ == '__main__':
    main()
