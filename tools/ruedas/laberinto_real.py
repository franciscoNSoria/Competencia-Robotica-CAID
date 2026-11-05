#!/usr/bin/env python3
"""Trémaux en el laberinto REAL con las piezas del panel de ruedas (LINEA RECTA enderezando, Giro 90° con tu sumador, 180° en dos giros de 90°).

    source /opt/ros/jazzy/setup.bash
    python3 tools/ruedas/laberinto_real.py            # g arranca, ESPACIO / p frena, q sale

Poné el robot en el CENTRO de la celda de partida mirando hacia el interior del laberinto (esa celda es (0,0) y el rumbo inicial +x).
Hace falta: el agente micro-ROS corriendo y la ESP32 con el firmware de robot_4ruedas. CERRAR el panel de ruedas y cualquier otro
programa que mande /cmd_vel. Lee el sumador del Giro 90° (giro90_corr_deg) y el de 180° de ~/caid/config/calibracion.json: son los que
dejaste en el panel de ruedas. Se detiene solo (y lo dice) si no logra separarse de una pared o el pivote no arranca."""
import argparse
import os
import select
import sys
import termios
import time
import tty

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import ruedas_logica as rl  # noqa: E402
import ruedas_panel as rp  # noqa: E402
import laberinto_ruedas as lr  # noqa: E402


def tecla():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--v', type=float, default=rl.V_LENTA_MS, help='velocidad de avance, m/s (0.16)')
    ap.add_argument('--vel-giro', type=float, default=153.0, help='velocidad de giro, grados/s (153 = maximo)')
    ap.add_argument('--frenada', type=float, default=0.03, help='m que el robot sigue despues de la orden de frenar (0.03)')
    a = ap.parse_args()
    enlace = rp.crear_enlace_ros()
    c = rl.Controlador(enlace)
    corr90 = float(c.cal.get('giro90_corr_deg', 0.0))
    corr180 = float(c.cal.get('giro180_corr_deg', 0.0))
    rec = lr.Recorrido(c, vel_dps=a.vel_giro, corr90=corr90, corr180=corr180, v=a.v, frenada_m=a.frenada)
    print('Giro 90: se piden %g° | sumador 180: %+g° | avance %.2f m/s | giro %.0f °/s' % (90 + corr90, corr180, a.v, a.vel_giro))
    print('  g arrancar    ESPACIO o p frenar    q salir')
    ant = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        t_ult = time.monotonic()
        while True:
            enlace.atender()
            c.tick()
            rec.tick()
            k = tecla()
            if k == 'g' and rec.estado in ('espera', 'fin', 'detenido'):
                if not c.conectado:
                    print('La ESP32 todavia no esta conectada.')
                else:
                    rec.arrancar()
            elif k in (' ', 'p'):
                rec.detener('frenado con el teclado')
            elif k == 'q':
                break
            time.sleep(max(0.0, 0.05 - (time.monotonic() - t_ult)))
            t_ult = time.monotonic()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ant)
        try:
            c.parar()
        finally:
            c.cerrar()
            enlace.cerrar()


if __name__ == '__main__':
    main()
