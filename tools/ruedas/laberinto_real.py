#!/usr/bin/env python3
"""Trémaux en el laberinto REAL con las piezas del panel de ruedas (LINEA RECTA enderezando, Giro 90° con tu sumador, 180° en dos giros de 90°).

    source /opt/ros/jazzy/setup.bash
    python3 tools/ruedas/laberinto_real.py            # g arranca, ESPACIO / p frena, q sale

Poné el robot en el CENTRO de la celda de partida mirando hacia el interior del laberinto (esa celda es (0,0) y el rumbo inicial +x).
Hace falta: el agente micro-ROS corriendo y la ESP32 con el firmware de robot_4ruedas. CERRAR el panel de ruedas y cualquier otro
programa que mande /cmd_vel. Lee el sumador del Giro 90° (giro90_corr_deg) y el de 180° de ~/caid/config/calibracion.json: son los que
dejaste en el panel de ruedas. Se detiene solo (y lo dice) si no logra separarse de una pared o el pivote no arranca.

OBJETIVO: llegar al cubo del color de las flechas que sigue y frenar ahi (sin vuelta). FLECHAS: si corre caid_vision (vision_node), escucha /vision/proximo_giro y el proximo giro lo decide la flecha FIJA (a 30 cm).
Con varias flechas a la vista sigue la del color que ya busca; al arrancar, la vision elige la P3 (la mas chica: camino mas
corto) y ese color queda fijo: se lo avisa a la vision por /nav/estado. --sin-flechas: Trémaux puro como antes."""
import argparse
import json
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


def conectar_flechas(enlace, rec, color=None):
    """/vision/proximo_giro -> rec.recibir_flecha; el color adoptado sale por /nav/estado (latched) para la vision."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = enlace.create_publisher(String, '/nav/estado', latched)

    def avisar(c):
        pub.publish(String(data=json.dumps({'busca': c})))

    def al_llegar(msg):
        try:
            datos = json.loads(msg.data)
        except ValueError:
            return
        rec.recibir_flecha(datos)

    from geometry_msgs.msg import Point
    rec.al_adoptar_color = avisar
    enlace.create_subscription(String, '/vision/proximo_giro', al_llegar, 10)
    for col in ('rojo', 'verde', 'azul'):
        enlace.create_subscription(Point, '/vision/cubo/' + col, lambda m, col=col: rec.recibir_cubo(col, m.z), 10)
    if color:
        rec.busca = color
        avisar(color)
    print('Flechas: escucho /vision/proximo_giro (%s)' % ('sigo ' + color if color else 'el primer color que aparezca; la vision prefiere la P3'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--v', type=float, default=rl.V_LENTA_MS, help='velocidad de avance, m/s (0.16)')
    ap.add_argument('--vel-giro', type=float, default=153.0, help='velocidad de giro, grados/s (153 = maximo)')
    ap.add_argument('--frenada', type=float, default=0.03, help='m que el robot sigue despues de la orden de frenar (0.03)')
    ap.add_argument('--sin-flechas', action='store_true', help='no escuchar /vision/proximo_giro (Trémaux puro)')
    ap.add_argument('--dist-cubo', type=float, default=lr.DIST_CUBO_M, help='frena al ver el cubo a esto o menos, m (0.20)')
    ap.add_argument('--fx', type=float, default=lr.FX_CAMARA_PX, help='foco de la camara en px para medir el cubo (950)')
    ap.add_argument('--color', default='', help='color de flecha a seguir desde el principio (rojo/verde/azul)')
    a = ap.parse_args()
    enlace = rp.crear_enlace_ros()
    c = rl.Controlador(enlace)
    corr90 = float(c.cal.get('giro90_corr_deg', 0.0))
    corr180 = float(c.cal.get('giro180_corr_deg', 0.0))
    rec = lr.Recorrido(c, vel_dps=a.vel_giro, corr90=corr90, corr180=corr180, v=a.v, frenada_m=a.frenada)
    rec.dist_cubo, rec.fx = a.dist_cubo, a.fx
    if not a.sin_flechas:
        conectar_flechas(enlace, rec, a.color or None)
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
