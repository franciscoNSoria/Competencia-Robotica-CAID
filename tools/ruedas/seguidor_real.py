#!/usr/bin/env python3
"""Seguidor de flechas en el laberinto REAL (ver seguidor.py): avanza derecho enderezandose con los sonares, dobla 90° hacia la flecha
que fija la camara apenas el sonar lateral ve el pasillo libre (+12 cm), avanza lento despues de doblar, se detiene a ~4 cm del cubo,
ESPERA la tecla Y, vuelve a la entrada, 'deja el cubo' (simulado) y vuelve a entrar a buscar el siguiente ignorando el color elegido.

    export CAID_DOMINIO=71; source ~/caid/env_ros.sh
    python3 tools/ruedas/seguidor_real.py            # g arranca | y captura el cubo (simulado) | ESPACIO o p frena | q sale

Hace falta: el agente micro-ROS con la ESP32 conectada y la camara + vision (robot.launch.xml) corriendo. Cerrar el panel de ruedas.
El robot en el CENTRO del pasillo de entrada, mirando hacia adentro. Lee los sumadores del Giro 90 / 180 de ~/caid/config/calibracion.json."""
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
import seguidor as sg  # noqa: E402

IMPORTANTE = ('CORRIJO', 'FRENO', 'Pienso', 'Pegado', 'Separado', 'ME DETENGO', 'ABORT', 'Giro hecho', 'Giro 1/2', 'Giro 2/2',
              'ESP32', 'Frene', 'Se cortaron', 'Maniobra')


def impresora(detalle):
    if detalle:
        return print
    return lambda m: print(m) if any(k in m for k in IMPORTANTE) else None


def tecla():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def conectar_vision(enlace, seg, color=None):
    """/vision/proximo_giro y /vision/cubo/* -> seguidor; el color elegido y los ignorados salen por /nav/estado (latched)."""
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    pub = enlace.create_publisher(String, '/nav/estado', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def avisar(busca, ignora):
        pub.publish(String(data=json.dumps({'busca': busca, 'ignora': ignora})))

    def al_llegar(msg):
        try:
            seg.recibir_flecha(json.loads(msg.data))
        except ValueError:
            pass

    seg.al_cambiar_estado = avisar
    enlace.create_subscription(String, '/vision/proximo_giro', al_llegar, 10)
    for col in ('rojo', 'verde', 'azul'):
        enlace.create_subscription(Point, '/vision/cubo/' + col, lambda m, col=col: seg.recibir_cubo(col, m.z), 10)
    if color:
        seg.busca = color
    avisar(seg.busca, [])
    print('Vision: escucho /vision/proximo_giro y /vision/cubo/* (%s)' % ('sigo ' + color if color else 'el primer color que aparezca'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--v', type=float, default=sg.V_NORMAL_MS, help='velocidad de avance, m/s (0.16)')
    ap.add_argument('--v-lento', type=float, default=sg.V_LENTO_MS, help='velocidad despues de doblar, m/s (0.10)')
    ap.add_argument('--lento-m', type=float, default=sg.LENTO_M, help='tramo lento despues de doblar, m (0.25)')
    ap.add_argument('--avance-giro', type=float, default=sg.AVANCE_GIRO_M,
                    help='m que sigue desde que el lateral ve el pasillo libre hasta doblar (0.12; ~0.30 deja el centro del robot en el centro del cruce)')
    ap.add_argument('--parada-cubo', type=float, default=sg.PARADA_CUBO_M, help='distancia a la que se detiene del cubo, m (0.04)')
    ap.add_argument('--fx', type=float, default=sg.FX_CAMARA_PX, help='foco de la camara en px para medir el cubo (950)')
    ap.add_argument('--vel-giro', type=float, default=153.0, help='velocidad de giro, grados/s')
    ap.add_argument('--color', default='', help='color de flecha a seguir desde el principio (rojo/verde/azul)')
    ap.add_argument('--volver', action='store_true', help='tras el OK volver solo a la entrada (por defecto: media vuelta, mapa y se queda)')
    ap.add_argument('--detalle', action='store_true', help='mostrar todo lo del controlador')
    a = ap.parse_args()
    enlace = rp.crear_enlace_ros()
    c = rl.Controlador(enlace, imprimir=impresora(a.detalle))
    corr90 = float(c.cal.get('giro90_corr_deg', 0.0))
    corr180 = float(c.cal.get('giro180_corr_deg', 0.0))
    seg = sg.Seguidor(c, vel_dps=a.vel_giro, corr90=corr90, corr180=corr180, v=a.v, v_lento=a.v_lento, lento_m=a.lento_m,
                      avance_giro_m=a.avance_giro, parada_cubo_m=a.parada_cubo, volver=a.volver)
    seg.fx = a.fx
    conectar_vision(enlace, seg, a.color or None)
    print('Giro 90: se piden %g° | avance %.2f m/s, lento %.2f m/s en %.0f cm | doblo %.0f cm despues del pasillo libre | paro a %.0f cm del cubo' % (
        90 + corr90, a.v, a.v_lento, 100 * a.lento_m, 100 * a.avance_giro, 100 * a.parada_cubo))
    print('  g arrancar    y OK frente al cubo (180° + mapa)    ESPACIO o p frenar    q salir')
    ant = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        t_ult = time.monotonic()
        while True:
            enlace.atender()
            c.tick()
            seg.tick()
            k = tecla()
            if k == 'g' and seg.estado in ('espera', 'fin', 'detenido'):
                if not c.conectado:
                    print('La ESP32 todavia no esta conectada.')
                else:
                    seg.arrancar()
            elif k in ('y', 'Y'):
                seg.confirmar()
            elif k in (' ', 'p'):
                seg.detener('frenado con el teclado')
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
