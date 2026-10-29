"""
Corre la misión completa en el simulador 2D y mide el resultado.

    python3 -m caid_navigation.simulacion.correr e3_obstaculos [--vision render] [--sin-ruido]

`--vision oraculo` (por defecto) usa detecciones perfectas; `--vision render` dibuja la imagen de
la
cámara y corre el detector de caid_vision (necesita OpenCV); `--vision no` navega sin cámara.
"""
import argparse
import math
import time

from caid_navigation.camara import Camara
from caid_navigation.geometria import a_mundo
from caid_navigation.mision import Navegador
from caid_navigation.simulacion import oraculo
from caid_navigation.simulacion.mundo import Mundo
from caid_navigation.simulacion.robot import RobotSim

ALCANCE_BRAZO = 0.20     # P: del eje del brazo al cubo o canasto para dar por hecho el agarre


class Resultado:
    def __init__(self):
        self.tiempo = 0.0
        self.entregados = []
        self.errores = []
        self.choques = []
        self.zonas = 0
        self.recorrido = 0.0
        self.fin = False
        self.en_inicio = False
        self.mensajes = []
        self.mapa = ''
        self.error_max = (0.0, 0.0)
        self.atascado = False
        self.errores_pose = []

    @property
    def completa(self):
        return self.fin and self.en_inicio and len(self.entregados) == 3 and not self.errores

    def resumen(self):
        return (f'{"COMPLETA" if self.completa else "INCOMPLETA"}: {self.tiempo:.0f} s, '
                f'entregados {self.entregados}, errores {self.errores}, '
                f'choques {len(self.choques)}, '
                f'pisadas de zona {self.zonas}, recorrido {self.recorrido:.1f} m, '
                f'error de pose máx {self.error_max[0] * 100:.1f} cm / '
                f'{math.degrees(self.error_max[1]):.1f}°')


def a_mapa(mundo, pose):
    """Pose del mundo de Gazebo expresada en el marco map (nace en la partida)."""
    x0, y0, th0 = mundo.inicio
    dx, dy = pose[0] - x0, pose[1] - y0
    c, s = math.cos(th0), math.sin(th0)
    return (c * dx + s * dy, -s * dx + c * dy, math.remainder(pose[2] - th0, 2 * math.pi))


def correr(nombre, t_max=900.0, dt=0.05, vision='oraculo', ruido=True, semilla=1,
           parametros=None, detalle=False, cada_imagen=0.2, seguir_error=True):
    mundo = Mundo(nombre)
    robot = RobotSim(mundo, ruido=ruido, semilla=semilla)
    params = dict(parametros or {})
    if vision == 'no':
        params['usar_camara'] = False
    nav = Navegador(params, significado=mundo.significado)
    cam = Camara()
    detector = None
    if vision == 'render':
        from caid_navigation.simulacion.render import Render
        from caid_vision.detector import detectar
        render = Render(mundo, cam)
        detector = detectar
    res = Resultado()
    t = 0.0
    prox_imagen = 0.0
    en_zona = False
    while t < t_max:
        rangos = {lado: robot.rango(lado) for lado in ('frente', 'izquierda', 'derecha')}
        v, w, pan = nav.actualizar(t, tuple(robot.odom), rangos, robot.pan)
        robot.paso(dt, v, w, pan)
        t += dt
        if vision != 'no' and t >= prox_imagen:
            prox_imagen = t + cada_imagen
            if detector is None:
                det = oraculo.detecciones(mundo, robot, cam, robot.pose, robot.pan)
            else:
                det = detector(render.imagen(robot.pose, robot.pan))
            nav.vision(t, det, nav.pose, robot.pan)
        while nav.eventos:
            accion, color, rel = nav.eventos.pop(0)
            nav.resultado_manipulacion(manipular(mundo, robot, accion, color, res))
        if seguir_error:
            verdad = a_mapa(mundo, robot.pose)
            est = nav.pose
            e = (math.hypot(verdad[0] - est[0], verdad[1] - est[1]),
                 abs(math.remainder(verdad[2] - est[2], 2 * math.pi)))
            res.error_max = (max(res.error_max[0], e[0]), max(res.error_max[1], e[1]))
            res.errores_pose.append((t, e[0], e[1]))
        rc = mundo.celda_rc(robot.pose[0], robot.pose[1])
        if rc in mundo.celdas_zona and not en_zona:
            res.zonas += 1
        en_zona = rc in mundo.celdas_zona
        if nav.estado == 'FIN':
            res.fin = True
            break
        if robot.choques and robot.t - robot.choques[0][0] > 15.0 and len(robot.choques) > 10:
            res.atascado = True
            break
    res.tiempo = t
    res.choques = robot.choques
    res.recorrido = robot.recorrido
    res.en_inicio = mundo.celda_rc(robot.pose[0], robot.pose[1]) == mundo.inicio_celda
    res.mensajes = nav.mensajes
    res.mapa = nav.con.dibujo(nav.pose)
    res.nav = nav
    res.robot = robot
    if detalle:
        for marca, texto in nav.mensajes:
            print(f'{marca:7.1f}  {texto}')
    return res


def manipular(mundo, robot, accion, color, res):
    """Brazo simulado: agarra el cubo si está al alcance; suelta en el canasto más cercano."""
    bx, by = a_mundo(robot.pose, -0.0213, 0.0)
    if accion == 'agarrar':
        cubo = mundo.cubos.get(color)
        if cubo is None or not cubo.activa:
            return False
        cx, cy = (cubo.caja[0] + cubo.caja[2]) / 2, (cubo.caja[1] + cubo.caja[3]) / 2
        if math.hypot(cx - bx, cy - by) > ALCANCE_BRAZO:
            dist = math.hypot(cx - bx, cy - by)
            res.errores.append(f'cubo {color} fuera de alcance ({dist:.2f} m)')
            return False
        cubo.activa = False
        return True
    mejor = min(mundo.canastos.items(), key=lambda kv: math.hypot(kv[1][0] - bx, kv[1][1] - by))
    dist = math.hypot(mejor[1][0] - bx, mejor[1][1] - by)
    if dist > ALCANCE_BRAZO + 0.05:
        res.errores.append(f'canasto fuera de alcance ({dist:.2f} m)')
        return False
    if mejor[0] != color:
        res.errores.append(f'cubo {color} en el canasto {mejor[0]}')
    res.entregados.append(color)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('laberintos', nargs='+')
    ap.add_argument('--vision', default='oraculo', choices=('oraculo', 'render', 'no'))
    ap.add_argument('--sin-ruido', action='store_true')
    ap.add_argument('--semilla', type=int, default=1)
    ap.add_argument('--t-max', type=float, default=900.0)
    ap.add_argument('--detalle', action='store_true')
    ap.add_argument('--mapa', action='store_true')
    a = ap.parse_args()
    for nombre in a.laberintos:
        t0 = time.time()
        r = correr(nombre, a.t_max, vision=a.vision, ruido=not a.sin_ruido, semilla=a.semilla,
                   detalle=a.detalle)
        print(f'{nombre}: {r.resumen()}  [{time.time() - t0:.0f} s reales]')
        for ch in r.choques:
            print(f'   choque t={ch[0]:.1f} contra {ch[2]} en {tuple(round(c, 3) for c in ch[1])}')
        if a.mapa:
            print(r.mapa)


if __name__ == '__main__':
    main()


def errores_de_mapa(mundo, con):
    """Lados del mapa del robot que no coinciden con el mundo: [(celda, d, dice, verdad)]."""
    x0, y0, th0 = mundo.inicio
    C = mundo.celda
    c, s = math.cos(th0), math.sin(th0)

    def a_rc(celda):
        mx, my = celda[0] * C, celda[1] * C
        return mundo.celda_rc(x0 + c * mx - s * my, y0 + s * mx + c * my)
    malos = []
    for (celda, d), abierto in con.mapa.lados.items():
        if (celda, d) in con.bloqueos:
            continue
        vec = (celda[0] + (1, 0, -1, 0)[d], celda[1] + (0, 1, 0, -1)[d])
        rc_a, rc_b = a_rc(celda), a_rc(vec)
        if rc_a in mundo.bloques:
            continue
        verdad = rc_b not in mundo.bloques and 0 <= rc_b[0] < mundo.lab.nfilas \
            and 0 <= rc_b[1] < mundo.lab.ncols
        if abierto != verdad:
            malos.append((celda, d, abierto, verdad))
    return malos
