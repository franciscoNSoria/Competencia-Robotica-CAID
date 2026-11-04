#!/usr/bin/env python3
"""
Simulador rápido de la misión: corre el explorador.py de verdad, sin ROS ni Gazebo, en segundos.

Sirve para probar la LÓGICA de la misión (exploración, flechas, carga, entrega, puerta y punto de
reingreso) antes de gastar una corrida de Gazebo. No prueba el control fino ni la visión real:

- El robot se teletransporta de centro de celda a centro de celda y gira exacto.
- Los ultrasonidos dan la distancia exacta a la pared de cada lado.
- La cámara es un oráculo: proyecta las flechas, los cubos y los canastos del mundo con el mismo
  modelo de cámara que usa la navegación (caid_navigation/camara.py), y le pasa al explorador las
  detecciones en píxeles, como haría vision_node. Así ubicar_flecha y ubicar_canasto trabajan
  con números de verdad. Tiene alcance (1,2 m para flechas) y respeta las paredes que tapan.
- El mundo sale del mismo YAML que usa gen_maze.py, con las mismas posiciones de flechas,
  cubos y canastos.

Uso (desde la raíz del repo):
    python3 tools/sim/mision_rapida.py ros2_ws/src/caid_gazebo/mazes/e2_bifurcaciones_simple.yaml
    python3 tools/sim/mision_rapida.py <yaml> --abierta        # zona de partida sin paredes
    python3 tools/sim/mision_rapida.py <yaml> --canasto-falso  # repite el falso canasto de E2
    python3 tools/sim/mision_rapida.py <yaml> -v               # con todo el log del explorador
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import types

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(RAIZ, 'ros2_ws', 'src', 'caid_navigation'))

T = [0.0]            # reloj simulado (s)
LOG = []
VERBOSO = [False]


# ---------------------------------------------------------------------------------------------
# ROS de mentira: lo justo para que explorador.py importe y corra
# ---------------------------------------------------------------------------------------------
class Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, nombre):
        if nombre.startswith('__'):
            raise AttributeError(nombre)
        v = Msg()
        setattr(self, nombre, v)
        return v


class _Logger:
    def _log(self, nivel, texto):
        LOG.append((T[0], nivel, texto))
        if VERBOSO[0]:
            print(f'[{T[0]:7.1f}] {nivel:5s} {texto}')

    def info(self, t):
        self._log('INFO', t)

    def warn(self, t):
        self._log('WARN', t)

    def error(self, t):
        self._log('ERROR', t)


class _Pub:
    def publish(self, msg):
        pass


class _Futuro:
    def __init__(self, r):
        self.r = r

    def result(self):
        return self.r

    def add_done_callback(self, cb):
        cb(self)


class _ClienteAccion:
    def __init__(self, *a):
        pass

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        meta = Msg(accepted=True,
                   get_result_async=lambda: _Futuro(Msg(result=Msg(exito=True))))
        return _Futuro(meta)


PARAMS = {}


class _Nodo:
    def __init__(self, nombre):
        self._p = {}

    def declare_parameter(self, n, v):
        self._p[n] = PARAMS.get(n, v)

    def get_parameter(self, n):
        return Msg(value=self._p[n])

    def create_publisher(self, *a):
        return _Pub()

    def create_subscription(self, *a):
        return None

    def create_timer(self, *a):
        return None

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return Msg(now=lambda: Msg(nanoseconds=int(T[0] * 1e9), to_msg=lambda: None))


def _modulo(nombre, **attrs):
    m = types.ModuleType(nombre)
    m.__dict__.update(attrs)
    sys.modules[nombre] = m
    return m


def stub_ros():
    _modulo('rclpy', init=lambda *a: None, spin=lambda *a: None, try_shutdown=lambda *a: None)
    _modulo('rclpy.node', Node=_Nodo)
    _modulo('rclpy.action', ActionClient=_ClienteAccion)
    _modulo('rclpy.qos', DurabilityPolicy=Msg(), QoSProfile=Msg)
    _modulo('caid_interfaces')
    _modulo('caid_interfaces.action', Cargar=Msg(Goal=lambda **kw: Msg(**kw)),
            Dejar=Msg(Goal=lambda **kw: Msg(**kw)))
    for paquete, clases in (('geometry_msgs', ('Point', 'TransformStamped', 'Twist')),
                            ('nav_msgs', ('Odometry',)),
                            ('sensor_msgs', ('JointState', 'Range')),
                            ('std_msgs', ('Float64', 'String')),
                            ('visualization_msgs', ('Marker', 'MarkerArray'))):
        _modulo(paquete)
        _modulo(paquete + '.msg', **{c: Msg for c in clases})
    _modulo('tf2_ros', TransformBroadcaster=lambda *a: _Pub())


# ---------------------------------------------------------------------------------------------
# El mundo, en el marco de la odometría (origen en la S, x hacia donde mira el robot)
# ---------------------------------------------------------------------------------------------
def cargar_gen_maze():
    ruta = os.path.join(RAIZ, 'ros2_ws', 'src', 'caid_gazebo', 'scripts', 'gen_maze.py')
    spec = importlib.util.spec_from_file_location('gen_maze', ruta)
    gm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gm)
    return gm


def abrir_partida(lab):
    """Saca las paredes de la zona de partida, igual que el mundo *_simple_abierta.world."""
    ruta = os.path.join(RAIZ, 'tools', 'sim', 'simplificar_laberintos.py')
    spec = importlib.util.spec_from_file_location('simplificar_laberintos', ruta)
    sl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sl)
    lab.filas = sl.partida_abierta(lab.filas)


class Mundo:
    def __init__(self, gm, lab, falso):
        self.gm, self.lab = gm, lab
        _, info = gm.validar(lab)
        (rs, cs), = lab.buscar('S')
        self.s = lab.centro(rs, cs)
        self.yaw0 = gm.YAW[lab.inicio_dir]
        self.tam = lab.celda
        self.paredes = set()
        for r in range(lab.nfilas):
            for c in range(len(lab.filas[r])):
                if lab.filas[r][c] == '#':
                    self.paredes.add(self.celda_de(*self.a_odom(*lab.centro(r, c))))
        # Una celda con objetos que no dejan el hueco que necesita el robot (el mismo criterio de
        # gen_maze: ancho del robot más 15 mm por lado) se toma como pared: el robot no pasa y el
        # ultrasonido lo ve. Un bloque de 8 cm pegado a la pared deja pasar; uno de 20 cm, según.
        self.bloqueos = []
        libres, pasable = gm.construir_grafo(lab)
        for (r, c) in libres:
            for eje, dirs in (('h', 'EO'), ('v', 'NS')):
                hay = any((r + gm.DIRS[d][0], c + gm.DIRS[d][1]) in libres for d in dirs)
                if hay and not pasable[(r, c)][eje] and lab.car(r, c) not in 'RGB':
                    celda = self.celda_de(*self.a_odom(*lab.centro(r, c)))
                    self.paredes.add(celda)
                    self.bloqueos.append(((r, c), celda))
                    break
        colores = {'R': 'rojo', 'G': 'verde', 'B': 'azul'}
        self.cubos = {}
        for simb, rc in info['cubos'].items():
            x, y, _ = gm.posicion_cubo(lab, simb, rc)
            self.cubos[colores[simb]] = (*self.a_odom(x, y), lab.altura_apoyo + gm.CUBO_M / 2)
        self.canastos = {}
        for k, rc in info['deposito'].items():
            x, y = gm.posicion_canasto(lab, rc)
            z = lab.altura_borde_canasto - gm.CANASTO_ALTO_M / 2
            self.canastos[colores[{'1': 'R', '2': 'G', '3': 'B'}[k]]] = (*self.a_odom(x, y), z)
        if falso:
            # Repite lo de la corrida en Gazebo: algo verde con forma de canasto en el pasillo,
            # en la celda del cubo verde (en E2 se anotó un "canasto verde" adentro).
            x, y, _ = self.cubos['verde']
            cx, cy = self.celda_de(x, y)
            self.falso = (cx * self.tam + 0.05, cy * self.tam - 0.05, 0.10)
        else:
            self.falso = None
        self.flechas = []
        for fl in lab.flechas:
            centro, (u, p, _) = gm.posicion_flecha(lab, fl)
            total, _, _, cuerpo, _ = gm.FLECHA_GEOM_M[int(fl['largo'])]
            pts = {}
            for clave, a in (('cola', -total / 2), ('cuello', -total / 2 + cuerpo),
                             ('punta', total / 2)):
                wx, wy, wz = (centro[i] + a * u[i] for i in range(3))
                pts[clave] = (*self.a_odom(wx, wy), wz)
            n = gm._vec_mundo(fl['pared'])
            self.flechas.append((fl['color'], pts, self.rotar(n[0], n[1])))

    def rotar(self, dx, dy):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        return c * dx + s * dy, -s * dx + c * dy

    def a_odom(self, x, y):
        return self.rotar(x - self.s[0], y - self.s[1])

    def celda_de(self, x, y):
        return (round(x / self.tam), round(y / self.tam))

    def libre(self, celda):
        return celda not in self.paredes

    def distancia(self, celda, d, desde_centro):
        """Distancia del sensor a la primera pared hacia d (4 m si no hay)."""
        paso = ((1, 0), (0, 1), (-1, 0), (0, -1))[d]
        for k in range(1, 12):
            if not self.libre((celda[0] + k * paso[0], celda[1] + k * paso[1])):
                return (k - 1) * self.tam + self.tam / 2 - desde_centro
        return 4.0

    def se_ve(self, o, p):
        """¿Ninguna pared entre el origen de la cámara y el punto (en planta)?"""
        largo = math.hypot(p[0] - o[0], p[1] - o[1])
        n = int(largo / 0.02)
        for i in range(1, max(n - 1, 1)):
            t = i / n
            if not self.libre(self.celda_de(o[0] + t * (p[0] - o[0]),
                                            o[1] + t * (p[1] - o[1]))):
                return False
        return True


def caja(cam, pose, pan, centro, lados):
    """Caja en píxeles (x, y, w, h) de un prisma, o None si no entra entero en la imagen."""
    uv = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                q = cam.proyectar(pose, pan, (centro[0] + sx * lados[0] / 2,
                                              centro[1] + sy * lados[1] / 2,
                                              centro[2] + sz * lados[2] / 2))
                if q is None:
                    return None
                uv.append(q)
    xs, ys = [a for a, _ in uv], [b for _, b in uv]
    if min(xs) < 2 or max(xs) > cam.ancho - 3 or min(ys) < 2 or max(ys) > cam.alto - 3:
        return None
    return (int(min(xs)), int(min(ys)), int(max(xs) - min(xs)), int(max(ys) - min(ys)))


def detecciones(mundo, e, pose):
    """Lo que vería vision_node desde esa pose, en el formato de /vision/detecciones."""
    cam, pan = e.cam, e.pan
    o = cam.origen(pose, pan)
    det = {'flechas': [], 'cubos': [], 'canastos': [], 'zonas': []}
    for color, pts, normal in mundo.flechas:
        p = pts['cuello']
        if math.hypot(p[0] - o[0], p[1] - o[1]) > 1.2 or not mundo.se_ve(o, p):
            continue
        px = {k: cam.proyectar(pose, pan, v) for k, v in pts.items()}
        if any(q is None or not (2 < q[0] < cam.ancho - 3 and 2 < q[1] < cam.alto - 3)
               for q in px.values()):
            continue
        det['flechas'].append({'color': color, 'area': 500.0, **px})
    for color, c in mundo.cubos.items():
        if mundo.se_ve(o, c):
            b = caja(cam, pose, pan, c, (0.02, 0.02, 0.02))
            if b:
                det['cubos'].append({'color': color, 'caja': b, 'area': float(b[2] * b[3])})
    objetos = list(mundo.canastos.items())
    if mundo.falso:
        objetos.append(('verde', mundo.falso))
    for color, c in objetos:
        if math.hypot(c[0] - o[0], c[1] - o[1]) < 2.0 and mundo.se_ve(o, c):
            b = caja(cam, pose, pan, c, (0.044, 0.044, 0.02))
            if b:
                det['canastos'].append({'color': color, 'caja': b, 'area': float(b[2] * b[3])})
    return det


# ---------------------------------------------------------------------------------------------
# Corrida
# ---------------------------------------------------------------------------------------------
def correr(ruta, abierta=False, falso=False, max_pasos=6000):
    stub_ros()
    gm = cargar_gen_maze()
    lab = gm.cargar(ruta)
    if abierta:
        abrir_partida(lab)
    mundo = Mundo(gm, lab, falso)
    PARAMS.update({'tam_celda': lab.celda, 'umbral_pared': 0.25, 'canastos_con_camara': True,
                   'colores': ['verde', 'rojo', 'azul']})
    from caid_navigation import explorador as ex
    e = ex.Explorador()
    e.publicar_rviz = lambda: None
    pose = [0.0, 0.0, 0.0]
    stats = {'celdas': 0, 'giros': 0, 'barridos': 0, 'alcance': []}
    for _ in range(max_pasos):
        T[0] += 0.35
        celda, rumbo = mundo.celda_de(pose[0], pose[1]), round(pose[2] / (math.pi / 2)) % 4
        e.odom = tuple(pose)
        e.pan = e.pan_pedido                       # el servo llega al toque
        for lado, rel, off in (('frente', 0, 0.1399), ('izquierda', 1, 0.0815),
                               ('derecha', 3, 0.0815)):
            m = mundo.distancia(celda, (rumbo + rel) % 4, off)
            e.lecturas[lado].clear()
            e.lecturas[lado].extend([m, m, m])
        if e.estado not in ('ESPERA', 'CARGAR'):
            texto = json.dumps(detecciones(mundo, e, tuple(pose)))
            for _ in range(3):                        # varias imágenes por paso
                e.on_detecciones(Msg(data=texto))
        antes = (e.estado, e.llevando)
        e.tick()
        if e.estado == 'CARGAR' and antes[0] != 'CARGAR':
            # Distancia REAL del eje de la base del brazo al objeto en el momento de agarrar o
            # dejar: es lo que tiene que quedar dentro del alcance del brazo (193 a 237 mm).
            accion, col, _ = e.accion
            obj = mundo.cubos.get(col) if accion == 'cargar' else mundo.canastos.get(col)
            if obj is not None:
                c, s_ = math.cos(pose[2]), math.sin(pose[2])
                ex, ey = pose[0] + c * e.cfg['eje_brazo_x'], pose[1] + s_ * e.cfg['eje_brazo_x']
                stats['alcance'].append((accion, col, math.hypot(obj[0] - ex, obj[1] - ey)))
        if e.llevando and e.llevando != antes[1]:
            mundo.cubos.pop(e.llevando, None)          # carga_simulada lo saca del mundo
        if e.estado == 'MIRAR' and antes[0] != 'MIRAR':
            stats['barridos'] += 1
        if e.estado in ('ACERCAR', 'ALEJAR'):
            pose[0], pose[1] = e.objetivo['punto']       # el acercamiento, de un salto
        if e.estado == 'AVANZAR':
            # La cámara real manda imágenes todo el camino: un cubo pegado a la pared se ve
            # entrando por el costado de la imagen entre dos celdas, no desde los centros.
            destino = (e.objetivo[0] * mundo.tam, e.objetivo[1] * mundo.tam)
            for f in (0.25, 0.5, 0.75):
                medio = (pose[0] + f * (destino[0] - pose[0]),
                         pose[1] + f * (destino[1] - pose[1]), pose[2])
                e.odom = medio
                texto = json.dumps(detecciones(mundo, e, medio))
                for _ in range(3):
                    e.on_detecciones(Msg(data=texto))
            pose[0], pose[1] = destino
            stats['celdas'] += 1
        elif e.estado == 'GIRAR' and antes[0] != 'GIRAR':
            pose[2] = e.objetivo * math.pi / 2
            stats['giros'] += 1
        if e.estado == 'TERMINADO':
            break
    return e, stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('yaml')
    ap.add_argument('--abierta', action='store_true', help='zona de partida sin paredes')
    ap.add_argument('--canasto-falso', action='store_true',
                    help='un canasto verde falso adentro, al lado del cubo verde')
    ap.add_argument('-v', action='store_true', help='mostrar todo el log del explorador')
    a = ap.parse_args()
    VERBOSO[0] = a.v
    e, st = correr(a.yaml, a.abierta, a.canasto_falso)
    claves = ('Reconocimiento', 'Entrada del', 'Primera bifurcación', 'Llegué al cubo',
              'Entregué', 'Vuelvo al punto', 'En el punto de reingreso', 'Misión terminada',
              'relocalic', 'no me relocalizo', 'borro el mapa', 'me detengo', 'Salí', 'Entré')
    if not a.v:
        for t, nivel, texto in LOG:
            if any(k in texto for k in claves):
                print(f'[{t:7.1f}] {nivel:5s} {texto.splitlines()[0]}')
    for accion, col, r in st['alcance']:
        print(f'  {accion:7s} {col:6s}: objeto a {r * 1000:5.1f} mm del eje del brazo'
              + ('' if 0.193 <= r <= 0.237 else '   <-- FUERA DEL ALCANCE (193 a 237 mm)'))
    print(f'\nEntregados: {e.entregados}  estado: {e.estado}  remapeos: {e.remapeos}  '
          f'celdas recorridas: {st["celdas"]}  giros: {st["giros"]}  barridos: {st["barridos"]}')
    return 0 if e.estado == 'TERMINADO' and len(e.entregados) == 3 else 1


if __name__ == '__main__':
    sys.exit(main())
