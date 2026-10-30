#!/usr/bin/env python3
"""Pruebas por componente desde la PC, inalambricas (ESP32 por micro-ROS), como el dia de la competencia.

Uso:  python3 tools/pruebas/prueba.py <prueba>        (sin argumento: lista de pruebas)
      <prueba> puede ser el numero (10), el nombre (sonar_frontal) o el archivo (10_sonar_frontal).

Requisitos: ROS 2 Jazzy cargado (source /opt/ros/jazzy/setup.bash), el agente micro-ROS corriendo y la ESP32 con el
firmware de ESA prueba (firmware/pruebas/compilar.sh). Si el firmware grabado es otra prueba, se avisa en pantalla.
Teclas: las de cada prueba (? muestra la ayuda). q sale y deja todo quieto.
Detalle de cada prueba, conexiones y que mirar: firmware/pruebas/README.md.
"""
import csv
import datetime
import json
import math
import os
import select
import statistics
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

CARPETA = os.path.expanduser('~/caid/config')
RUEDAS = ['FL', 'FR', 'RL', 'RR']
MM_POR_FLANCO = math.pi * 67.0 / 40      # rueda de 67 mm, 40 flancos por vuelta (M)
TERM = None                               # atributos originales de la terminal


# ---------------------------------------------------------------------------------------------------------------------
# Terminal
# ---------------------------------------------------------------------------------------------------------------------
def tecla():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def pedir(texto):
    """Pide una linea con la terminal normal (para escribir numeros)."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, TERM)
    try:
        print()
        return input(texto).strip()
    except EOFError:
        return ''
    finally:
        tty.setcbreak(sys.stdin.fileno())


def pedir_numeros(texto, n):
    t = pedir(texto).replace(',', '.').split()
    try:
        v = [float(x) for x in t]
    except ValueError:
        print('  no entendi los numeros')
        return None
    if len(v) != n:
        print(f'  hacen falta {n} numeros')
        return None
    return v


def confirmar(texto):
    return pedir(texto + ' (s/n): ').lower().startswith('s')


def linea(texto):
    sys.stdout.write('\r\033[K' + texto)
    sys.stdout.flush()


def msg(texto):
    sys.stdout.write('\r\033[K' + texto + '\n')
    sys.stdout.flush()


def dist(x):
    return '  ---  ' if x < 0 else f'{x * 100:6.1f} cm'


# ---------------------------------------------------------------------------------------------------------------------
# Pruebas
# ---------------------------------------------------------------------------------------------------------------------
class Prueba:
    ayuda = ''
    repetir_s = 0.0          # >0: reenviar orden() cada tanto (motores con perro guardian)

    def __init__(self, ident, nombre):
        self.id = ident
        self.nombre = nombre
        self.enviar = None   # lo pone el nodo: enviar(lista)
        self.e = None        # ultimo estado (lista)

    def mandar(self, *valores):
        self.enviar([float(self.id)] + [float(v) for v in valores])

    def conectado(self):
        """Primer estado despues de conectar (o de un reinicio de la ESP32)."""

    def tecla(self, k):
        return False

    def orden(self):
        return None

    def texto(self, e):
        return str(e)

    def salir(self):
        """Dejar todo quieto antes de salir."""


class Enlace(Prueba):
    ayuda = 'l prender/apagar el LED    r estadisticas en cero'

    def __init__(self, *a):
        super().__init__(*a)
        self.n = 0
        self.envios = {}
        self.rtt = []
        self.led = False
        self.t_eco = 0.0

    def tecla(self, k):
        if k == 'l':
            self.led = not self.led
            self.mandar(2, 1 if self.led else 0)
        elif k == 'r':
            self.rtt.clear()
        else:
            return False
        return True

    def periodico(self):
        if time.time() - self.t_eco >= 0.2:
            self.t_eco = time.time()
            self.n = (self.n + 1) % 100000
            self.envios[self.n] = time.time()
            self.mandar(1, self.n)
        if self.e is not None:
            n = int(round(self.e[2]))
            if n in self.envios:
                self.rtt.append((time.time() - self.envios.pop(n)) * 1000)
                self.rtt = self.rtt[-300:]
        for n in [n for n, t in self.envios.items() if time.time() - t > 5]:
            del self.envios[n]

    def texto(self, e):
        r = self.rtt
        s = f'RSSI {e[4]:.0f} dBm | encendida {e[5]:.0f} s | memoria {e[6]:.0f} kB | orden hace {e[3]:.0f} ms'
        if r:
            lentos = 100 * sum(1 for x in r if x > 300) / len(r)
            s += (f' | ida y vuelta: min {min(r):.0f} / med {statistics.median(r):.0f} / max {max(r):.0f} ms, '
                  f'>300 ms {lentos:.0f} % ({len(r)})')
        return s


class SonarUno(Prueba):
    ayuda = 'r cuentas en cero'

    def tecla(self, k):
        if k == 'r':
            self.mandar(1)
            return True
        return False

    def texto(self, e):
        tot = e[5] + e[6]
        perd = 100 * e[6] / tot if tot else 0
        return (f'distancia {dist(e[2])} | mediana {dist(e[4])} | eco {e[3]:6.0f} us | '
                f'validas {e[5]:.0f}  perdidas {e[6]:.0f} ({perd:.0f} %)')


class Sonares(Prueba):
    ayuda = 'm modo secuencial / simultaneo    r cuentas en cero'

    def tecla(self, k):
        if k == 'm':
            modo = 0 if (self.e and self.e[2] > 0.5) else 1
            self.mandar(2, modo)
            msg(f'  modo {"SIMULTANEO (los 3 juntos)" if modo else "SECUENCIAL (de a uno)"}')
        elif k == 'r':
            self.mandar(1)
        else:
            return False
        return True

    def texto(self, e):
        partes = []
        for i, nom in enumerate(('frente', 'der', 'izq')):
            d, med, v, p = e[3 + 4 * i:7 + 4 * i]
            perd = 100 * p / (v + p) if (v + p) else 0
            partes.append(f'{nom} {dist(d)} (med {dist(med)}, perd {perd:.0f} %)')
        return f'[{"simult." if e[2] > 0.5 else "secuen."}] ' + ' | '.join(partes)


class Encoder(Prueba):
    ayuda = 'girar la rueda A MANO.   r cuentas en cero'

    def __init__(self, ident, nombre, rueda):
        super().__init__(ident, nombre)
        self.rueda = rueda

    def tecla(self, k):
        if k == 'r':
            self.mandar(1)
            return True
        return False

    def texto(self, e):
        otros = '  '.join(f'{RUEDAS[i]} {e[5 + i]:.0f}' for i in range(4))
        aviso = ''
        if any(e[5 + i] > 2 for i in range(4) if i != self.rueda):
            aviso = '  <<< CUENTA OTRO ENCODER: cables cruzados?'
        return (f'{RUEDAS[self.rueda]}: {e[2]:.0f} flancos = {e[4]:.2f} vueltas, {e[3]:.1f} flancos/s | '
                f'todos: {otros}{aviso}')


class Motor(Prueba):
    ayuda = 'w / s duty +0,05 / -0,05   1..9 duty 0,1..0,9   0 o Espacio parar   r cuentas en cero'
    repetir_s = 0.1

    def __init__(self, ident, nombre, rueda):
        super().__init__(ident, nombre)
        self.rueda = rueda
        self.duty = 0.0

    def tecla(self, k):
        if k == 'w':
            self.duty = min(1.0, round(self.duty + 0.05, 2))
        elif k == 's':
            self.duty = max(-1.0, round(self.duty - 0.05, 2))
        elif k in '123456789':
            self.duty = int(k) / 10
        elif k in '0 x':
            self.duty = 0.0
        elif k == 'r':
            self.mandar(2)
            return True
        else:
            return False
        return True

    def orden(self):
        return [1, self.duty]

    def salir(self):
        self.duty = 0.0
        self.mandar(3)

    def texto(self, e):
        fps = e[5]
        otros = [i for i in range(4) if i != self.rueda and e[10 + i] > 2]
        aviso = f'  <<< se mueve el encoder de {", ".join(RUEDAS[i] for i in otros)}' if otros else ''
        return (f'{RUEDAS[self.rueda]}: duty pedido {e[2]:+.2f} sale {e[3]:+.2f} | {fps:5.1f} flancos/s = '
                f'{fps / 40:4.2f} vueltas/s = {fps * MM_POR_FLANCO / 1000:4.2f} m/s (sin carga) | {e[4]:.0f} flancos{aviso}')


class Ruedas4(Prueba):
    ayuda = ('1 2 3 4 elegir FL FR RL RR   t las 4   w / s duty +/-0,05 de lo elegido   0 o Espacio parar todo\n'
             '  v verificacion automatica de cableado (una rueda por vez)   r cuentas en cero')
    repetir_s = 0.1

    def __init__(self, *a):
        super().__init__(*a)
        self.duty = [0.0] * 4
        self.sel = [0, 1, 2, 3]
        self.mostro_matriz = True

    def tecla(self, k):
        if k in '1234':
            self.sel = [int(k) - 1]
            msg(f'  elegida {RUEDAS[self.sel[0]]}')
        elif k == 't':
            self.sel = [0, 1, 2, 3]
            msg('  elegidas las 4')
        elif k in 'ws':
            for i in self.sel:
                self.duty[i] = max(-1.0, min(1.0, round(self.duty[i] + (0.05 if k == 'w' else -0.05), 2)))
        elif k in '0 x':
            self.duty = [0.0] * 4
            self.mandar(4)
        elif k == 'r':
            self.mandar(3)
        elif k == 'v':
            t = pedir('Duty para la verificacion (Enter = 0.6). EL ROBOT TIENE QUE ESTAR EN EL AIRE: ').replace(',', '.')
            try:
                d = float(t) if t else 0.6
            except ValueError:
                d = 0.6
            self.duty = [0.0] * 4
            self.mandar(5, d)
            self.mostro_matriz = False
            msg(f'  verificacion con duty {d:.2f}: FL, FR, RL, RR, 2,5 s cada una')
        else:
            return False
        return True

    def orden(self):
        if self.e is not None and 0 <= self.e[14] < 4:
            return None                      # verificacion en marcha: no mandar duty
        return [1] + self.duty

    def salir(self):
        self.duty = [0.0] * 4
        self.mandar(4)

    def periodico(self):
        e = self.e
        if e is None or self.mostro_matriz or e[14] != 4:
            return
        self.mostro_matriz = True
        m = [e[15 + 4 * i:19 + 4 * i] for i in range(4)]
        msg('  Verificacion (fila = rueda movida, columna = encoder que conto):')
        msg('            ' + ''.join(f'{r:>8}' for r in RUEDAS))
        for i in range(4):
            msg(f'  movi {RUEDAS[i]}: ' + ''.join(f'{x:8.0f}' for x in m[i]))
        for i in range(4):
            fila = m[i]
            mayor = max(range(4), key=lambda j: fila[j])
            if fila[mayor] < 5:
                msg(f'  {RUEDAS[i]}: ningun encoder conto -> el motor no giro (mirar si giro) o su encoder no anda')
            elif mayor != i:
                msg(f'  {RUEDAS[i]}: conto el encoder de {RUEDAS[mayor]} -> encoders cruzados')
            else:
                msg(f'  {RUEDAS[i]}: bien ({fila[i]:.0f} flancos en 2 s)')
        msg('  Ademas, MIRAR que las 4 hayan girado hacia ADELANTE (si no: polaridad en comun/ruedas.h)')

    def texto(self, e):
        partes = [f'{RUEDAS[i]} {e[2 + i]:+.2f} {e[10 + i]:5.1f}f/s' for i in range(4)]
        paso = int(e[14])
        estado = f' | verificando {RUEDAS[paso]}' if 0 <= paso < 4 else ''
        sel = '/'.join(RUEDAS[i] for i in self.sel)
        return f'[{sel}] ' + ' | '.join(partes) + estado


class Servo(Prueba):
    ayuda = ('g activar (empieza a mandar pulsos)   a / d -10 / +10 us   A / D -50 / +50 us   i ir a un valor\n'
             '  c centro 1500   v velocidad   x limites   s soltar (sin pulsos)   m marcar (anota el pulso actual)')

    def __init__(self, ident, nombre, reposo):
        super().__init__(ident, nombre)
        self.reposo = reposo

    def tecla(self, k):
        e = self.e
        if k == 'g':
            if confirmar(f'El servo va a saltar a {self.reposo} us (reposo). Esta en esa posicion o suelto?'):
                self.mandar(1, 0)
        elif k in 'adAD':
            self.mandar(3, {'a': -10, 'd': 10, 'A': -50, 'D': 50}[k])
        elif k == 'i':
            v = pedir_numeros('Pulso en us: ', 1)
            if v:
                self.mandar(2, v[0])
        elif k == 'c':
            self.mandar(2, 1500)
        elif k == 'v':
            v = pedir_numeros('Velocidad en us/s (200 = unos 20 grados/s): ', 1)
            if v:
                self.mandar(5, v[0])
        elif k == 'x':
            v = pedir_numeros('Limites en us, minimo y maximo (dentro de 500..2500): ', 2)
            if v:
                self.mandar(6, v[0], v[1])
        elif k == 's':
            self.mandar(4)
        elif k == 'm' and e is not None:
            nota = pedir('Nota para esta marca (que se ve, angulo...): ')
            os.makedirs(CARPETA, exist_ok=True)
            ruta = os.path.join(CARPETA, 'marcas_servos.csv')
            nuevo = not os.path.exists(ruta)
            with open(ruta, 'a', newline='') as f:
                w = csv.writer(f)
                if nuevo:
                    w.writerow(['fecha', 'prueba', 'pulso_us', 'nota'])
                w.writerow([datetime.datetime.now().isoformat(timespec='seconds'), self.nombre, round(e[3]), nota])
            msg(f'  anotado en {ruta}: {self.nombre} {e[3]:.0f} us "{nota}"')
        else:
            return False
        return True

    def texto(self, e):
        estado = 'ACTIVO ' if e[2] > 0.5 else 'sin pulsos'
        extra = '  barriendo' if len(e) > 8 and e[8] > 0.5 else ''
        return (f'{self.nombre}: {estado} pulso {e[3]:5.0f} us -> objetivo {e[4]:5.0f} us | limites {e[5]:.0f}..{e[6]:.0f} '
                f'| {e[7]:.0f} us/s{extra}')


class Pan(Servo):
    ayuda = Servo.ayuda + '\n  w barrido continuo entre los limites (prender / apagar)'

    def tecla(self, k):
        if k == 'w':
            barriendo = self.e is not None and len(self.e) > 8 and self.e[8] > 0.5
            self.mandar(7, 0 if barriendo else 1)
            return True
        return super().tecla(k)


SERVOS = ['base', 'hombro', 'codo', 'pinza']
POSES = ['reposo', 'arriba', 'lateral', 'abierta', 'extendida', 'cerrada', 'retraida', 'levantada', 'atras',
         'reposo con cubo']
POSES_DEFECTO = [[1500, 1700, 1700, 1500], [1500, 1300, 1700, 1500], [1000, 1300, 1700, 1500],
                 [1000, 1300, 1700, 1900], [1000, 1450, 1300, 1900], [1000, 1450, 1300, 1100],
                 [1000, 1450, 1700, 1100], [1000, 1300, 1700, 1100], [1500, 1300, 1700, 1100],
                 [1500, 1700, 1700, 1100]]
RUTA_POSES = os.path.join(CARPETA, 'brazo_poses.json')


class Brazo(Prueba):
    ayuda = ('b h c p elegir base / hombro / codo / pinza   a / d -10 / +10 us   A / D -50 / +50 us\n'
             '  g activar los 4 (en reposo)   s soltar los 4   0..9 ir a la pose   r secuencia completa   n siguiente paso\n'
             '  Espacio detener   k guardar la posicion actual como pose   e editar una pose   l listar poses\n'
             '  t duracion de cada paso   P pausa entre pasos')

    def __init__(self, *a):
        super().__init__(*a)
        self.sel = 0
        self.poses = [list(p) for p in POSES_DEFECTO]
        try:
            with open(RUTA_POSES) as f:
                guardadas = json.load(f)
            for k, p in enumerate(guardadas[:len(self.poses)]):
                if len(p) == 4:
                    self.poses[k] = [int(x) for x in p]
            msg(f'  poses cargadas de {RUTA_POSES}')
        except (OSError, ValueError):
            pass

    def guardar(self):
        os.makedirs(CARPETA, exist_ok=True)
        with open(RUTA_POSES, 'w') as f:
            json.dump(self.poses, f, indent=1)

    def conectado(self):
        for k, p in enumerate(self.poses):
            self.mandar(18, k, *p)
            time.sleep(0.05)                 # de a una: best effort puede perder mensajes en rafaga
        msg('  poses enviadas a la ESP32 (l para listarlas)')

    def listar(self):
        msg('  pose               ' + ''.join(f'{s:>8}' for s in SERVOS))
        for k, p in enumerate(self.poses):
            msg(f'  {k} {POSES[k]:<16}' + ''.join(f'{x:8d}' for x in p))

    def tecla(self, k):
        if k in 'bhcp':
            self.sel = 'bhcp'.index(k)
            msg(f'  elegido: {SERVOS[self.sel]}')
        elif k in 'adAD':
            self.mandar(3, self.sel, {'a': -10, 'd': 10, 'A': -50, 'D': 50}[k])
        elif k == 'g':
            if confirmar('Los 4 servos van a saltar a la pose de reposo. El brazo esta en el reposo?'):
                self.mandar(1)
        elif k == 's':
            if confirmar('Soltar los 4 servos (el brazo cae por su peso)?'):
                self.mandar(4)
        elif k in '0123456789':
            self.mandar(10, int(k))
            msg(f'  a la pose {k} ({POSES[int(k)]})')
        elif k == 'r':
            if confirmar('Secuencia completa 1..9 (agarre lateral y vuelta al reposo)?'):
                self.mandar(11)
        elif k == 'n':
            self.mandar(12)
        elif k in ' x':
            self.mandar(13)
        elif k == 'k' and self.e is not None:
            v = pedir_numeros('Guardar la posicion actual como la pose numero (0..9): ', 1)
            if v and 0 <= int(v[0]) <= 9:
                n = int(v[0])
                self.poses[n] = [int(round(self.e[3 + 3 * j])) for j in range(4)]
                self.mandar(16, n)
                self.guardar()
                msg(f'  pose {n} ({POSES[n]}) = {self.poses[n]}, guardada en {RUTA_POSES}')
        elif k == 'e':
            v = pedir_numeros('Pose (0..9), servo (0 base 1 hombro 2 codo 3 pinza) y us: ', 3)
            if v and 0 <= int(v[0]) <= 9 and 0 <= int(v[1]) <= 3:
                n, j = int(v[0]), int(v[1])
                self.poses[n][j] = int(v[2])
                self.mandar(14, n, j, v[2])
                self.guardar()
                msg(f'  pose {n} ({POSES[n]}) = {self.poses[n]}')
        elif k == 'l':
            self.listar()
        elif k == 't':
            v = pedir_numeros('Duracion de cada paso en ms (300..15000): ', 1)
            if v:
                self.mandar(15, v[0])
        elif k == 'P':
            v = pedir_numeros('Pausa entre pasos en ms (0..10000): ', 1)
            if v:
                self.mandar(17, v[0])
        else:
            return False
        return True

    def salir(self):
        self.mandar(13)

    def texto(self, e):
        partes = []
        for j in range(4):
            act, pul = e[2 + 3 * j], e[3 + 3 * j]
            marca = '>' if j == self.sel else ' '
            partes.append(f'{marca}{SERVOS[j]} {pul:5.0f}' + ('' if act > 0.5 else '(suelto)'))
        paso, corr, mov, dest = int(e[14]), e[15] > 0.5, e[16] > 0.5, int(e[17])
        estado = ''
        if mov and 0 <= dest < len(POSES):
            estado = f' | moviendo a {dest} ({POSES[dest]})'
        if corr or paso >= 0:
            estado += f' | paso {paso + 1}/9' + (' (secuencia)' if corr else '')
        return ' '.join(partes) + estado


def pruebas():
    t = {0: ('enlace', lambda i, n: Enlace(i, n), 'prueba'),
         10: ('sonar_frontal', lambda i, n: SonarUno(i, n), 'prueba'),
         11: ('sonar_derecho', lambda i, n: SonarUno(i, n), 'prueba'),
         12: ('sonar_izquierdo', lambda i, n: SonarUno(i, n), 'prueba'),
         13: ('sonares_3', lambda i, n: Sonares(i, n), 'prueba'),
         34: ('ruedas_4', lambda i, n: Ruedas4(i, n), 'prueba'),
         40: ('servo_base', lambda i, n: Servo(i, n, 1500), 'prueba'),
         41: ('servo_hombro', lambda i, n: Servo(i, n, 1700), 'prueba'),
         42: ('servo_codo', lambda i, n: Servo(i, n, 1700), 'prueba'),
         43: ('servo_pinza', lambda i, n: Servo(i, n, 1500), 'prueba'),
         44: ('brazo_coordinado', lambda i, n: Brazo(i, n), 'prueba'),
         50: ('servo_pan_cam', lambda i, n: Pan(i, n, 1500), 'prueba_cam')}
    for r, nom in enumerate(RUEDAS):
        t[20 + r] = (f'encoder_{nom}', (lambda rr: lambda i, n: Encoder(i, n, rr))(r), 'prueba')
        t[30 + r] = (f'motor_{nom}', (lambda rr: lambda i, n: Motor(i, n, rr))(r), 'prueba')
    return dict(sorted(t.items()))


def elegir(arg):
    t = pruebas()
    for ident, (nombre, fabrica, topico) in t.items():
        if arg in (str(ident), f'{ident:02d}', nombre, f'{ident:02d}_{nombre}'):
            return fabrica(ident, nombre), topico
    return None, None


class Nodo(Node):
    def __init__(self, prueba, topico):
        super().__init__('pc_pruebas')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Float32MultiArray, f'/{topico}/cmd', qos)
        self.create_subscription(Float32MultiArray, f'/{topico}/estado', self.cb, qos)
        self.prueba = prueba
        prueba.enviar = self.enviar
        self.t_estado = 0.0
        self.contador = -1
        self.nuevo = False
        self.otro = None

    def enviar(self, valores):
        self.pub.publish(Float32MultiArray(data=[float(v) for v in valores]))

    def cb(self, m):
        e = list(m.data)
        if len(e) < 2:
            return
        if int(round(e[0])) != self.prueba.id:
            self.otro = int(round(e[0]))
            self.t_estado = time.time()
            self.prueba.e = None
            return
        self.otro = None
        if e[1] < self.contador or self.contador < 0:
            self.nuevo = True                 # primera vez o la ESP32 se reinicio
        self.contador = e[1]
        self.t_estado = time.time()
        self.prueba.e = e


def main():
    global TERM
    if len(sys.argv) < 2:
        print(__doc__)
        for ident, (nombre, _, topico) in pruebas().items():
            print(f'  {ident:2d}  {nombre:<18} (/{topico})')
        return
    prueba, topico = elegir(sys.argv[1])
    if prueba is None:
        print(f'No conozco la prueba "{sys.argv[1]}". Sin argumentos se listan.')
        return
    rclpy.init()
    nodo = Nodo(prueba, topico)
    TERM = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(f'Prueba {prueba.id}: {prueba.nombre}.  Teclas: {prueba.ayuda}\n  ? ayuda   q salir')
    t_linea = t_rep = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(nodo, timeout_sec=0.02)
            ahora = time.time()
            if nodo.nuevo:
                nodo.nuevo = False
                msg('  ESP32 conectada')
                prueba.conectado()
            k = tecla()
            if k == 'q':
                break
            if k == '?':
                msg(f'  Teclas: {prueba.ayuda}\n  ? ayuda   q salir')
            elif k is not None:
                prueba.tecla(k)
            if prueba.repetir_s and ahora - t_rep >= prueba.repetir_s:
                t_rep = ahora
                o = prueba.orden()
                if o is not None:
                    prueba.mandar(*o)
            if hasattr(prueba, 'periodico'):
                prueba.periodico()
            if ahora - t_linea >= 0.2:
                t_linea = ahora
                if nodo.otro is not None:
                    linea(f'La ESP32 tiene grabada la prueba {nodo.otro}, no la {prueba.id}. Grabar la correcta.')
                elif ahora - nodo.t_estado > 2.0:
                    linea('Esperando a la ESP32... (agente corriendo? ESP32 encendida? LED parpadeando = no encuentra al agente)')
                elif prueba.e is not None:
                    linea(prueba.texto(prueba.e))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            prueba.salir()
            for _ in range(5):
                rclpy.spin_once(nodo, timeout_sec=0.02)
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, TERM)
        print()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
