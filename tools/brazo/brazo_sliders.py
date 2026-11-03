#!/usr/bin/env python3
"""Ventana con sliders para mover los 4 servos del brazo, por Wi-Fi (ESP32 con micro-ROS).

Requisitos: ROS 2 Jazzy cargado (source /opt/ros/jazzy/setup.bash), python3-tk, el agente micro-ROS corriendo y la ESP32
con firmware/brazo_sliders grabado.

Habla con la ESP32 por dos topicos (el significado de cada numero esta en firmware/brazo_sliders/main.c):
  /brazo/cmd     esta PC -> ESP32   [base, hombro, codo, pinza (grados), mascara de activos, velocidad en grados/s]
  /brazo/estado  ESP32 -> esta PC   [4 angulos actuales, mascara, velocidad, contador, ms sin orden, 8 limites]
La orden se manda 10 veces por segundo, siempre completa: la ESP32 va hacia lo ultimo que recibio. Asi, si se pierde un
mensaje por el Wi-Fi, el siguiente corrige.
"""
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

SERVOS = ['base', 'hombro', 'codo', 'pinza']          # el orden es el de la ESP32 (indices 0 a 3)
MOSTRAR = [3, 2, 1, 0]                                 # de arriba hacia abajo en la ventana, como en la imagen de referencia
PERIODO_ORDEN_MS = 100
VELOCIDAD_DEFECTO = 90.0
VELOCIDAD_MIN, VELOCIDAD_MAX = 10.0, 360.0

FONDO, TARJETA, TEXTO, APAGADO, ROJO = '#0d0d10', '#15151a', '#f2f2f2', '#8a8a93', '#ff3b30'


# ---------------------------------------------------------------------------------------------------------------------
# Lo que se manda y lo que se recibe (sin ventana: se puede probar solo)
# ---------------------------------------------------------------------------------------------------------------------
def armar_orden(angulos, activos, velocidad, salto):
    """Lista de 6 numeros para /brazo/cmd. Velocidad 0 = salto directo."""
    mascara = sum(1 << i for i, a in enumerate(activos) if a)
    return [float(a) for a in angulos] + [float(mascara), 0.0 if salto else float(velocidad)]


def leer_estado(datos):
    """Convierte la lista que publica la ESP32 en un diccionario. None si esta incompleta."""
    if len(datos) < 16:
        return None
    mascara = int(round(datos[4]))
    return {
        'actuales': list(datos[0:4]),
        'activos': [bool((mascara >> i) & 1) for i in range(4)],
        'velocidad': datos[5],
        'contador': datos[6],
        'ms_sin_orden': datos[7],
        'limites': [(datos[8 + 2 * i], datos[9 + 2 * i]) for i in range(4)],
    }


# ---------------------------------------------------------------------------------------------------------------------
# ROS 2
# ---------------------------------------------------------------------------------------------------------------------
class Enlace(Node):
    def __init__(self):
        super().__init__('pc_brazo_sliders')
        # best effort: igual que la ESP32. Una suscripcion "reliable" no recibe de un publicador "best effort"
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Float32MultiArray, '/brazo/cmd', qos)
        self.create_subscription(Float32MultiArray, '/brazo/estado', self.al_llegar_estado, qos)
        self.estado = None
        self.estado_nuevo = False
        self.t_estado = 0.0

    def al_llegar_estado(self, msg):
        e = leer_estado(list(msg.data))
        if e is not None:
            self.estado = e
            self.estado_nuevo = True
            self.t_estado = self.get_clock().now().nanoseconds / 1e9

    def enviar(self, orden):
        self.pub.publish(Float32MultiArray(data=orden))

    def segundos_sin_estado(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.t_estado if self.estado else None


# ---------------------------------------------------------------------------------------------------------------------
# Ventana
# ---------------------------------------------------------------------------------------------------------------------
class Ventana:
    def __init__(self, raiz, enlace):
        self.raiz = raiz
        self.enlace = enlace
        self.sincronizado = False          # no se manda nada hasta saber como esta la ESP32 (evita soltar el brazo al abrir)
        self.ultimo_contador = None
        self.angulo = [tk.DoubleVar(value=90.0) for _ in SERVOS]
        self.activo = [tk.BooleanVar(value=False) for _ in SERVOS]
        self.velocidad = tk.DoubleVar(value=VELOCIDAD_DEFECTO)
        self.salto = tk.BooleanVar(value=False)
        self.etiqueta_valor, self.etiqueta_real, self.deslizador, self.fila = [], [], [], []
        self.seleccionado = None
        raiz.title('Brazo: servos')
        raiz.configure(bg=FONDO)
        self._armar()
        self._tarea_ros()
        self._tarea_orden()

    def _armar(self):
        tarjeta = tk.Frame(self.raiz, bg=TARJETA, padx=24, pady=18)
        tarjeta.pack(padx=24, pady=24)
        cab = tk.Frame(tarjeta, bg=TARJETA)
        cab.pack(fill='x', pady=(0, 10))
        tk.Label(cab, text='■', fg=ROJO, bg=TARJETA).pack(side='left')
        tk.Label(cab, text=' MODO PRO', fg=TEXTO, bg=TARJETA, font=('DejaVu Sans Mono', 10, 'bold')).pack(side='left')
        tk.Label(cab, text='SERVOS · GRADOS', fg=APAGADO, bg=TARJETA, font=('DejaVu Sans Mono', 10)).pack(side='right')

        self.etiqueta_valor = [None] * 4
        self.etiqueta_real = [None] * 4
        self.deslizador = [None] * 4
        self.fila = [None] * 4
        for i in MOSTRAR:
            fila = tk.Frame(tarjeta, bg=TARJETA, padx=8, pady=4)
            fila.pack(fill='x')
            nombre = tk.Label(fila, text=SERVOS[i].capitalize(), fg=APAGADO, bg=TARJETA, width=9, anchor='w',
                              font=('DejaVu Sans', 12))
            nombre.pack(side='left')
            desliz = tk.Scale(fila, from_=0, to=180, orient='horizontal', length=420, showvalue=False,
                              variable=self.angulo[i], resolution=1, bg=TARJETA, fg=TEXTO, troughcolor='#2b2b33',
                              highlightthickness=0, bd=0, sliderlength=22, activebackground=ROJO,
                              command=lambda _v, k=i: self._movio(k))
            desliz.pack(side='left', padx=10)
            valor = tk.Label(fila, text='090°', fg=TEXTO, bg=TARJETA, width=5, font=('DejaVu Sans Mono', 15, 'bold'))
            valor.pack(side='left')
            real = tk.Label(fila, text='', fg=APAGADO, bg=TARJETA, width=11, font=('DejaVu Sans Mono', 9))
            real.pack(side='left')
            tk.Checkbutton(fila, text='activo', variable=self.activo[i], fg=TEXTO, bg=TARJETA, selectcolor='#2b2b33',
                           activebackground=TARJETA, activeforeground=TEXTO,
                           command=lambda k=i: self._movio(k)).pack(side='left')
            self.etiqueta_valor[i], self.etiqueta_real[i], self.deslizador[i], self.fila[i] = valor, real, desliz, fila
            self.fila[i].nombre = nombre

        pie = tk.Frame(tarjeta, bg=TARJETA, pady=10)
        pie.pack(fill='x')
        tk.Label(pie, text='Velocidad', fg=APAGADO, bg=TARJETA).pack(side='left')
        tk.Scale(pie, from_=VELOCIDAD_MIN, to=VELOCIDAD_MAX, orient='horizontal', length=200, variable=self.velocidad,
                 showvalue=True, bg=TARJETA, fg=TEXTO, troughcolor='#2b2b33', highlightthickness=0, bd=0).pack(side='left')
        tk.Label(pie, text='grados/s', fg=APAGADO, bg=TARJETA).pack(side='left')
        tk.Checkbutton(pie, text='salto directo (mas fuerza)', variable=self.salto, fg=TEXTO, bg=TARJETA,
                       selectcolor='#2b2b33', activebackground=TARJETA, activeforeground=TEXTO).pack(side='left', padx=14)

        botones = tk.Frame(tarjeta, bg=TARJETA)
        botones.pack(fill='x', pady=(4, 8))
        for texto, accion in (('ACTIVAR TODOS', lambda: self._poner_activos(True)),
                              ('SOLTAR TODOS', lambda: self._poner_activos(False)),
                              ('CENTRAR 90°', self._centrar)):
            tk.Button(botones, text=texto, command=accion, bg='#2b2b33', fg=TEXTO, activebackground=ROJO,
                      activeforeground=TEXTO, bd=0, padx=12, pady=6).pack(side='left', padx=(0, 8))

        self.estado = tk.Label(tarjeta, text='Esperando a la ESP32...', fg=APAGADO, bg=TARJETA, anchor='w',
                               font=('DejaVu Sans Mono', 9))
        self.estado.pack(fill='x')

    # --- acciones de la ventana
    def _movio(self, k):
        """Se movio el slider (o el 'activo') del servo k: queda resaltado en rojo y se actualiza el numero."""
        self.etiqueta_valor[k].config(text=f'{int(self.angulo[k].get()):03d}°')
        if self.seleccionado is not None:
            self._pintar(self.seleccionado, False)
        self.seleccionado = k
        self._pintar(k, True)

    def _pintar(self, k, resaltado):
        color = ROJO if resaltado else TEXTO
        self.etiqueta_valor[k].config(fg=color)
        self.fila[k].nombre.config(fg=color if resaltado else APAGADO)

    def _poner_activos(self, valor):
        for v in self.activo:
            v.set(valor)

    def _centrar(self):
        for i in range(4):
            self.angulo[i].set(90.0)
            self.etiqueta_valor[i].config(text='090°')

    # --- tareas periodicas
    def _tarea_ros(self):
        rclpy.spin_once(self.enlace, timeout_sec=0)           # atiende los mensajes que llegaron, sin esperar
        e = self.enlace.estado
        if e is not None and self.enlace.estado_nuevo:
            self.enlace.estado_nuevo = False
            reinicio = self.ultimo_contador is not None and e['contador'] < self.ultimo_contador
            self.ultimo_contador = e['contador']
            if not self.sincronizado or reinicio:
                self._sincronizar(e)
            for i in range(4):
                self.etiqueta_real[i].config(text=f'real {e["actuales"][i]:5.1f}°' if e['activos'][i] else 'suelto')
        self._mostrar_estado()
        self.raiz.after(20, self._tarea_ros)

    def _sincronizar(self, e):
        """Primera vez (o la ESP32 se reinicio): los sliders toman lo que tiene la ESP32 (sin saltos ni soltar nada)."""
        for i in range(4):
            lo, hi = e['limites'][i]
            self.deslizador[i].config(from_=lo, to=hi)
            self.angulo[i].set(e['actuales'][i])
            self.activo[i].set(e['activos'][i])
            self.etiqueta_valor[i].config(text=f'{int(e["actuales"][i]):03d}°')
        self.velocidad.set(max(VELOCIDAD_MIN, min(VELOCIDAD_MAX, e['velocidad'] or VELOCIDAD_DEFECTO)))
        self.sincronizado = True

    def _tarea_orden(self):
        if self.sincronizado:
            orden = armar_orden([v.get() for v in self.angulo], [v.get() for v in self.activo], self.velocidad.get(),
                                self.salto.get())
            self.enlace.enviar(orden)
        self.raiz.after(PERIODO_ORDEN_MS, self._tarea_orden)

    def _mostrar_estado(self):
        seg = self.enlace.segundos_sin_estado()
        if seg is None:
            self.estado.config(text='Esperando a la ESP32... (agente corriendo? LED apagado = conectada)', fg=APAGADO)
        elif seg > 1.5:
            self.estado.config(text=f'Sin datos de la ESP32 hace {seg:.0f} s. Sigue en la ultima posicion.', fg=ROJO)
        else:
            e = self.enlace.estado
            self.estado.config(text=f'ESP32 conectada | ultima orden recibida hace {e["ms_sin_orden"]:.0f} ms', fg=APAGADO)


def main():
    rclpy.init()
    enlace = Enlace()
    raiz = tk.Tk()
    Ventana(raiz, enlace)
    try:
        raiz.mainloop()
    finally:
        enlace.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()