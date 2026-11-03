#!/usr/bin/env python3
"""Ventana con sliders para mover los 4 servos del brazo, por Wi-Fi (ESP32 con micro-ROS).
Version 3: limites y reposo de cada servo ajustables, tope propio de cada servo y reposo seguro antes de girar la base.

Requisitos: ROS 2 Jazzy cargado (source /opt/ros/jazzy/setup.bash), python3-tk, el agente micro-ROS corriendo y la ESP32
con firmware/brazo_sliders (v3) grabado.

Habla con la ESP32 por dos topicos (el significado de cada numero esta en firmware/brazo_sliders/main.c):
  /brazo/cmd     esta PC -> ESP32   [base, hombro, codo, pinza (grados), mascara de activos, velocidad en grados/s,
                                     los limites minimo y maximo de cada servo (8 numeros) y el reposo seguro del hombro y del codo]
  /brazo/estado  ESP32 -> esta PC   [4 angulos actuales, mascara, velocidad, contador, ms sin orden, 8 limites,
                                     4 topes, fase de la base, reposo seguro en uso]
La orden se manda 10 veces por segundo, siempre completa: la ESP32 va hacia lo ultimo que recibio. Asi, si se pierde un
mensaje por el Wi-Fi, el siguiente corrige.

Los limites y el reposo de cada servo se editan en la ventana y se guardan con GUARDAR LIMITES en
~/caid/config/brazo_calibracion.json (fuera del repositorio). Se cargan solos la proxima vez que abras la ventana.
El reposo del hombro y del codo es ademas el REPOSO SEGURO: la base solo gira con el brazo ahi (lo impone la ESP32).
Los grados son NOMINALES: 500 us = 0 grados y 2500 us = 180 grados (11,1 us por grado). No son grados reales de la articulacion.
"""
import json
import os

try:
    import tkinter as tk
except ImportError:                                    # PC sin python3-tk: las funciones de calculo se pueden usar igual
    tk = None

SERVOS = ['base', 'hombro', 'codo', 'pinza']          # el orden es el de la ESP32 (indices 0 a 3)
MOSTRAR = [3, 2, 1, 0]                                 # de arriba hacia abajo en la ventana, como en la imagen de referencia
PERIODO_ORDEN_MS = 100
VELOCIDAD_DEFECTO = 90.0
VELOCIDAD_MIN, VELOCIDAD_MAX = 10.0, 360.0
GRADOS_MIN, GRADOS_MAX = 0.0, 200.0                    # el tope mas alto de cualquier servo (el codo: 200 grados = 2722 us); el de cada uno lo informa la ESP32
TOPE_DEFECTO = 180.0                                   # tope de un servo si la ESP32 no lo informa (firmware viejo)
ANCHO_MIN = 1.0                                        # un rango de limites tiene que medir al menos 1 grado
PULSO_0_US, PULSO_180_US = 500.0, 2500.0
# Reposo de cada servo (boton REPOSO). Hombro y codo son el reposo seguro para girar la base. Base: alineada con el robot (95, M a ojo).
REPOSO_DEFECTO = {'base': 95.0, 'hombro': 60.0, 'codo': 180.0, 'pinza': 114.0}
CARPETA = os.path.expanduser('~/caid/config')
RUTA_CAL = os.path.join(CARPETA, 'brazo_calibracion.json')

FONDO, TARJETA, TEXTO, APAGADO, ROJO, AMBAR = '#0d0d10', '#15151a', '#f2f2f2', '#8a8a93', '#ff3b30', '#ffb020'


# ---------------------------------------------------------------------------------------------------------------------
# Lo que se calcula, se manda y se recibe (sin ventana ni ROS: se puede probar solo)
# ---------------------------------------------------------------------------------------------------------------------
def limitar(x, lo, hi):
    return max(lo, min(hi, x))


def us_de_grados(deg):
    """Ancho de pulso en microsegundos para un angulo nominal (500 us = 0 grados, 2500 us = 180 grados)."""
    return PULSO_0_US + (PULSO_180_US - PULSO_0_US) * deg / 180.0


def rango_valido(lo, hi):
    return GRADOS_MIN <= lo and hi <= GRADOS_MAX and hi - lo >= ANCHO_MIN


def armar_orden(angulos, activos, velocidad, salto, limites=None, pose_segura=None):
    """Lista de numeros para /brazo/cmd: 6 (angulos, mascara, velocidad), 14 si se pasan los limites y 16 si ademas se pasa el
    reposo seguro (hombro, codo). Velocidad 0 = salto directo."""
    mascara = sum(1 << i for i, a in enumerate(activos) if a)
    orden = [float(a) for a in angulos] + [float(mascara), 0.0 if salto else float(velocidad)]
    if limites is not None:
        for lo, hi in limites:
            orden += [float(lo), float(hi)]
        if pose_segura is not None:
            orden += [float(pose_segura[0]), float(pose_segura[1])]
    return orden


def leer_estado(datos):
    """Convierte la lista que publica la ESP32 en un diccionario. None si esta incompleta. Un firmware viejo (16 numeros)
    no informa topes, fase ni reposo seguro: se ponen valores neutros."""
    if len(datos) < 16:
        return None
    mascara = int(round(datos[4]))
    nuevo = len(datos) >= 23
    return {
        'actuales': list(datos[0:4]),
        'activos': [bool((mascara >> i) & 1) for i in range(4)],
        'velocidad': datos[5],
        'contador': datos[6],
        'ms_sin_orden': datos[7],
        'limites': [(datos[8 + 2 * i], datos[9 + 2 * i]) for i in range(4)],
        'topes': list(datos[16:20]) if nuevo else [TOPE_DEFECTO] * 4,
        'fase': int(round(datos[20])) if nuevo else 0,                 # 0 libre, 1 esperando el reposo seguro, 2 girando
        'pose_segura': (datos[21], datos[22]) if nuevo else None,
    }


def cargar_calibracion(ruta=RUTA_CAL):
    """{nombre: {'min', 'max', 'reposo'}} con lo guardado. Lo que no sea valido se descarta; sin archivo, {}.
    Acepta la clave vieja 'centro' (v2) como 'reposo'."""
    try:
        with open(ruta) as f:
            crudo = json.load(f)
    except (OSError, ValueError):
        return {}
    cal = {}
    for nombre in SERVOS:
        try:
            c = crudo[nombre]
            lo, hi = float(c['min']), float(c['max'])
        except (KeyError, TypeError, ValueError):
            continue
        if not rango_valido(lo, hi):
            continue
        try:
            reposo = limitar(float(c['reposo'] if 'reposo' in c else c['centro']), lo, hi)
        except (KeyError, TypeError, ValueError):
            reposo = limitar(REPOSO_DEFECTO[nombre], lo, hi)
        cal[nombre] = {'min': lo, 'max': hi, 'reposo': reposo}
    return cal


def guardar_calibracion(cal, ruta=RUTA_CAL):
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, 'w') as f:
        json.dump(cal, f, indent=2)


def limites_de_arranque(cal, limites_esp32, topes=None):
    """Limites y reposos con los que arranca la ventana: lo guardado si hay (acotado al tope de cada servo) y si no, lo que
    informa la ESP32."""
    topes = topes or [TOPE_DEFECTO] * 4
    salida = []
    for i, nombre in enumerate(SERVOS):
        lo_e, hi_e = limites_esp32[i]
        if nombre in cal:
            c = cal[nombre]
            lo, hi = c['min'], min(c['max'], topes[i])
            if hi - lo < ANCHO_MIN:                       # lo guardado ya no entra en el tope de este servo: se usa lo de la ESP32
                lo, hi = lo_e, hi_e
            salida.append((lo, hi, limitar(c['reposo'], lo, hi)))
        else:
            salida.append((lo_e, hi_e, limitar(REPOSO_DEFECTO[nombre], lo_e, hi_e)))
    return salida


def texto_grados(x):
    return f'{x:g}'


# ---------------------------------------------------------------------------------------------------------------------
# ROS 2 (solo se importa al abrir la ventana de verdad: asi lo demas se puede probar sin ROS)
# ---------------------------------------------------------------------------------------------------------------------
def crear_enlace_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray

    rclpy.init()

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

        def atender(self):
            rclpy.spin_once(self, timeout_sec=0)          # atiende los mensajes que llegaron, sin esperar

        def enviar(self, orden):
            self.pub.publish(Float32MultiArray(data=orden))

        def segundos_sin_estado(self):
            return self.get_clock().now().nanoseconds / 1e9 - self.t_estado if self.estado else None

        def cerrar(self):
            self.destroy_node()
            rclpy.try_shutdown()

    return Enlace()


# ---------------------------------------------------------------------------------------------------------------------
# Ventana
# ---------------------------------------------------------------------------------------------------------------------
class Ventana:
    def __init__(self, raiz, enlace, ruta_cal=RUTA_CAL):
        self.raiz = raiz
        self.enlace = enlace
        self.ruta_cal = ruta_cal
        self.cal = cargar_calibracion(ruta_cal)
        self.sincronizado = False          # no se manda nada hasta saber como esta la ESP32 (evita soltar el brazo al abrir)
        self.ultimo_contador = None
        self.ignorar_movimientos = False   # mientras la ventana se pone al dia con la ESP32, los sliders no cuentan como "tocados"
        self.limites = [(GRADOS_MIN, TOPE_DEFECTO)] * 4      # los que se mandan a la ESP32
        self.topes = [TOPE_DEFECTO] * 4                      # tope absoluto de cada servo: lo informa la ESP32
        self.cajas = [[] for _ in SERVOS]                    # las casillas min / max / reposo de cada servo
        self.angulo = [tk.DoubleVar(value=90.0) for _ in SERVOS]
        self.activo = [tk.BooleanVar(value=False) for _ in SERVOS]
        self.v_min = [tk.StringVar() for _ in SERVOS]
        self.v_max = [tk.StringVar() for _ in SERVOS]
        self.v_reposo = [tk.StringVar() for _ in SERVOS]
        self.velocidad = tk.DoubleVar(value=VELOCIDAD_DEFECTO)
        self.salto = tk.BooleanVar(value=False)
        self.seleccionado = None
        raiz.title('Brazo: servos')
        raiz.configure(bg=FONDO)
        self._armar()
        self._id_ros = self._id_orden = self._id_sync = None
        self._tarea_ros()
        self._tarea_orden()

    # --- construccion
    def _estilo_entrada(self, **extra):
        return dict(bg='#2b2b33', fg=TEXTO, insertbackground=TEXTO, highlightthickness=1, highlightbackground='#3a3a44',
                    highlightcolor=ROJO, bd=0, justify='center', **extra)

    def _armar(self):
        tarjeta = tk.Frame(self.raiz, bg=TARJETA, padx=24, pady=18)
        tarjeta.pack(padx=24, pady=24)
        cab = tk.Frame(tarjeta, bg=TARJETA)
        cab.pack(fill='x', pady=(0, 10))
        tk.Label(cab, text='■', fg=ROJO, bg=TARJETA).pack(side='left')
        tk.Label(cab, text=' MODO PRO', fg=TEXTO, bg=TARJETA, font=('DejaVu Sans Mono', 10, 'bold')).pack(side='left')
        tk.Label(cab, text='SERVOS · GRADOS', fg=APAGADO, bg=TARJETA, font=('DejaVu Sans Mono', 10)).pack(side='right')

        self.etiqueta_valor = [None] * 4
        self.etiqueta_us = [None] * 4
        self.etiqueta_va = [None] * 4
        self.deslizador = [None] * 4
        self.fila = [None] * 4
        for i in MOSTRAR:
            fila = tk.Frame(tarjeta, bg=TARJETA, padx=8, pady=4)
            fila.pack(fill='x')
            nombre = tk.Label(fila, text=SERVOS[i].capitalize(), fg=APAGADO, bg=TARJETA, width=9, anchor='w',
                              font=('DejaVu Sans', 12))
            nombre.pack(side='left')
            desliz = tk.Scale(fila, from_=GRADOS_MIN, to=GRADOS_MAX, orient='horizontal', length=380, showvalue=False,
                              variable=self.angulo[i], resolution=1, bg=TARJETA, fg=TEXTO, troughcolor='#2b2b33',
                              highlightthickness=0, bd=0, sliderlength=22, activebackground=ROJO,
                              command=lambda _v, k=i: self._movio(k))
            desliz.pack(side='left', padx=10)
            valor = tk.Label(fila, text='090°', fg=TEXTO, bg=TARJETA, width=5, font=('DejaVu Sans Mono', 15, 'bold'))
            valor.pack(side='left')
            us = tk.Label(fila, text='', fg=APAGADO, bg=TARJETA, width=8, anchor='w', font=('DejaVu Sans Mono', 9))
            us.pack(side='left')
            va = tk.Label(fila, text='', fg=APAGADO, bg=TARJETA, width=11, anchor='w', font=('DejaVu Sans Mono', 9))
            va.pack(side='left')
            tk.Checkbutton(fila, text='activo', variable=self.activo[i], fg=TEXTO, bg=TARJETA, selectcolor='#2b2b33',
                           activebackground=TARJETA, activeforeground=TEXTO,
                           command=lambda k=i: self._movio(k)).pack(side='left')
            self.etiqueta_valor[i], self.etiqueta_us[i], self.etiqueta_va[i] = valor, us, va
            self.deslizador[i], self.fila[i] = desliz, fila
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
                              ('REPOSO', self._ir_a_reposo)):
            tk.Button(botones, text=texto, command=accion, bg='#2b2b33', fg=TEXTO, activebackground=ROJO,
                      activeforeground=TEXTO, bd=0, padx=12, pady=6).pack(side='left', padx=(0, 8))

        # --- limites y reposo de cada servo
        tk.Frame(tarjeta, bg='#2b2b33', height=1).pack(fill='x', pady=(6, 8))
        tk.Label(tarjeta, text='LIMITES Y REPOSO (grados nominales: 500 us = 0, 2500 us = 180). El reposo de hombro y codo es el reposo seguro de la base',
                 fg=APAGADO, bg=TARJETA, anchor='w',
                 font=('DejaVu Sans Mono', 9, 'bold')).pack(fill='x')
        self.etiqueta_pulsos = [None] * 4
        for i in MOSTRAR:
            fila = tk.Frame(tarjeta, bg=TARJETA, padx=8, pady=2)
            fila.pack(fill='x')
            tk.Label(fila, text=SERVOS[i].capitalize(), fg=TEXTO, bg=TARJETA, width=9, anchor='w',
                     font=('DejaVu Sans', 11)).pack(side='left')
            for texto, var in (('min', self.v_min[i]), ('max', self.v_max[i]), ('reposo', self.v_reposo[i])):
                tk.Label(fila, text=texto, fg=APAGADO, bg=TARJETA, font=('DejaVu Sans Mono', 9)).pack(side='left', padx=(12, 4))
                e = tk.Spinbox(fila, from_=GRADOS_MIN, to=GRADOS_MAX, increment=1, width=6, textvariable=var,
                               command=lambda k=i: self._aplicar_limites(k), buttonbackground='#2b2b33',
                               **self._estilo_entrada(font=('DejaVu Sans Mono', 11)))
                e.pack(side='left')
                self.cajas[i].append(e)
                e.bind('<Return>', lambda _e, k=i: self._aplicar_limites(k))
                e.bind('<FocusOut>', lambda _e, k=i: self._aplicar_limites(k))
            p = tk.Label(fila, text='', fg=APAGADO, bg=TARJETA, anchor='w', font=('DejaVu Sans Mono', 9))
            p.pack(side='left', padx=14)
            self.etiqueta_pulsos[i] = p
        tk.Button(tarjeta, text='GUARDAR LIMITES', command=self._guardar, bg='#2b2b33', fg=TEXTO, activebackground=ROJO,
                  activeforeground=TEXTO, bd=0, padx=12, pady=6).pack(anchor='w', pady=(8, 4))
        for i in range(4):
            self._mostrar_limites(i)

        self.fase = tk.Label(tarjeta, text='', fg=AMBAR, bg=TARJETA, anchor='w', font=('DejaVu Sans Mono', 10, 'bold'))
        self.fase.pack(fill='x')
        self.aviso = tk.Label(tarjeta, text='', fg=APAGADO, bg=TARJETA, anchor='w', font=('DejaVu Sans Mono', 9))
        self.aviso.pack(fill='x')
        self.estado = tk.Label(tarjeta, text='Esperando a la ESP32...', fg=APAGADO, bg=TARJETA, anchor='w',
                               font=('DejaVu Sans Mono', 9))
        self.estado.pack(fill='x')

    # --- acciones de la ventana
    def _movio(self, k):
        """Se movio el slider (o el 'activo') del servo k: queda resaltado en rojo y se actualiza el numero."""
        if self.ignorar_movimientos:
            return
        self._actualizar_valor(k)
        if self.seleccionado is not None:
            self._pintar(self.seleccionado, False)
        self.seleccionado = k
        self._pintar(k, True)

    def _actualizar_valor(self, k):
        a = self.angulo[k].get()
        self.etiqueta_valor[k].config(text=f'{int(a):03d}°')
        self.etiqueta_us[k].config(text=f'{us_de_grados(a):4.0f} us')

    def _pintar(self, k, resaltado):
        color = ROJO if resaltado else TEXTO
        self.etiqueta_valor[k].config(fg=color)
        self.fila[k].nombre.config(fg=color if resaltado else APAGADO)

    def _poner_activos(self, valor):
        for v in self.activo:
            v.set(valor)

    def _reposo(self, k):
        """Reposo del servo k, siempre dentro de sus limites."""
        lo, hi = self.limites[k]
        try:
            r = float(self.v_reposo[k].get().replace(',', '.'))
        except ValueError:
            r = REPOSO_DEFECTO[SERVOS[k]]
        return limitar(r, lo, hi)

    def _ir_a_reposo(self):
        """Lleva los 4 sliders a su reposo. La ESP32 no deja girar la base hasta que el hombro y el codo esten en el reposo seguro."""
        for i in range(4):
            self.angulo[i].set(self._reposo(i))
            self._actualizar_valor(i)

    def _mostrar_limites(self, k):
        """Escribe en las casillas los limites vigentes del servo k y el rango de pulsos que significan."""
        lo, hi = self.limites[k]
        self.v_min[k].set(texto_grados(lo))
        self.v_max[k].set(texto_grados(hi))
        self.v_reposo[k].set(texto_grados(self._reposo(k)))
        self.etiqueta_pulsos[k].config(text=f'{us_de_grados(lo):.0f} - {us_de_grados(hi):.0f} us')

    def _aplicar_limites(self, k):
        """Se edito una casilla de limites o de reposo del servo k: se valida (maximo: el tope de ese servo), se acota el slider y
        se manda a la ESP32."""
        try:
            lo = float(self.v_min[k].get().replace(',', '.'))
            hi = float(self.v_max[k].get().replace(',', '.'))
        except ValueError:
            self.aviso.config(text=f'{SERVOS[k].capitalize()}: numero no valido, se mantienen los limites anteriores.', fg=ROJO)
            self._mostrar_limites(k)
            return
        lo, hi = limitar(lo, GRADOS_MIN, self.topes[k]), limitar(hi, GRADOS_MIN, self.topes[k])
        if hi - lo < ANCHO_MIN:
            self.aviso.config(text=f'{SERVOS[k].capitalize()}: el maximo tiene que superar al minimo en 1 grado como minimo.',
                              fg=ROJO)
            self._mostrar_limites(k)
            return
        self.limites[k] = (lo, hi)
        self.aviso.config(text='', fg=APAGADO)
        self.deslizador[k].config(from_=lo, to=hi)           # Tk acota el valor del slider si quedo afuera
        self._mostrar_limites(k)
        self._actualizar_valor(k)

    def _guardar(self):
        cal = {SERVOS[i]: {'min': self.limites[i][0], 'max': self.limites[i][1], 'reposo': self._reposo(i)}
               for i in range(4)}
        try:
            guardar_calibracion(cal, self.ruta_cal)
        except OSError as e:
            self.aviso.config(text=f'No se pudo guardar: {e}', fg=ROJO)
            return
        self.cal = cal
        self.aviso.config(text=f'Guardado en {self.ruta_cal}', fg=APAGADO)

    # --- tareas periodicas
    def _tarea_ros(self):
        self.enlace.atender()
        e = self.enlace.estado
        if e is not None and self.enlace.estado_nuevo:
            self.enlace.estado_nuevo = False
            reinicio = self.ultimo_contador is not None and e['contador'] < self.ultimo_contador
            self.ultimo_contador = e['contador']
            if not self.sincronizado or reinicio:
                self._sincronizar(e)
            for i in range(4):
                self.etiqueta_va[i].config(text=f'va {e["actuales"][i]:5.1f}°' if e['activos'][i] else 'suelto')
        self._mostrar_estado()
        self._id_ros = self.raiz.after(20, self._tarea_ros)

    def _sincronizar(self, e):
        """Primera vez (o la ESP32 se reinicio): los sliders toman lo que tiene la ESP32 (sin saltos ni soltar nada).
        Los limites de la primera vez salen de lo guardado en el archivo; si no hay, de lo que informa la ESP32. Si la ESP32
        se reinicia, los limites que hay en la ventana se conservan: la PC es quien manda y se los vuelve a pasar."""
        primera = not self.sincronizado
        self.ignorar_movimientos = True
        avisos = []
        self.topes = [min(t, GRADOS_MAX) for t in e['topes']]
        for i in range(4):
            for caja in self.cajas[i]:
                caja.config(to=self.topes[i])                # las casillas no dejan pasar el tope de cada servo
        arranque = limites_de_arranque(self.cal, e['limites'], self.topes)
        for i in range(4):
            if primera:
                lo, hi, reposo = arranque[i]
                self.limites[i] = (lo, hi)
                self.v_reposo[i].set(texto_grados(reposo))
                self._mostrar_limites(i)
                self.deslizador[i].config(from_=lo, to=hi)
            lo, hi = self.limites[i]
            actual = e['actuales'][i]
            if e['activos'][i] and not lo <= actual <= hi:
                avisos.append(f'{SERVOS[i].capitalize()} estaba en {actual:.0f}° (fuera de sus limites): va a {limitar(actual, lo, hi):.0f}°')
            self.angulo[i].set(limitar(actual, lo, hi))
            self.activo[i].set(e['activos'][i])
            self._actualizar_valor(i)
        self.velocidad.set(max(VELOCIDAD_MIN, min(VELOCIDAD_MAX, e['velocidad'] or VELOCIDAD_DEFECTO)))
        self.aviso.config(text=' | '.join(avisos), fg=ROJO if avisos else APAGADO)
        self.sincronizado = True
        self._id_sync = self.raiz.after(250, self._fin_sincronizacion)       # los avisos de los sliders llegan un poco despues

    def _fin_sincronizacion(self):
        self.ignorar_movimientos = False

    def detener(self):
        """Cancela las tareas periodicas (para cerrar la ventana sin errores de Tk)."""
        for id_ in (self._id_ros, self._id_orden, self._id_sync):
            if id_ is not None:
                self.raiz.after_cancel(id_)

    def _tarea_orden(self):
        if self.sincronizado:
            orden = armar_orden([v.get() for v in self.angulo], [v.get() for v in self.activo], self.velocidad.get(),
                                self.salto.get(), self.limites, (self._reposo(1), self._reposo(2)))
            self.enlace.enviar(orden)
        self._id_orden = self.raiz.after(PERIODO_ORDEN_MS, self._tarea_orden)

    def _mostrar_fase(self, e):
        """Avisa cuando la ESP32 retiene la base: espera el reposo seguro del brazo, o gira con el brazo ahi."""
        if e is None or e['fase'] == 0:
            self.fase.config(text='')
        elif e['fase'] == 1 and not (e['activos'][1] and e['activos'][2]):
            self.fase.config(text='BASE EN ESPERA: activa el hombro y el codo. La base no gira con el brazo suelto.')
        elif e['fase'] == 1:
            hs, cs = e['pose_segura']
            self.fase.config(text=f'BASE EN ESPERA: el brazo va al reposo seguro (hombro {hs:.0f}°, codo {cs:.0f}°) antes de girar.')
        else:
            self.fase.config(text='BASE GIRANDO con el brazo en reposo seguro.')

    def _mostrar_estado(self):
        self._mostrar_fase(self.enlace.estado)
        seg = self.enlace.segundos_sin_estado()
        if seg is None:
            self.estado.config(text='Esperando a la ESP32... (agente corriendo? LED apagado = conectada)', fg=APAGADO)
        elif seg > 1.5:
            self.estado.config(text=f'Sin datos de la ESP32 hace {seg:.0f} s. Sigue en la ultima posicion.', fg=ROJO)
        else:
            e = self.enlace.estado
            self.estado.config(text=f'ESP32 conectada | ultima orden recibida hace {e["ms_sin_orden"]:.0f} ms', fg=APAGADO)


def main():
    enlace = crear_enlace_ros()
    raiz = tk.Tk()
    ventana = Ventana(raiz, enlace)
    try:
        raiz.mainloop()
    finally:
        ventana.detener()
        enlace.cerrar()


if __name__ == '__main__':
    main()