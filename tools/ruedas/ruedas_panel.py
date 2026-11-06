#!/usr/bin/env python3
"""Panel de las 4 ruedas con sus encoders, por Wi-Fi (ESP32 con micro-ROS): barras por rueda, maniobras de 1 m / giros y odometria.

Requisitos: ROS 2 Jazzy cargado (source /opt/ros/jazzy/setup.bash), python3-tk, el agente micro-ROS corriendo y la ESP32 con
firmware/banco/robot_4ruedas/main.c grabado (la version con el giro en el lugar, orden 7).

  Barras      una por rueda. Modo "Duty": el porcentaje de potencia va directo al motor (lazo abierto, sin PI): sirve para ver como
              responde cada motor. Modo "Velocidad": m/s por rueda con el PI del firmware. "Aceleracion" limita cuanto puede cambiar
              cada barra por segundo (la PC sube el valor de a poco: no pega 4 arranques a fondo juntos contra el 5 V).
              Las barras solo mueven las ruedas con ARMADO tildado.
  Maniobras   1 m adelante / atras y giros en el lugar de 180 y 360 grados, medidos con los encoders. Al terminar muestra lo
              calculado por la ESP32 y por la odometria de la PC; tu medis con cinta o transportador y lo cargas abajo.
  Marchar     una velocidad (m/s y rad/s) durante unos segundos con /cmd_vel, la misma interfaz que usara la navegacion.
  Odometria   x, y, angulo y recorrido integrados con los ticks de las 4 ruedas, y por rueda: velocidad pedida contra medida.
  PARAR       tambien con la tecla Esc. Frena todo, cancela la maniobra y desarma las barras.

La logica esta en ruedas_logica.py. Todo se imprime tambien en esta terminal, y se guarda en ~/caid/config/:
  ruedas_maniobras.csv   una fila por maniobra registrada (pedido, calculado, odometria, lo que mediste)
  ruedas_AAAAMMDD_HHMMSS.csv   con la casilla "Grabar CSV": todo a 10 Hz para comparar comandos contra tacometros en una planilla
  calibracion.json       diametro, trocha y adelanto de frenado (el mismo archivo de robot_teclado.py)
"""
import math
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ruedas_logica as rl  # noqa: E402

try:
    import tkinter as tk
except ImportError:                                     # PC sin python3-tk
    tk = None

PERIODO_MS = 50
FONDO, TARJETA, TEXTO, APAGADO, ROJO, AMBAR = '#0d0d10', '#15151a', '#f2f2f2', '#8a8a93', '#ff3b30', '#ffb020'
MONO = 'DejaVu Sans Mono'
V_BARRA_MAX = 0.40                                      # m/s por rueda (el firmware acota a 75 flancos/s = 0,395 m/s)
COLUMNAS = [('Rueda', 21), ('Pedido PC', 11), ('Objetivo', 9), ('Medida', 8), ('Medida', 9), ('Error', 7), ('vs media', 8),
            ('Duty', 6), ('Ticks', 7), ('', 5)]
SUBTITULOS = ['', '', 'm/s', 'm/s', 'flanc/s', 'vs obj.', '4 ruedas', 'sale', 'desde 0', 'bloq.']


# ---------------------------------------------------------------------------------------------------------------------
# ROS 2 (solo se importa al abrir la ventana de verdad: asi lo demas se puede probar sin ROS)
# ---------------------------------------------------------------------------------------------------------------------
def crear_enlace_ros():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray, Int32MultiArray

    rclpy.init()

    class Enlace(Node):
        def __init__(self):
            super().__init__('pc_ruedas_panel')
            # best effort, igual que la ESP32: una suscripcion "reliable" no recibe de un publicador "best effort"
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
            self.pub_vel = self.create_publisher(Twist, '/cmd_vel', qos)
            self.pub_cmd = self.create_publisher(Float32MultiArray, '/robot_cmd', qos)
            self.create_subscription(Int32MultiArray, '/wheel_ticks', self._al_llegar_ticks, qos)
            self.create_subscription(Float32MultiArray, '/wheel_state', self._al_llegar_estado, qos)
            self.estado, self.n_estado = None, 0
            self.ticks, self.n_ticks = None, 0

        def _al_llegar_ticks(self, msg):
            self.ticks = list(msg.data)
            self.n_ticks += 1

        def _al_llegar_estado(self, msg):
            self.estado = list(msg.data)
            self.n_estado += 1

        def atender(self):
            for _ in range(10):                                  # spin_once atiende un mensaje por llamada
                rclpy.spin_once(self, timeout_sec=0)

        def enviar_cmd(self, ident, *args):
            self.pub_cmd.publish(Float32MultiArray(data=[float(ident)] + [float(a) for a in args]))

        def enviar_vel(self, lineal, angular):
            m = Twist()
            m.linear.x = float(lineal)
            m.angular.z = float(angular)
            self.pub_vel.publish(m)

        def cerrar(self):
            self.destroy_node()
            rclpy.try_shutdown()

    return Enlace()


# ---------------------------------------------------------------------------------------------------------------------
# Ventana
# ---------------------------------------------------------------------------------------------------------------------
class Panel:
    def __init__(self, raiz, enlace, carpeta=rl.CARPETA, reloj=None, periodo_ms=PERIODO_MS):
        self.raiz = raiz
        self.enlace = enlace
        self.periodo_ms = periodo_ms
        kw = {'reloj': reloj} if reloj else {}
        self.c = rl.Controlador(enlace, carpeta=carpeta, imprimir=self._log, **kw)
        self.armado = tk.BooleanVar(value=False)
        self.modo = tk.StringVar(value='duty')
        self.enlazar = tk.BooleanVar(value=False)
        self.tanque = tk.BooleanVar(value=True)              # FL+RL y FR+RR se mueven juntas (las delanteras solas patinan)
        self.corr90 = tk.StringVar(value='0')                # grados que se suman al giro de 90 hasta que gire 90 reales
        self.txt90 = tk.StringVar(value='Giro 90°')
        self.corr180 = tk.StringVar(value='0')               # grados que se suman al giro de 180 con retroceso
        self.txt180 = tk.StringVar(value='180° HACIA LA PARED')
        self.cm_enderezar = tk.StringVar(value='100')
        self.despues = tk.BooleanVar(value=True)             # despues del 180 avanzar enderezandose
        self.cm_despues = tk.StringVar(value='20')
        self.f_del = tk.StringVar(value='1.15')              # giros: delanteras = factor x traseras
        self.k_del = tk.StringVar(value='0')                 # giros: nivel del empujon de arranque de FL y FR
        self.pre_duty = tk.StringVar(value='0')              # giros de 90: duty (%) con el que FR arranca antes
        self.pre_ms = tk.StringVar(value='100')              # ... y cuantos ms antes
        self.esc_atras = tk.StringVar(value='6')             # escape de pared: cm que retrocede (una vez)
        self.esc_giro = tk.StringVar(value='45')             # ... y grados que gira hacia el lado contrario de la pared
        self.esc_lado = tk.IntVar(value=0)                   # 0 = la pared mas cercana (sonares), +1 pared a la izquierda, -1 a la derecha
        self.sec_d1 = tk.StringVar(value='40')
        self.sec_ang = tk.StringVar(value='90')
        self.sec_d2 = tk.StringVar(value='20')
        self.aceleracion = tk.DoubleVar(value=100.0)
        self.barra = [tk.DoubleVar(value=0.0) for _ in rl.RUEDAS]
        self.sentido = tk.IntVar(value=1)                    # +1 izquierda (antihorario), -1 derecha
        self.v_marcha = tk.StringVar(value='0.21')
        self.v_giro = tk.StringVar(value='153')            # 153 °/s = 0,40 m/s por lado con trocha 300 mm (el firmware acota a 75 flancos/s)
        self.dist = tk.StringVar(value='1.00')
        self.ang = tk.StringVar(value='90')
        self.m_v = tk.StringVar(value='0.20')
        self.m_w = tk.StringVar(value='0.0')
        self.m_t = tk.StringVar(value='3.0')
        self.corr90.set(f"{float(self.c.cal.get('giro90_corr_deg', 0.0)):g}")
        self._texto90()
        self.corr180.set(f"{float(self.c.cal.get('giro180_corr_deg', 0.0)):g}")
        self._texto180()
        self.esc_atras.set(f"{self.c.cal.get('escape_atras_cm', 6.0):g}")
        self.esc_giro.set(f"{self.c.cal.get('escape_giro_deg', 45.0):g}")
        self.f_del.set(f"{self.c.cal.get('giro_factor_del', 1.15):g}")
        self.k_del.set(str(int(self.c.cal.get('kick_del', 0))))
        self.pre_duty.set(f"{self.c.cal.get('fr_pre_duty_pct', 0.0):g}")
        self.pre_ms.set(f"{self.c.cal.get('fr_pre_ms', 100.0):g}")
        self.real = tk.StringVar()
        self.desplaz = tk.StringVar()
        self.grabar = tk.BooleanVar(value=False)
        self.ajustar = tk.BooleanVar(value=True)
        self._enlazando = False
        raiz.title('Ruedas: encoders y odometria')
        raiz.configure(bg=FONDO)
        self._armar()
        raiz.bind('<Escape>', lambda _e: self._parar())
        raiz.protocol('WM_DELETE_WINDOW', self._cerrar)
        self._id = None
        self._tarea()

    # --- utilidades de construccion
    def _boton(self, padre, texto, accion, **extra):
        opciones = dict(bg='#2b2b33', fg=TEXTO, activebackground=ROJO, activeforeground=TEXTO, bd=0, padx=10, pady=5)
        opciones.update(extra)
        return tk.Button(padre, text=texto, command=accion, **opciones)

    def _casilla(self, padre, texto, variable, **extra):
        return tk.Checkbutton(padre, text=texto, variable=variable, fg=TEXTO, bg=TARJETA, selectcolor='#2b2b33',
                              activebackground=TARJETA, activeforeground=TEXTO, **extra)

    def _opcion(self, padre, texto, variable, valor, **extra):
        return tk.Radiobutton(padre, text=texto, variable=variable, value=valor, fg=TEXTO, bg=TARJETA, selectcolor='#2b2b33',
                              activebackground=TARJETA, activeforeground=TEXTO, **extra)

    def _entrada(self, padre, variable, ancho=6):
        return tk.Entry(padre, textvariable=variable, width=ancho, bg='#2b2b33', fg=TEXTO, insertbackground=TEXTO,
                        highlightthickness=1, highlightbackground='#3a3a44', highlightcolor=ROJO, bd=0, justify='center',
                        font=(MONO, 10))

    def _etiqueta(self, padre, texto, **extra):
        return tk.Label(padre, text=texto, fg=extra.pop('fg', APAGADO), bg=TARJETA, **extra)

    def _linea(self, padre):
        tk.Frame(padre, bg='#2b2b33', height=1).pack(fill='x', pady=(8, 8))

    def _armar(self):
        t = tk.Frame(self.raiz, bg=TARJETA, padx=22, pady=14)
        t.pack(padx=18, pady=14)

        cab = tk.Frame(t, bg=TARJETA)
        cab.pack(fill='x', pady=(0, 8))
        tk.Label(cab, text='■', fg=ROJO, bg=TARJETA).pack(side='left')
        tk.Label(cab, text=' RUEDAS', fg=TEXTO, bg=TARJETA, font=(MONO, 10, 'bold')).pack(side='left')
        self.estado = tk.Label(cab, text='Esperando a la ESP32...', fg=APAGADO, bg=TARJETA, font=(MONO, 9))
        self.estado.pack(side='right')

        # --- barras
        fila = tk.Frame(t, bg=TARJETA)
        fila.pack(fill='x')
        self._casilla(fila, 'ARMADO (las barras mueven las ruedas)', self.armado, command=self._armo).pack(side='left')
        self._opcion(fila, 'Duty', self.modo, 'duty', command=self._cambio_modo).pack(side='left', padx=(16, 0))
        self._opcion(fila, 'Velocidad (PI)', self.modo, 'vel', command=self._cambio_modo).pack(side='left')
        self._casilla(fila, 'Enlazar las 4', self.enlazar).pack(side='left', padx=(12, 0))
        self._casilla(fila, 'Tanque (izq. y der.)', self.tanque, command=self._movio_tanque).pack(side='left', padx=(12, 0))
        self._etiqueta(fila, 'Aceleracion').pack(side='left', padx=(12, 0))
        tk.Scale(fila, from_=25, to=400, orient='horizontal', length=110, variable=self.aceleracion, showvalue=True, bg=TARJETA,
                 fg=TEXTO, troughcolor='#2b2b33', highlightthickness=0, bd=0, resolution=5,
                 command=lambda _v: self._act_aceleracion()).pack(side='left')
        self._etiqueta(fila, '%/s').pack(side='left')

        self.valor_barra, self.deslizador = [], []
        for i, nombre in enumerate(rl.NOMBRES):
            f = tk.Frame(t, bg=TARJETA, pady=2)
            f.pack(fill='x')
            self._etiqueta(f, nombre, width=19, anchor='w', font=('DejaVu Sans', 11)).pack(side='left')
            d = tk.Scale(f, from_=-100, to=100, orient='horizontal', length=420, showvalue=False, variable=self.barra[i],
                         resolution=1, bg=TARJETA, fg=TEXTO, troughcolor='#2b2b33', highlightthickness=0, bd=0, sliderlength=22,
                         activebackground=ROJO, command=lambda _v, k=i: self._movio(k))
            d.pack(side='left', padx=8)
            v = tk.Label(f, text='+0 %', fg=TEXTO, bg=TARJETA, width=11, anchor='w', font=(MONO, 13, 'bold'))
            v.pack(side='left')
            self.deslizador.append(d)
            self.valor_barra.append(v)
        self.pista = self._etiqueta(t, '', anchor='w', font=(MONO, 8))
        self.pista.pack(fill='x')

        self._linea(t)
        # --- tabla de ruedas
        tabla = tk.Frame(t, bg=TARJETA)
        tabla.pack(fill='x')
        self.celdas = []
        for j, (titulo, ancho) in enumerate(COLUMNAS):
            tk.Label(tabla, text=titulo, fg=TEXTO, bg=TARJETA, width=ancho, anchor='w', font=(MONO, 9, 'bold')).grid(row=0, column=j)
            tk.Label(tabla, text=SUBTITULOS[j], fg=APAGADO, bg=TARJETA, width=ancho, anchor='w', font=(MONO, 8)).grid(row=1, column=j)
        for i, nombre in enumerate(rl.NOMBRES):
            fila_celdas = []
            for j, (_titulo, ancho) in enumerate(COLUMNAS):
                texto = nombre if j == 0 else ''
                lb = tk.Label(tabla, text=texto, fg=TEXTO, bg=TARJETA, width=ancho, anchor='w', font=(MONO, 9))
                lb.grid(row=2 + i, column=j)
                fila_celdas.append(lb)
            self.celdas.append(fila_celdas)

        f = tk.Frame(t, bg=TARJETA, pady=6)
        f.pack(fill='x')
        self.odo = tk.Label(f, text='', fg=TEXTO, bg=TARJETA, font=(MONO, 10), anchor='w')
        self.odo.pack(side='left')
        self._boton(f, 'ODOMETRIA EN CERO', self._cero).pack(side='right')
        self._casilla(f, 'Grabar CSV', self.grabar, command=self._grabar).pack(side='right', padx=10)

        self._linea(t)
        # --- maniobras
        f = tk.Frame(t, bg=TARJETA)
        f.pack(fill='x')
        self._boton(f, '1 m adelante', lambda: self._distancia(+1.0)).pack(side='left', padx=(0, 6))
        self._boton(f, '1 m atras', lambda: self._distancia(-1.0)).pack(side='left', padx=(0, 6))
        self._boton(f, 'Giro 90°', self._giro90, textvariable=self.txt90).pack(side='left', padx=(0, 2))
        self._boton(f, '-1', lambda: self._ajuste90(-1.0), padx=5).pack(side='left')
        self._boton(f, '+1', lambda: self._ajuste90(+1.0), padx=5).pack(side='left', padx=(0, 6))
        self._boton(f, 'Giro 180°', lambda: self._giro(180.0)).pack(side='left', padx=(0, 6))
        self._boton(f, '180° AUTO (sonar)', self._giro180_auto).pack(side='left', padx=(0, 6))
        self._boton(f, 'Giro 360°', lambda: self._giro(360.0)).pack(side='left', padx=(0, 10))
        self._opcion(f, 'izquierda', self.sentido, 1).pack(side='left')
        self._opcion(f, 'derecha', self.sentido, -1).pack(side='left')

        f = tk.Frame(t, bg=TARJETA, pady=5)
        f.pack(fill='x')
        self._etiqueta(f, 'Distancia').pack(side='left')
        self._entrada(f, self.dist, 5).pack(side='left', padx=3)
        self._etiqueta(f, 'm  a').pack(side='left')
        self._entrada(f, self.v_marcha, 5).pack(side='left', padx=3)
        self._etiqueta(f, 'm/s (+ adelante, - atras)').pack(side='left')
        self._boton(f, 'IR', lambda: self._distancia(None)).pack(side='left', padx=(6, 18))
        self._etiqueta(f, 'Giro').pack(side='left')
        self._entrada(f, self.ang, 5).pack(side='left', padx=3)
        self._etiqueta(f, '°  a').pack(side='left')
        self._entrada(f, self.v_giro, 5).pack(side='left', padx=3)
        self._etiqueta(f, '°/s').pack(side='left')
        self._boton(f, 'GIRAR', lambda: self._giro(None)).pack(side='left', padx=6)

        f = tk.Frame(t, bg=TARJETA, pady=3)
        f.pack(fill='x')
        self._etiqueta(f, 'Con sonares:').pack(side='left')
        self._boton(f, 'LINEA RECTA enderezando', self._enderezar).pack(side='left', padx=(6, 2))
        self._entrada(f, self.cm_enderezar, 4).pack(side='left', padx=3)
        self._etiqueta(f, 'cm (0 = hasta la pared)     ').pack(side='left')
        self._boton(f, '180° HACIA LA PARED', self._giro180_pared, textvariable=self.txt180).pack(side='left', padx=(6, 2))
        self._boton(f, '-1', lambda: self._ajuste180(-1.0), padx=5).pack(side='left')
        self._boton(f, '+1', lambda: self._ajuste180(+1.0), padx=5).pack(side='left', padx=(0, 6))
        self._casilla(f, 'despues avanzar', self.despues).pack(side='left')
        self._entrada(f, self.cm_despues, 4).pack(side='left', padx=3)
        self._etiqueta(f, 'cm').pack(side='left')

        f = tk.Frame(t, bg=TARJETA, pady=3)
        f.pack(fill='x')
        self._etiqueta(f, 'En los giros las delanteras van x').pack(side='left')
        self._entrada(f, self.f_del, 5).pack(side='left', padx=3)
        self._etiqueta(f, 'las traseras; empujon de arranque delanteras (0-3, traseras = 2):').pack(side='left')
        self._entrada(f, self.k_del, 3).pack(side='left', padx=3)
        self._boton(f, 'APLICAR', self._aplicar_delanteras).pack(side='left', padx=6)

        f = tk.Frame(t, bg=TARJETA, pady=3)
        f.pack(fill='x')
        self._etiqueta(f, 'Giro 90: FR arranca antes con duty').pack(side='left')
        self._entrada(f, self.pre_duty, 4).pack(side='left', padx=3)
        self._etiqueta(f, '% durante').pack(side='left')
        self._entrada(f, self.pre_ms, 4).pack(side='left', padx=3)
        self._etiqueta(f, 'ms (duty 0 = apagado)').pack(side='left')
        self._boton(f, 'APLICAR', self._aplicar_pre_fr).pack(side='left', padx=6)

        f = tk.Frame(t, bg=TARJETA, pady=3)
        f.pack(fill='x')
        self._etiqueta(f, 'ESCAPE de pared: retrocede').pack(side='left')
        self._entrada(f, self.esc_atras, 4).pack(side='left', padx=3)
        self._etiqueta(f, 'cm (una vez) y gira').pack(side='left')
        self._entrada(f, self.esc_giro, 4).pack(side='left', padx=3)
        self._etiqueta(f, '° al lado contrario de la pared:').pack(side='left')
        self._opcion(f, 'la mas cercana', self.esc_lado, 0).pack(side='left')
        self._opcion(f, 'izq', self.esc_lado, 1).pack(side='left')
        self._opcion(f, 'der', self.esc_lado, -1).pack(side='left')
        self._boton(f, 'ESCAPE', self._escape).pack(side='left', padx=6)

        f = tk.Frame(t, bg=TARJETA, pady=3)
        f.pack(fill='x')
        self._etiqueta(f, 'Secuencia:').pack(side='left')
        self._entrada(f, self.sec_d1, 4).pack(side='left', padx=3)
        self._etiqueta(f, 'cm recto, giro').pack(side='left')
        self._entrada(f, self.sec_ang, 4).pack(side='left', padx=3)
        self._etiqueta(f, '° (lado de arriba), recto').pack(side='left')
        self._entrada(f, self.sec_d2, 4).pack(side='left', padx=3)
        self._etiqueta(f, 'cm').pack(side='left')
        self._boton(f, 'SECUENCIA', self._secuencia).pack(side='left', padx=6)

        f = tk.Frame(t, bg=TARJETA, pady=3)
        f.pack(fill='x')
        self._etiqueta(f, 'Marchar a').pack(side='left')
        self._entrada(f, self.m_v, 5).pack(side='left', padx=3)
        self._etiqueta(f, 'm/s y').pack(side='left')
        self._entrada(f, self.m_w, 5).pack(side='left', padx=3)
        self._etiqueta(f, 'rad/s durante').pack(side='left')
        self._entrada(f, self.m_t, 5).pack(side='left', padx=3)
        self._etiqueta(f, 's').pack(side='left')
        self._boton(f, 'MARCHAR', self._marchar).pack(side='left', padx=6)
        self._casilla(f, 'ajustar solo el adelanto de frenado', self.ajustar, command=self._act_ajuste).pack(side='left', padx=12)

        tk.Button(t, text='PARAR  (Esc)', command=self._parar, bg=ROJO, fg='white', activebackground='#b02018',
                  activeforeground='white', bd=0, font=('DejaVu Sans', 13, 'bold'), pady=8).pack(fill='x', pady=(8, 4))

        self._linea(t)
        # --- resultado de la ultima maniobra
        self.resultado = tk.Label(t, text='Todavia no hay una maniobra terminada.', fg=APAGADO, bg=TARJETA, anchor='w',
                                  justify='left', wraplength=900, font=(MONO, 9))
        self.resultado.pack(fill='x')
        f = tk.Frame(t, bg=TARJETA, pady=4)
        f.pack(fill='x')
        self.unidad = self._etiqueta(f, 'Real medido:')
        self.unidad.pack(side='left')
        self._entrada(f, self.real, 7).pack(side='left', padx=4)
        self.unidad_real = self._etiqueta(f, 'm')
        self.unidad_real.pack(side='left')
        self._etiqueta(f, '   se desplazo el centro').pack(side='left')
        self._entrada(f, self.desplaz, 6).pack(side='left', padx=4)
        self._etiqueta(f, 'cm').pack(side='left')
        self._boton(f, 'REGISTRAR', lambda: self._registrar(False)).pack(side='left', padx=(14, 6))
        self._boton(f, 'REGISTRAR Y CORREGIR', lambda: self._registrar(True)).pack(side='left')

        self.texto_log = tk.Text(t, height=7, width=118, bg='#0d0d10', fg='#c8c8d0', bd=0, font=(MONO, 8), wrap='word',
                                 highlightthickness=0, state='disabled')
        self.texto_log.pack(fill='x', pady=(8, 0))
        self._cambio_modo()

    # --- registro de mensajes: terminal y ventana
    def _log(self, texto):
        print(texto, flush=True)
        try:
            self.texto_log.config(state='normal')
            self.texto_log.insert('end', texto + '\n')
            lineas = int(self.texto_log.index('end-1c').split('.')[0])
            if lineas > 300:
                self.texto_log.delete('1.0', f'{lineas - 300}.0')
            self.texto_log.see('end')
            self.texto_log.config(state='disabled')
        except tk.TclError:
            pass

    # --- barras
    def _armo(self):
        for b in self.barra:
            b.set(0.0)
        self.c.objetivo = [0.0] * 4
        self.c.armado = self.armado.get()
        if not self.c.armado:
            self.c.parar()
        self._refrescar_barras()

    def _cambio_modo(self):
        modo = self.modo.get()
        self.c.poner_modo(modo)
        for b in self.barra:
            b.set(0.0)
        for d in self.deslizador:
            if modo == 'duty':
                d.config(from_=-100, to=100, resolution=1)
            else:
                d.config(from_=-V_BARRA_MAX, to=V_BARRA_MAX, resolution=0.01)
        self.pista.config(text=('Duty: potencia directa al motor, sin PI. Los motores no arrancan por debajo de ~0,35 (varia por motor).'
                                if modo == 'duty' else
                                'Velocidad con PI: minimo ~0,13 m/s por rueda; por debajo de 0,07 m/s el firmware lo toma como 0.'))
        self._refrescar_barras()

    def _act_aceleracion(self):
        self.c.aceleracion = float(self.aceleracion.get())

    def _act_ajuste(self):
        self.c.ajustar_frenado = self.ajustar.get()

    PAREJA = {0: 2, 2: 0, 1: 3, 3: 1}                        # FL<->RL (lado izquierdo) y FR<->RR (lado derecho)

    def _movio_tanque(self):
        """Al tildar 'Tanque' las barras de cada lado se igualan (la trasera copia a la delantera)."""
        if self.tanque.get():
            for k in (0, 1):
                self.barra[self.PAREJA[k]].set(self.barra[k].get())
        self._refrescar_barras()

    def _movio(self, k):
        if self.tanque.get() and not self._enlazando and not self.enlazar.get():
            self._enlazando = True
            try:
                self.barra[self.PAREJA[k]].set(self.barra[k].get())
            finally:
                self._enlazando = False
        if self.enlazar.get() and not self._enlazando:
            self._enlazando = True
            try:
                for b in self.barra:
                    b.set(self.barra[k].get())
            finally:
                self._enlazando = False
        self._refrescar_barras()

    def _refrescar_barras(self):
        unidad = '%' if self.modo.get() == 'duty' else 'm/s'
        for i in range(4):
            v = self.barra[i].get()
            self.valor_barra[i].config(text=f'{v:+.0f} %' if unidad == '%' else f'{v:+.2f} m/s')
        self.c.objetivo = [b.get() for b in self.barra]

    # --- maniobras
    def _numero(self, variable, nombre, lo, hi):
        try:
            x = float(variable.get().replace(',', '.'))
        except ValueError:
            self._log(f'{nombre}: "{variable.get()}" no es un numero.')
            return None
        if not lo <= x <= hi:
            self._log(f'{nombre}: tiene que estar entre {lo:g} y {hi:g}.')
            return None
        return x

    def _distancia(self, fija):
        vel = self._numero(self.v_marcha, 'Velocidad de marcha', 0.05, 0.40)
        if vel is None:
            return
        if fija is None:
            d = self._numero(self.dist, 'Distancia', -5.0, 5.0)
            if d is None or d == 0:
                return
        else:
            d = fija
        self._antes_de_maniobra()
        self.c.distancia(d, vel)

    def _giro(self, fijo):
        vel = self._numero(self.v_giro, 'Velocidad de giro', 20.0, 400.0)
        if vel is None:
            return
        if fijo is None:
            g = self._numero(self.ang, 'Giro', 1.0, 720.0)
            if g is None:
                return
        else:
            g = fijo
        self._antes_de_maniobra()
        self.c.girar(g * self.sentido.get(), vel)

    def _texto90(self):
        try:
            c = float(self.corr90.get().replace(',', '.'))
        except ValueError:
            c = 0.0
        self.txt90.set(f'Giro 90° ({c:+g}° = {90.0 + c:g}° pedidos)')

    def _giro180_auto(self):
        """180 grados hacia el lado contrario de la pared lateral mas cercana (sonares); informa las distancias al terminar."""
        vel = self._numero(self.v_giro, 'Velocidad de giro', 20.0, 400.0)
        if vel is None:
            return
        self._antes_de_maniobra()
        self.c.girar_180_auto(vel)

    def _texto180(self):
        try:
            c = float(self.corr180.get().replace(',', '.'))
        except ValueError:
            c = 0.0
        self.txt180.set(f'180° HACIA LA PARED ({c:+g}°)')

    def _ajuste180(self, paso):
        try:
            c = float(self.corr180.get().replace(',', '.'))
        except ValueError:
            c = 0.0
        c = max(-45.0, min(45.0, c + paso))
        self.corr180.set(f'{c:g}')
        self._texto180()
        self.c.cal['giro180_corr_deg'] = c
        rl.guardar_cal(self.c.cal, self.c.ruta_cal)
        self._log(f'Sumador del giro de 180° con retroceso: {c:+g}° (se piden {180.0 + c:g}°).')

    def _enderezar(self):
        """Linea recta por el pasillo: avanza despacio manteniendose alineado y centrado con los sonares laterales (0 cm = hasta
        que haya una pared de frente). Para probarlo se pone el robot inclinado dentro del pasillo."""
        cm = self._numero(self.cm_enderezar, 'Linea recta: distancia (cm)', 0.0, 1000.0)
        if cm is None:
            return
        self._antes_de_maniobra()
        self.c.enderezar_avanzando(cm / 100.0 if cm > 0 else None)

    def _giro180_pared(self):
        """180 grados HACIA la pared lateral mas cercana, retrocediendo mientras gira; despues, si esta tildado, avanza enderezandose."""
        vel = self._numero(self.v_giro, 'Velocidad de giro', 20.0, 400.0)
        corr = self._numero(self.corr180, 'Sumador del giro de 180°', -45.0, 45.0)
        despues = None
        if self.despues.get():
            cm = self._numero(self.cm_despues, 'Despues del giro: avanzar (cm)', 5.0, 300.0)
            if cm is None:
                return
            despues = cm / 100.0
        corr90 = self._numero(self.corr90, 'Sumador del giro de 90°', -45.0, 45.0)
        if vel is None or corr is None or corr90 is None:
            return
        self._texto180()
        self._antes_de_maniobra()
        self.c.giro_180_hacia_la_pared(vel, corr, despues, corr90)

    def _giro90(self):
        """Giro de 90 mas el sumador de grados (se ajusta con -1 / +1 hasta que el robot gire 90 reales)."""
        c = self._numero(self.corr90, 'Sumador del giro de 90°', -45.0, 45.0)
        if c is None:
            return
        self._texto90()
        self._giro(90.0 + c)

    def _ajuste90(self, paso):
        try:
            c = float(self.corr90.get().replace(',', '.'))
        except ValueError:
            c = 0.0
        c = max(-45.0, min(45.0, c + paso))
        self.corr90.set(f'{c:g}')
        self._texto90()
        self.c.cal['giro90_corr_deg'] = c                     # queda en calibracion.json: es el valor que hay que recordar
        guardar = getattr(rl, 'guardar_cal', None)
        if guardar:
            guardar(self.c.cal, self.c.ruta_cal)
        self._log(f'Sumador del giro de 90°: {c:+g}° (se piden {90.0 + c:g}°).')

    def _aplicar_delanteras(self):
        fac = self._numero(self.f_del, 'Delanteras: factor', 0.5, 2.0)
        niv = self._numero(self.k_del, 'Delanteras: nivel de arranque', 0, 3)
        if None in (fac, niv):
            return
        self.c.config_delanteras(fac, int(round(niv)))

    def _aplicar_pre_fr(self):
        d = self._numero(self.pre_duty, 'Pre-arranque FR: duty (%)', 0, 100)
        ms = self._numero(self.pre_ms, 'Pre-arranque FR: ms', 0, 500)
        if None in (d, ms):
            return
        self.c.config_pre_fr(d, ms)

    def _escape(self):
        """Retrocede una vez y gira hacia el lado contrario de la pared (cm y grados ajustables; quedan en calibracion.json)."""
        vel = self._numero(self.v_marcha, 'Velocidad de marcha', 0.05, 0.40)
        vg = self._numero(self.v_giro, 'Velocidad de giro', 20.0, 400.0)
        atras = self._numero(self.esc_atras, 'Escape: retroceso (cm)', 1.0, 30.0)
        giro = self._numero(self.esc_giro, 'Escape: giro (grados)', 1.0, 180.0)
        if None in (vel, vg, atras, giro):
            return
        self.c.cal['escape_atras_cm'], self.c.cal['escape_giro_deg'] = atras, giro
        guardar = getattr(rl, 'guardar_cal', None)
        if guardar:
            guardar(self.c.cal, self.c.ruta_cal)
        self._antes_de_maniobra()
        self.c.escape_de_pared(atras / 100.0, giro, vel, vg, self.esc_lado.get())

    def _secuencia(self):
        """Recto d1 cm, giro (izquierda o derecha segun la opcion de arriba) y recto d2 cm, uno tras otro."""
        vel = self._numero(self.v_marcha, 'Velocidad de marcha', 0.05, 0.40)
        vg = self._numero(self.v_giro, 'Velocidad de giro', 20.0, 400.0)
        d1 = self._numero(self.sec_d1, 'Secuencia: primer tramo (cm)', 1.0, 500.0)
        g = self._numero(self.sec_ang, 'Secuencia: giro', 1.0, 720.0)
        d2 = self._numero(self.sec_d2, 'Secuencia: segundo tramo (cm)', 1.0, 500.0)
        if None in (vel, vg, d1, g, d2):
            return
        self._antes_de_maniobra()
        self.c.secuenciar([('distancia', d1 / 100.0), ('giro', g * self.sentido.get()), ('distancia', d2 / 100.0)], vel, vg)

    def _marchar(self):
        v = self._numero(self.m_v, 'Marchar: velocidad', -0.40, 0.40)
        w = self._numero(self.m_w, 'Marchar: giro', -4.0, 4.0)
        t = self._numero(self.m_t, 'Marchar: tiempo', 0.5, 30.0)
        if None in (v, w, t):
            return
        self._antes_de_maniobra()
        self.c.marchar(v, w, t)

    def _antes_de_maniobra(self):
        self.armado.set(False)
        for b in self.barra:
            b.set(0.0)
        self.c.armado = False
        self._refrescar_barras()

    def _parar(self):
        self.armado.set(False)
        for b in self.barra:
            b.set(0.0)
        self.c.parar()
        self._refrescar_barras()

    def _cero(self):
        self.c.poner_cero()

    def _grabar(self):
        self.c.grabar(self.grabar.get())

    def _registrar(self, corregir):
        real = desplaz = None
        if self.real.get().strip():
            real = self._numero(self.real, 'Real medido', 0.0, 100000.0)
            if real is None:
                return
        if self.desplaz.get().strip():
            desplaz = self._numero(self.desplaz, 'Desplazamiento', -10000.0, 10000.0)
            if desplaz is None:
                return
        if corregir and real is None:
            self._log('Para corregir hace falta cargar el valor real medido.')
            return
        if self.c.registrar(real, desplaz, corregir):
            self.real.set('')
            self.desplaz.set('')

    def _cerrar(self):
        self._parar()
        self.detener()
        self.c.cerrar()
        self.raiz.destroy()

    def detener(self):
        """Cancela la tarea periodica (para cerrar la ventana sin errores de Tk)."""
        if self._id is not None:
            try:
                self.raiz.after_cancel(self._id)
            except tk.TclError:
                pass
            self._id = None

    # --- el reloj de la ventana
    def _tarea(self):
        self.enlace.atender()
        self.c.tick()
        self._mostrar()
        self._id = self.raiz.after(self.periodo_ms, self._tarea)

    def _mostrar(self):
        c = self.c
        if not c.conectado:
            self.estado.config(text='Esperando a la ESP32...  (agente corriendo? LED apagado = conectada)', fg=APAGADO)
        else:
            self.estado.config(text=f'ESP32 conectada | diametro {c.diametro * 1000:.2f} mm | trocha {c.trocha * 1000:.1f} mm | '
                                    f'frenado {c.frenado * 1000:.0f} mm, giro {c.frenado_giro * 1000:.0f} mm', fg=APAGADO)
        e = c.estado
        if e is not None:
            vm, vo = c.v_medida_ms(), c.v_objetivo_ms()
            media = sum(abs(x) for x in vm) / 4.0
            ticks = c.ticks_desde_cero()
            for i in range(4):
                fila = self.celdas[i]
                maniobra = c.maniobra is not None or c.marcha is not None or c.guiado is not None
                if maniobra:
                    pedido = 'maniobra'
                elif not c.armado:
                    pedido = '-'
                elif c.modo == 'duty':
                    pedido = f'{c.aplicado[i]:+.0f} %'
                else:
                    pedido = f'{c.aplicado[i]:+.3f}'
                obj_fps = e[4 + i]
                err = f'{100.0 * (abs(e[i]) - abs(obj_fps)) / abs(obj_fps):+.0f} %' if abs(obj_fps) > 1.0 else '-'
                vs = f'{100.0 * (abs(vm[i]) - media) / media:+.0f} %' if media * rl.flancos_por_m(c.diametro) > 5.0 else '-'
                textos = [None, pedido, f'{vo[i]:+.3f}', f'{vm[i]:+.3f}', f'{e[i]:+.1f}', err, vs, f'{e[8 + i]:+.2f}',
                          f'{ticks[i]:+d}', 'SI' if e[12 + i] > 0.5 else '']
                for j, txt in enumerate(textos):
                    if txt is not None:
                        fila[j].config(text=txt, fg=self._color_celda(j, txt))
        o = c.odo
        self.odo.config(text=f'Odometria  x {o.x:+.3f} m   y {o.y:+.3f} m   giro {math.degrees(o.theta):+.1f}°   '
                             f'avance {o.s:+.3f} m   recorrido {o.recorrido:.3f} m')
        self._mostrar_resultado()

    @staticmethod
    def _color_celda(columna, texto):
        """Rojo si la rueda esta bloqueada; ambar si su velocidad se aparta mas de 15 % del objetivo o de las otras ruedas."""
        if columna == 9:
            return ROJO if texto else TEXTO
        if columna in (5, 6) and texto.endswith('%'):
            return AMBAR if abs(float(texto.split()[0])) > 15.0 else TEXTO
        return TEXTO

    def _mostrar_resultado(self):
        r = self.c.resultado
        m = self.c.maniobra
        if m is not None:
            avance = f'{m["recorrido"]:+.3f} m' if m['activa'] else 'esperando a la ESP32'
            self.resultado.config(text=f'En curso: {m["nombre"]} | recorrido de cada rueda {avance}', fg=AMBAR)
            return
        if self.c.marcha is not None:
            self.resultado.config(text=f'En curso: marcha {self.c.marcha["v"]:+.2f} m/s, {self.c.marcha["w"]:+.2f} rad/s', fg=AMBAR)
            return
        if r is None:
            self.resultado.config(text='Todavia no hay una maniobra terminada.', fg=APAGADO)
            return
        if r['tipo'] == 'giro':
            txt = (f'Terminada: {r["nombre"]}.  Pedido {r["pedido"]:+.1f}° | ESP32 {r["calculado"]:+.1f}° | odometria de la PC '
                   f'{r["odometria"]:+.1f}° | el centro se movio {r["desplazamiento_odo_cm"]:.1f} cm segun la odometria.')
            unidad = '° (lo que giro de verdad)'
        elif r['tipo'] == 'marcha':
            txt = (f'Terminada: {r["nombre"]}.  Esperado v x t {r["pedido"]:+.3f} m | odometria de la PC {r["odometria"]:+.3f} m '
                   f'y {r["giro_odo_deg"]:+.1f}°.')
            unidad = 'm (lo que avanzo de verdad)'
        else:
            txt = (f'Terminada: {r["nombre"]}.  Pedido {r["pedido"]:+.3f} m | ESP32 {r["calculado"]:+.3f} m | odometria de la PC '
                   f'{r["odometria"]:+.3f} m.')
            unidad = 'm (lo que avanzo de verdad)'
        self.resultado.config(text=txt + '  Ticks: ' + ' '.join(f'{n} {t:+d}' for n, t in zip(rl.RUEDAS, r['ticks'])), fg=TEXTO)
        self.unidad_real.config(text=unidad)


def main():
    enlace = crear_enlace_ros()
    raiz = tk.Tk()
    panel = Panel(raiz, enlace)
    try:
        raiz.mainloop()
    finally:
        panel.detener()
        panel.c.cerrar()
        enlace.cerrar()


if __name__ == '__main__':
    main()
