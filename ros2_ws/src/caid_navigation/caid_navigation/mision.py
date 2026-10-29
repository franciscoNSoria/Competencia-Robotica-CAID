"""
Navegador de la misión (núcleo puro, sin ROS).

Une todo: pose, mapa, planificador, ultrasonidos, cámara y la política de la misión. Lo usan el
nodo
de ROS (navegador_node.py) y el simulador 2D de las pruebas, con la misma interfaz:

    nav = Navegador()
    v, w, pan = nav.actualizar(t, odom=(x, y, yaw), rangos={'frente': ..}, pan_real=..)
    nav.vision(t, detecciones, pose, pan)          # cada imagen procesada
    nav.resultado_manipulacion(ok)                 # respuesta del brazo
    nav.eventos                                    # pedidos al brazo: ('agarrar' | 'soltar',
    color, (x, y, z))

Política (docs/roadmap/ROADMAP.md, "Comportamiento de la misión"):
1. Si sostiene un cubo y conoce su canasto, va a dejarlo. Si no lo conoce, explora.
2. Si tiene la pinza libre y conoce un cubo que falta entregar, va a buscarlo.
3. Si no, explora la frontera más barata. Las flechas de los colores que faltan encarecen los otros
   lados de la celda donde están: empujan la exploración, no la reemplazan.
4. Con los tres entregados (o si ya no queda nada por explorar) vuelve a la celda de partida.

Antes de entrar a una celda nueva: si no la vio de lejos con el ultrasonido, barre la celda de
enfrente
girando ±24° (detecta bloques chicos pegados a la pared); si la cámara no vio su piso, la mira
(zona
roja).
"""
import math

from caid_navigation.camara import (
    Camara, CANASTO_ANCHO, CANASTO_BASE_Z, celda_visible, CUBO_BASE_Z, CUBO_LADO, forma_de_cubo,
    ubicar_flecha, ubicar_objeto, zonas_en_suelo)
from caid_navigation.conocimiento import Conocimiento
from caid_navigation.control import Giro, Recta
from caid_navigation.geometria import (
    a_mundo, a_robot, ang_dif, CELDA, celda_de, centro, HUELLA_X, limitar, MEDIA, PASOS, rumbo_de,
    vecina, yaw_de)
from caid_navigation.localizacion import Estimador
from caid_navigation.planificador import Costos, planificar
from caid_navigation.sonar import Barrido, leer_frente, RegistroLateral

COLORES = ('rojo', 'verde', 'azul')

PARAMETROS = {
    'v_avance': 0.25,           # m/s (el PI de las ruedas llega a ~0,39 m/s)
    'v_retro': 0.15,            # m/s
    'v_acercar': 0.08,          # m/s, acercamiento final a un cubo o canasto
    'w_giro': 1.5,              # rad/s
    'margen_giro': 0.006,       # m, holgura mínima con las paredes al planificar un giro
    'angulo_barrido': 0.42,     # rad (24°)
    'barrer_celdas_nuevas': True,
    'usar_camara': True,
    'mirar_cruces': True,
    'inspeccion_inicial': True,
    'agarre_dx': -0.02,         # m, x del cubo en base_link al agarrarlo (P: eje del brazo)
    'deposito_dist': 0.19,      # m, del centro del robot al centro del canasto (queda atrás)
    'radio_abierto': 3,         # celdas: lejos de toda pared solo se explora cerca del inicio
    'penal_abierto': 25.0,      # s
    'penal_flecha': 40.0,       # s
    'penal_trabado': 60.0,      # s, por cada vez que el robot se trabó en ese paso
    'tiempo_max': 900.0,        # s, después vuelve al inicio (ajustar al tiempo de la ronda)
    'tiempo_manipular': 30.0,   # s, espera máxima de la respuesta del brazo
    'confirmar_objeto': 2,      # detecciones para dar por visto un cubo o canasto
    'confirmar_zona': 2,        # imágenes con votos para prohibir una celda
    'confirmar_flecha': 2,      # imágenes iguales para usar una flecha
    'margen_giro_ejecucion': 0.006,  # m: al girar de verdad (se planifica con margen_giro)
    'mirar_en_barrido': True,   # quieto en los extremos del barrido para que mire la cámara
    'factor_giro': 1.0,         # giro real / giro de la odometría (calibrar en el banco)
    'penal_partida': 60.0,      # s: explorar alrededor de la partida va al final
    'penal_sala': 15.0,         # s: celdas con menos de 2 paredes (salas, exterior)
    'penal_afuera': 60.0,       # s: fronteras que no están "adentro" (detrás de la entrada)
    'volver_a_la_entrada': True,  # después de entregar, volver a entrar al laberinto
    'punto_de_reingreso': 'entrada',  # 'entrada' o 'bifurcacion' (la primera, si se guardó)
    'reposo_al_final': True,    # con 3 entregados queda quieto junto al canasto (zona de inicio)
    'v_carga': 0.15,            # m/s con un cubo en el 2.o piso
    'w_carga': 0.9,             # rad/s con un cubo
    'a_carga': 0.4,             # m/s², aceleración lineal máxima con un cubo
    'alfa_carga': 2.0,          # rad/s², aceleración angular máxima con un cubo
}


class Observacion:
    """Promedio de las posiciones vistas de un objeto."""

    def __init__(self, x, y):
        self.x, self.y, self.n = x, y, 1

    def agregar(self, x, y):
        self.n += 1
        k = 1.0 / min(self.n, 8)
        self.x += k * (x - self.x)
        self.y += k * (y - self.y)


# ---------------------------------------------------------------------------------------------
# Tareas: cada una mueve el robot hasta terminar
# ---------------------------------------------------------------------------------------------
class Tarea:
    nombre = 'tarea'
    pan = 0.0

    def paso(self, nav, pose, rangos, dt):
        return 0.0, 0.0, True

    def __repr__(self):
        return self.nombre


class TRecta(Tarea):
    """Recta por el eje h hasta el centro de `fin` (+ s), adelante (+1) o atrás (-1)."""

    def __init__(self, nav, h, signo, fin, s=0.0, lat=None, extender=False, v_max=None,
                 a_fin=None, lat_fin=None, nombre=None):
        self.nombre = nombre or ('avanzar' if signo > 0 else 'retroceder')
        self.h, self.signo = h, signo
        self.fin = fin
        self.extender = extender
        u = PASOS[h]
        n = PASOS[(h + 1) % 4]
        cx, cy = centro(fin)
        self.base_lat = cx * n[0] + cy * n[1]
        self.a_fin = a_fin if a_fin is not None else cx * u[0] + cy * u[1] + s
        self.l_fin = lat
        self.lat_fin = lat_fin
        self.nav = nav
        vel = v_max or (nav.p['v_avance'] if signo > 0 else nav.p['v_retro'])
        self.ctrl = Recta(h, signo, self.a_fin, self.lateral, v_max=vel)
        self.ultima = None
        self.version = nav.con.version

    def lateral(self, a):
        """
        Lateral objetivo en la coordenada a: lo más cerca del preferido (el eje, o el corrimiento
        para un giro o un acople) dentro de lo libre de todas las celdas que el robot ocupa y de
        las dos que vienen (bloques chicos, soportes de cubos, canastos).
        """
        u = PASOS[self.h]
        n = PASOS[(self.h + 1) % 4]
        signo_eje = 1 if self.h < 2 else -1
        if self.lat_fin is not None:
            preferido = self.lat_fin - self.base_lat
        elif self.l_fin is not None and abs(self.a_fin - a) < 0.40:
            preferido = self.l_fin
        else:
            preferido = 0.0
        celdas = []
        for desp in (-0.14, 0.0, 0.14, 0.30 * self.signo, 0.45 * self.signo):
            p = a + desp
            if (p - self.a_fin) * self.signo > 0.16:
                continue        # más allá de donde termina la recta
            celda = celda_de(p * u[0] + self.base_lat * n[0], p * u[1] + self.base_lat * n[1])
            if celda not in celdas:
                celdas.append(celda)
        # Margen adaptable: 2,5 cm si entra; si no, menos (y el robot va más despacio).
        tramos = [(-1.0, 1.0)]
        self.ajustado = False
        for margen in (0.025, 0.018, 0.012, 0.008, 0.005):
            tramos = [(-1.0, 1.0)]
            for celda in celdas:
                libres = [(lo, hi) if signo_eje > 0 else (-hi, -lo)
                          for lo, hi in self.nav.con.libres_laterales(celda, self.h % 2, margen)]
                tramos = [(max(t0, l0), min(t1, l1)) for t0, t1 in tramos for l0, l1 in libres
                          if min(t1, l1) > max(t0, l0)]
                if not tramos:
                    break
            if tramos:
                self.ajustado = margen < 0.02
                break
        if not tramos:
            tramos = [(-1.0, 1.0)]
        mejor = min(tramos, key=lambda t: 0.0 if t[0] <= preferido <= t[1]
                    else min(abs(preferido - t[0]), abs(preferido - t[1])))
        return self.base_lat + max(mejor[0], min(mejor[1], preferido))

    def paso(self, nav, pose, rangos, dt):
        u = PASOS[self.h]
        celda = celda_de(pose[0], pose[1])
        if self.extender and celda != self.ultima:
            self.ultima = celda
            if celda == self.fin:
                nuevo = nav.extension(celda, self.h)
                if nuevo is not None:
                    self.fin, s, lat = nuevo
                    cx, cy = centro(self.fin)
                    self.a_fin = cx * u[0] + cy * u[1] + s
                    self.l_fin = lat
                    self.ctrl.a_fin = self.a_fin
        if nav.con.version != self.version and self.signo > 0:
            self.version = nav.con.version
            self._recortar(nav, pose)
        v, w, listo = self.ctrl.comando(pose)
        if getattr(self, 'ajustado', False):
            v = limitar(v, 0.06)
        return v, w, listo

    def _recortar(self, nav, pose):
        """Si se cerró o prohibió un paso por delante, frena en la última celda segura."""
        u = PASOS[self.h]
        celda = celda_de(pose[0], pose[1])
        a_celda = centro(celda)[0] * u[0] + centro(celda)[1] * u[1]
        a_pose = pose[0] * u[0] + pose[1] * u[1]
        if self.a_fin <= a_celda + 0.01:
            return
        if a_pose > a_celda + 0.02:
            # Ya pasó el centro: la celda de enfrente tiene que ser transitable.
            if not nav.con.transitable(celda, self.h):
                self.a_fin = max(a_pose, a_celda)
                self.ctrl.a_fin = self.a_fin
            return
        c = celda
        while centro(c)[0] * u[0] + centro(c)[1] * u[1] < self.a_fin - 0.17:
            if not nav.con.transitable(c, self.h):
                self.fin = c
                self.a_fin = centro(c)[0] * u[0] + centro(c)[1] * u[1]
                self.ctrl.a_fin = self.a_fin
                self.l_fin = None
                return
            c = vecina(c, self.h)


class TMover(Tarea):
    """Avanza (d > 0) o retrocede d metros en línea recta manteniendo el rumbo con que arrancó."""

    nombre = 'mover'

    def __init__(self, d, v=0.06):
        self.d = d
        self.v = v
        self.inicio = None

    def paso(self, nav, pose, rangos, dt):
        if self.inicio is None:
            self.inicio = tuple(pose)
        hecho = (pose[0] - self.inicio[0]) * math.cos(self.inicio[2]) + \
            (pose[1] - self.inicio[1]) * math.sin(self.inicio[2])
        falta = self.d - hecho
        if abs(falta) < 0.003 or falta * self.d < 0:
            return 0.0, 0.0, True
        v = math.copysign(max(0.025, min(self.v, 1.5 * abs(falta))), falta)
        w = limitar(3.0 * ang_dif(self.inicio[2], pose[2]), 0.5)
        return v, w, False


class TGiro(Tarea):
    """
    Giro en el lugar. Si se pide `sobrepaso`, se pasa ese ángulo y vuelve: como el ultrasonido de
    frente llega barriendo desde un lado y el sobrepaso cubre el otro, la celda nueva queda barrida
    entera sin pararse (reemplaza al barrido de ±24°).
    """
    nombre = 'girar'

    def __init__(self, yaw, nav, celda=None, h_nuevo=None, sentido=0, sobrepaso=0.0):
        self.yaw = yaw
        self.objetivos = ([yaw + math.copysign(sobrepaso, sentido)] if sobrepaso and sentido
                          else []) + [yaw]
        self.ctrl = Giro(self.objetivos.pop(0), w_max=nav.p['w_giro'])
        self.barrido = Barrido(celda, h_nuevo) if celda is not None and sentido else None
        self.sentido = sentido
        self.completo = bool(sobrepaso and sentido)

    def paso(self, nav, pose, rangos, dt):
        if self.barrido is not None and abs(ang_dif(pose[2], self.yaw)) < 0.6:
            self.barrido.agregar(pose, rangos.get('frente', math.inf))
        v, w, listo = self.ctrl.comando(pose)
        if listo and self.objetivos:
            self.ctrl = Giro(self.objetivos.pop(0), w_max=nav.p['w_giro'])
            return v, w, False
        if listo and self.barrido is not None:
            # Girando a la izquierda el sensor llega desde la derecha: cubrió el lado negativo.
            cubierto = 2 if self.completo else -self.sentido
            nav.barrido_previo = (self.barrido.celda, self.barrido.h, self.barrido, cubierto)
        return v, w, listo


class TBarrido(Tarea):
    """Gira ±ang en el lugar leyendo el ultrasonido de frente (celda de enfrente)."""

    nombre = 'barrer'

    def __init__(self, nav, celda, h, ang, barrido=None, cubierto=0):
        self.barrido = barrido or Barrido(celda, h)
        base = yaw_de(h)
        if cubierto < 0:
            self.objetivos = [base + ang, base]
        elif cubierto > 0:
            self.objetivos = [base - ang, base]
        else:
            self.objetivos = [base - ang, base + ang, base]
        self.giro = None
        self.quieto = 0.0
        self.cuadros0 = 0

    def paso(self, nav, pose, rangos, dt):
        self.barrido.agregar(pose, rangos.get('frente', math.inf))
        if self.quieto > 0:
            # Quieto en un extremo: la cámara también mira (cubos a los costados del pasillo).
            if nav.cuadros - self.cuadros0 >= 2 or self.quieto > 1.0:
                self.quieto = 0.0
            else:
                self.quieto += dt
                return 0.0, 0.0, False
        if self.giro is None:
            if not self.objetivos:
                nav.fin_barrido(self.barrido)
                return 0.0, 0.0, True
            self.giro = Giro(self.objetivos.pop(0), w_max=0.9, w_min=0.2)
        v, w, listo = self.giro.comando(pose)
        if listo:
            self.giro = None
            if self.objetivos and nav.p['usar_camara'] and nav.p['mirar_en_barrido']:
                self.quieto = 1e-6
                self.cuadros0 = nav.cuadros
        return v, w, False


class TMirar(Tarea):
    """Lleva la cámara a cada ángulo de la lista y espera `cuadros` imágenes en cada uno."""

    nombre = 'mirar'

    def __init__(self, pans, cuadros=3, t_max=2.5):
        self.pans = list(pans)
        self.cuadros = cuadros
        self.t_max = t_max
        self.pan = self.pans[0] if self.pans else 0.0
        self.t = 0.0
        self.inicio_cuadros = None

    def paso(self, nav, pose, rangos, dt):
        if not self.pans:
            return 0.0, 0.0, True
        self.pan = self.pans[0]
        self.t += dt
        if abs(nav.pan_real - self.pan) < 0.05 or self.t > self.t_max:
            if self.inicio_cuadros is None:
                self.inicio_cuadros = nav.cuadros
            if nav.cuadros - self.inicio_cuadros >= self.cuadros or self.t > self.t_max + 1.0:
                self.pans.pop(0)
                self.t = 0.0
                self.inicio_cuadros = None
        return 0.0, 0.0, False


class TManipular(Tarea):
    """Pide agarrar o soltar y espera la respuesta del brazo."""

    def __init__(self, nav, accion, color, rel):
        self.nombre = accion
        self.accion, self.color, self.rel = accion, color, rel
        self.enviado = False
        self.t = 0.0

    def paso(self, nav, pose, rangos, dt):
        if not self.enviado:
            nav.manip_resultado = None
            nav.eventos.append((self.accion, self.color, self.rel))
            nav.registro(f'{self.accion} {self.color} en {tuple(round(c, 3) for c in self.rel)}')
            self.enviado = True
        self.t += dt
        if nav.manip_resultado is not None or self.t > nav.p['tiempo_manipular']:
            nav.fin_manipulacion(self.accion, self.color, bool(nav.manip_resultado))
            return 0.0, 0.0, True
        return 0.0, 0.0, False


# ---------------------------------------------------------------------------------------------
# Navegador
# ---------------------------------------------------------------------------------------------
class Navegador:

    def __init__(self, parametros=None, significado=None, camara=None):
        self.p = dict(PARAMETROS)
        self.p.update(parametros or {})
        # Qué cubos indica cada color de flecha (gris = nada). Parámetro, como en laberintos.md.
        self.significado = significado or {'rojo': ['rojo'], 'verde': ['verde'], 'azul': ['azul'],
                                           'gris': []}
        self.con = Conocimiento(self.p['margen_giro'])
        self.est = Estimador()
        self.est.factor_giro = self.p['factor_giro']
        self.lateral = RegistroLateral()
        self.costos = Costos(self.p['v_avance'], self.p['v_retro'], self.p['w_giro'])
        self.cam = camara or Camara()
        self.t = None
        self.t0 = None
        self.estado = 'ESPERA'
        self.tarea = None
        self.cola = []
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.pan_real = 0.0
        self.cuadros = 0
        self.eventos = []
        self.manip_resultado = None
        self.mensajes = []
        # Memoria de la misión
        self.cubos = {}             # color -> Observacion
        self.canastos = {}          # color -> Observacion
        self.pistas = {}            # celda -> {color: (dir, largo)}
        self.sosteniendo = None
        self.entregados = []
        self.descartados = {}       # objetivo -> versión del mapa en que no se llegó
        # Sensores
        self.sospechas = {}
        self.vistas_lejos = set()
        self.barridos = set()
        self.inspecciones = {}
        self.miradas = set()
        self.votos_zona = {}
        self.vistas_flecha = {}
        self.ultimo_objetivo = None
        self.barrido_previo = None
        self.lentas = set()
        self.cajas_objeto = {}      # (tipo, color) -> (caja, celdas donde se guardó)
        self.franjas = {}
        self.reacomodos = {}
        self.segundas_oportunidades = 0
        self.trabados = {}          # (celda, dirección) -> veces que el robot se trabó ahí
        self._ref_mov = None        # (t, odometría) del último avance real
        self._ultimo_trabado = None  # (t, girando)
        # Puntos guardados (marco map): la partida es (0, 0) con rumbo 0.
        self.entrada = None         # (celda, rumbo) donde el robot entró al laberinto
        self.bifurcacion = None     # (celda, rumbo) de la primera bifurcación adentro
        self.ir_a_entrada = False
        self._dentro = (None, set())

    # ---------- registro ----------

    def registro(self, texto):
        marca = 0.0 if self.t is None or self.t0 is None else self.t - self.t0
        self.mensajes.append((marca, texto))

    # ---------- entrada principal ----------

    def actualizar(self, t, odom, rangos, pan_real=0.0):
        dt = 0.0 if self.t is None else max(0.0, t - self.t)
        self.t = t
        if self.t0 is None:
            self.t0 = t
        self.pan_real = pan_real
        self.est.odometria(*odom)
        trabado = self._vigilar_trabado(odom)
        pose = self.est.pose
        girando = isinstance(self.tarea, (TGiro, TBarrido))
        if not girando:
            self.est.corregir(self.con, rangos, self.v_cmd)
            pose = self.est.pose
        self._registrar(pose, rangos, girando)
        if self.estado == 'ESPERA':
            self.estado = 'INICIO'
            self.registro('arranco')
            if self.p['inspeccion_inicial']:
                self.cola = self._inspeccion_inicial()
        if self.estado == 'FIN':
            self.v_cmd = 0.0
            return 0.0, 0.0, 0.0
        if trabado:
            self._destrabar(pose)
        if self.tarea is None:
            self.tarea = self.cola.pop(0) if self.cola else self._decidir()
            if self.tarea is None:
                return 0.0, 0.0, 0.0
        v, w, listo = self.tarea.paso(self, pose, rangos, dt)
        pan = getattr(self.tarea, 'pan', 0.0)
        if listo:
            self.tarea = None
        # Seguridad: algo a menos de 5 cm adelante que no es la pared conocida donde va a frenar.
        f = rangos.get('frente', math.inf)
        if v > 0 and f < 0.05 and not self._pared_esperada(pose, f):
            self.registro(f'freno de emergencia: frente a {f:.3f} m')
            c = celda_de(pose[0], pose[1])
            h = rumbo_de(pose[2])
            self.barridos.discard((c, h))
            self.sospechas[vecina(c, h)] = self.sospechas.get(vecina(c, h), 0) + 2
            self.tarea = None
            self.cola = []
            v = 0.0
        elif self.sosteniendo:
            v, w = self._limitar_carga(v, w, dt)
        self.v_cmd = v
        self.w_cmd = w
        return v, w, pan

    def _vigilar_trabado(self, odom):
        """
        True si se pide avanzar o retroceder y las ruedas no se mueven hace 0,8 s: el robot tocó
        algo que no está en el mapa (un pilar que los ultrasonidos no vieron, un roce de esquina).
        Con los motores frenados contra un obstáculo los encoders casi no cuentan.
        """
        moviendo = abs(self.v_cmd) >= 0.02
        girando = not moviendo and abs(self.w_cmd) >= 0.08
        if not (moviendo or girando) or self._ref_mov is None:
            self._ref_mov = (self.t, odom)
            return False
        t0, o0 = self._ref_mov
        if math.hypot(odom[0] - o0[0], odom[1] - o0[1]) > 0.01 or \
                abs(ang_dif(odom[2], o0[2])) > 0.03:
            self._ref_mov = (self.t, odom)
            return False
        return self.t - t0 > 0.8

    def _destrabar(self, pose):
        """
        Sale de un trabado y aprende de él:
        - girando: vuelve al eje (girar para el otro lado); si eso también se traba, se mueve 3 cm;
        - avanzando o retrocediendo: vuelve 5 cm, encarece el paso y lo barre de nuevo; a la
          tercera vez en el mismo paso, lo da por cerrado (no entra, aunque el mapa diga que sí);
        - si estaba en la maniobra de un giro, a la segunda vez ese giro se marca imposible.
        """
        c = celda_de(pose[0], pose[1])
        h = rumbo_de(pose[2])
        clave = getattr(self.tarea, 'clave_giro', None)
        if clave is not None:
            self.trabados[clave] = self.trabados.get(clave, 0) + 1
            if self.trabados[clave] >= 2:
                self.registro(f'me trabé dos veces girando en {clave[0]}: giro imposible')
                self.con.giros_imposibles.add(clave)
        previo, self._ultimo_trabado = self._ultimo_trabado, (self.t, abs(self.v_cmd) < 0.02)
        self.cola = []
        self._ref_mov = None
        self.con.version += 1
        if abs(self.v_cmd) < 0.02:
            if previo and previo[1] and self.t - previo[0] < 4.0:
                # Tampoco puede volver girando: está apretado contra algo, se aleja en recta.
                atras = (h + 2) % 4
                d = -0.03 if self.con.transitable(c, atras) else 0.03
                self.registro(f'giro trabado otra vez en {c}: me muevo {d * 100:.0f} cm')
                self.tarea = TMover(d)
                return
            self.registro(f'giro trabado en {c}: vuelvo al eje y busco otro camino')
            self.tarea = TGiro(yaw_de(h), self)
            return
        sentido = 1 if self.v_cmd > 0 else -1
        d = h if sentido > 0 else (h + 2) % 4
        # Paso donde tocó: el de la celda adonde llegó el borde de adelante (o de atrás).
        borde = HUELLA_X[1] if sentido > 0 else -HUELLA_X[0]
        punta = celda_de(pose[0] + (borde + 0.01) * PASOS[d][0],
                         pose[1] + (borde + 0.01) * PASOS[d][1])
        paso = (c, d) if punta != c else None
        for k in ((c, d), (vecina(c, d), (d + 2) % 4)):
            self.trabados[k] = self.trabados.get(k, 0) + 1
        self.registro(f'trabado en {c} yendo hacia {d}: retrocedo y busco otro camino')
        # Solo hacia una celda donde nunca entró: si ya pasó por ahí, el paso existe y lo que
        # falló es la pose.
        if paso and self.trabados[paso] >= 3 and self.con.transitable(*paso) and \
                not self.con.visitada(vecina(*paso)):
            self.registro(f'el paso de {c} hacia {d} no da en la práctica: lo cierro')
            self.con.bloquear(*paso)
        self.sospechas[vecina(c, d)] = self.sospechas.get(vecina(c, d), 0) + 2
        self.barridos.discard((c, h))
        self.tarea = TMover(-0.05 * sentido)

    def _limitar_carga(self, v, w, dt):
        """Con un cubo apoyado en el 2.o piso: más despacio y sin tirones, para que no se caiga."""
        v = limitar(v, self.p['v_carga'])
        w = limitar(w, self.p['w_carga'])
        # Solo se suaviza el arranque (lo que aumenta la velocidad): las frenadas ya son suaves
        # en los controladores y achicarlas acá haría que el robot se pase de donde debe parar.
        if dt > 0 and abs(v) > abs(self.v_cmd):
            v = self.v_cmd + limitar(v - self.v_cmd, self.p['a_carga'] * dt)
        if dt > 0 and abs(w) > abs(self.w_cmd):
            w = self.w_cmd + limitar(w - self.w_cmd, self.p['alfa_carga'] * dt)
        return v, w

    def _pared_esperada(self, pose, f):
        """True si la lectura de frente corta es la pared conocida del borde de la celda."""
        if f < 0.008:
            return False
        _, cerrado, _ = leer_frente(pose, f, tolerancia=0.02)
        return cerrado is not None and self.con.abierto(*cerrado) is False

    def resultado_manipulacion(self, ok):
        self.manip_resultado = bool(ok)

    @property
    def pose(self):
        return tuple(self.est.pose)

    # ---------- sensores ----------

    def _registrar(self, pose, rangos, girando):
        x, y, th = pose
        h = rumbo_de(th)
        celda = celda_de(x, y)
        cx, cy = centro(celda)
        u, n = PASOS[h], PASOS[(h + 1) % 4]
        along = (x - cx) * u[0] + (y - cy) * u[1]
        lat = (x - cx) * n[0] + (y - cy) * n[1]
        if abs(along) < 0.10 and abs(lat) < 0.10:
            if not self.con.visitada(celda):
                self.con.visitar(celda)
            if self.entrada is None or self.bifurcacion is None:
                self._guardar_puntos(celda, h)
        if girando:
            return
        for c, d, abierto in self.lateral.actualizar(self.con, pose, rangos):
            if not abierto:
                self._revisar_costado(c, d)
        err = ang_dif(th, yaw_de(h))
        f = rangos.get('frente', math.inf)
        abiertos, cerrado, sospecha = leer_frente(pose, f)
        for c, d in abiertos:
            self.con.marcar(c, d, True, cerca=False)
        if cerrado:
            self.con.marcar(cerrado[0], cerrado[1], False, cerca=f < 0.5)
        if sospecha:
            c = sospecha[0]
            if c != celda and not self.con.visitada(c):
                self.sospechas[c] = self.sospechas.get(c, 0) + 1
        # Celdas vistas de lejos: el cono (±7,5°) cubre ±8 cm a partir de 0,61 m.
        if abs(err) < 0.04 and abs(lat) < 0.02:
            a_s = along + 0.1399
            for k in range(1, 4):
                cerca_borde = (k - 0.5) * CELDA - a_s
                lejos_borde = (k + 0.5) * CELDA - a_s
                if cerca_borde >= 0.58 and lejos_borde <= min(1.05, f - 0.05):
                    self.vistas_lejos.add((celda[0] + k * u[0], celda[1] + k * u[1]))
        # Lectura lateral corta contra una pared conocida: algo apoyado contra esa pared.
        self._revisar_lateral_corta(pose, rangos, h, err)

    def _revisar_lateral_corta(self, pose, rangos, h, err):
        """
        Algo apoyado contra una pared conocida (soporte de un cubo, bloque chico, canasto).

        Se compara cada ultrasonido lateral con lo que debería medir según el lateral estimado
        del robot (que se corrige con la pared del otro lado). El lado con algo apoyado mide 2,5 cm
        o más de menos; hacen falta 3 lecturas seguidas y el otro lado tiene que medir normal (si
        los dos miden distinto, el que está mal es el lateral estimado, no la pared).
        """
        if abs(err) > 0.08:
            self.cortas = {}
            return
        from caid_navigation.geometria import SONARES
        u, n = PASOS[h], PASOS[(h + 1) % 4]
        datos = {}
        for lado, d_rel, signo in (('izquierda', 1, 1), ('derecha', 3, -1)):
            r = rangos.get(lado, math.inf)
            sx, sy = a_mundo(pose, *SONARES[lado][:2])
            celda = celda_de(sx, sy)
            cx, cy = centro(celda)
            if abs((sx - cx) * u[0] + (sy - cy) * u[1]) > 0.06:
                continue
            d = (h + d_rel) % 4
            if self.con.abierto(celda, d) is not False or not math.isfinite(r):
                continue
            lat_s = (sx - cx) * n[0] + (sy - cy) * n[1]
            datos[lado] = (MEDIA - signo * lat_s - r, celda, d, (sx, sy))
        if len(datos) < 2:
            self.cortas = {}
            return
        cortos = [lado for lado, (dif, *_) in datos.items() if dif > 0.025]
        normales = [lado for lado, (dif, *_) in datos.items() if abs(dif) < 0.015]
        if len(cortos) != 1 or len(normales) != 1:
            self.cortas = {}
            return
        lado = cortos[0]
        self.cortas = {lado: getattr(self, 'cortas', {}).get(lado, 0) + 1}
        if self.cortas[lado] >= 3:
            dif, celda, d, punto = datos[lado]
            self._agregar_franja(celda, d, dif + 0.003, punto)

    def _agregar_franja(self, celda, d, prof, punto):
        """Caja contra la pared d de la celda, ±6 cm alrededor del punto donde la vio el sensor."""
        clave = ('franja', celda, d)
        if self.franjas.get(clave, 0) >= prof - 0.005:
            return
        self.franjas[clave] = prof
        cx, cy = centro(celda)
        u = PASOS[d]
        prof = min(prof, 0.10)
        if u[0]:
            x0 = cx + u[0] * MEDIA
            x1 = x0 - u[0] * prof
            y0, y1 = max(cy - MEDIA, punto[1] - 0.06), min(cy + MEDIA, punto[1] + 0.06)
            caja = (min(x0, x1), y0, max(x0, x1), y1)
        else:
            y0 = cy + u[1] * MEDIA
            y1 = y0 - u[1] * prof
            x0, x1 = max(cx - MEDIA, punto[0] - 0.06), min(cx + MEDIA, punto[0] + 0.06)
            caja = (x0, min(y0, y1), x1, max(y0, y1))
        # La franja medida con el ultrasonido lateral es más precisa que la caja del barrido (el
        # cono la agranda): reemplaza a las cajas de esa celda pegadas a la misma pared.
        previas = self.con.cajas.get(celda, [])
        for vieja in list(previas):
            if self._toca_pared(vieja, celda, d):
                previas.remove(vieja)
        self.con.agregar_caja(celda, caja)
        self.registro(f'algo apoyado en la pared {d} de {celda} ({prof * 100:.0f} cm): '
                      'no giro ahí')

    @staticmethod
    def _toca_pared(caja, celda, d):
        cx, cy = centro(celda)
        u = PASOS[d]
        if u[0]:
            borde = cx + u[0] * MEDIA
            return min(abs(caja[0] - borde), abs(caja[2] - borde)) < 0.05
        borde = cy + u[1] * MEDIA
        return min(abs(caja[1] - borde), abs(caja[3] - borde)) < 0.05

    def _revisar_costado(self, celda, d):
        pass

    # ---------- visión ----------

    def vision(self, t, det, pose, pan):
        """Procesa las detecciones de una imagen tomada con esa pose y ese ángulo de cámara."""
        if not self.p['usar_camara']:
            return
        self.cuadros += 1
        if abs(self.w_cmd) > 0.15 or abs(self.pan_real - getattr(self.tarea, 'pan', 0.0)) > 0.05:
            return      # girando: la pose de la imagen no es confiable
        for f in det.get('flechas', []):
            r = ubicar_flecha(self.con, self.cam, pose, pan, f)
            if r is None:
                continue
            colores = self.significado.get(r['color'], [])
            if not colores:
                continue
            clave = (r['celda'], r['color'], r['dir'], r['largo'])
            self.vistas_flecha[clave] = self.vistas_flecha.get(clave, 0) + 1
            if self.vistas_flecha[clave] < self.p['confirmar_flecha']:
                continue
            pistas = self.pistas.setdefault(r['celda'], {})
            for col in colores:
                previa = pistas.get(col)
                pistas[col] = (r['dir'], r['largo'])
                if previa != pistas[col]:
                    self.con.version += 1
                    self.registro(f'flecha {r["color"]} en {r["celda"]}: {col} hacia {r["dir"]}, '
                                  f'largo {r["largo"]} (cuerpo {r["cuerpo"] * 1000:.0f} mm)')
        for c in det.get('cubos', []):
            if not forma_de_cubo(self.cam, c['caja']):
                continue
            xy = ubicar_objeto(self.con, self.cam, pose, pan, c['caja'], CUBO_BASE_Z, CUBO_LADO)
            if xy is None or xy[2] > 1.6:
                continue
            # Un canasto visto de cerca puede parecer un cubo: cerca de un canasto conocido no
            # se acepta un cubo (además ahí van a parar los cubos entregados).
            if any(math.hypot(o.x - xy[0], o.y - xy[1]) < 0.35 for o in self.canastos.values()
                   if o.n >= self.p['confirmar_objeto']):
                continue
            self._observar(self.cubos, c['color'], xy, 'cubo')
        for c in det.get('canastos', []):
            xy = ubicar_objeto(self.con, self.cam, pose, pan, c['caja'], CANASTO_BASE_Z,
                               CANASTO_ANCHO)
            if xy is not None and xy[2] < 1.8:
                self._observar(self.canastos, c['color'], xy, 'canasto')
        puntos = det.get('zonas', [])
        if puntos:
            for celda, _ in zonas_en_suelo(self.con, self.cam, pose, pan, puntos).items():
                self.votos_zona[celda] = self.votos_zona.get(celda, 0) + 1
                if self.votos_zona[celda] >= self.p['confirmar_zona'] and self.con.prohibir(celda):
                    self.registro(f'zona prohibida en {celda}')
        # Celdas cuyo piso se vio (para no entrar a una celda sin haber buscado una zona roja).
        x, y = pose[0], pose[1]
        c0 = celda_de(x, y)
        for i in range(-4, 5):
            for j in range(-4, 5):
                c = (c0[0] + i, c0[1] + j)
                if c in self.con.bloques or self.con.visitada(c):
                    continue
                if celda_visible(self.con, self.cam, pose, pan, c):
                    self.inspecciones[c] = self.inspecciones.get(c, 0) + 1

    def _observar(self, tabla, color, xy, tipo):
        if tipo == 'cubo' and (color in self.entregados or color == self.sosteniendo):
            return
        obs = tabla.get(color)
        if obs is None or math.hypot(obs.x - xy[0], obs.y - xy[1]) > 0.25 and obs.n < 2:
            tabla[color] = Observacion(xy[0], xy[1])
        elif math.hypot(obs.x - xy[0], obs.y - xy[1]) <= 0.25:
            obs.agregar(xy[0], xy[1])
            if obs.n == self.p['confirmar_objeto']:
                self.registro(f'{tipo} {color} en ({obs.x:.2f}, {obs.y:.2f}), celda '
                              f'{celda_de(obs.x, obs.y)}')
                self.con.version += 1
            if obs.n >= self.p['confirmar_objeto']:
                # Debajo del cubo está su pilar (cilindro de 40 mm): +2 mm por lado. El canasto
                # mide 44 mm: +4 mm por lado. La caja sigue al promedio de lo que se ve.
                self._caja_objeto((tipo, color), obs, 0.044 if tipo == 'cubo' else 0.052)

    def _caja_objeto(self, clave, obs, lado):
        previa = self.cajas_objeto.get(clave)
        if previa:
            caja, celdas = previa
            cx, cy = (caja[0] + caja[2]) / 2, (caja[1] + caja[3]) / 2
            if math.hypot(cx - obs.x, cy - obs.y) < 0.005:
                return
            for c in celdas:
                if caja in self.con.cajas.get(c, []):
                    self.con.cajas[c].remove(caja)
        caja = (obs.x - lado / 2, obs.y - lado / 2, obs.x + lado / 2, obs.y + lado / 2)
        # En todas las celdas que toca: visto de lejos puede caer sobre el borde de la celda.
        celdas = {celda_de(x, y) for x in (caja[0], caja[2]) for y in (caja[1], caja[3])}
        celdas = [c for c in celdas if c not in self.con.bloques] or [celda_de(obs.x, obs.y)]
        for c in celdas:
            self.con.agregar_caja(c, caja)
        self.cajas_objeto[clave] = (caja, celdas)

    # ---------- decisiones ----------

    def _inspeccion_inicial(self):
        """Mira alrededor de la partida para encontrar los canastos (quedan detrás del robot)."""
        tareas = [TMirar([1.3, -1.3], cuadros=3)]
        if not self.p['usar_camara']:
            return []
        return tareas + [TInspeccionAtras()]

    def objetivo(self):
        """El objetivo de la misión ahora: dict con 'tipo' y lo necesario para planificar."""
        if self.t - self.t0 > self.p['tiempo_max'] and self.estado != 'VOLVER':
            self.registro('se acabó el tiempo: vuelvo al inicio')
            self.estado = 'VOLVER'
        if self.estado == 'VOLVER':
            return {'tipo': 'inicio'}
        if self.sosteniendo:
            obs = self.canastos.get(self.sosteniendo)
            if obs and obs.n >= self.p['confirmar_objeto']:
                obj = {'tipo': 'canasto', 'color': self.sosteniendo, 'obs': obs}
                if not self._descartado(('canasto', self.sosteniendo)) and self._metas_acople(obj):
                    return obj
                # Se conoce el canasto pero no su entorno: explorar cerca de él.
                return {'tipo': 'explorar', 'cerca': (obs.x, obs.y)}
            return {'tipo': 'explorar'}
        faltan = [c for c in COLORES if c not in self.entregados]
        if not faltan:
            return {'tipo': 'reposo'} if self.p['reposo_al_final'] else {'tipo': 'inicio'}
        x, y = self.est.pose[0], self.est.pose[1]
        candidatos = sorted(
            (math.hypot(o.x - x, o.y - y), col) for col, o in self.cubos.items()
            if col in faltan and o.n >= self.p['confirmar_objeto']
            and not self._descartado(('cubo', col)))
        if candidatos:
            col = candidatos[0][1]
            return {'tipo': 'cubo', 'color': col, 'obs': self.cubos[col]}
        if self.ir_a_entrada and self.entrada and not self._descartado(('entrada', None)):
            return {'tipo': 'entrada'}
        return {'tipo': 'explorar'}

    def _reingreso(self):
        """(celda, rumbo) adonde vuelve después de entregar: entrada o primera bifurcación."""
        if self.p['punto_de_reingreso'] == 'bifurcacion' and self.bifurcacion:
            return self.bifurcacion
        return self.entrada

    def dentro(self):
        """
        Celdas "adentro" del laberinto: las que se alcanzan desde la entrada sin volver a salir
        por donde se entró. Vacío mientras no se conoce la entrada.
        """
        if self.entrada is None:
            return set()
        if self._dentro[0] == self.con.version:
            return self._dentro[1]
        celda, h = self.entrada
        salida = (celda, (h + 2) % 4)
        vistos = {celda}
        pila = [celda]
        while pila:
            c = pila.pop()
            for d in range(4):
                if (c, d) == salida or not self.con.transitable(c, d):
                    continue
                n = vecina(c, d)
                if n not in vistos:
                    vistos.add(n)
                    pila.append(n)
        self._dentro = (self.con.version, vistos)
        return vistos

    def _guardar_puntos(self, celda, h):
        """Entrada: primera celda con pared a los dos costados. Bifurcación: la primera adentro."""
        lados = [self.con.abierto(celda, d) for d in range(4)]
        if None in lados or celda == (0, 0):
            return
        if self.entrada is None:
            if lados[(h + 1) % 4] is False and lados[(h + 3) % 4] is False:
                self.entrada = (celda, h)
                self.registro(f'entrada al laberinto guardada: celda {celda}, rumbo {h}')
        elif self.bifurcacion is None and sum(lados) >= 3 and celda in self.dentro():
            self.bifurcacion = (celda, h)
            self.registro(f'primera bifurcación guardada: celda {celda}, rumbo {h}')

    def _descartado(self, clave):
        return self.descartados.get(clave) == self.con.version

    def _metas(self, obj):
        """(es_objetivo, bonus) del planificador para un objetivo."""
        con = self.con
        if obj['tipo'] == 'inicio':
            return (lambda c, h: c == (0, 0)), None
        if obj['tipo'] == 'entrada':
            celda = self._reingreso()[0]
            return (lambda c, h: c == celda), None
        if obj['tipo'] == 'explorar':
            def es(c, h):
                return self._explorable(c)

            cerca = obj.get('cerca')

            dentro = self.dentro()

            def bonus(c, h):
                if cerca:
                    cx, cy = centro(c)
                    return -8.0 * math.hypot(cx - cerca[0], cy - cerca[1]) / CELDA
                b = 0.0
                if dentro and not any(vecina(c, d) in dentro for d in range(4)) \
                        and c not in dentro:
                    b -= self.p['penal_afuera']     # afuera del laberinto: al final
                if self.entrada is None and (c[1] != 0 or c[0] < 1):
                    # Supuesto (P): el robot arranca mirando hacia el laberinto. Hasta encontrar
                    # la entrada sigue derecho; el resto del área de partida, al final.
                    b -= self.p['penal_afuera']
                if not con.junto_a_pared(c):
                    b -= self.p['penal_abierto']
                elif abs(c[0]) + abs(c[1]) > self.p['radio_abierto'] and sum(
                        vecina(c, d) in con.bloques for d in range(4)) < 2:
                    # Menos de dos paredes al lado: sala o afuera del laberinto (recorriendo el
                    # muro exterior). Primero los pasillos.
                    b -= self.p['penal_sala']
                if max(abs(c[0]), abs(c[1])) <= 1 and c != (1, 0):
                    # Alrededor de la partida: ya se miró con la cámara en la inspección inicial y
                    # girar ahí (sin paredes cerca) acumula error de rumbo. Se deja para el final.
                    b -= self.p['penal_partida']
                return b
            return es, bonus
        if obj['tipo'] in ('cubo', 'canasto'):
            metas = self._metas_acople(obj)
            return (lambda c, h: (c, h) in metas), None
        return (lambda c, h: False), None

    def _explorable(self, c):
        con = self.con
        if con.visitada(c) or c in con.prohibidas or c not in con.libres or c in con.bloques:
            return False
        if any(math.hypot(o.x - centro(c)[0], o.y - centro(c)[1]) < 0.2
               for o in self.canastos.values() if o.n >= self.p['confirmar_objeto']):
            return False
        if con.junto_a_pared(c):
            return True
        if self.entrada is None and c[1] == 0 and c[0] >= 1:
            return True     # supuesto (P): arranca mirando al laberinto; la entrada está adelante
        return abs(c[0]) + abs(c[1]) <= self.p['radio_abierto']

    def costo_extra(self, celda, d):
        trabas = self.p['penal_trabado'] * self.trabados.get((celda, d), 0)
        return trabas + self._costo_flecha(celda, d)

    def _costo_flecha(self, celda, d):
        pistas = self.pistas.get(celda)
        if not pistas:
            return 0.0
        faltan = [c for c in COLORES if c not in self.entregados and c != self.sosteniendo]
        if self.sosteniendo or not faltan:
            return 0.0
        dirs = {pistas[c][0] for c in faltan if c in pistas}
        if not dirs or d in dirs:
            return 0.0
        return self.p['penal_flecha']

    def _acople(self, obj):
        """Poses (x, y, h) donde el robot agarra el cubo o suelta en el canasto."""
        obs = obj['obs']
        poses = []
        if obj['tipo'] == 'cubo':
            celda = celda_de(obs.x, obs.y)
            cx, cy = centro(celda)
            dx, dy = obs.x - cx, obs.y - cy
            if max(abs(dx), abs(dy)) < 0.06:
                hs = range(4)
            else:
                dw = (0 if dx > 0 else 2) if abs(dx) > abs(dy) else (1 if dy > 0 else 3)
                hs = ((dw + 1) % 4, (dw + 3) % 4)
            for h in hs:
                u = PASOS[h]
                a = obs.x * u[0] + obs.y * u[1] - self.p['agarre_dx']
                n = PASOS[(h + 1) % 4]
                lat = cx * n[0] + cy * n[1]
                poses.append((a * u[0] + lat * n[0], a * u[1] + lat * n[1], h))
        else:
            for h in range(4):
                u = PASOS[h]
                px = obs.x + self.p['deposito_dist'] * u[0]
                py = obs.y + self.p['deposito_dist'] * u[1]
                poses.append((px, py, h))
        return poses

    def _metas_acople(self, obj):
        metas = set()
        for px, py, h in self._acople(obj):
            c = celda_de(px, py)
            if c in self.con.bloques or c in self.con.prohibidas:
                continue
            if not self._acople_libre(obj, px, py, h):
                continue
            metas.add((c, h))
            metas.add((vecina(c, (h + 2) % 4), h))
            metas.add((vecina(c, h), h))
        return {(c, h) for c, h in metas if c in self.con.libres and c not in self.con.bloques}

    def _acople_libre(self, obj, px, py, h):
        """La pose de agarre o descarga no puede tocar nada salvo el objeto (o su soporte)."""
        obs = obj['obs']
        propias = []
        c0 = celda_de(obs.x, obs.y)
        cercanas = [((c0[0] + i, c0[1] + j), caja) for i in (-1, 0, 1) for j in (-1, 0, 1)
                    for caja in self.con.cajas.get((c0[0] + i, c0[1] + j), [])]
        for celda_obj, caja in cercanas:
            # El soporte está debajo del objeto. Medido desde lejos con unos grados de error de
            # rumbo puede aparecer corrido unos centímetros (y dos veces): todo lo que queda a
            # menos de 6 cm del objeto se toma como su soporte.
            dx = max(caja[0] - obs.x, 0.0, obs.x - caja[2])
            dy = max(caja[1] - obs.y, 0.0, obs.y - caja[3])
            if math.hypot(dx, dy) < 0.06:
                propias.append((celda_obj, caja))
        for celda_obj, caja in propias:
            self.con.cajas[celda_obj].remove(caja)
        try:
            return self.con.huella_libre(px, py, yaw_de(h), 0.008)
        finally:
            for celda_obj, caja in propias:
                self.con.cajas[celda_obj].append(caja)

    def _decidir(self):
        pose = self.est.pose
        c = celda_de(pose[0], pose[1])
        h = rumbo_de(pose[2])
        if abs(ang_dif(pose[2], yaw_de(h))) > 0.08:
            return TGiro(yaw_de(h), self)
        # Mirar los costados nuevos en un cruce (cubos y flechas en las ramas).
        if self.p['usar_camara'] and self.p['mirar_cruces'] and ('cruce', c) not in self.miradas \
                and self.con.visitada(c):
            pans = []
            for rel, pan in ((1, 1.45), (3, -1.45)):
                d = (h + rel) % 4
                if self.con.transitable(c, d) and not self.con.visitada(vecina(c, d)):
                    pans.append(pan)
            self.miradas.add(('cruce', c))
            if pans:
                return TMirar(pans + [0.0], cuadros=3)
        obj = self.objetivo()
        if obj['tipo'] == 'reposo':
            self.registro('misión terminada: 3 cubos entregados, quedo en reposo junto al canasto')
            self.estado = 'FIN'
            return None
        es, bonus = self._metas(obj)
        plan = planificar(self.con, (c, h), es, self.costos, self.costo_extra, bonus)
        clave = (obj['tipo'], obj.get('color'))
        if clave != self.ultimo_objetivo:
            self.ultimo_objetivo = clave
            self.registro(f'objetivo: {obj["tipo"]} {obj.get("color", "")}'.strip())
        if plan is None:
            if obj['tipo'] == 'entrada':
                self.descartados[('entrada', None)] = self.con.version
                return TEspera(0.05)
            if obj['tipo'] in ('cubo', 'canasto'):
                self.descartados[(obj['tipo'], obj['color'])] = self.con.version
                self.registro(f'no hay camino conocido a {obj["tipo"]} {obj["color"]}')
                return TEspera(0.05)
            if self._segunda_oportunidad():
                return TEspera(0.05)
            if obj['tipo'] == 'explorar':
                self.registro('no queda nada por explorar: vuelvo al inicio')
                self.estado = 'VOLVER'
                return TEspera(0.05)
            self.registro('no encuentro el camino al inicio: me detengo')
            self.estado = 'FIN'
            return None
        acciones, final, _ = plan
        if not acciones:
            return self._llegada(obj, c, h)
        return self._tareas(acciones, c, h)

    def _segunda_oportunidad(self):
        """
        Sin camino: los giros marcados imposibles fallaron por cómo llegó el robot esa vez (pose,
        lateral), no por el mapa. Antes de rendirse se olvidan y se reintenta (hasta 2 veces).
        """
        if not self.con.giros_imposibles or self.segundas_oportunidades >= 2:
            return False
        self.segundas_oportunidades += 1
        self.registro(f'sin camino: olvido {len(self.con.giros_imposibles)} giros imposibles y '
                      f'vuelvo a probar')
        self.con.giros_imposibles.clear()
        self.reacomodos.clear()
        self.con.version += 1
        return True

    def _tareas(self, acciones, c, h):
        tipo = acciones[0][0]
        if tipo == 'avanzar':
            n = acciones[0][1]
            previa = self._previa(c, h, n)
            if previa is not None:
                return previa
            s, lat = 0.0, None
            if len(acciones) > 1 and acciones[1][0] == 'girar':
                s, lat = acciones[1][2]
            return TRecta(self, h, 1, n, s, lat, extender=True)
        if tipo == 'retroceder':
            k = 0
            while k < len(acciones) and acciones[k][0] == 'retroceder':
                k += 1
            fin = acciones[k - 1][1]
            s, lat = 0.0, None
            if k < len(acciones) and acciones[k][0] == 'girar':
                s, lat = acciones[k][2]
            return TRecta(self, h, -1, fin, s, lat)
        # Girar: con el lateral que tiene ahora el robot, buscar dónde frenar.
        nuevo = acciones[0][1]
        giro = {1: math.pi / 2, 3: -math.pi / 2, 2: math.pi}[(nuevo - h) % 4]
        pose = self.est.pose
        cx, cy = centro(c)
        u, n = PASOS[h], PASOS[(h + 1) % 4]
        s0 = (pose[0] - cx) * u[0] + (pose[1] - cy) * u[1]
        l0 = (pose[0] - cx) * n[0] + (pose[1] - cy) * n[1]
        corr = self.con.giro(c, h, giro, l_fijo=l0, s_actual=s0,
                             margen=self.p['margen_giro_ejecucion'])
        if corr is None:
            s, lat = acciones[0][2]
            maniobra = self._desplazamiento(c, h, s0, l0, s, lat)
            if maniobra is not None:
                self.registro(f'corrimiento lateral en {c}: {l0 * 100:.1f} -> {lat * 100:.1f} cm '
                              f'para girar')
                sentido = 1 if giro > 0 else -1
                tareas = maniobra + [TGiro(yaw_de(nuevo), self, c, nuevo,
                                           sentido if abs(giro) < 2 else 0)]
                for t in tareas:
                    t.clave_giro = (c, h, nuevo)
                self.cola = tareas[1:] + self.cola
                return tareas[0]
            # Con este lateral no entra: alejarse por el lado libre y volver a llegar corrido.
            self.registro(f'reacomodo para girar en {c} (lateral {l0 * 100:.1f} cm, '
                          f'pide {lat * 100:.0f} cm)')
            clave = (c, h, nuevo)
            self.reacomodos[clave] = self.reacomodos.get(clave, 0) + 1
            if self.reacomodos[clave] > 2:
                self.registro(f'giro en {c} imposible en la práctica: busco otro camino')
                self.con.giros_imposibles.add(clave)
                self.con.version += 1
                return TEspera(0.05)
            atras = (h + 2) % 4
            if self.con.transitable(c, atras):
                sentido = -1
            else:
                sentido = 1 if self.con.transitable(c, h) else 0
            if sentido == 0:
                self.registro(f'no hay lugar para reacomodarse en {c}: busco otro camino')
                self.con.giros_imposibles.add((c, h, nuevo))
                self.con.version += 1
                return TEspera(0.05)
            a_lejos = cx * u[0] + cy * u[1] + s + 0.30 * sentido
            volver = TRecta(self, h, -sentido, c, s, nombre='acomodar', v_max=0.10)
            volver.ctrl.k_lat = 10.0
            volver.lat_fin = (cx * n[0] + cy * n[1]) + lat + math.copysign(0.004, lat)
            ida = TRecta(self, h, sentido, c, a_fin=a_lejos, nombre='acomodar')
            ida.clave_giro = volver.clave_giro = clave
            self.cola = [volver]
            return ida
        tareas = []
        if abs(corr[0] - s0) > 0.006:
            signo = 1 if corr[0] > s0 else -1
            tareas.append(TRecta(self, h, signo, c, corr[0], lat_fin=(cx * n[0] + cy * n[1]) + l0,
                                 v_max=0.06, nombre='acomodar'))
        sentido = 1 if giro > 0 else -1
        tareas.append(TGiro(yaw_de(nuevo), self, c, nuevo, sentido if abs(giro) < 2 else 0,
                            self._sobrepaso(c, h, nuevo, giro, corr[0], l0)))
        for t in tareas:
            t.clave_giro = (c, h, nuevo)
        self.cola = tareas[1:] + self.cola
        return tareas[0]

    def _sobrepaso(self, c, h, nuevo, giro, s, lat):
        """Ángulo extra (hasta 15°) que entra en la celda al terminar el giro, o 0."""
        n = vecina(c, nuevo)
        if abs(giro) > 2 or self.con.visitada(n) or not self.p['barrer_celdas_nuevas']:
            return 0.0
        # Corrimiento del robot en el marco del rumbo nuevo.
        s2, l2 = (lat, -s) if (nuevo - h) % 4 == 1 else (-lat, s)
        ang = self.con.barrido_posible(c, nuevo, s2, l2, 0.26, self.p['margen_giro_ejecucion'])
        return ang if ang >= 0.17 else 0.0

    def _desplazamiento(self, c, h, s0, l0, s_obj, l_obj, angulo=0.35):
        """
        Corrimiento lateral en el lugar, como al estacionar: girar ±20°, avanzar (o retroceder) lo
        justo, volver a alinear y quedar en s_obj. Se prueba la huella en cada paso contra el mapa
        (paredes, celdas desconocidas y obstáculos). Devuelve la lista de tareas o None.
        """
        dl = l_obj - l0
        if abs(dl) < 0.004:
            return [TMover(s_obj - s0)] if abs(s_obj - s0) > 0.004 else []
        cx, cy = centro(c)
        u, n = PASOS[h], PASOS[(h + 1) % 4]
        base = yaw_de(h)
        margen = self.p['margen_giro_ejecucion']

        def punto(s_, l_):
            return cx + s_ * u[0] + l_ * n[0], cy + s_ * u[1] + l_ * n[1]

        def recta_libre(sa, sb, l_):
            pasos = max(2, int(abs(sb - sa) / 0.01))
            return all(self.con.huella_libre(*punto(sa + (sb - sa) * k / pasos, l_), base, margen)
                       for k in range(pasos + 1))
        # Primero, si hace falta, un poco adelante o atrás para tener lugar para girar 20°.
        for previo in (0.0, 0.03, -0.03, 0.06, -0.06):
            sp = s0 + previo
            if previo and not recta_libre(s0, sp, l0):
                continue
            for avance in (1, -1):
                giro = math.copysign(angulo, dl) * avance
                d = abs(dl) / math.sin(angulo) * avance
                if not all(self.con.huella_libre(*punto(sp, l0), base + giro * k / 6, margen)
                           for k in range(7)):
                    continue
                s1 = sp + d * math.cos(angulo)
                if not all(self.con.huella_libre(*punto(sp + (s1 - sp) * k / 10, l0 + dl * k / 10),
                                                 base + giro, margen) for k in range(11)):
                    continue
                if not all(self.con.huella_libre(*punto(s1, l_obj), base + giro * (1 - k / 6),
                                                 margen) for k in range(7)):
                    continue
                if not recta_libre(s1, s_obj, l_obj):
                    continue
                tareas = [TMover(previo)] if abs(previo) > 0.003 else []
                return tareas + [TGiro(base + giro, self), TMover(d), TGiro(base, self),
                                 TMover(s_obj - s1)]
        return None

    def _previa(self, c, h, n):
        """Lo que hay que hacer antes de entrar a la celda n: barrer o mirar."""
        if self.con.visitada(n):
            return None
        if self.p['barrer_celdas_nuevas'] and (c, h) not in self.barridos and (
                n not in self.vistas_lejos or self.sospechas.get(n, 0) >= 2):
            self.barridos.add((c, h))
            pose = self.est.pose
            cx, cy = centro(c)
            u, nn = PASOS[h], PASOS[(h + 1) % 4]
            s0 = (pose[0] - cx) * u[0] + (pose[1] - cy) * u[1]
            l0 = (pose[0] - cx) * nn[0] + (pose[1] - cy) * nn[1]
            ang = self.con.barrido_posible(c, h, s0, l0, self.p['angulo_barrido'],
                                           self.p['margen_giro_ejecucion'])
            previo = self.barrido_previo if self.barrido_previo and \
                self.barrido_previo[:2] == (c, h) else None
            self.barrido_previo = None
            if previo and previo[3] == 2:
                self.fin_barrido(previo[2])
                return TEspera(0.05)        # replanificar con lo que vio el giro
            if ang >= 0.17:
                if previo:
                    return TBarrido(self, c, h, ang, previo[2], previo[3])
                return TBarrido(self, c, h, ang)
            self.registro(f'sin lugar para barrer en {c}: entro despacio a {n}')
            self.lentas.add(n)
        if self.p['usar_camara'] and ('costados', c, h) not in self.miradas and \
                self._sin_mirada_previa(c, h) and self._faltan_cubos():
            # Los cubos están pegados a las paredes de los costados: con 60° de campo no se ven
            # hasta que están a 2 celdas. Al entrar a un pasillo nuevo se mira a los dos lados.
            self.miradas.add(('costados', c, h))
            return TMirar([0.75, -0.75, 0.0], cuadros=2)
        if self.p['usar_camara'] and self.inspecciones.get(n, 0) < 2 and \
                ('piso', n) not in self.miradas:
            self.miradas.add(('piso', n))
            pose = self.est.pose
            cx, cy = centro(n)
            ox, oy = a_mundo(pose, 0.094, 0.0)
            ang = ang_dif(math.atan2(cy - oy, cx - ox), pose[2])
            return TMirar([limitar(ang, 1.5), 0.0], cuadros=3)
        return None

    def _sin_mirada_previa(self, c, h):
        """True si el robot no viene derecho por este pasillo (entonces no vio los costados)."""
        atras = vecina(c, (h + 2) % 4)
        return not (self.con.visitada(atras) and self.con.visitada(vecina(atras, (h + 2) % 4))
                    and self.con.transitable(atras, h))

    def _faltan_cubos(self):
        faltan = [c for c in COLORES if c not in self.entregados and c != self.sosteniendo]
        return any(c not in self.cubos or self.cubos[c].n < self.p['confirmar_objeto']
                   for c in faltan)

    def extension(self, c, h):
        """Al entrar a la celda c en una recta: (celda_final, s, lat) para seguir o frenar."""
        obj = self.objetivo()
        es, bonus = self._metas(obj)
        plan = planificar(self.con, (c, h), es, self.costos, self.costo_extra, bonus)
        if plan is None or not plan[0]:
            return c, 0.0, None
        acciones = plan[0]
        if acciones[0][0] == 'avanzar':
            n = acciones[0][1]
            if self._necesita_previa(c, h, n):
                return c, 0.0, None
            if len(acciones) > 1 and acciones[1][0] == 'girar':
                return n, acciones[1][2][0], acciones[1][2][1]
            return n, 0.0, None
        if acciones[0][0] == 'girar':
            return c, acciones[0][2][0], acciones[0][2][1]
        return c, 0.0, None

    def _necesita_previa(self, c, h, n):
        if self.con.visitada(n):
            return False
        if self.p['usar_camara'] and ('costados', c, h) not in self.miradas and \
                self._sin_mirada_previa(c, h) and self._faltan_cubos():
            return True
        if self.p['barrer_celdas_nuevas'] and (c, h) not in self.barridos and (
                n not in self.vistas_lejos or self.sospechas.get(n, 0) >= 2):
            return True
        return self.p['usar_camara'] and self.inspecciones.get(n, 0) < 2 and \
            ('piso', n) not in self.miradas

    def _llegada(self, obj, c, h):
        if obj['tipo'] == 'entrada':
            self.ir_a_entrada = False
            self.registro(f'de vuelta en el punto de reingreso {c}: sigo explorando')
            return TEspera(0.05)
        if obj['tipo'] == 'inicio':
            self.registro('misión terminada: en la celda de partida')
            self.estado = 'FIN'
            return None
        if obj['tipo'] == 'explorar':
            return TEspera(0.05)
        poses = [p for p in self._acople(obj) if p[2] == h and self._acople_libre(obj, *p)]
        if not poses:
            self.descartados[(obj['tipo'], obj['color'])] = self.con.version
            return TEspera(0.05)
        pose = self.est.pose
        u, n = PASOS[h], PASOS[(h + 1) % 4]
        mejor = min(poses, key=lambda p: math.hypot(p[0] - pose[0], p[1] - pose[1]))
        a_obj = mejor[0] * u[0] + mejor[1] * u[1]
        lat_obj = mejor[0] * n[0] + mejor[1] * n[1]
        # Sin tocar las paredes de adelante o de atrás de la celda donde queda el robot.
        celda_obj = celda_de(mejor[0], mejor[1])
        a_c = centro(celda_obj)[0] * u[0] + centro(celda_obj)[1] * u[1]
        m = 0.012
        if self.con.abierto(celda_obj, h) is not True:
            a_obj = min(a_obj, a_c + MEDIA - 0.1399 - m)
        if self.con.abierto(celda_obj, (h + 2) % 4) is not True:
            a_obj = max(a_obj, a_c - MEDIA + 0.1382 + m)
        a = pose[0] * u[0] + pose[1] * u[1]
        signo = 1 if a_obj > a else -1
        acercar = TRecta(self, h, signo, c, a_fin=a_obj, lat_fin=lat_obj,
                         v_max=self.p['v_acercar'], nombre='acercar')
        obs = obj['obs']
        accion = 'agarrar' if obj['tipo'] == 'cubo' else 'soltar'
        self.cola = [TManipularDiferido(accion, obj['color'], obs)]
        return acercar

    def fin_barrido(self, barrido):
        pasable, desvio, pts = barrido.resultado()
        celda, h = barrido.celda, barrido.h
        n = barrido.objetivo
        if not pasable:
            self.con.bloquear(celda, h)
            self.registro(f'paso {celda} -> {n} cerrado por un obstáculo ({len(pts)} ecos)')
        elif len(barrido.ocupados_centrales()) >= 2:
            u, nn = PASOS[h], PASOS[(h + 1) % 4]
            cx, cy = centro(celda)
            for caja in self._cajas_de_barrido(pts):
                esquinas = [(cx + x * u[0] + y * nn[0], cy + x * u[1] + y * nn[1])
                            for x in (caja[0], caja[2]) for y in (caja[1], caja[3])]
                xs = [e[0] for e in esquinas]
                ys = [e[1] for e in esquinas]
                self.con.agregar_caja(n, (min(xs), min(ys), max(xs), max(ys)))
            self.registro(f'obstáculo en {n}: paso corrido {desvio * 100:.1f} cm')
        self.sospechas.pop(n, None)
        self.vistas_lejos.add(n)

    @staticmethod
    def _cajas_de_barrido(pts):
        """
        Cajas (marco de la celda del robot) de los obstáculos que vio un barrido.

        El ultrasonido solo ve la cara de adelante: la profundidad no se mide. Supuesto (P): los
        obstáculos son prismas de base cuadrada (bloques de 8 o 20 cm, soportes de 40 mm,
        canastos), así que la profundidad es igual al ancho medido. Un obstáculo que toca una pared
        lateral se extiende hasta esa pared.
        """
        cajas = []
        for lado in (1, -1):
            grupo = [p for p in pts if p[1] * lado > -0.02]
            if not grupo:
                continue
            interior = min(p[1] * lado for p in grupo)
            # Entre el obstáculo y la pared no pasa el robot (163 mm): se lo toma pegado a la
            # pared.
            exterior = MEDIA
            ancho = max(0.03, exterior - interior)
            x0 = min(p[0] for p in grupo) - 0.005
            y0, y1 = sorted((interior * lado, exterior * lado))
            cajas.append((x0, y0, x0 + ancho + 0.01, y1))
        return cajas

    def fin_manipulacion(self, accion, color, ok):
        if accion == 'agarrar':
            if ok:
                self.sosteniendo = color
                self.cubos.pop(color, None)
                self.registro(f'cubo {color} agarrado')
            else:
                self.descartados[('cubo', color)] = self.con.version
                self.registro(f'no pude agarrar el cubo {color}')
        else:
            if ok:
                self.entregados.append(color)
                self.sosteniendo = None
                self.registro(f'cubo {color} entregado ({len(self.entregados)}/3)')
                self.ir_a_entrada = self.p['volver_a_la_entrada'] and len(self.entregados) < 3
            else:
                self.descartados[('canasto', color)] = self.con.version
                self.registro(f'no pude soltar el cubo {color}')


class TEspera(Tarea):
    nombre = 'esperar'

    def __init__(self, t):
        self.t = t

    def paso(self, nav, pose, rangos, dt):
        self.t -= dt
        return 0.0, 0.0, self.t <= 0


class TManipularDiferido(Tarea):
    """Calcula la posición del objeto en base_link recién al llegar (la pose final real)."""

    def __init__(self, accion, color, obs):
        self.nombre = accion
        self.accion, self.color, self.obs = accion, color, obs
        self.interna = None

    def paso(self, nav, pose, rangos, dt):
        if self.interna is None:
            rx, ry = a_robot(pose, self.obs.x, self.obs.y)
            if self.accion == 'agarrar':
                z = CUBO_BASE_Z + CUBO_LADO / 2
            else:
                z = CANASTO_BASE_Z + 0.02
            self.interna = TManipular(nav, self.accion, self.color, (rx, ry, z))
        return self.interna.paso(nav, pose, rangos, dt)


class TInspeccionAtras(Tarea):
    """En la partida: gira 90° a cada lado y mira hacia atrás con la cámara (canastos)."""

    nombre = 'inspeccion'

    def __init__(self):
        self.pasos = None

    def paso(self, nav, pose, rangos, dt):
        if self.pasos is None:
            self.pasos = []
            c = celda_de(pose[0], pose[1])
            h = rumbo_de(pose[2])
            base = yaw_de(h)
            lados = []
            for giro, pans in ((math.pi / 2, [1.5, 0.75, 0.0]),
                               (-math.pi / 2, [-1.5, -0.75, 0.0])):
                corr = nav.con.giro(c, h, giro, l_fijo=0.0, s_actual=0.0)
                if corr is None or abs(corr[0]) > 0.04:
                    continue
                if abs(corr[0]) > 0.006:
                    cx, cy = centro(c)
                    n = PASOS[(h + 1) % 4]
                    self.pasos.append(TRecta(nav, h, 1 if corr[0] > 0 else -1, c, corr[0],
                                             lat_fin=cx * n[0] + cy * n[1], v_max=0.06,
                                             nombre='acomodar'))
                self.pasos += [TGiro(base + giro, nav), TMirar(pans, cuadros=3), TGiro(base, nav)]
                lados.append('izquierda' if giro > 0 else 'derecha')
            nav.registro(f'inspección inicial: {lados or "sin lugar para girar"}')
        while self.pasos:
            t = self.pasos[0]
            v, w, listo = t.paso(nav, pose, rangos, dt)
            self.pan = getattr(t, 'pan', 0.0)
            if listo:
                self.pasos.pop(0)
                continue
            return v, w, False
        return 0.0, 0.0, True
