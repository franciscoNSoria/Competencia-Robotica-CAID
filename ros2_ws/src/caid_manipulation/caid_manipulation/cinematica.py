"""
Cinemática del brazo (tipo EEZYbotARM), sin ROS.

Une tres cosas: los grados que se le mandan a cada servo (nominales: 500 us = 0, 2500 us = 180),
los ángulos de las articulaciones del URDF (caid_description/urdf/arm.xacro) y la posición de la
punta de la pinza en base_link. Es lo mismo que usan el brazo simulado (brazo_sim) y la carga
(carga_brazo), así el brazo de Gazebo y el real se mandan con los mismos números.

Calibración (docs/guias/brazo-calibracion.md), medida sobre las fotos del 04/11 con los trazos del
equipo: 8 fotos del barrido del hombro (codo fijo en 180) y 7 del barrido del codo (hombro fijo en
60), vistas de frente con el brazo girado al costado.

- Brazo principal: alfa = ALFA_0 - hombro. Pendiente -1 (transmisión directa: con 6 puntos da
  -1,044 +- 0,033). Hombro 100 y 115 se apartan 7 grados: el servo no llega con el brazo
  extendido (ya se había visto que el hombro no levanta solo).
- Antebrazo: beta = BETA_GANANCIA * codo + BETA_0, pendiente 0,833 +- 0,025 (rms 2,2 grados). No es
  1 porque el cuadrilátero del codo no es un paralelogramo perfecto. El antebrazo depende solo
  del codo: en el barrido del hombro se movió menos de 5 grados.
alfa y beta son ángulos absolutos desde la horizontal, hacia el lado de la pinza (90 = vertical,
negativo = hacia abajo).

Las longitudes salen del CAD (hardware/cad/dimensions.yaml). OJO: con la pose de agarre de las
fotos (hombro 104, codo 140) el CAD pone la punta 241 mm delante del eje del brazo y la foto de
costado muestra unos 200 mm. Hasta medirlo con regla, el desplazamiento muñeca -> punta es dato P.
"""
import math

# --- Calibración medida (grados) ---
ALFA_0 = 132.5            # alfa = ALFA_0 - hombro
BETA_GANANCIA = 0.833     # beta = BETA_GANANCIA * codo + BETA_0
BETA_0 = -139.2
# Base: 95 nominal = brazo hacia adelante. Ganancia y sentido sin medir (P): la base recorre más
# de 180 grados (136 a la izquierda y 120 a la derecha), así que no es 1 a 1.
BASE_REPOSO = 95.0
# Grados de giro por grado nominal (P): 256 grados medidos de tope a tope sobre 180 nominales.
BASE_GANANCIA = 1.42
BASE_SIGNO = 1.0          # +1: más grados nominales = hacia la izquierda (P, falta medir)
# Pinza (P): apertura del URDF (rad de la garra A) entre cerrada y abierta.
PINZA_CERRADA = 114.0
PINZA_ABIERTA = 150.0
GARRA_ABIERTA_RAD = -0.6

REPOSO = {'base': 95.0, 'hombro': 60.0, 'codo': 180.0, 'pinza': 114.0}
TOPES = {'base': (0.0, 180.0), 'hombro': (0.0, 180.0), 'codo': (0.0, 200.0),
         'pinza': (0.0, 180.0)}

# --- Geometría del CAD, marco del brazo (mm): x hacia la pinza, z hacia arriba ---
# Base del brazo en base_link (base_link: cara de arriba del 2.o piso)
BASE_X, BASE_Z = -21.3, 0.0
EJE_Z = 32.0
HOMBRO = (0.0, 66.7)
CODO = (5.9, 146.5)
MUNECA = (77.8, 111.5)
PUNTA = (174.7, 96.3)
PUNTA_Y = -4.0                        # la punta está 4 mm al costado del plano del brazo
L1 = math.hypot(CODO[0] - HOMBRO[0], CODO[1] - HOMBRO[1])          # 80,0 mm
L2 = math.hypot(MUNECA[0] - CODO[0], MUNECA[1] - CODO[1])          # 80,0 mm
ALFA_CAD = math.degrees(math.atan2(CODO[1] - HOMBRO[1], CODO[0] - HOMBRO[0]))  # 85,8
BETA_CAD = math.degrees(math.atan2(MUNECA[1] - CODO[1], MUNECA[0] - CODO[0]))  # -26,0
PINZA_DX, PINZA_DZ = PUNTA[0] - MUNECA[0], PUNTA[1] - MUNECA[1]    # la pinza queda horizontal


def alfa(hombro):
    return ALFA_0 - hombro


def beta(codo):
    return BETA_GANANCIA * codo + BETA_0


def servos_a_articulaciones(base, hombro, codo, pinza=PINZA_CERRADA):
    """Ángulos (rad) de las articulaciones del URDF para unos grados nominales de los servos."""
    t1 = ALFA_CAD - alfa(hombro)          # el eje +y baja la punta: alfa = ALFA_CAD - t1
    t2 = (BETA_CAD - beta(codo)) - t1                 # el codo es relativo al brazo principal
    t3 = -(t1 + t2)                       # muñeca virtual: la pinza no cambia de inclinación
    yaw = BASE_SIGNO * BASE_GANANCIA * (base - BASE_REPOSO)
    f = min(max((pinza - PINZA_CERRADA) / (PINZA_ABIERTA - PINZA_CERRADA), 0.0), 1.0)
    garra = GARRA_ABIERTA_RAD * f
    return {'arm_yaw_joint': math.radians(yaw), 'arm_shoulder_joint': math.radians(t1),
            'arm_elbow_joint': math.radians(t2), 'arm_wrist_joint': math.radians(t3),
            'arm_claw_a_joint': garra, 'arm_claw_b_joint': -garra}


def punta_plano(hombro, codo):
    """(r, z) de la punta en el plano del brazo (mm): r desde el eje de giro, z desde la base."""
    a, b = math.radians(alfa(hombro)), math.radians(beta(codo))
    r = HOMBRO[0] + L1 * math.cos(a) + L2 * math.cos(b) + PINZA_DX
    z = HOMBRO[1] + L1 * math.sin(a) + L2 * math.sin(b) + PINZA_DZ
    return r, z


def punta(base, hombro, codo):
    """Punta de la pinza en base_link (m)."""
    r, z = punta_plano(hombro, codo)
    psi = math.radians(BASE_SIGNO * BASE_GANANCIA * (base - BASE_REPOSO))
    c, s = math.cos(psi), math.sin(psi)
    return ((BASE_X + r * c - PUNTA_Y * s) / 1000.0, (r * s + PUNTA_Y * c) / 1000.0,
            (BASE_Z + z) / 1000.0)


def base_para(x, y):
    """Grados nominales de la base para que la punta pase por (x, y) de base_link (m)."""
    dx, dy = x * 1000.0 - BASE_X, y * 1000.0
    r = math.sqrt(max(dx * dx + dy * dy - PUNTA_Y * PUNTA_Y, 0.0))
    psi = math.degrees(math.atan2(dy, dx) - math.atan2(PUNTA_Y, r))
    return BASE_REPOSO + psi / (BASE_SIGNO * BASE_GANANCIA)


def ik_plano(r, z):
    """
    Hombro y codo (grados nominales) para poner la punta en (r, z) del plano del brazo (mm).

    Codo arriba, como trabaja el EEZYbotARM. None si no se llega.
    """
    wr, wz = r - PINZA_DX - HOMBRO[0], z - PINZA_DZ - HOMBRO[1]       # muñeca respecto del hombro
    d = math.hypot(wr, wz)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    c = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)
    a = math.atan2(wz, wr) + math.acos(max(-1.0, min(1.0, c)))
    ex, ez = L1 * math.cos(a), L1 * math.sin(a)
    b = math.atan2(wz - ez, wr - ex)
    hombro = ALFA_0 - math.degrees(a)
    codo = (math.degrees(b) - BETA_0) / BETA_GANANCIA
    return hombro, codo


def ik(x, y, z):
    """(base, hombro, codo) en grados nominales para la punta en (x, y, z) de base_link (m)."""
    base = base_para(x, y)
    r = math.sqrt(max((x * 1000.0 - BASE_X) ** 2 + (y * 1000.0) ** 2 - PUNTA_Y ** 2, 0.0))
    sol = ik_plano(r, z * 1000.0 - BASE_Z)
    if sol is None:
        return None
    return base, sol[0], sol[1]


def en_rango(base, hombro, codo, margen=0.0):
    return all(TOPES[n][0] + margen <= v <= TOPES[n][1] - margen
               for n, v in (('base', base), ('hombro', hombro), ('codo', codo)))
