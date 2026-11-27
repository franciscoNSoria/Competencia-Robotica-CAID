"""
Detección de ruta (espacio libre del piso) y rumbo, en una imagen BGR. OpenCV clásico, sin ROS.

Idea: justo delante del robot siempre hay piso. Se toma el color de una franja de la parte baja de
la imagen como "color del piso", se marca como piso todo lo conectado a esa franja que tenga un
color parecido, y para cada columna se mide hasta dónde llega el piso antes de topar con una pared u
obstáculo. Las columnas que llegan lejos forman la ruta. El rumbo apunta al centro del hueco más
amplio y profundo.

    analizar_ruta(img) devuelve:
      {'rumbo_deg':  ángulo hacia donde ir (0 = derecho, + = derecha, - = izquierda) o None,
       'estado':     'libre' | 'bloqueado' | 'sin_piso',
       'confianza':  0..1 (qué tan claro es el hueco elegido),
       'profundidad': [0..1, ...]  alcance del piso en cada columna (1 = hasta el horizonte),
       'hueco':      (col_izq, col_der) del hueco elegido, en píxeles de la imagen original, o None,
       'libre':      {'izq': 0..1, 'centro': 0..1, 'der': 0..1},   # alcance medio por zona
       'mascara':    máscara uint8 del piso (tamaño de la imagen original)}

El rumbo es un ángulo de la imagen (usa la distancia focal fx). Para un robot real hay que
chequearlo con fotos del laberinto: el piso puede tener sombras o el color puede cambiar con la luz.
"""
import math

import cv2
import numpy as np

ANCHO_TRABAJO = 320          # se achica la imagen para que sea rápido (ms por cuadro)
FRANJA_PISO = (0.86, 0.98)   # filas (fracción del alto) de donde se toma el color del piso
FRANJA_COLS = (0.30, 0.70)   # columnas (fracción del ancho) de esa franja
UMBRAL_MIN = 14.0            # distancia mínima de color (Lab) para considerar "no es piso"
UMBRAL_MAX = 45.0
COLUMNAS = 32                # en cuántas bandas se divide la imagen
PROF_MIN = 0.18              # una banda con menos alcance que esto es pared/obstáculo
HUECO_REL = 0.65             # una banda cuenta para el hueco si llega al 65 % de la mejor
FOV_GRADOS = 60.0            # campo visual horizontal si no se pasa fx


def _umbral_color(lab_franja):
    """Umbral según qué tan uniforme es la franja: piso liso -> umbral chico."""
    ref = np.median(lab_franja.reshape(-1, 3), axis=0)
    d = np.linalg.norm(lab_franja.reshape(-1, 3) - ref, axis=1)
    return float(np.clip(np.percentile(d, 95) * 1.8 + 6.0, UMBRAL_MIN, UMBRAL_MAX)), ref


def analizar_ruta(img, fx=None, horizonte=None):
    """Ver el formato de la salida arriba. `horizonte`: fila (px) hasta donde se mira."""
    alto0, ancho0 = img.shape[:2]
    esc = ANCHO_TRABAJO / ancho0
    chica = cv2.resize(img, (ANCHO_TRABAJO, int(round(alto0 * esc))), interpolation=cv2.INTER_AREA)
    chica = cv2.GaussianBlur(chica, (5, 5), 0)
    alto, ancho = chica.shape[:2]
    lab = cv2.cvtColor(chica, cv2.COLOR_BGR2LAB).astype(np.float32)

    f0, f1 = int(alto * FRANJA_PISO[0]), int(alto * FRANJA_PISO[1])
    c0, c1 = int(ancho * FRANJA_COLS[0]), int(ancho * FRANJA_COLS[1])
    umbral, ref = _umbral_color(lab[f0:f1, c0:c1])

    dist = np.linalg.norm(lab - ref, axis=2)
    piso = (dist < umbral).astype(np.uint8)
    piso = cv2.morphologyEx(piso, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    piso = cv2.morphologyEx(piso, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    # Solo el piso conectado a la franja de abajo (un cartel gris en la pared no cuenta)
    n, etiquetas = cv2.connectedComponents(piso)
    semilla = etiquetas[f0:f1, c0:c1]
    ids = np.unique(semilla[semilla > 0])
    conectado = np.isin(etiquetas, ids).astype(np.uint8) if len(ids) else np.zeros_like(piso)

    cobertura = float(conectado[f0:f1, c0:c1].mean())
    salida = {'rumbo_deg': None, 'estado': 'sin_piso', 'confianza': 0.0, 'profundidad': [],
              'hueco': None, 'libre': {'izq': 0.0, 'centro': 0.0, 'der': 0.0},
              'mascara': cv2.resize(conectado * 255, (ancho0, alto0),
                                    interpolation=cv2.INTER_NEAREST)}
    if cobertura < 0.3:           # la franja no es piso uniforme (algo adelante, o imagen negra)
        return salida

    # Alcance del piso por columna: filas contiguas de piso desde abajo hasta el primer "no piso"
    fila_tope = int(alto * 0.15) if horizonte is None else int(horizonte * esc)
    util = max(alto - fila_tope, 1)
    ancho_banda = ancho / COLUMNAS
    prof = []
    for i in range(COLUMNAS):
        a, b = int(i * ancho_banda), max(int((i + 1) * ancho_banda), int(i * ancho_banda) + 1)
        col = conectado[fila_tope:, a:b].mean(axis=1) > 0.5       # por fila: ¿es piso?
        libres = 0
        for v in col[::-1]:                                        # de abajo hacia arriba
            if not v:
                break
            libres += 1
        prof.append(libres / util)
    prof = np.array(prof)
    salida['profundidad'] = [round(float(p), 3) for p in prof]

    tercio = COLUMNAS // 3
    salida['libre'] = {'izq': float(prof[:tercio].mean()),
                       'centro': float(prof[tercio:2 * tercio].mean()),
                       'der': float(prof[2 * tercio:].mean())}

    mejor = float(prof.max())
    if mejor < PROF_MIN:
        salida['estado'] = 'bloqueado'
        return salida

    # Huecos: tramos seguidos de bandas que llegan lejos. Gana el de más área (ancho x profundidad)
    abierto = prof >= max(PROF_MIN, HUECO_REL * mejor)
    huecos, ini = [], None
    for i, ok in enumerate(list(abierto) + [False]):
        if ok and ini is None:
            ini = i
        elif not ok and ini is not None:
            huecos.append((ini, i - 1))
            ini = None
    puntaje = [(prof[a:b + 1].sum(), (a, b)) for a, b in huecos]
    mejor_puntaje, (a, b) = max(puntaje)
    pesos = prof[a:b + 1]
    col_centro = (np.arange(a, b + 1) + 0.5) * ancho_banda
    u = float(np.sum(col_centro * pesos) / np.sum(pesos)) / esc        # px de la imagen original

    f = fx if fx else (ancho0 / 2) / math.tan(math.radians(FOV_GRADOS) / 2)
    salida['rumbo_deg'] = math.degrees(math.atan2(u - ancho0 / 2, f))
    salida['hueco'] = (int(a * ancho_banda / esc), int((b + 1) * ancho_banda / esc))
    total = float(prof.sum())
    confianza = float(mejor_puntaje / total) * min(1.0, mejor / 0.5) if total else 0.0
    salida['confianza'] = round(confianza, 3)
    salida['estado'] = 'libre'
    return salida


def dibujar_ruta(img, ruta, fx=None):
    """Dibuja el piso, el hueco elegido y una flecha con el rumbo sobre `img` (se modifica)."""
    alto, ancho = img.shape[:2]
    if ruta['mascara'] is not None:
        verde = np.zeros_like(img)
        verde[:, :, 1] = 255
        m = ruta['mascara'] > 0
        img[m] = cv2.addWeighted(img, 0.78, verde, 0.22, 0)[m]
    n = len(ruta['profundidad'])
    for i, p in enumerate(ruta['profundidad']):
        x0, x1 = int(i * ancho / n), int((i + 1) * ancho / n)
        y = int(alto * (1 - p * 0.85))
        cv2.line(img, (x0, y), (x1, y), (0, 255, 255), 2)
    if ruta['rumbo_deg'] is not None:
        f = fx if fx else (ancho / 2) / math.tan(math.radians(FOV_GRADOS) / 2)
        u = ancho / 2 + f * math.tan(math.radians(ruta['rumbo_deg']))
        base, tope = (ancho // 2, alto - 6), (int(np.clip(u, 0, ancho - 1)), int(alto * 0.62))
        cv2.arrowedLine(img, base, tope, (0, 0, 0), 7, tipLength=0.18)
        cv2.arrowedLine(img, base, tope, (255, 200, 0), 3, tipLength=0.18)
    return img
