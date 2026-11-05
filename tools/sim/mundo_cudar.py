#!/usr/bin/env python3
"""
Genera ros2_ws/src/caid_gazebo/worlds/cudar.world: el laberinto visto en el campo de competencia.

Uso: python3 tools/sim/mundo_cudar.py

A diferencia de los escenarios de DESC-04 (bloques del tamaño de una celda), este tiene paredes
finas (2 cm) sobre una grilla de 0,40 m, así que no sirve gen_maze.py: los datos están acá. Se
reutilizan sus funciones de SDF (materiales, cajas, cilindros, cubos) y las medidas del equipo.

Todo cae en la grilla de la navegación (celdas de 0,40 m, centros en x = 0,2 + 0,4 i y
y = -0,4 j): líneas de pared en x = 0,4 i y en y = 0,2 - 0,4 j. Respecto de lo medido en el campo
se corrieron la pared de arriba (y = 0 -> 0,2), el fondo del pasillo rojo (-2,8 -> -3,0), la pared
derecha (desde -1,72 -> -1,8), el final de la #10 (x = 2,2 -> 2,4), el pie de la estantería
(2,24 -> 2,4) y las flechas al centro de la pared de su celda.

- Origen en la esquina de arriba a la izquierda, x al este, y al norte (casi todas las y < 0).
- Cubos a recoger: 20 mm, con el centro a la altura de la cámara, sobre un pilar cilíndrico de
  80 mm, en la zona de cubos (junto a la estantería). Se llaman cubo_<color>, como en los otros
  mundos, así la carga simulada los saca junto con su pilar.
- Zona de partida cerrada como la de E1 a E5 (para que todo funcione igual que ahí): arriba de la
  zona de cubos, 3 x 2 celdas, con la S mirando a la entrada y los canastos 1, 2 y 3 (rojo, verde,
  azul) en la fila de atrás, contra la pared. La abertura derecha de la zona de cubos se cierra.
  Los destinos de la especificación (cubos de 8 cm en los cuartos) no se ponen.
- Flechas: 0,14 m de largo y 0,056 m de punta, pegadas a 3 mm de la cara de la pared.
"""
import importlib.util
import math
import os

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.join(AQUI, '..', '..', 'ros2_ws', 'src', 'caid_gazebo')
spec = importlib.util.spec_from_file_location('gen_maze', os.path.join(RAIZ, 'scripts', 'gen_maze.py'))
gm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gm)

PARED_ALTO = 0.25
PARED_ESPESOR = 0.02
COLOR_MADERA = (0.72, 0.55, 0.36)
CAMARA_ALTURA = 0.1385            # hardware/cad/dimensions.yaml
PILAR_DIAMETRO = 0.080

# Inicio del robot: la S de la zona de partida, junto a la entrada, mirando al sur. La celda de
# atrás es la de los canastos (ahí "sale" a entregar, como en E1 a E5).
INICIO = (2.60, -0.80, -math.pi / 2)

PAREDES = [  # (x0, y0, x1, y1), numeradas como en la especificación, ya en la grilla
    (0.40, 0.20, 1.20, 0.20), (0.40, 0.20, 0.40, -0.60), (0.00, -0.60, 0.40, -0.60),
    (0.00, -0.20, 0.00, -1.80), (0.80, 0.20, 0.80, -1.40), (0.40, -1.00, 0.80, -1.00),
    (0.40, -1.00, 0.40, -1.40), (0.40, -1.40, 0.80, -1.40), (1.20, 0.20, 1.20, -1.00),
    (1.20, -1.00, 2.40, -1.00), (2.00, -1.00, 2.00, -1.80), (2.00, -1.80, 2.40, -1.80),
    (2.80, -1.00, 2.80, -2.20), (1.20, -1.40, 1.60, -1.40), (1.60, -1.40, 1.60, -1.80),
    (1.20, -1.80, 1.60, -1.80), (1.20, -1.40, 1.20, -1.80), (0.00, -1.80, 0.80, -1.80),
    (0.80, -1.80, 0.80, -3.00), (0.80, -3.00, 1.20, -3.00), (1.20, -2.20, 1.20, -3.00),
    (1.20, -2.20, 2.80, -2.20),
    # Zona de partida (la #13 ya sube hasta y = -1,0 y cierra la abertura derecha)
    (1.60, -0.20, 2.80, -0.20), (1.60, -0.20, 1.60, -1.00), (2.80, -0.20, 2.80, -1.00),
]

# A recoger: donde llevan las flechas de su color, cada uno en el centro de la última celda del
# camino (como en E1 a E6): azul y verde arriba de su cuarto, rojo al fondo del pasillo de abajo.
CUBOS = {'azul': (0.60, 0.00), 'verde': (1.00, 0.00), 'rojo': (1.00, -2.80)}
# Canastos de la zona de partida: fila de atrás de la S, contra la pared norte (como E1 a E5).
_CONTRA_PARED = -0.20 - PARED_ESPESOR / 2 - gm.CANASTO_SEPARACION_M - gm.CANASTO_EXT_M / 2
CANASTOS = {'rojo': (1.80, _CONTRA_PARED), 'verde': (2.20, _CONTRA_PARED),
            'azul': (2.60, _CONTRA_PARED)}
CANASTO_BORDE = 0.1085            # como en los escenarios de DESC-04

DIRS = {'norte': (0, 1), 'sur': (0, -1), 'este': (1, 0), 'oeste': (-1, 0)}
FLECHAS = [  # (centro x, y, cara, z, color, apunta a)
    (2.60, -2.20, 'norte', 0.19, 'azul', 'oeste'),
    (2.60, -2.20, 'norte', 0.125, 'rojo', 'oeste'),
    (2.60, -2.20, 'norte', 0.06, 'verde', 'oeste'),
    (0.80, -2.00, 'este', 0.19, 'verde', 'norte'),
    (0.80, -2.00, 'este', 0.125, 'azul', 'norte'),
    (0.80, -2.00, 'este', 0.06, 'rojo', 'sur'),
    (1.20, -1.60, 'oeste', 0.12, 'verde', 'norte'),
    (0.60, -1.40, 'sur', 0.12, 'azul', 'oeste'),
    (0.80, -1.20, 'este', 0.12, 'verde', 'norte'),
    (0.00, -1.20, 'este', 0.12, 'azul', 'norte'),
    (0.80, -0.80, 'oeste', 0.12, 'azul', 'norte'),
]
FLECHA_LARGO = 0.14
FLECHA_PUNTA = 0.056               # alto de la punta, perpendicular al eje
# Proporciones del reglamento (punta 70 x 50 mm, cuerpo de 30 mm de ancho) escaladas a esa punta.
FLECHA_PUNTA_EJE = FLECHA_PUNTA * 50 / 70
FLECHA_CUERPO_ANCHO = FLECHA_PUNTA * 30 / 70
FLECHA_SEPARACION = 0.003
TIRAS_PUNTA = 25

f = gm.f


def modelo_estatico(nombre, x, y, z, yaw, cuerpo):
    return ('    <model name="%s">\n      <static>true</static>\n'
            '      <pose>%s %s %s 0 0 %s</pose>\n      <link name="link">\n%s'
            '      </link>\n    </model>\n\n' % (nombre, f(x), f(y), f(z), f(yaw), cuerpo))


def paredes():
    cuerpo = ''
    for i, (x0, y0, x1, y1) in enumerate(PAREDES, 1):
        largo = math.hypot(x1 - x0, y1 - y0) + PARED_ESPESOR     # cierra las esquinas
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        yaw = math.atan2(y1 - y0, x1 - x0)
        pose = '%s %s %s 0 0 %s' % (f(cx), f(cy), f(PARED_ALTO / 2), f(yaw))
        size = '%s %s %s' % (f(largo), f(PARED_ESPESOR), f(PARED_ALTO))
        cuerpo += ('        <collision name="c%d"><pose>%s</pose><geometry><box><size>%s</size>'
                   '</box></geometry></collision>\n' % (i, pose, size))
        cuerpo += ('        <visual name="v%d"><pose>%s</pose><geometry><box><size>%s</size></box>'
                   '</geometry>\n%s        </visual>\n'
                   % (i, pose, size, gm.material(COLOR_MADERA, 10)))
    return modelo_estatico('paredes', 0, 0, 0, 0, cuerpo)


def cubos(posiciones, prefijo):
    """Cubo de 20 mm a la altura de la cámara sobre un pilar cilíndrico: <prefijo>_<color>."""
    salida = ''
    apoyo = CAMARA_ALTURA - gm.CUBO_M / 2
    for color, (x, y) in posiciones.items():
        rgb = dict(gm.COLORES_CUBO.values())[color]
        pilar = (gm.cilindro_colision('colision', PILAR_DIAMETRO / 2, apoyo)
                 + gm.cilindro_visual('visual', PILAR_DIAMETRO / 2, apoyo, (0.05, 0.05, 0.05)))
        salida += modelo_estatico(f'soporte_{prefijo}_{color}', x, y, apoyo / 2, 0, pilar)
        salida += gm.cubo_sdf(f'{prefijo}_{color}', x, y, rgb, apoyo)
    return salida


def canastos():
    """Canasto de 44 mm sobre su columna, con el borde a 108,5 mm, como los de gen_maze.py."""
    salida = ''
    E, T, F, A = gm.CANASTO_EXT_M, gm.CANASTO_PARED_M, gm.CANASTO_FONDO_M, gm.CANASTO_ALTO_M
    base = CANASTO_BORDE - A
    for color, (x, y) in CANASTOS.items():
        rgb = dict(gm.COLORES_CUBO.values())[color]
        size = (gm.CANASTO_SOPORTE_M, gm.CANASTO_SOPORTE_M, base)
        columna = (gm.caja_colision('colision', (0, 0, 0), size)
                   + gm.caja_visual('visual', (0, 0, 0), size, gm.COLOR_SOPORTE))
        salida += modelo_estatico('soporte_canasto_' + color, x, y, base / 2, 0, columna)
        partes = [('fondo', (0, 0, F / 2), (E, E, F)),
                  ('pared_n', (0, E / 2 - T / 2, A / 2), (E, T, A)),
                  ('pared_s', (0, -E / 2 + T / 2, A / 2), (E, T, A)),
                  ('pared_e', (E / 2 - T / 2, 0, A / 2), (T, E - 2 * T, A)),
                  ('pared_o', (-E / 2 + T / 2, 0, A / 2), (T, E - 2 * T, A))]
        cuerpo = ''.join(gm.caja_colision('c_' + n, p, s, friccion=1.0)
                         + gm.caja_visual('v_' + n, p, s, rgb) for n, p, s in partes)
        salida += modelo_estatico('canasto_' + color, x, y, base, 0, cuerpo)
    return salida


def flechas():
    cuerpo = ''
    for i, (x, y, cara, z, color, apunta) in enumerate(FLECHAS):
        nx, ny = DIRS[cara]
        d = FLECHA_SEPARACION + PARED_ESPESOR / 2 + gm.FLECHA_GROSOR_M / 2
        ux, uy = DIRS[apunta]
        yaw = math.atan2(uy, ux)
        cuerpo += '      <link name="flecha_%d">\n        <pose>%s %s %s 0 0 %s</pose>\n' % (
            i, f(x + nx * d), f(y + ny * d), f(z), f(yaw))
        # Eje x del link = hacia donde apunta; z = arriba. Cuerpo y punta en tiras (SDF no tiene
        # prismas), con el origen en el centro del largo total.
        u0 = -FLECHA_LARGO / 2
        largo_cuerpo = FLECHA_LARGO - FLECHA_PUNTA_EJE
        rgb = gm.COLOR_FLECHA[color]
        piezas = [(u0 + largo_cuerpo / 2, largo_cuerpo, FLECHA_CUERPO_ANCHO)]
        paso = FLECHA_PUNTA_EJE / TIRAS_PUNTA
        for j in range(TIRAS_PUNTA):
            piezas.append((u0 + largo_cuerpo + (j + 0.5) * paso, paso,
                           FLECHA_PUNTA * (1 - j / TIRAS_PUNTA)))
        for j, (uc, largo, alto) in enumerate(piezas):
            cuerpo += gm.caja_visual('p%d' % j, (uc, 0, 0), (largo, gm.FLECHA_GROSOR_M, alto),
                                     rgb, indent=8)
        cuerpo += '      </link>\n'
    return '    <model name="flechas">\n      <static>true</static>\n%s    </model>\n\n' % cuerpo


def main():
    x, y, yaw = INICIO
    salida = os.path.join(RAIZ, 'worlds', 'cudar.world')
    with open(salida, 'w') as out:
        out.write('<?xml version="1.0"?>\n<!-- Generado por tools/sim/mundo_cudar.py. No editar a '
                  'mano. -->\n<!-- Inicio del robot: x=%s y=%s yaw=%s -->\n'
                  '<sdf version="1.9">\n  <world name="ros2_world">\n\n' % (f(x), f(y), f(yaw)))
        out.write('''    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>

    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material>
            <ambient>0.55 0.55 0.55 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

''')
        out.write(paredes() + cubos(CUBOS, 'cubo') + canastos() + flechas())
        out.write('  </world>\n</sdf>\n')
    print(f'{salida}: {len(PAREDES)} paredes, {len(CUBOS)} cubos, {len(CANASTOS)} canastos, '
          f'{len(FLECHAS)} flechas. Inicio x={x} y={y} yaw={yaw:.4f}')


if __name__ == '__main__':
    main()
