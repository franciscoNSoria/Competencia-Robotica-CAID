#!/usr/bin/env python3
"""
Genera un mundo de Gazebo a partir de un dibujo del laberinto en texto.

Uso: python3 tools/sim/mundo_desde_ascii.py laberinto.txt salida.world [--celda 0.45]

En el dibujo, # es un bloque de pared, . una celda libre y E la entrada (celda libre con un
marcador azul en el piso). La primera línea es el lado +y y el laberinto queda centrado en el
origen. Imprime la posición de cada celda marcada con E para pasarla al launch.
"""
import argparse

ENCABEZADO = """<?xml version="1.0"?>
<!-- Generado con tools/sim/mundo_desde_ascii.py a partir de este dibujo (# pared, E entrada):
{dibujo}
-->
<sdf version="1.9">
  <world name="ros2_world">

    <plugin filename="gz-sim-physics-system"
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
          <geometry><plane><normal>0 0 1</normal><size>{piso} {piso}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{piso} {piso}</size></plane></geometry>
          <material>
            <ambient>0.82 0.82 0.82 1</ambient>
            <diffuse>0.82 0.82 0.82 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

PARED = """
    <model name="wall_{n}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.140 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{c:.3f} {c:.3f} 0.280</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{c:.3f} {c:.3f} 0.280</size></box></geometry>
          <material>
            <ambient>0.05 0.05 0.05 1</ambient>
            <diffuse>0.05 0.05 0.05 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

ENTRADA = """
    <model name="entrada_{n}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.002 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.099</radius><length>0.004</length></cylinder></geometry>
          <material>
            <ambient>0.1 0.3 0.9 1</ambient>
            <diffuse>0.1 0.3 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dibujo')
    ap.add_argument('salida')
    ap.add_argument('--celda', type=float, default=0.45)
    args = ap.parse_args()

    filas = open(args.dibujo).read().strip().splitlines()
    alto, ancho, c = len(filas), max(len(f) for f in filas), args.celda
    partes = [ENCABEZADO.format(dibujo='\n'.join(filas), piso=round(max(alto, ancho) * c + 4))]
    paredes = entradas = 0
    for j, fila in enumerate(filas):
        for i, ch in enumerate(fila):
            x = (i - (ancho - 1) / 2) * c
            y = ((alto - 1) / 2 - j) * c
            if ch == '#':
                partes.append(PARED.format(n=paredes, x=x, y=y, c=c))
                paredes += 1
            elif ch == 'E':
                partes.append(ENTRADA.format(n=entradas, x=x, y=y))
                entradas += 1
                print(f'Entrada {entradas}: x={x:.3f} y={y:.3f}')
    partes.append('\n  </world>\n</sdf>\n')
    with open(args.salida, 'w') as f:
        f.write(''.join(partes))
    print(f'{args.salida}: {paredes} paredes, {ancho}x{alto} celdas de {c} m')


if __name__ == '__main__':
    main()
