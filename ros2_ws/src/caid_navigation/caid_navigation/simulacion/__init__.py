"""
Simulador 2D liviano para probar la navegación sin Gazebo (no reemplaza a Gazebo).

Arma el mismo mundo que gen_maze.py (paredes, soportes, obstáculos, canastos, flechas y zonas), mueve
el robot con límites de velocidad y aceleración, simula los tres ultrasonidos como en gazebo.xacro
(5 rayos en un cono de 15°) y dibuja la imagen de la cámara para correr la visión de verdad.
"""
