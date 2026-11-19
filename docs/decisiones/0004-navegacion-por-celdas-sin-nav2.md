# 0004. Navegación por celdas con planificador propio, sin Nav2 ni SLAM

- **Estado:** Propuesta
- **Fecha:** 2026-10-29

## Contexto

Hay que recorrer un laberinto desconocido, encontrar tres cubos, llevarlos a sus canastos y volver,
usando la cámara (con servo de giro), la odometría de las ruedas y tres ultrasonidos. Se preguntó si
conviene usar Nav2 y SLAM.

Datos que pesan (verificados en el repositorio):

- **No hay LIDAR.** Los "lidar" de Gazebo son tres ultrasonidos simulados como 5 rayos en un cono de
  15° (`gazebo.xacro`): tres lecturas por ciclo, no un barrido de 360°. El robot real tiene un HC-SR04
  al frente y dos RCWL-9610A a los costados (`hardware/cad/dimensions.yaml`).
- **El laberinto es una grilla.** Bloques de 320 × 320 mm: cada celda es piso o pared
  (`docs/arquitectura/laberintos.md`).
- **El robot casi no entra.** Mide 278 × 163 mm; sus esquinas quedan a 162 mm del centro de giro y el
  pasillo deja 160 mm a cada lado del eje (C). No puede girar 180° en un pasillo y para girar 90° tiene
  que frenar y llegar corrido 2 a 4 cm (C, `giros.py`).
- **Tiempo:** las fechas del roadmap son de esta semana.

## Decisión

Navegación propia sobre una **grilla de celdas de 0,32 m** (paquete `caid_navigation`):

- Mapa de lados (paso / pared / desconocido) que se arma con los ultrasonidos, vacío al empezar.
- Planificador de Dijkstra sobre estados (celda, rumbo) con tres acciones: avanzar, retroceder y girar.
  Un giro solo existe si la huella del robot, ensanchada con un margen y ±3° de duda de rumbo, no toca
  nada (`giros.py`). En un callejón el robot sale marcha atrás.
- Pose = odometría corregida con las paredes conocidas (lateral, frente, bordes de aberturas, rumbo por
  la pendiente lateral).
- Obstáculos: barrido con el ultrasonido de frente al girar; detección lateral al pasar.
- Cámara: flechas (celda, pared, dirección y largo medidos sobre el plano de la pared), cubos,
  canastos y zonas rojas.

## Alternativas consideradas

- **Nav2 + slam_toolbox.** slam_toolbox necesita un `LaserScan` denso; con tres rayos el mapa de
  ocupación queda casi vacío y la localización no converge. El costmap de Nav2 con inflado no deja
  pasar un robot de 163 mm por un pasillo de 320 mm (sobran 78 mm por lado) y sus controladores
  (DWB, RPP) no saben que un giro de 180° es imposible ahí. Ajustarlo llevaría más tiempo que el
  disponible y agregaría dependencias que hay que instalar en la PC de la competencia.
- **Seguir paredes (mano derecha).** No funciona con islas (E4) ni permite volver por el camino más
  corto a un canasto.
- **Trémaux puro (marcar pasillos) sin pose.** Es la base del explorador de Martín
  (`explorador.py`); se mantiene la idea (explorar la frontera más cercana marcando lo recorrido) y se
  le suma la pose corregida, los giros verificados y la misión completa.

## Consecuencias

- Todo es Python puro y se prueba sin ROS con un simulador 2D (`caid_navigation/simulacion`) que usa
  el mismo `gen_maze.py` que los mundos de Gazebo.
- Las medidas del robot y de la cámara están en `geometria.py` y salen de `dimensions.yaml`: si cambia
  el robot, hay que actualizarlas.
- Si en algún momento se agrega un LIDAR de verdad, Nav2 vuelve a ser una opción; el contrato
  (`/sim/cmd_vel`, odometría, `Range`) no cambia.
