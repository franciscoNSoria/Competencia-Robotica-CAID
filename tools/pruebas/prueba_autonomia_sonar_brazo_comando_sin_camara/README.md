# Prueba: autonomía con sonar y brazo a comando (sin cámara)

> El nombre que pediste tenía espacios; en el sistema de archivos quedó con guiones bajos
> (`prueba_autonomia_sonar_brazo_comando_sin_camara`). Es la misma carpeta.

Prueba de banco, **no la estrategia de competencia**: valida por separado dos cosas que la
prueba completa junta (navegar con sonar, y agarrar con el brazo), sin la cámara, que queda
para después. El agarre acá es a mano porque no hay cámara para ubicar el cubo; en la
competencia real el brazo tiene que agarrar solo (el reglamento, Categoría A, prohíbe controlar
el brazo a mano durante la prueba — ver `docs/reglamento/desafio-laberinto.md`).

## Qué hace

1. El robot entra al laberinto y lo recorre explorando con los 3 sonares (sin flechas: no las
   necesita, explora por sí solo con `caid_navigation.mapa.Mapa`, lo mismo que usa el
   explorador completo).
2. Cuando ya no queda nada por explorar y tiene una pared justo adelante, asume que llegó al
   fondo (donde dejaste el cubo) y se acerca despacio hasta que el sonar frontal mide 3 cm.
   **Supone un solo camino, sin ramas**: si tu laberinto tiene una bifurcación hacia otro lado
   sin cubo, el robot puede terminar ahí en vez de en el fondo.
3. Se frena y espera. En la terminal donde corre este nodo, parado ahí: escribís `y` y Enter
   (mientras tanto movés el brazo con `tools/brazo/brazo_sliders.py`, como en tu captura).
4. Al confirmar, vuelve por el camino más corto conocido hasta la entrada y termina.

## Cómo correrlo

Tres terminales, con el agente de micro-ROS corriendo y la ESP32 conectada (como para cualquier
otra prueba). Primero el bringup (calibra el firmware, activa la guardia por sonar, levanta la
odometría y separa los 3 sonares de `/wheel_state`; no toca cámara ni visión):

```bash
ros2 launch caid_bringup prueba_sin_camara.launch.py
```

Después el que explora (es un script suelto, se corre directo con python3, igual que
`brazo_sliders.py`, no con `ros2 launch`):

```bash
python3 tools/pruebas/prueba_autonomia_sonar_brazo_comando_sin_camara/explorar_sin_camara.py
```

Y en la tercera, para mover el brazo y agarrar el cubo a mano:

```bash
python3 tools/brazo/brazo_sliders.py
```

## Parámetros que capaz hay que ajustar

Van en `explorar_sin_camara.py` (es el que tiene `tam_celda`, no el launch del bringup):

- `tam_celda` (0,40 m): el tamaño de las celdas de tu mini-laberinto.
- `umbral_pared` (0,25 m): a partir de qué distancia un sonar dice "hay paso".
- `distancia_parada` (0,03 m): a qué lectura del sonar frontal se frena (3 cm, lo que pediste).

```bash
python3 tools/pruebas/prueba_autonomia_sonar_brazo_comando_sin_camara/explorar_sin_camara.py \
  --ros-args -p tam_celda:=0.45
```
