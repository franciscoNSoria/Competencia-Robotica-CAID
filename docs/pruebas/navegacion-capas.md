# Informe: navegación (capas 1 a 3) y calibración de cámara

> Fecha: 2026-11-19. Código entregado: 9 módulos nuevos (1.605 líneas), 68 pruebas, 2 herramientas. Todo en `nav_capas_1_a_3.zip` (el código se integra en NAV-08 a NAV-10: #97, #98 y #99).
> **Lo más importante, primero: todo esto está probado en un simulador 2D y con imágenes sintéticas. NADA está probado en el robot.**
> Los parámetros del simulador salen de lo medido en el robot (diámetro, trocha, piso de velocidad, pérdidas del sonar a 5,5 cm, huecos de Wi-Fi)
> y de supuestos míos (retardo de las ruedas de 0,25 s, latencia de 50 ms, ruido del sonar de 5 mm, frecuencia de los huecos de Wi-Fi). Los supuestos están
> marcados **(S)**. Los números de abajo se vuelven datos cuando se reemplacen por los de la campaña de calibración ([`calibracion-sim2real.md`](../guias/calibracion-sim2real.md)).

---

## 1. Qué se hizo

| Capa | Módulo (`ros2_ws/src/caid_navigation/caid_navigation/`) | Qué hace | Pruebas |
|---|---|---|---|
| 1a | `control_pared.py` | Control lateral por paredes: modo "las dos / una / ninguna", histéresis, rumbo por la pendiente de los sonares, PD en cascada | 17 |
| 1b | `movimiento_celda.py` | Avanzar UNA celda de centro a centro: perfil de velocidad, freno, parada contra pared de frente, zona ciega del sonar | 9 |
| banco | `sim2d.py`, `bucle_pasillo.py`, `bucle_celdas.py` | Simulador 2D con los defectos del robot real y los bucles cerrados para probar | (se usan en las pruebas de 1a/1b) |
| 2 | `decision_flechas.py` | Decisión en un nodo con las flechas, cuenta de segmentos, conflicto flecha-vs-sonar, plan de barrido lateral, verificación entre lecturas | 16 |
| 3 | `grafo.py`, `marcadores_grafo.py` | Grafo topológico desde el mapa de celdas, camino más corto con penalización de giro, traducción a comandos, comparación contra el recorrido dibujado, marcadores de RViz | 9 + 2 |
| cámara | `camara_geometria.py`, `tools/calibrar_camara_extrinseca.py` | Pixel ↔ piso, distancia a la pared por la base, calibración de altura y cabeceo, rumbo por punto de fuga, flip/mirror parametrizados | 15 |
| informe | `tools/experimentos_capa1.py` | Reproduce las tablas de este informe | — |

**No se hizo (y por qué):** el port de la capa 1 a C para la ESP32 (hace falta antes el firmware S3 compilando); los nodos ROS que conectan todo (hace falta el robot
y los puentes reales en `dev`); ninguna validación con datos reales. Ver sección 9.

---

## 2. Respuestas a lo que planteaste

**¿Por qué PD y no PID?** La planta del rumbo es un integrador (la velocidad angular se acumula en ángulo) y la posición lateral otro: el lazo es de segundo orden
y un PD en cascada lo estabiliza sin error de régimen para una orden constante. El problema es el sesgo del robot (la deriva): con solo PD queda un corrimiento
lateral constante proporcional al sesgo. **Medí cuánto:** con una deriva de 0,05 rad/s (2,5 veces la que equivale a tus 5 cm por metro) el corrimiento medio es de
1,0 cm; con 0,10 rad/s, 1,75 cm. Una integral **gateada** (solo con el robot casi derecho y casi centrado, con tope) lo baja a 1,3 cm. No la activé por defecto
(`ki_sesgo = 0`): con el pasillo de 78,5 mm de holgura por lado no hace falta. Si la deriva real resulta mayor, es la perilla.

**Tu regla de las dos paredes.** Implementada tal cual la aclaraste: manda la **diferencia entre las dos distancias**. Hasta ~10 cm de diferencia se endereza con las dos;
con más, se sigue solo la cercana y la lejana se trata como apertura o hall. Agregué histéresis (entra con ≤ 8 cm, vuelve a "una sola" con > 12 cm) para que un sonar
ruidoso en el límite no haga saltar el modo.

**"Mantener ~7 cm de la pared": hay que debatirlo.** Con el pasillo de 320 mm, el robot de 163 mm y los sonares laterales a 81,5 mm del eje
(`dimensions.yaml`, ese valor figura como provisorio), **con el robot centrado cada sonar lee 78,5 mm**. O sea que 7–8 cm **es** ir centrado. Eso tiene dos consecuencias:
(a) seguir una pared a 7 cm equivale a ir por el medio (conviene), y (b) tu regla "al acercarse a menos de 7/8 cm de una pared, corregir rápido" **se dispararía
todo el tiempo**. Por eso el umbral de alerta (`d_alerta`) quedó en 4 cm, a 3,5 cm del centro. Es un parámetro: si preferís otro criterio (por ejemplo, por velocidad de
acercamiento), se cambia ahí.

**"Iterar muy rápido los dos laterales".** Hoy el firmware dispara en la secuencia frontal, derecho, frontal, izquierdo a 60 ms: cada lateral se mide cada **240 ms (4 Hz)**,
y a 0,18 m/s son 4,3 cm entre lecturas. Medí qué pasa con más rapidez (tabla 1): **con las ganancias ajustadas el control anda bien incluso a 4 Hz** (y RMS 0,31 cm) y mejora
poco a 17 Hz. Conviene igual medir más rápido si se puede (más margen), pero ya no es un requisito.

**Estimar el rumbo después de un giro con una sola lectura de sonares: no se puede.** Con el robot girado 10°, la suma de las dos distancias laterales cambia solo ~5 mm
(157 → 162 mm), por debajo del ruido y la resolución del sonar. El rumbo sale de la **pendiente de la distancia mientras se avanza** (lo que hace `control_pared.py`) o de la
cámara (punto de fuga, sección 6). Esto respalda tu idea de realimentar con cámara.

**Cámara boca abajo.** Parametrizado: `Camara.flip` y `Camara.mirror` (y `Camara.desde_dict` con las claves `flip`/`mirror` del YAML). Hoy el firmware ya endereza la
imagen (`vflip 0, hmirror 1`), así que los dos quedan en `False`; si la cámara cambia de montaje, se corrige en un archivo y no en el código.

**Navegación por cámara (IA) como alternativa más suave:** no la toqué. Primero hay que ver cuánto aporta el punto de fuga.

---

## 3. Capa 1a: control por paredes

**Geometría.** Con el robot en `(y, psi)` en el pasillo, cada sonar lee `d_izq = (W − y_s_izq)/cos(psi)`, con `y_s_izq = y + x_sonar·sin(psi) + y_sonar·cos(psi)` y simétrico
para la derecha. El módulo implementa el modelo directo y su inversa (probados: error de inversión del orden de 1e-17) y estima `psi` como la pendiente por mínimos
cuadrados de la posición lateral contra el avance de la odometría, con una ventana de 10 cm. Un salto de más de 4 cm entre lecturas se toma como un hueco en la pared (no
como pendiente) y reinicia la ventana. Se combina con el giro que dicen los encoders (filtro complementario, `tau_psi` = 0,8 s).

**Ley de control.** `psi_ref = clamp(−k_y·y, ±psi_max)`, `omega = k_psi·(psi_ref − psi_est)`, con `k_psi = 2·zeta·omega_n` y `k_y = omega_n/(2·zeta·v)`: el lazo de posición queda
como un sistema de segundo orden con la frecuencia natural y el amortiguamiento que se elijan, a cualquier velocidad.

**Cómo cambia cada parámetro** (los defaults salieron de una búsqueda sobre 6 escenarios con defectos; ver sección 7):

| Parámetro | Default | Si lo subís | Si lo bajás |
|---|---|---|---|
| `omega_n` | 1,6 rad/s | Corrige más rápido, pero amplifica el ruido del sonar y oscila si la trocha efectiva real es menor que la del firmware. Con 2,5 (y los demás parámetros de entonces) hubo contactos en 3 de 36 corridas | Más lento: un error de 4 cm tarda ~0,5 m (1,5 celdas) en corregirse |
| `zeta` | 0,9 | Más amortiguado, más lento | Más rebote |
| `psi_max` | 0,30 rad | Permite corregir errores grandes más rápido | Limita el ángulo de ataque a la pared |
| `omega_max` | 1,2 rad/s | — | Protege cuando una orden llega retenida por un hueco de Wi-Fi |
| `tau_psi` | 0,8 s | Confía más en los encoders (se desvía si patinan) | Confía más en la pendiente de los sonares (más ruido) |
| `ventana_m` / `n_min_ventana` | 10 cm / 3 | Pendiente más estable pero más lenta | Pendiente más ruidosa |
| `d_alerta` | 4 cm | Corrección fuerte más lejos de la pared (dispara en el centro si ≥ 7,85 cm) | Más tarde |
| `dif_entra` / `dif_sale` | 8 / 12 cm | Más tiempo con "las dos paredes" | Más tiempo siguiendo una sola |
| `ki_sesgo` | 0 | Compensa deriva constante (1–4 sirve; probado) | Sin integral |

---

## 4. Capa 1b: avanzar una celda

Estados: acelerando, crucero, frenando, listo, bloqueado. Llama a la capa 1a, frena `v·t_freno` antes del final (se sigue andando después de cortar la orden), y si la celda
de destino tiene pared de frente, para con el sonar (a 2,01 cm con el robot centrado, con las medidas actuales).

**Hallazgos de diseño (todos con prueba):**
- **Sonar frontal ciego.** Con pared de frente y el robot centrado, el frontal lee ~2 cm, **justo en el límite del sonar** (`d_min` 2 cm): ahí devuelve "sin eco". Un sonar que "no ve" es indistinguible de uno
  que no tiene nada enfrente. Se resolvió con memoria: si venía midiendo algo cerca (< 10 cm) y de golpe no hay eco, es la pared pegada (`pared_pegada`), hasta que se gire
  (`giro_hecho()`). Sin esto, pidiéndole más celdas contra el fondo, el simulador atravesaba la pared; con esto, 0 contactos en 12 pruebas.
- **El freno hay que calibrarlo:** el valor de `t_freno` que deja el error medio en cero fue 0,48 s en el simulador (con un retardo de rueda de 0,25 s **(S)**); con 0,36 s el
  robot se pasa ~1 cm por celda y en 10 celdas acumula ~10 cm. En el robot real se mide (la campaña de calibración incluye "avanzar N celdas y medir").
- **La corrección por aperturas laterales NO quedó validada** (ver tabla 4 y sección 8). Está implementada, apagada por defecto, y la prueba lo deja asentado.

---

## 5. Capa 2 y Capa 3

**Capa 2 (`decision_flechas.py`).** Elige la flecha del color buscado (mayor largo primero y, a igualdad, la más cercana, como el seguidor real), cuenta los segmentos rectos que faltan
y decide: seguir derecho, girar, o **marcar conflicto** si la flecha pide un lado que los sonares ven como pared (en ese caso confía en los sonares y vuelve a Trémaux).
Una lectura sola no alcanza (`min_votos`). Planifica el barrido lateral del servo (0, +30, +60, +90, 0, −30, −60, −90, 0) y lo acepta si hay ≥ 3 lecturas y ≥ 70 % de acuerdo.

**Supuestos a confirmar con la pista (P):** (1) corta/media/larga = 1/2/3 segmentos; (2) un "segmento" se cuenta por **nodo con alternativa** que se pasa derecho, no por celda;
(3) el sentido de la flecha es relativo al sentido de marcha cuando se la ve de frente. Si la pista contradice alguno, es un parámetro/constante, no una reescritura.

**Capa 3 (`grafo.py`).** Nodos: inicio, cruces, fondos ciegos, celdas especiales (cubo, depósito) y, opcional, esquinas. Aristas con largo, celdas, direcciones y "recorrida".
Camino más corto con penalización por giro (media vuelta pesa el doble) y `rumbo_inicial`. `ruta_a_comandos` traduce a "avanzar N / girar izq-der". **Verificación de tu idea:**
`mapa_desde_ascii` arma el grafo esperado de un dibujo y `comparar_grafos` lo contrasta con el que arma el robot; la prueba explora dos laberintos con el mapa de Trémaux y
el grafo aprendido coincide con el dibujado. `marcadores_grafo.py` lo dibuja en RViz (probado con mensajes de mentira; **el dibujo real en RViz no lo vi**).

---

## 6. Cámara

**Hallazgos (todos con prueba y salen del lente calibrado del repo):**

| Hallazgo | Valor | Consecuencia |
|---|---|---|
| Campo visual horizontal real (de `fx` = 947 px a 640 px) | **37,3°** | La simulación usa **60°**: la visión ve cosas distintas en sim y en real. Hay que cambiar `horizontal_fov` en `gazebo.xacro` |
| Distancia mínima a la que se ve la base de una pared de frente | 0,41 m con cabeceo 0,15 rad; 0,34 m con 0,25 rad; 0,29 m con 0,35 rad | **A una celda de distancia (0,32 m) la cámara casi no ve la base de la pared.** Sirve para medir desde ~0,35–0,4 m, no como segundo sonar a corta distancia |
| Base de la pared lateral con el servo a 90° | Solo se ve con cabeceo ≥ ~0,45 rad (26°) | A 90° se ven las flechas (están más arriba) pero **no el borde piso-pared**: la distancia lateral por cámara hay que sacarla de otra pista (borde superior de la pared o tamaño de la flecha) |
| Rumbo por punto de fuga (imágenes sintéticas con la distorsión real) | error < 0,2° | Promete como medición independiente; **falta probarlo con imágenes reales** |

**Calibración (protocolo para hacer con el robot):**
1. **Altura y cabeceo:** robot quieto y recto frente a una pared a 0,40; 0,50; 0,65; 0,85; 1,10 y 1,50 m (cinta desde el centro del robot). Guardar un cuadro de cada una y anotar en
   `medidas.csv` la fila donde la pared toca el piso. `python3 tools/calibrar_camara_extrinseca.py medidas.csv --yaml .../calibracion_esp32cam.yaml` da la altura, el cabeceo y el
   error que queda (en una prueba con datos sintéticos de 1 px y 3 mm de ruido, recuperó 145,8 mm y 0,269 rad cuando los verdaderos eran 147 mm y 0,27 rad).
2. **Punto de fuga:** robot en el centro de un pasillo recto, alineado con una regla; comparar `punto_fuga_imagen` + `rumbo_desde_fuga` con el rumbo real a 0°, ±5°, ±10°.
3. **Retraso y fps:** ya medidos en el PR #88 (~44 ms, ~12 detecciones/s); se incorporan al emulador.
4. **Verificar el servo:** pedir pan = 0 y medir con transportador el ángulo real (`pan_cero`).

---

## 7. Resultados (simulador 2D; se reproducen con `tools/experimentos_capa1.py`)

Tabla 1: periodo de muestreo de los sonares laterales (inicio con 4 cm de desvío y 0,1 rad de rumbo, con los defectos supuestos; 8 repeticiones).

| Periodo por lateral | Contactos | y RMS (cm) | y máx (cm) | rumbo final (°) |
|---|---|---|---|---|
| 240 ms (4 Hz) | 0/8 | 0,31 | 4,4 | 1,4 |
| 120 ms (8 Hz) | 0/8 | 0,26 | 4,2 | 0,8 |
| 60 ms (17 Hz) | 0/8 | 0,25 | 4,2 | 0,4 |
| 30 ms (33 Hz) | 0/8 | 0,28 | 4,1 | 0,6 |

Tabla 2: robustez del control con lateral cada 240 ms (como el firmware actual).

| Caso | Contactos | y RMS (cm) | y máx (cm) | rumbo final (°) |
|---|---|---|---|---|
| referencia | 0/8 | 0,31 | 4,4 | 1,4 |
| trocha física 25 % menor que la del firmware | 0/8 | 0,61 | 4,3 | 2,7 |
| trocha física 25 % mayor | 0/8 | 0,26 | 4,4 | 1,6 |
| deriva 0,05 rad/s | 0/8 | 0,99 | 1,6 | 1,0 |
| deriva 0,10 rad/s | 0/8 | 1,75 | 3,0 | 2,0 |
| deriva 0,10 rad/s + integral gateada (ki = 2) | 0/8 | 1,32 | 2,5 | 1,6 |
| huecos de Wi-Fi (0,2–0,4 s), 0,1 por segundo, control en la PC | 0/8 | 0,36 | 2,2 | 2,1 |
| huecos de Wi-Fi, 0,5 por segundo | 0/8 | 0,42 | 2,2 | 1,8 |

Tabla 3: una celda (0,32 m).

| Caso | Error medio (cm) | Desvío (cm) |
|---|---|---|
| `t_freno` calibrado (0,48 s) | +0,30 | 0,26 |
| `t_freno` = 0,36 s (sin calibrar) | +1,10 | 0,18 |

Tabla 4: diez celdas seguidas, error al final según el error de escala de la odometría (sin corrección por aperturas).

| Escala de la odometría | Error final medio (cm) | Peor (cm) |
|---|---|---|
| 1,00 | −0,3 | 2,4 |
| 1,05 | −13,2 | 15,7 |
| 0,95 | +14,1 | 15,7 |
| 0,90 | +29,9 | 33,1 |

**Lectura de la tabla 4:** un error de escala de la odometría del 5 % ya son ~1,4 cm por celda y ~14 cm en diez. **La calibración del diámetro de rueda (el logger de calibración) importa más que cualquier
corrección por aperturas.** El error longitudinal crece en línea recta con la distancia (~1,4 cm por celda con un 5 % de error) y solo se corrige con una referencia
externa (pared de frente, una apertura bien fechada) o calibrando bien.

---

## 8. Lo que corrijo de lo que dije antes

1. **Huecos de Wi-Fi.** En una primera versión (ganancias altas, `omega_n` = 2,5) los huecos hacían que el robot tocara una pared (6 de 8 corridas). Lo usé como argumento a favor de
   mover el movimiento por celdas a la ESP32. **Con las ganancias ajustadas ese argumento se debilita**: con 0,5 huecos por segundo de 0,2–0,4 s (mi supuesto, la frecuencia real no la sé) hay 0 contactos y el error por celda
   sube de 0,5 a 1,1 cm de desvío. Hacerlo en la ESP32 sigue siendo mejor (parada más precisa y no depende del enlace), pero **ya no es necesario por ese motivo**.
2. **Corrección por aperturas laterales.** Esperaba que corrigiera el error de la odometría. En el simulador **no lo hace de forma confiable**: con 4–33 Hz en los laterales mejora algo con un error de escala del 5 %
   (de 15 a 6–12 cm en 14 celdas) pero **empeora cuando la odometría es buena** (de ~1 a 3–6 cm), y el aprendizaje de la escala con aperturas consecutivas (probado) lo empeoró. Queda apagado y documentado;
   puede servir con mejor temporización, no se justifica ahora.

---

## 9. Qué falta y riesgos

| Falta | Quién | Cuándo |
|---|---|---|
| Datos reales: calibración (REAL-04) para reemplazar los supuestos (S) | equipo, con el robot | primero |
| Port de la capa 1 a C (ESP32) con los mismos vectores de prueba que el Python | yo, cuando el firmware S3 compile | después de la calibración |
| Nodos ROS que conecten capas 1 a 3 con `/wheel_state` y `/vision/*` | yo / equipo | cuando los puentes reales estén en `dev` |
| Probar el barrido lateral con flechas reales (enfoque a 7–16 cm, ángulo de visión) | equipo | con la pista |
| Mejorar el control con el robot real (la sim no tiene el piso lustrado ni los traseros al 45 %) | equipo | con datos |

**Riesgos:** (1) los parámetros del simulador son en parte supuestos: un buen resultado en la sim **no** garantiza lo mismo en el piso; (2) el sonar real puede comportarse peor que mi modelo de
pérdidas a ángulos altos (no medí ángulos); (3) la semántica de las flechas (P1–P3) puede ser distinta; (4) las medidas de montaje de los sonares y la cámara en `dimensions.yaml` están marcadas como provisorias.

---

## 10. Cómo integrarlo (respetando las reglas del repo)

Issues sugeridos (a numerar según el tablero; no inventé números): `NAV-08` capa 1, `NAV-09` capa 2, `NAV-10` capa 3, `CAM-06` geometría y calibración de la cámara.
Una rama por issue desde `dev` actualizado, p. ej. `feature/NAV-08-capa1-control-celda`, PR a `dev` con `Closes #n`, commits `feat(navegacion): ...`.

```bash
# desde la raíz del repo, con la rama ya creada
unzip -o nav_capas_1_a_3.zip -d /tmp/nav && cp -r /tmp/nav/nav_capas_1_a_3/* .
cd ros2_ws/src/caid_navigation && PYTHONPATH=. python3 -m pytest -q test/test_control_pared.py test/test_movimiento_celda.py \
  test/test_decision_flechas.py test/test_grafo.py test/test_marcadores_grafo.py test/test_camara_geometria.py   # 68 pruebas
```

`test_camara_geometria.py` necesita `opencv-python` y `numpy`; los demás solo la biblioteca estándar. Los módulos dependen de `caid_navigation.mapa` (ya en `dev`).
