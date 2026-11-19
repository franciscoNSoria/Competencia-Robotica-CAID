# Guía para Claude Code: calibrar el robot armado para que Gazebo se parezca al robot en el piso

> **Para quién:** una sesión de Claude Code que trabaje en el repo `franciscoNSoria/Competencia-Robotica-CAID`.
> **Estado de partida:** ver [`estado-repo-2026-11-19.md`](../analisis/estado-repo-2026-11-19.md) (sección 6 = tabla de diferencias sim vs real).
> **Qué es cada marca:** **M** medido en el robot, **G** leído en el repo, **P** propuesta (sin probar), **?** dato que falta.

---

## 0. Reglas del repo que NO se rompen

- Una rama por tarea: `feature/<ID>-<descripcion>`, creada desde `dev` actualizado. PR **a `dev`**, con `Closes #n`. Squash and merge. Otra persona aprueba.
- Commits con formato `tipo(alcance): descripción` (`feat`, `fix`, `docs`, `test`, `refactor`, `chore`).
- No hacer push a `main` ni a `dev`. No crear ramas sin ID. Al abrir un PR con `gh`, **siempre** `--head <rama>` (si no, toma la rama que esté en uso).
- Issues relacionados: **#59 REAL-04** (calibración de movimiento, odometría y modelo medido), **#58 REAL-03** (alimentación), **#42 SIM-01** (emulador de firmware), **#43 SIM-02**.
- Cada tarea documenta en el lugar que indica su issue (`docs/pruebas/odometria.md`, etc.). Sin documentación no está terminada.
- Probar sin robot siempre que se pueda. **No inventar números**: si un parámetro no está medido, dejarlo en `calibracion_real.yaml` con `valor: null` y `fuente: pendiente`.

---

## 1. Objetivo y qué significa "100 %"

**Objetivo:** que un mismo programa de navegación se comporte en Gazebo como en el robot sobre el piso.

Igualar al 100 % es imposible (el piso, la batería y el patinaje cambian de un día a otro). Entonces el 100 % se define como **criterios de aceptación medibles**, verificados con un informe automático (`validar_sim_vs_real.py`, sección 6):

| Magnitud | Criterio (propuesto) |
|---|---|
| Recta de 1 m con la misma orden | media sim vs media real: diferencia ≤ la dispersión real (±1,3 cm) |
| Recta de 2 m | ≤ 3 cm de diferencia entre medias |
| Giro de 90° | diferencia entre medias ≤ 3°; dispersión simulada del orden de la real |
| Desvío lateral por metro | distribución simulada con media y desvío del orden del real (real ≈ 5 cm/m, sin signo fijo) |
| Respuesta de velocidad a un escalón | tiempo de subida y piso de velocidad dentro del 15 % |
| Sonar a distancia fija | sesgo ≤ 5 mm y tasa de ecos perdidos del mismo orden |
| Cámara | FOV y distorsión iguales; retraso y fps del mismo orden |
| Ticks de los encoders | para la misma orden de movimiento, diferencia ≤ 5 % |

**Estos umbrales son propuestos:** el equipo los confirma después de ver la dispersión real de sus datos.

---

## 2. Lo que existe hoy

**En `dev` (G):**
- `ros2_ws/src/caid_description/urdf/{caid_bot.urdf.xacro,gazebo.xacro}`: el modelo y el plugin de Gazebo. Parámetros provisorios en `gazebo.xacro`: `wheel_separation` 0,204, límites de velocidad ±0,5 m/s y ±3 rad/s, aceleración ±1 m/s² y ±5 rad/s², sonares `gpu_lidar` de 5 rayos (cono 15°, σ = 5 mm, 20 Hz), cámara 640×480 a 15 Hz con FOV 1,047 rad.
- `hardware/cad/dimensions.yaml`: cotas del robot (el URDF lee de acá).
- `ros2_ws/src/caid_bringup/{launch/sim.launch.py,config/sim/bridge.yaml,caid_bringup/scan_to_range.py}`.
- `tools/sim/prueba_giro.sh`: calibra `wheel_separation` comparando el giro real de Gazebo contra la odometría.
- `ros2_ws/src/caid_gazebo/scripts/gen_maze.py` y mundos con `celda_m` = 0,32 (pasillo real de 320 mm).
- Visión calibrada: `caid_vision/config/{calibracion_esp32cam.yaml,colores.yaml}`.

**Solo en ramas (G):** `feature/laberinto-cuenta-giros` trae los nodos del robot **real**: `sonar_bridge.py` (separa los sonares de `/wheel_state`), `odom_ticks.py` (odometría por flancos con 64,5 mm y 291 mm), `calibracion_robot.py` (manda diámetro, trocha y activa la guardia por sonar), el firmware `robot_4ruedas` con freno por sonar y `tools/ruedas/*`.

**Del equipo, fuera del repo todavía:** `calibracion_logger.py`, `analizar_calibracion.py` y `PROTOCOLO_CALIBRACION.md` (los hizo la sesión anterior; registra CSV de ticks, sonares y eventos, y calcula diámetro/trocha/sesgo). **Primer paso: traerlos al repo en `tools/calibracion/`.**

**El robot real (contrato de tópicos, M/G):**

| Tópico | Tipo | Notas |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | m/s y rad/s |
| `/robot_cmd` | `std_msgs/Float32MultiArray` | ids 1–9 (duty, velocidad por rueda, mover distancia, parar, configurar, curva, girar, guardia, relación de giro) |
| `/wheel_ticks` | `std_msgs/Int32MultiArray` | 4 flancos con signo `[FL, FR, RL, RR]`, 20 Hz |
| `/wheel_state` | `std_msgs/Float32MultiArray` | 34 valores, 10 Hz: [0–3] velocidad medida, [4–7] pedida, [8–11] duty, [26–28] sonares (frontal, izq., der.; −1 = sin eco), [29] guardia |

La simulación usa `/sim/*`. Un nodo que consuma `/cmd_vel`, `/wheel_ticks`, `/wheel_state` debería correr igual en los dos mundos: **ese es el objetivo del emulador**.

---

## 3. Cómo se acopla todo: un archivo de calibración único

Un solo archivo, `ros2_ws/src/caid_bringup/config/real/calibracion_real.yaml`, con cada parámetro, su valor, su incertidumbre y su fuente. De ahí lo leen: el emulador, el xacro (por argumento) y el informe de validación. **Nada de números sueltos en el código.**

```yaml
# Estructura (valores de ejemplo; los reales salen de las pruebas)
meta: {fecha: null, piso: null, bateria: null, version_firmware: null}
ruedas:
  diametro_efectivo_m: {valor: 0.0645, incert: null, fuente: "3/11, falta repetir"}
  trocha_efectiva_m:   {valor: 0.291,  incert: 0.02, fuente: "giros de 360°; falta 90°"}
  flancos_por_vuelta:  {valor: 40,     fuente: "video"}
  relacion_trasera_delantera: {valor: null, fuente: pendiente}   # ~0.45 según banco
control:
  v_min_ms: {valor: 0.13}   # piso del PI: 25 flancos/s
  zona_muerta_flancos_s: {valor: 12.5}
  tau_velocidad_s: {valor: null, fuente: "prueba de escalón"}
  watchdog_s: {valor: 1.0}
movimiento:
  deriva_lateral_cm_por_m: {media: null, desvio: null}
  ruido_giro_deg: {desvio: null}
sonares:                         # uno por sensor: frontal, izquierdo, derecho
  frontal:  {sesgo_m: null, escala: null, sigma_m: null, perdida_pct: null, rango_min_m: null, cono_deg: null, periodo_s: 0.12}
  izquierdo: {periodo_s: 0.24}
  derecho:  {periodo_s: 0.24}
red: {latencia_s: null, huecos_s: null}
camara: {fx: 947.24, fy: 950.24, cx: 292.90, cy: 218.53, k1: 0.0915, k2: 0.1189, ancho: 640, alto: 480, fps: null, retraso_s: 0.044}
fisica_gazebo: {masa_kg: null, mu_piso: null, mu_rueda: null}
```

---

## 4. Códigos a escribir

Cada ítem dice **dónde va**, **qué entra y sale**, **cómo se prueba sin robot** y **de qué issue es**. Orden de trabajo sugerido: 4.1 → 4.2 → 4.3 → 4.4 → 4.5 → 4.6.

### 4.1 `tools/calibracion/` (issue #59)
Subir `calibracion_logger.py`, `analizar_calibracion.py`, `test_calibracion.py` y el protocolo. **Ampliar el logger** para que también registre, en el mismo CSV, los eventos de los experimentos 4.2 a 4.5.

### 4.2 Experimentos de movimiento (#59)
`tools/calibracion/exp_movimiento.py`, con tres modos:
1. **Escalón de velocidad:** orden de `0` a `v` (varios `v` entre 0,13 y 0,39 m/s), registrar `/wheel_state[0:4]` a 10 Hz. Salida: tiempo de subida, constante de tiempo `tau`, piso y zona muerta, relación trasera/delantera.
2. **Recta de N m:** como `recta1m`/`recta2m` del logger. Salida: diámetro corregido, desvío lateral (cm/m) y su signo.
3. **Giros de 90/180/360°:** a varias velocidades de giro. Salida: trocha efectiva por ángulo (puede **no** ser la misma para 90° y para 360°).
Cada experimento: **≥ 5 repeticiones**; el informe da media y desvío, y **no** propone cambiar nada con menos de 5.

### 4.3 Caracterización del sonar (#59)
`tools/calibracion/exp_sonar.py`: con el robot quieto frente a una pared de bloque real, distancias de 3 a 60 cm (paso de 3 cm hasta 30, luego de 10), **y a distintos ángulos** (0°, 15°, 30°, 45°). Por cada punto, 100 lecturas. Salida por sensor: **sesgo, escala, desvío, % de ecos perdidos, distancia mínima fiable, ángulo máximo con eco**. Además la **tela** (el sillón) y una esquina. Las tres unidades (frontal HC-SR04, laterales RCWL-9610A) por separado.

### 4.4 Caracterización de la cámara (#59 o issue nuevo `CAM-06`)
`tools/calibracion/exp_camara.py`:
- **Intrínsecos:** ya hay `calibrar_lente` en `caid_vision`; **no reescribir**. Verificar que el FOV de la simulación salga de acá: `fx = 947 px` a 640 px de ancho da un FOV horizontal ≈ 37° (la sim usa 60°).
- **Retraso y fps reales:** marcar el cuadro en que aparece un evento (un LED o un golpe de color a la cámara) y medir el retraso hasta el nodo; fps efectivos.
- **Extrínseca:** la cámara va a 138,5 mm del piso (`dimensions.yaml`, derivada; "la medición a mano dio 137,5"). Medir en el robot armado: altura, cabeceo y posición. Con un objeto a distancias conocidas se ajustan altura y cabeceo (módulo `camara_geometria.py`, ver el informe de navegación).

### 4.5 Latencia y huecos de red (#59)
`tools/calibracion/exp_red.py`: publicar `/robot_cmd` con marcas y medir ida y vuelta; estadística de huecos de `/wheel_state` (esperado 10 Hz) durante 10 minutos en movimiento. Salida: latencia media, p95 y distribución de huecos.

### 4.6 `ros2_ws/src/caid_gazebo/caid_gazebo/sim_fw_emulator.py` (issue #42 SIM-01)
Nodo que reproduce el **contrato del firmware real** sobre Gazebo, leyendo `calibracion_real.yaml`:

- **Entra:** `/cmd_vel`, `/robot_cmd` (al menos ids 2, 3, 4, 5, 7 y 8), `/sim/joint_states`, `/sim/ultrasonic/*`.
- **Sale:** `/sim/cmd_vel` (a Gazebo), `/wheel_ticks`, `/wheel_state` (los mismos 34 índices), con la misma frecuencia que el robot.
- **Reproduce:**
  1. Conversión de `/cmd_vel` a velocidades por rueda **con la trocha real (0,291)**, no la de Gazebo.
  2. Piso de velocidad (0,13 m/s), zona muerta, tope de 0,395 m/s, relación trasera/delantera.
  3. Retraso de primer orden con `tau`.
  4. Watchdog de 1 s sin órdenes.
  5. Cuantización de ticks a 40 flancos por vuelta con el diámetro efectivo.
  6. Sonares: sesgo, escala, ruido, pérdidas, rango mínimo, ángulo máximo, período 120/240 ms.
  7. Huecos y latencia de red (muestrear de la distribución medida).
  8. Guardia por sonar (misma lógica que `firmware/pruebas/comun/guardia.h`).
- **Prueba:** unitaria (sin Gazebo) con entradas sintéticas; el emulador tiene que dar `/wheel_state` con 34 valores y el mismo orden que `main.c`.
- **Importante:** la física de giro (patinaje) hay que **ajustarla en Gazebo** (fricción, masa) hasta que la trocha efectiva física sea la medida; si no se logra, el emulador **corrige en la orden** y lo documenta.

### 4.7 `tools/calibracion/ajustar_sim.py` (#59)
Optimiza los parámetros físicos de Gazebo (fricción de ruedas, masa, `wheel_separation` del plugin) para minimizar el error entre las pruebas simuladas y las reales:
- Corre en Gazebo **sin ventana** (`gui:=false`) los mismos escenarios de 4.2 (recta, giro) con parámetros candidatos.
- Métrica: diferencia de medias normalizada por la dispersión real.
- Método: búsqueda por rejilla (pocos parámetros: ≤ 3) antes que cualquier optimizador; guardar cada evaluación en un CSV.
- Resultado: parámetros nuevos en `calibracion_real.yaml` y en `gazebo.xacro` (por argumento xacro, no editando a mano).
- **Cuidado:** el RTF (factor de tiempo real) cambia con la GPU; usar tiempo de simulación (`use_sim_time`), no de pared.

### 4.8 Actualizar el mundo y el modelo (issues `DESC-05`, nuevo)
- Verificar que **todos** los mundos usados para validar tengan `celda_m` = 0,32 (el pasillo real) y bloques de 320×320×170 mm. `maze_2` usa 0,45 y no sirve para validar.
- Material de las paredes: cómo las ve el sonar (en Gazebo `gpu_lidar` refleja todo).
- **Mover el brazo:** `hardware/cad/dimensions.yaml` (`brazo.base_x`, etc.) y los límites del brazo; medir el desplazamiento real cuando el robot esté armado. Después correr `test_urdf.py`.
- FOV de la cámara y distorsión: leerlos de `calibracion_real.yaml`.

### 4.9 `tools/calibracion/validar_sim_vs_real.py` (#59)
Lee una sesión real y una simulada con los **mismos escenarios**, calcula las métricas de la sección 1 y escribe `docs/pruebas/sim_vs_real.md` con una tabla **cumple / no cumple** por magnitud. Es lo que se corre antes de afirmar que "la sim se parece al robot".

---

## 5. Orden de trabajo y ramas

| Paso | Issue | Rama | Entregable |
|---|---|---|---|
| 1 | #59 | `feature/REAL-04-calibracion-movimiento` | 4.1 + 4.2 + 4.3 con pruebas y protocolo en `docs/pruebas/odometria.md` |
| 2 | #59 | misma | datos reales de la sesión de calibración (CSV, no más de unos MB) |
| 3 | #42 | `feature/SIM-01-emulador-firmware` | 4.6, con `calibracion_real.yaml` completo |
| 4 | #59 | `feature/REAL-04-ajuste-sim` | 4.7 |
| 5 | nuevo | `feature/DESC-05-mundo-y-modelo` | 4.8 |
| 6 | #59 | misma que 4 | 4.9 + informe `sim_vs_real.md` |

**Antes del paso 1:** integrar a `dev` los nodos del robot real (`sonar_bridge`, `odom_ticks`, `calibracion_robot`) que hoy están en `feature/laberinto-cuenta-giros`. Si eso no se hace, el emulador y el robot no comparten tópicos.

---

## 6. Trampas conocidas (no repetir)

- **Dominio ROS fijo en 71** en el firmware: todas las terminales en `ROS_DOMAIN_ID=71`.
- La ESP32 **no guarda calibración** al apagarse: manda `[5, diametro, trocha, frenado]` por `/robot_cmd` al conectar (lo hace `calibracion_robot.py`, varias veces, porque no hay confirmación).
- **Dos programas mandando `/robot_cmd` se pisan.** Cerrar el panel de ruedas antes de calibrar.
- La trocha de 291 mm sale de giros de 360°: **no asumir** que vale para 90°.
- El sonar pierde lecturas cerca de la pared (32 % a 5,5 cm) y **no ve telas en diagonal**; la sim hoy ve todo.
- Los traseros giran al ~45 % de los delanteros; un solo factor no los iguala.
- El encoder es de **un canal**: en marcha atrás el signo de los ticks es el del último duty y puede equivocarse unos flancos.
- El monitor serie de la ESP32 imprime la contraseña del Wi-Fi: no pegarla en ningún chat, issue ni PR.
- `gui:=false` para correr la sim sin ventana; verificar que no quedó `gz sim` vivo (ver `simulacion.md`).

---

## 7. Datos que faltan (pedírselos al equipo)

- Masa del robot armado y su reparto (pesar; `REAL-04` lo pide).
- Posición final del brazo (cuánto se corre hacia el frente).
- Tipo exacto de cada sonar (HC-SR04 vs RCWL) y cómo se alimentan.
- Resultado de la competencia y rama que corrió.
- Si el piso de la sala de pruebas es el de la competencia.

---

## 8. Prompt de arranque sugerido para Claude Code

> Leé `CONTRIBUTING.md`, [`estado-repo-2026-11-19.md`](../analisis/estado-repo-2026-11-19.md) y esta guía. Trabajá el paso 1 de la sección 5: creá la rama `feature/REAL-04-calibracion-movimiento` desde `dev`, subí `tools/calibracion/` con el logger, el análisis, las pruebas y el protocolo; ampliá el logger con los experimentos 4.2 y 4.3; mantené `calibracion_real.yaml` con `valor: null` donde no haya dato. No hagas push a `dev` ni abras PR sin que el equipo lo pida. Mostrá qué probaste y qué no pudiste probar.
