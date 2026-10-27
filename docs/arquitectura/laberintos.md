# Formato de laberintos (YAML con grilla ASCII)

> Documento de INF-02 (formato) y DESC-04 (generador). Lo lee `ros2_ws/src/caid_gazebo/scripts/gen_maze.py`, que escribe el mundo de Gazebo en `ros2_ws/src/caid_gazebo/worlds/`. Pensado para que también lo lea `tools/gridsim`.
> Estados: **O** informado por el equipo como oficial, **R** del reglamento, **M** medido en un archivo del equipo, **C** calculado, **P** supuesto nuestro que hay que confirmar con la organización.

## 1. Medidas que usa el generador

| Cosa | Valor | Estado | Fuente |
|---|---|---|---|
| Ancho del pasillo | 320 mm | O | Dato del equipo (24/10) |
| Bloque de pared | 320 x 320 x 170 mm (macizo) | O, M | Dato del equipo y `Bloque de pared reglamentario.stl` (320 x 320 x 170) |
| Cubos | 20 mm de lado, rojo / verde / azul | R | `desafio-laberinto.md` |
| **Altura del cubo** | Apoya a **108,5 mm** del piso: la altura de la base del brazo (cara superior del 2.º piso del robot) | O, C | Dato del equipo (26/10); 108,5 = 104,5 + 4 en `dimensions.yaml` |
| Soporte del cubo | Columna cuadrada de 40 mm de lado, de 108,5 mm de alto | P | Desconocido (equipo, 26/10): "probablemente sobre algún cuerpo de esa altura" |
| Punta de la flecha | 70 mm de ancho, 50 mm de alto | R | Anexo del desafío |
| Cuerpo de la flecha | 30 mm de ancho; largo L = 30 / 60 / 100 mm | R | Anexo del desafío |
| Largo total de la flecha | 80 / 110 / 150 mm | C | L + 50; coincide con los STL `Flecha corta / media / Larga` |
| Espesor de la flecha | 2 mm | M | STL del equipo |
| Altura del centro de la flecha | 115 mm sobre el piso | P | **Arbitraria** (dato del equipo, 26/10). Se cambia con `altura_flechas_mm` |
| Inclinación de las flechas | hasta ±30° | R | Reglamento del desafío |
| Canasto | 44 x 44 x 20 mm, pared de 2 mm, fondo de 1 mm | M, P | `Canasto Ficticio.stl`: es un canasto de prueba, no el oficial |
| **Altura del canasto** | El borde de su pared trasera a **108,5 mm** (la base queda a 88,5 mm), sobre una columna de 36 mm | O, P | Dato del equipo (confirmado 26/10): las paredes superiores del agujero a la altura del 2.º piso, y la base que lo sostiene más baja. La columna es nuestra |
| Robot | 163 mm de ancho, 278 mm de largo (sin el brazo) | M | `hardware/cad/dimensions.yaml` |

Números que salen de acá (C):

- Con un pasillo de 320 mm y un robot de 163 mm sobran **78,5 mm por lado**.
- Para girar en el lugar en un pasillo recto hace falta que quepa la diagonal del robot: √(278,1² + 163²) = **322,3 mm**, que es **2,3 mm más** que el pasillo. Con el brazo extendido en la pose del ensamblado el largo es mayor (336 mm de punta a punta), así que con el brazo así no hay forma de girar 180° dentro de una sola celda de 320 mm.
- Un soporte de 40 mm pegado a una pared deja **275 mm** libres: el robot pasa. El mismo soporte **en el eje del pasillo** deja 140 mm por lado: no pasa. Y en una **esquina** también molesta, porque al girar el robot barre casi toda la celda: los cubos van en tramos rectos, fondos ciegos o zonas abiertas, nunca en una esquina.

## 2. Coordenadas

- La grilla se dibuja con el norte arriba: la fila 0 es la de más al norte y la columna 0 la de más al oeste.
- Mundo de Gazebo: el origen está en la esquina suroeste de la grilla, **x al este, y al norte**, z hacia arriba desde el piso. El centro de la celda (fila `r`, columna `c`) es `x = (c + 0,5)·0,32`, `y = (filas − r − 0,5)·0,32`.
- Direcciones: `N`, `E`, `S`, `O` (oeste). El `yaw` del robot es 0 al este, π/2 al norte, π al oeste y −π/2 al sur.

## 3. El archivo YAML

```yaml
nombre: 'E6 Salida abierta: laberinto grande, sala abierta y entrada entre paredes'
descripcion: texto libre
celda_m: 0.32               # ancho del pasillo y lado del bloque (opcional, 0.32 por defecto)
alto_pared_m: 0.17          # alto del bloque (opcional, 0.17 por defecto)
altura_flechas_mm: 115      # centro de las flechas sobre el piso (opcional, 115 por defecto)
altura_apoyo_cubo_mm: 108.5 # donde apoyan los cubos: la base del brazo (opcional)
altura_borde_canasto_mm: 108.5  # borde de la pared trasera del canasto (opcional)
pedestal_lado_mm: 40        # lado de la columna que sostiene cada cubo (opcional)
inicio_dir: E               # hacia dónde mira el robot en la S
canastos_pared: O           # opcional: pared contra la que se apoyan los canastos; sin esto, al centro de su celda
mapa: |
  ~~~~~~~~################
  ~~~~~~~~##.............#
  ...
significado_colores:        # opcional: qué cubo indica cada color de flecha (esto es lo que se puede cambiar)
  rojo: [R]
  verde: [G]
  azul: [B]
  gris: []                  # lista vacía = la flecha no significa nada y se ignora
cubos:                      # opcional: dónde apoya cada cubo
  R: {lado: N}              # contra la pared norte de su celda (por defecto, el generador elige una)
flechas:
- {celda: [2, 6], pared: N, dir: E, largo: 3, color: rojo, inclinacion: 12}
objetos:
- {tipo: bloque_8cm, celda: [5, 4], lado: S}
```

El nombre del archivo (sin `.yaml`) es el nombre del mundo: `mazes/e6_salida_abierta.yaml` produce `worlds/e6_salida_abierta.world`.

### 3.1 Símbolos del mapa

| Símbolo | Significado |
|---|---|
| `#` | Bloque de pared |
| `.` | Piso libre (pasillo o sala) |
| `~` | Piso libre del **área abierta exterior**, sin paredes. Puede estar en el borde de la grilla |
| `S` | Punto de partida del robot (centro de la celda), mirando hacia `inicio_dir` |
| `R` `G` `B` | Cubo rojo, verde, azul. Cada uno apoya sobre una columna de 108,5 mm, pegada a una pared de su celda (o al centro si la celda no tiene paredes). Uno de cada |
| `1` `2` `3` | Celda del canasto del cubo rojo (1), verde (2) y azul (3), con el borde a 108,5 mm. Uno de cada |
| `Z` | Zona prohibida: piso libre con un rectángulo rojo pintado. El robot no debe entrar |

Hay dos formas de zona de inicio:

- **Cerrada** (E1 a E5): un área de 2 x 3 celdas dentro de los muros, con la `S` en el medio de la columna este y los canastos contra la pared oeste (`canastos_pared: O`).
- **Abierta** (E6): un área de `~` sin paredes con la `S` y los tres canastos; el laberinto está a un lado y se entra por una puerta. Ver 3.5.

El borde de la grilla solo puede tener `#` o piso abierto (`~`, `S`, canastos, `Z`): un pasillo no puede llegar al borde.

### 3.2 Flechas

Cada flecha se pega a la **cara de un bloque** que da a una celda libre. Campos:

| Campo | Qué es |
|---|---|
| `celda` | `[fila, columna]` de la celda libre desde cuyo lado se ve la flecha |
| `pared` | Pared de esa celda donde se pega: `N`, `E`, `S`, `O`. Tiene que ser un bloque |
| `dir` | Hacia dónde apunta, en brújula. Tiene que ser **paralela a la pared** (en una pared N o S apunta `E` u `O`; en una E u O apunta `N` o `S`) |
| `largo` | 1, 2 o 3 (cuerpo de 30, 60 o 100 mm) |
| `color` | `gris`, `rojo`, `verde` o `azul`. Qué significa cada uno lo define `significado_colores` (ver abajo) |
| `inclinacion` | Grados, de −30 a 30, opcional. Positiva = antihoraria vista desde el pasillo (una flecha que apunta a la derecha sube la punta) |
| `desplaza_mm` | Opcional. Corrimiento a lo largo de la pared (positivo al este en paredes N/S, al norte en E/O). Sirve para poner varias flechas en la misma pared |
| `altura_mm` | Opcional. Altura del centro sobre el piso. Por defecto `altura_flechas_mm` |

**Qué significa cada color: es un parámetro.** El reglamento solo dice que en las bifurcaciones el color indica el camino al objetivo, así que no fijamos más que eso. `significado_colores` asigna a cada color una lista de cubos (`R`, `G`, `B`):

- Por defecto: `rojo: [R]`, `verde: [G]`, `azul: [B]` y `gris: []`.
- **`gris` (y cualquier color con la lista vacía) no significa nada: la flecha es decorativa y el robot tiene que ignorarla.** El generador no la verifica ni la cuenta, y no pone ninguna sola por su cuenta (los 6 escenarios no tienen flechas grises).
- Para entrenar con otra regla se cambia la lista de ese color (por ejemplo `rojo: [G]`, o `azul: [R, G]`, que valdría para dos cubos) y el validador comprueba las flechas contra el nuevo significado. `--sugerir-flechas` también lo respeta.

**Cómo se lee una flecha con significado.** El reglamento dice que la longitud indica cuántos segmentos rectos siguen antes de la curva. Lo bajamos a estas reglas, que el generador verifica para cada flecha con significado:

1. La flecha apunta en la dirección `dir` y esa dirección es el **primer paso de un camino más corto** desde su celda hasta el cubo (o cubos) que indica su color.
2. `largo` es la **clase** de tramo recto que sigue en esa dirección, contando celdas desde la de la flecha (sin contarla): **1** = un tramo recto antes de la curva (cuerpo de 30 mm), **2** = dos tramos (60 mm), **3** = **tres o más** tramos (100 mm). El 3 no es un tope: es un mínimo. La flecha larga asegura al menos 3 tramos rectos seguidos, que alcanza para acelerar.
3. En cada punto donde decide un camino hay **una flecha por cubo**, del color de ese cubo, y todas apuntan al lado que corresponde a ese cubo.

Qué es del reglamento y qué es nuestro: el reglamento (anexo) dibuja las flechas en gris oscuro, da la regla de 1 / 2 / 3 y dice que el color indica el camino en las bifurcaciones. Que **se cuenten los pasos desde la celda de la flecha** y la asignación rojo-R, verde-G, azul-B son convenciones nuestras (P), por eso son parámetros.

Dónde se pegan (lo que sugiere `--sugerir-flechas`):

- **En una curva o en una T**: en la pared del frente, apuntando hacia el lado al que gira la ruta. El `largo` es lo que sigue recto después de girar.
- **En un tramo recto o en una bifurcación en que la ruta sigue derecho**: en una pared lateral, apuntando hacia adelante. En la entrada del laberinto hay una de este tipo.
- Si en una misma pared hay varias, `--sugerir-flechas` las reparte en ranuras (desplazamiento a lo largo de la pared, altura respecto de la base): 1 flecha = (0, 0); 2 = (−80, 0) y (+80, 0); 3 = (−80, 0), (+80, +10) y (+80, −70); 4 = dos columnas (±80) con dos alturas (+10 y −70). Las flechas de una pared compartida no se inclinan, para que quepan en los 320 mm de la cara.
- No se ponen flechas dentro de las áreas abiertas (donde hay un bloque de 2 x 2 celdas libres): ahí no hay pasillo ni curva.

Limitación de la convención: una flecha va a lo largo de la pared, así que **no puede indicar un giro en una bifurcación pasante** (la ruta dobla hacia una abertura lateral mientras el pasillo sigue de frente). El validador lo avisa como `AVISO`. En esos puntos el robot decide explorando.

### 3.3 Objetos

| Tipo | Tamaño | Cómo es |
|---|---|---|
| `bloque_8cm` | 80 x 80 x 80 mm, gris | Fijo. Deja pasar al robot **solo si está pegado a una pared** (quedan 240 mm libres; el robot necesita 163 mm + 15 mm de holgura por lado = 193 mm) |
| `bloque_20cm` | 200 x 200 x 200 mm, gris | Fijo. **Cierra el pasillo de 320 mm** (quedan 120 mm); en una sala abierta se esquiva. Es el bloque de PVC del reglamento general |
| `distractor_amarillo`, `distractor_naranja`, `distractor_violeta` | cubo de 20 mm | Móvil (9,9 g), como los cubos objetivo pero de otro color. Por defecto también va sobre una columna a 108,5 mm; con `elevado: false` queda en el piso |

Campos: `tipo`, `celda`, y la posición con **una** de estas formas: `lado: N|E|S|O` (apoyado contra esa pared de la celda), `lado: auto` (el generador elige una pared lateral libre de flechas), o `dx_mm` y `dy_mm` (corrimiento respecto del centro de la celda, este y norte). Sin nada, queda al centro.

**Cubos objetivo.** Se configuran en la sección `cubos` con `lado` (pared), `dx_mm`/`dy_mm` o `elevado: false`. Si no se dice nada, el generador elige una pared lateral de la celda que no tenga una flecha. Que el soporte vaya **pegado a una pared lateral** deja el cubo al costado del robot cuando este avanza por el pasillo, que es desde donde el brazo (apunta hacia atrás y gira hasta cerca de las ruedas delanteras) puede tomarlo.

### 3.4 Qué valida el generador

Errores (no se escribe el mundo): borde abierto; símbolos desconocidos; falta o sobra algún `S`, cubo o canasto; el robot mira a una pared; un cubo, un canasto o un objeto que no se puede alcanzar o que está en una zona prohibida o dentro de un bloque (esto incluye un soporte en el eje del pasillo o en una esquina, que cierra la celda); canastos o soportes sin la pared donde apoyarse; una flecha fuera de la cara del bloque, solapada con otra, **tapada por el soporte de un cubo** de su misma pared, con inclinación mayor de 30° o con la dirección o el largo que no cumplen la regla de 3.2; una entrada mal hecha (3.5).

Avisos e información: bifurcaciones de la ruta a un cubo sin una flecha que lo indique o pasantes, flechas decorativas ignoradas, celdas donde un objeto no deja pasar al robot, largo de las rutas, la altura a la que apoya cada cubo y las cuentas de la sección 1.

### 3.5 Área abierta exterior y entrada

Con `~` se dibuja un área **sin paredes**, donde está la `S` y están los canastos. El laberinto se separa de ella con muros y tiene que haber **una única entrada**:

- la entrada es una celda libre que toca el área abierta, y las dos celdas de sus costados (perpendiculares al sentido de entrada) tienen que ser bloques: la entrada está **limitada entre paredes**;
- si hay más de una celda libre que toque el área abierta, es un error;
- la `S` y los canastos tienen que estar en el área abierta (si no, aviso).

En E6 la entrada son 2 celdas de largo entre paredes: el robot sale, deja el cubo en su canasto sin tener ninguna pared cerca, y tiene que volver a encontrar la abertura en el muro.

## 4. Uso

Desde la raíz del repo:

```bash
# generar y validar todos los escenarios (escribe worlds/*.world y las vistas en docs/pruebas/img/*.svg)
python3 ros2_ws/src/caid_gazebo/scripts/gen_maze.py ros2_ws/src/caid_gazebo/mazes/*.yaml --svg docs/pruebas/img --rutas

# solo validar un laberinto nuevo
python3 ros2_ws/src/caid_gazebo/scripts/gen_maze.py ros2_ws/src/caid_gazebo/mazes/mi_laberinto.yaml --solo-validar

# proponer las flechas de un laberinto nuevo (se pegan en el YAML y se ajustan a mano)
python3 ros2_ws/src/caid_gazebo/scripts/gen_maze.py ros2_ws/src/caid_gazebo/mazes/mi_laberinto.yaml --solo-validar --sugerir-flechas

# pruebas del generador
python3 ros2_ws/src/caid_gazebo/scripts/test_gen_maze.py
```

Necesita `python3-yaml` (ROS 2 lo instala; se comprueba con `python3 -c 'import yaml'`). Para ver cada escenario con la simulación, ver `docs/pruebas/escenarios.md`.

## 5. Supuestos que hay que confirmar con la organización

1. **Lectura de las flechas**: qué cubo indica cada color (hoy rojo-R, verde-G, azul-B, parametrizable), si las grises significan algo, si se cuentan las celdas desde la flecha (nuestra regla) o desde otro punto, y si la celda de la curva cuenta.
2. **Ubicación de las flechas en una bifurcación**: pared frontal o lateral, y si pueden apuntar hacia arriba o abajo para indicar "seguir derecho".
3. **Zonas rojas**: si hay dentro del laberinto, de qué tamaño y si son rectángulos con línea o rellenos. El reglamento general habla de 40 x 40 cm, que **no entra en un pasillo de 32 cm**: usamos celdas completas (32 x 32 cm con línea roja de 20 mm) y las ponemos en áreas abiertas.
4. **Cubos**: cómo se sostienen a 108,5 mm del piso (columna, estante, pared) y de qué tamaño. Y el **canasto**: forma, medidas, ubicación y cómo queda su borde a esa altura.
5. **Obstáculos** que ponen el día de la prueba: tamaños. Un bloque de 20 cm cierra un pasillo de 32 cm, y uno de 8 cm solo se esquiva si está pegado a la pared.
6. **Colores** del piso y de los bloques: usamos un piso gris medio y bloques claros.
7. **Área abierta**: si el robot sale a un área sin paredes y cómo se marca la entrada.

## 6. Lo que no cubre todavía

- Mundos aleatorios por semilla y `legacy_maze2` (el resto de DESC-04): el generador acepta `celda_m` y `alto_pared_m`, así que `legacy_maze2` se carga convirtiendo su grilla a este formato.
- Textura PNG de las flechas: la punta triangular se arma con 25 tiras de 2 mm porque el SDF no tiene prismas. Si hace falta más fidelidad, se reemplaza por la malla del STL.
- La orientación y el alcance del brazo en el mundo: se resuelve con MAN-02 (poses con nombre y límites).

## Historial

- 26/10/2026: primera versión.
- 26/10/2026: cubos y canastos a la altura de la base del brazo (108,5 mm) sobre columnas; área abierta exterior con entrada entre paredes (E6); altura de flechas configurable; el largo 3 es "3 o más".
- 26/10/2026: el significado de cada color de flecha pasa a ser un parámetro (`significado_colores`); las grises no significan nada y se ignoran; una flecha por cubo en cada punto de decisión.
