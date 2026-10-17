# Índice de STL

Fuente de nombres, cantidades y pesos: `hardware/bom/Lista_de_partes` (provista por la organización).
Fuente de medidas: caja envolvente (*bounding box*) leída de los vértices de cada STL, en
**milímetros** y en los ejes propios del archivo (x × y × z). No son medidas de la pieza montada.

Cada STL tiene su propio origen, en una esquina de la pieza o cerca. Por eso la posición de cada
pieza en el robot no sale del STL: sale de `hardware/cad/dimensions.yaml`.

## Chasis de acrílico y cubo

| Archivo | Qué es | x × y × z (mm) | Uso en el URDF |
|---|---|---|---|
| `Chasis Armadillo.stl` | Placa de acrílico. Se usa dos veces: piso 1 y piso 2 (la lista de partes pide 2 chasis 4WD) | 256,0 × 150,0 × 4,0 | Sí: visual y colisión de ambos pisos |
| `Cubo.STL` | Cubo de la competencia | 20,0 × 20,0 × 20,0 | Sí: objeto del mundo de simulación |

## Piezas impresas (`ARM2-CUDAR-<conjunto>-<subconjunto>-<pieza>`)

Todas tienen `ARM2-CUDAR-` delante en el nombre del archivo; acá se abrevia.
"Peso" es el peso unitario en gramos.


### Subconjunto 1: brazo

| Archivo | Descripción | Cant. | Peso (g) | x × y × z (mm) |
|---|---|---|---|---|
| `0-1-10 Base` | Base | 1 | 40 | 100,0 × 101,0 × 31,0 |
| `0-1-20 Engranaje conductor` | Engranaje conductor | 1 | 1 | 20,0 × 20,0 × 5,0 |
| `0-1-30 Soporte de garras` | Soporte de garras | 1 | 11 | 54,0 × 25,0 × 78,2 |
| `0-1-40 Engranaje conducido` | Engranaje conducido | 1 | 1 | 28,0 × 21,5 × 7,0 |
| `0-1-50 Garra izquierda` | Garra izquierda | 1 | 3 | 60,4 × 14,5 × 24,0 |
| `0-1-60 Garra derecha` | Garra derecha | 1 | 3 | 60,4 × 14,5 × 24,0 |
| `0-1-70 Brazo para servo` | Brazo principal (*) | 1 | 10 | 46,5 × 7,5 × 13,0 |
| `0-1-80 Soporte de servo` | Soporte de servo | 2 | 1 | 34,0 × 26,0 × 2,5 |
| `0-1-90 Soporte triangular` | Soporte triangular | 1 | 4 | 9,0 × 58,3 × 29,2 |
| `0-1-100 Brazo Superior` | Brazo superior | 1 | 7 | 127,5 × 15,0 × 8,0 |
| `0-1-110 Articulación` | Articulación | 3 | 3 | 92,0 × 12,0 × 5,0 |
| `0-1-120 Soporte principal` | Soporte principal | 1 | 19 | 52,0 × 61,1 × 52,6 |
| `0-1-130 Plato circular` | Plato circular | 1 | 14 | 71,9 × 8,6 × 72,0 |
| `0-1-140 Brazo principal` | Brazo para servo (*) | 1 | 3 | 29,0 × 92,3 × 15,0 |
| `0-1-150 Arandela de separacion` | Arandela de separación | 1 | 15 | 73,9 × 12,5 × 74,0 |

### Subconjunto 2: chasis

| Archivo | Descripción | Cant. | Peso (g) | x × y × z (mm) |
|---|---|---|---|---|
| `0-2-10` | Frente | 1 | 45 | 163,0 × 46,9 × 56,0 |
| `0-2-20` | Posterior | 1 | 50 | 163,0 × 45,0 × 56,0 |
| `0-2-30` | Lateral | 2 | 8 | 27,0 × 61,0 × 13,5 |
| `0-2-40` | Tapa superior | 1 | 8 | 134,0 × 45,0 × 7,0 |
| `0-2-50` | Tapas inferiores / superior trasera | 3 | 33 | 163,0 × 45,0 × 7,0 |
| `0-2-60` | Soporte motores | 2 | 28 | 102,0 × 34,0 × 30,0 |

### Subconjunto 3: cámara

| Archivo | Descripción | Cant. | Peso (g) | x × y × z (mm) |
|---|---|---|---|---|
| `0-3-10` | Caja cámara frente | 1 | 5 | 44,7 × 35,0 × 10,5 |
| `0-3-20` | Caja cámara posterior | 1 | 4 | 35,2 × 44,6 × 7,5 |
| `0-3-30` | Unión cámara servo | 1 | 6 | 40,0 × 23,0 × 17,0 |
| `0-3-40` | Soporte servo | 1 | 5 | 69,2 × 27,0 × 12,0 |

Masa impresa total (cantidad × peso unitario): brazo 142 g, chasis 274 g,
cámara 20 g; en total 436 g. No incluye acrílico, motores, servos, placas ni baterías.

## Observaciones

- (*) **Conflicto de nombres.** La lista de partes llama `0-1-70` "Brazo principal" (10 g) y `0-1-140`
  "Brazo para servo" (3 g). Los archivos STL tienen los nombres al revés: `0-1-70 Brazo para servo`
  y `0-1-140 Brazo principal`. Además, las medidas no coinciden con los pesos (la pieza de
  46,5 × 7,5 × 13 mm pesaría 10 g y la de 92,3 × 29 × 15 mm, 3 g). Hay que revisar cuál es cuál
  mirando las piezas físicas.
- En la lista de partes el código de `0-1-150` figura como `ARM2-CUDAR-0-150` (falta el `-1-`); es un
  error de tipeo.
- `0-2-10` (Frente) y `0-2-20` (Posterior) miden 163 mm de ancho y 56 mm de alto. Coinciden con el ancho
  máximo del robot (162 mm) y con el hueco de 55 mm entre pisos: son las paredes/paragolpes que van
  entre las dos placas. El frente es un poco más grueso (46,9 mm contra 45,0 mm).
- La lista de partes dice servos SG90 (5 unidades); el robot usa MG90S.
