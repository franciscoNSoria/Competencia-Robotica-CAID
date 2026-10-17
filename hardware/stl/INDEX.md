# Índice de STL

Medidas de la caja envolvente de cada pieza (*bounding box*), en **milímetros** y en los ejes
propios de cada STL (x × y × z). Las medí leyendo los vértices de cada archivo. No son medidas de
la pieza ya montada en el robot.

Cada STL tiene su propio origen, en una esquina de la pieza o cerca de ella. Por eso la posición de
cada pieza en el robot no sale del STL: sale de `hardware/cad/dimensions.yaml`.

## Chasis de acrílico y cubo

| Archivo | Qué es | x × y × z (mm) | Uso en el URDF |
|---|---|---|---|
| `Chasis Armadillo.stl` | Placa de acrílico. Se usa dos veces: piso 1 y piso 2 | 256,0 × 150,0 × 4,0 | Sí: visual y colisión de ambos pisos |
| `Cubo.STL` | Cubo de la competencia | 20,0 × 20,0 × 20,0 | Sí: objeto del mundo de simulación |

## Subconjunto 1: brazo (`ARM2-CUDAR-0-1-*`)

| Archivo | Pieza | x × y × z (mm) |
|---|---|---|
| `0-1-10 Base` | Base | 100,0 × 101,0 × 31,0 |
| `0-1-20 Engranaje conductor` | Engranaje conductor | 20,0 × 20,0 × 5,0 |
| `0-1-30 Soporte de garras` | Soporte de garras | 54,0 × 25,0 × 78,2 |
| `0-1-40 Engranaje conducido` | Engranaje conducido | 28,0 × 21,5 × 7,0 |
| `0-1-50 Garra izquierda` | Garra izquierda | 60,4 × 14,5 × 24,0 |
| `0-1-60 Garra derecha` | Garra derecha | 60,4 × 14,5 × 24,0 |
| `0-1-70 Brazo para servo` | Brazo para servo | 46,5 × 7,5 × 13,0 |
| `0-1-80 Soporte de servo` | Soporte de servo | 34,0 × 26,0 × 2,5 |
| `0-1-90 Soporte triangular` | Soporte triangular | 9,0 × 58,3 × 29,2 |
| `0-1-100 Brazo Superior` | Brazo superior | 127,5 × 15,0 × 8,0 |
| `0-1-110 Articulación` | Articulación | 92,0 × 12,0 × 5,0 |
| `0-1-120 Soporte principal` | Soporte principal | 52,0 × 61,1 × 52,6 |
| `0-1-130 Plato circular` | Plato circular | 71,9 × 8,6 × 72,0 |
| `0-1-140 Brazo principal` | Brazo principal | 29,0 × 92,3 × 15,0 |
| `0-1-150 Arandela de separacion` | Arandela de separación | 73,9 × 12,5 × 74,0 |

Uso en el URDF: a definir en DESC-02.

## Subconjunto 2: chasis impreso (`ARM2-CUDAR-0-2-*`)

Es el chasis impreso del diseño original. El robot usa la placa de acrílico, así que estas piezas
no van en el URDF salvo que se confirme lo contrario.

| Archivo | x × y × z (mm) |
|---|---|
| `0-2-10 ` (el nombre original tiene un espacio antes del punto) | 163,0 × 46,9 × 56,0 |
| `0-2-20` | 163,0 × 45,0 × 56,0 |
| `0-2-30` | 27,0 × 61,0 × 13,5 |
| `0-2-40` | 134,0 × 45,0 × 7,0 |
| `0-2-50` | 163,0 × 45,0 × 7,0 |
| `0-2-60` | 102,0 × 34,0 × 30,0 |

## Subconjunto 3: cámara (`ARM2-CUDAR-0-3-*`)

| Archivo | x × y × z (mm) |
|---|---|
| `0-3-10` | 44,7 × 35,0 × 10,5 |
| `0-3-20` | 35,2 × 44,6 × 7,5 |
| `0-3-30` | 40,0 × 23,0 × 17,0 |
| `0-3-40` | 69,2 × 27,0 × 12,0 |

Uso en el URDF: a definir en DESC-02. Falta anotar qué es cada pieza.
