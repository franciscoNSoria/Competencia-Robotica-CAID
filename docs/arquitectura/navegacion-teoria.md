# Navegación: algoritmos y teoría

> Complemento de `navegacion.md` (qué hace y cómo se prueba). Acá está **cómo** lo hace: cada
> algoritmo, de dónde sale, sus fórmulas, sus números y dónde está en el código.
> Estados: **M** medido, **C** calculado, **P** supuesto a confirmar, **S** simulado.
> Todos los números de este documento salen del código (`ros2_ws/src/caid_navigation`) al 29/10/2026.

## 0. Mapa de algoritmos

| Problema | Algoritmo | Código | Referencia |
|---|---|---|---|
| Qué se sabe del laberinto | Grilla de celdas con votos por lado (mapa de ocupación simplificado) | `conocimiento.py`, `sonar.py` | Moravec y Elfes 1985; Elfes 1989 |
| Adónde ir a explorar | Exploración por fronteras sobre la grilla | `mision.py` (`_explorable`, `_metas`) | Yamauchi 1997 |
| Cómo llegar | Dijkstra sobre estados (celda, rumbo) | `planificador.py` | Dijkstra 1959 |
| Si puede girar sin tocar | Barrido de la huella + teorema del eje separador (SAT) | `giros.py`, `geometria.py` | Ericson 2004; Gottschalk y otros 1996 |
| Dónde está (sin IMU) | Odometría + correcciones de ganancia fija con las paredes; rumbo por regresión lineal | `localizacion.py` | Siegwart y otros 2011; Borenstein y Feng 1996; Thrun y otros 2005 |
| Obstáculos dentro de una celda | Barrido del ultrasonido de frente (arcos + espacio libre) | `sonar.py` (`Barrido`) | Moravec y Elfes 1985 |
| Seguir un pasillo, girar | Control proporcional de rumbo con referencia lateral | `control.py` | Siegwart y otros 2011 |
| De píxel a metros | Modelo estenopeico (pinhole), intersección rayo-plano | `camara.py` | Hartley y Zisserman 2004 |
| Zonas rojas | Envolvente convexa (cadena monótona) + votos por celda | `camara.py` | Andrew 1979 |
| Formas en la imagen | Contornos (seguimiento de bordes), solidez, polígono aproximado | `caid_vision/detector.py` | Suzuki y Abe 1985 (OpenCV `findContours`) |
| Qué hacer ahora | Máquina de estados con prioridades y puntos guardados | `mision.py` (`objetivo`, `_decidir`) | — |

No se usan Nav2 ni SLAM: ver ADR 0004 (en resumen: en pasillos de 32 cm donde el robot ni siquiera
puede girar centrado (2.3), un costmap con inflado no lo deja pasar, y tres HC-SR04 de 15° no dan un
escaneo que un SLAM 2D pueda alinear).

---

## 1. Representación del mundo

### 1.1 Marcos y grilla

- Marco `map`: nace en el **centro de la celda de partida**, con x hacia donde mira el robot al
  encenderse. La celda (i, j) tiene su centro en (0,32·i ; 0,32·j) m. Por eso **el punto de partida
  queda guardado solo**: es (0, 0) con rumbo 0, desde el primer instante.
- Direcciones absolutas: 0 = +x, 1 = +y, 2 = −x, 3 = −y (antihorario, como el yaw).
- Estado del robot para planificar: **(celda, rumbo)**, con rumbo ∈ {0, 1, 2, 3}. En E6
  (13 × 24 = 312 celdas) son como mucho 1248 estados.

### 1.2 Lados con votos

Cada lado entre dos celdas tiene una suma de votos V (+ paso, − pared):

```
V ← limitar(V + p·s, −6, +6)       p = 2 si se midió de cerca (≥ 2 lecturas), 1 si de lejos
                                    s = +1 abierto, −1 pared
estado = signo(V)                  (V = 0: desconocido)
```

Es una versión discreta de la grilla de ocupación con log-odds (Thrun y otros 2005, cap. 9): cada
lectura suma evidencia y el límite ±6 hace que el mapa pueda corregirse si la evidencia cambia. Una
lectura mal asignada (por 2-3 cm de error de pose) suma 1 o 2; un lado medido varias veces está en
±6 y no se da vuelta por una sola.

**Registro lateral** (`sonar.py`, `RegistroLateral`): mientras el sensor lateral está a menos de
11 cm del centro de una celda (a lo largo del pasillo) se juntan sus lecturas. Al salir (o cada 6
lecturas) se cierra ese lado con la mediana:

```
esperada = distancia del sensor al borde de la celda (con la pose estimada)
pared    si  mediana(lectura) ≤ 0,20 m  y  mediana(lectura − esperada) ≤ 0,06 m
abierto  en otro caso
```

La segunda condición (agregada el 29/10) evita tomar como pared algo que está **dentro de la celda
vecina** (un pilar a 18 cm visto con el sensor casi en el borde de la celda). Además, si el robot
está quieto (el sensor no se movió 3 cm desde el último registro de ese lado) no vuelve a votar: es
la misma medida repetida.

**Frente** (`leer_frente`): el eco de frente cierra el lado donde cae (si cae en un borde de celda,
±4,5 cm) y abre los lados que el rayo atravesó, solo hasta el alcance confiable del cono (1.4).

### 1.3 Obstáculos como cajas

Bloques chicos, pilares de cubos y canastos se guardan como **cajas alineadas** (xmin, ymin, xmax,
ymax) en cada celda que tocan. Las usan los giros (2.3), el control lateral (3.1) y las poses de
agarre. Supuesto (P): los obstáculos son de base cuadrada o circular, así que su profundidad es su
ancho; y un obstáculo a menos de 5 cm de una pared se toma pegado a ella (entre los dos no pasa el
robot).

Los pilares de los cubos son **cilindros negros de 40 mm** (equipo, 29/10; el diámetro es P). Como
caja se toman de 44 mm (el cuadrado que circunscribe al cilindro + 2 mm por lado). La cámara no ve el
pilar (negro), pero sí el cubo que está encima: **en cuanto un cubo se confirma, su pilar entra al
mapa** debajo de él, y la caja se mueve con el promedio de las vistas (al principio, visto de lejos,
puede estar corrido unos centímetros).

---

## 2. Planificación

### 2.1 Dijkstra sobre (celda, rumbo)

Acciones y costos en segundos (`Costos`, con los parámetros por defecto, C):

| Acción | Condición | Costo |
|---|---|---|
| avanzar | el paso al frente es transitable (abierto, no prohibido, no cerrado por obstáculo) | 0,32 / 0,25 = 1,28 s |
| retroceder | el paso de atrás es transitable **y la celda de atrás ya se visitó** (no hay sensor atrás) | 0,32 / 0,15 + 0,6 = 2,73 s |
| girar 90° | `giros.py` encuentra un corrimiento que no toca nada (2.3) | (π/2) / 1,5 + 1,0 = 2,05 s + 3·(\|s\| + \|l\|) |
| girar 180° | ídem | π / 1,5 + 1,0 = 3,09 s + 3·(\|s\| + \|l\|) |

Más un costo extra por paso (`costo_extra`): +40 s por las flechas (2.4) y +60 s por cada vez que el
robot se trabó en ese paso (4.4).

Dijkstra (1959) da el camino de menor costo desde el estado actual a **todos** los estados en
O(E log V) con una cola de prioridad (`heapq`). Como el giro imposible directamente no existe como
acción, el plan nunca lo pide: en un callejón donde no puede girar sale marcha atrás hasta una celda
donde sí puede (y solo por celdas conocidas).

Para un objetivo con varias celdas posibles (explorar, poses de agarre) se elige el estado que
minimiza `costo − bonus`, con el bonus de 2.2.

### 2.2 Exploración por fronteras

Frontera (Yamauchi 1997): una celda **libre y todavía no visitada**, al lado de lo conocido. En
nuestra grilla (`_explorable`):

- no visitada, no prohibida (zona roja), no es un bloque, no está a menos de 0,2 m de un canasto;
- y además: tiene una pared conocida al lado, **o** está a ≤ 3 celdas (Manhattan) de la partida,
  **o** (mientras no se encontró la entrada) está sobre la recta de partida hacia adelante.

El objetivo es la frontera de menor `costo Dijkstra − bonus`. El bonus resta (es decir, posterga):

| Término | Valor | Para qué |
|---|---|---|
| `penal_afuera` | 60 s | fronteras fuera del laberinto una vez conocida la entrada (salida abierta, E6) |
| `penal_afuera` | 60 s | antes de encontrar la entrada, fronteras fuera de la recta de partida (P: arranca mirando al laberinto) |
| `penal_abierto` | 25 s | celdas sin ninguna pared conocida al lado |
| `penal_sala` | 15 s | celdas con menos de 2 bloques al lado, lejos de la partida (salas) |
| `penal_partida` | 60 s | alrededor de la partida (ya se miró con la cámara al arrancar) |
| `cerca` | 8 s por celda | con un cubo en la pinza y el canasto visto pero sin camino: explorar cerca del canasto |

Explorar por fronteras es **completo** en un grafo finito: cada paso visita una celda nueva, así que
termina habiendo recorrido todo lo alcanzable. Por eso **las islas no lo traban**: a diferencia de
seguir una pared (regla de la mano derecha), que con una isla puede dar vueltas para siempre
alrededor de ella, acá no se sigue ninguna pared; se va a la celda sin visitar más barata. El
algoritmo de Trémaux (Lucas 1882) tiene la misma garantía marcando pasos; la exploración por
fronteras con un planificador óptimo es su generalización cuando se conoce el mapa recorrido (no
recorre dos veces un tramo para volver: va por el camino más corto).

### 2.3 Giros: barrido de la huella

Geometría (C): huella de 278,1 × 163 mm con el centro de giro en `base_link`; las esquinas
delanteras están a √(0,1399² + 0,0815²) = **161,9 mm** y las traseras a √(0,1382² + 0,0815²) =
**160,4 mm** del centro. El pasillo deja 160 mm a cada lado del eje: **girar centrado roza**.

Algoritmo (`buscar_giro`):

1. Las paredes y obstáculos alrededor de la celda se pasan al marco de la celda (x hacia donde mira
   el robot) como cajas.
2. Para cada corrimiento candidato (s, l), s ∈ [−6, +6] cm y l ∈ [−5, +5] cm cada 1 cm, se
   "barre" el giro: 24 posiciones de la huella entre −3° y giro + 3° (los ±3° = 0,05 rad son la
   duda del rumbo estimado), cada una inflada con un margen m.
3. Cada posición es un rectángulo rotado; se prueba contra cada caja con el **teorema del eje
   separador** (SAT): dos polígonos convexos no se tocan si y solo si existe un eje (normal de algún
   lado de alguno de los dos) sobre el que sus proyecciones no se solapan (Ericson 2004, cap. 4;
   `poligono_toca_caja`).
4. La holgura de (s, l) es el mayor m de (40, 30, 22, 16, 12, 8, 6, 4) mm con el que el barrido no
   toca nada. Se elige el (s, l) de **mayor holgura**; entre iguales, el más chico.

Entre dos posiciones del barrido hay 4° (90° + 6°, en 24 pasos); un punto de la esquina se aparta
de la cuerda r·(1 − cos(Δθ/2)) = 162 mm × (1 − cos 2°) ≈ 0,1 mm, despreciable frente al margen
mínimo de 4 mm (C).

Resultado típico (C): en una esquina en L la mejor opción es s = −4 cm, l = +4 cm, con 22 mm de
holgura. El planificador usa el giro con 6 mm de margen mínimo; si al llegar el robot no está en el
lateral pedido, hace un **corrimiento lateral** como al estacionar (girar ±20°, moverse lo justo,
volver a alinear), verificando cada paso con la misma prueba de huella.

### 2.4 Flechas

Una flecha confirmada (vista igual en 2 imágenes) en la celda C, que indica la dirección d para el
color de un cubo que falta, encarece **los otros pasos** de C en 40 s. No es una orden: si el
camino de la flecha está cerrado o es mucho más largo, el robot igual va por otro lado. Así la
flecha acelera la búsqueda sin poder dejarlo encerrado si se la leyó mal. Con un cubo en la pinza,
las flechas no cuentan (el objetivo es el canasto).

---

## 3. Control

### 3.1 Recta por un pasillo (`control.py`, `Recta`)

Coordenadas a lo largo (a) y lateral (l) del eje del pasillo:

```
v      = signo · max(v_min, min(v_max, √(2 · d_frenado · falta)))     d_frenado = 0,35 m/s²
φ_ref  = asin( limitar(−k_lat · (l − l_obj) · signo, 0,35) )           k_lat = 6 /m
ω      = limitar( k_φ · (φ_ref − φ), ω_max )                          k_φ = 3 /s, ω_max = 1,2 rad/s
```

La velocidad baja con la raíz de lo que falta (desaceleración constante). El rumbo de referencia
lleva el error lateral a cero; `asin` y el tope de 0,35 (20°) evitan que apunte a la pared.

`l_obj` no es fijo: es el punto más cercano al preferido (el eje, o el corrimiento que pide el giro
siguiente) **dentro del tramo libre** de todas las celdas que ocupa el robot y las dos siguientes,
dejando 2,5 cm de margen a cada obstáculo; si no entra, 1,8 / 1,2 / 0,8 / 0,5 cm, y entonces va a
0,06 m/s.

### 3.2 Giro en el lugar (`Giro`)

```
ω = signo(e) · max(ω_min, min(ω_max, k · |e|))      k = 2,5 /s, ω_min = 0,22 rad/s
termina con |e| < 0,025 rad (1,4°) durante 3 ciclos
```

Si la celda de enfrente es nueva, el giro **se pasa hasta 15° y vuelve** (`sobrepaso`): el
ultrasonido de frente barre la celda entera sin parar (es el barrido de obstáculos de 4.1).

### 3.3 Con un cubo en el 2.º piso

Con el cubo apoyado arriba, el robot limita (`_limitar_carga`):

| | Valor | Por qué (C, con supuestos P) |
|---|---|---|
| v | ≤ 0,15 m/s | menos energía si frena de golpe |
| ω | ≤ 0,9 rad/s | aceleración centrípeta ω²·r = 0,08 m/s² con el cubo a r = 0,10 m del centro de giro (P) |
| a | ≤ 0,4 m/s² | un cubo apoyado se desliza si a > μ·g; con μ = 0,3 (P) eso es 2,9 m/s²: 7 veces de margen |
| α | ≤ 2 rad/s² | aceleración tangencial α·r = 0,2 m/s² a r = 0,10 m (P) |

Solo se suavizan los **arranques**: las frenadas ya son suaves en los controladores. El freno de
emergencia (algo a menos de 5 cm adelante) corta la velocidad sin rampa; en el simulador eso es
1 m/s² (S), todavía debajo de 2,9 m/s². Un cubo de 20 mm desliza antes de volcar (vuelca con
a > g·(b/h) = g, porque b = h). **Falta confirmar con MAN** dónde queda el cubo en el 2.º piso
(la foto que se mencionó no la tengo) y el μ real entre el cubo y la placa.

---

## 4. Sensado

### 4.1 Barrido de obstáculos con el ultrasonido de frente (`Barrido`)

Antes de entrar a una celda que no se vio de lejos, el robot gira hacia ella pasándose (3.2) y
cada lectura de frente f se trata con el cono de 15° como **5 rayos**:

- Todo lo que el rayo atraviesa antes de f − 1,2 cm es **libre** (grilla de 1 cm).
- El punto donde termina cada rayo es **candidato a ocupado** (arco de Moravec y Elfes 1985).
- Una lectura solo se usa si es al menos 3 cm más corta que la pared más cercana posible del
  pasillo con ±3° de error de rumbo (si no, el eco lo explica la pared).

Al final: los candidatos que caen en celdas libres se descartan; si hay cuadros de 2 cm marcados
por 2 lecturas o más, se usan solo esos (el "ensanche" del cono cae en lugares distintos en cada
lectura); si ninguno se repite, todos. Con esos puntos se busca el corrimiento lateral (hasta
6,5 cm) con el que la franja del robot (163 mm + 2,5 cm por lado) queda libre: si no hay, el paso
está **cerrado** (un bloque de 20 cm).

### 4.2 Obstáculos al costado

Mientras avanza, si un ultrasonido lateral mide 2,5 cm o más de menos de lo que debería medir con
la pared conocida (3 lecturas seguidas) y el otro lado mide normal, hay **algo apoyado en esa
pared** (pilar, bloque chico, canasto). Se mide su ancho a lo largo del pasillo y se guarda como
caja (supuesto de base cuadrada).

### 4.3 Localización sin IMU (`localizacion.py`)

**Odometría (predicción).** Se integran los incrementos de la odometría de las ruedas (no la pose
absoluta) en el marco del robot:

```
(Δx, Δy) = R(−θ_odo,anterior) · (x_odo − x_odo,anterior)
Δθ       = factor_giro · Δθ_odo
pose     ← pose ⊕ (Δx, Δy, Δθ)
```

Integrar incrementos hace que una corrección del rumbo también corrija hacia dónde se acumula el
avance que sigue.

**Correcciones (actualización).** Son de **ganancia fija**: pose ← pose + g·innovación, solo si la
innovación pasa una compuerta. Es un observador de ganancia constante (lo que queda de un filtro de
Kalman cuando la ganancia se fija en su valor estacionario); se eligió porque no hay IMU, los
errores dominantes son sistemáticos y así es fácil de entender y de ajustar.

| Corrección | Medida | Ganancia | Compuerta |
|---|---|---|---|
| Lateral | ultrasonido de costado contra una pared conocida de esta celda o de la vecina (salas de 2 celdas) | 0,25 | \|innovación\| < 3 cm |
| A lo largo, frente | ultrasonido de frente contra una pared **confirmada** (votos ≤ −4, sin nada apoyado adelante), solo dentro del alcance confiable | 0,25 | < 2,5 cm |
| A lo largo, bordes | cambio pared ↔ abierto de un ultrasonido lateral = borde de celda | 0,6 | a ≤ 7 cm del borde, y la lectura del lado con pared tiene que ser la de la pared (±2,5 cm) |
| Rumbo | pendiente de la posición lateral medida (abajo) | 0,7 | ≥ 8 muestras en ≥ 16 cm, residuo < 12 mm |

*Alcance confiable del frente* (C): el cono de 15° no puede tocar las paredes laterales hasta
`(0,16 − |l| − 0,015) / tan(7,5° + |φ| + 0,05)`; centrado y alineado da 0,79 m.

*Borde* (C): el cambio de lectura se ve con el rayo extremo del cono cuando pasa los 0,20 m del
umbral: 0,20 × tan 7,5° = 2,6 cm después del borde (pared → abierto) o antes (abierto → pared). Con
el sensor real hay que medirlo (`offset_borde`).

**Rumbo por regresión lineal** (reescrito el 29/10). Mientras avanza junto a una pared, cada
muestra guarda la posición a lo largo (a), el lateral **medido con la pared** l_med (no depende de
la odometría) y el rumbo estimado φ̂. Si el rumbo estimado tiene un error δ = φ̂ − φ:

```
l_med(a)  = l_0 + ∫ tan φ da                  (lo que hizo el robot)
L_pred(a) = ∫ tan φ̂ da    (trapecios)         (lo que cree la odometría desde la 1.ª muestra)
r(a) = l_med − L_pred ≈ l_0 − δ · (a − a_0)   (porque tan φ̂ − tan φ ≈ δ)
```

La pendiente p de la recta de mínimos cuadrados de r(a) da δ = −atan(p), y se corrige
φ̂ ← φ̂ − 0,7·δ. La versión anterior ajustaba la recta a l_med directamente y suponía el rumbo
constante en la ventana; cuando el control lateral corregía el rumbo dentro de la ventana, el
ajuste daba cualquier cosa. Restar L_pred saca esa parte y deja solo el error. Vale igual marcha
atrás (dl/da = tan φ en los dos sentidos).

**Factor de giro.** Un 4WD patina al girar y la odometría se equivoca siempre en la misma
proporción (error sistemático, Borenstein y Feng 1996). Cada corrección de rumbo δ (con |δ| < 0,15
rad) después de haber girado G rad según la odometría (|G| > 0,8 rad):

```
factor ← limitar(factor · (0,75 + 0,25 · (G − δ)/G), 0,85, 1,15)
```

Es un promedio móvil exponencial (peso 0,25) de la razón giro real / giro medido. Conviene medir el
valor inicial en el banco (parámetro `factor_giro`).

Resultado (S, 29/10): error máximo de pose en las corridas que terminan, 2-3 cm y 6-8°; 5-8 cm en
el área abierta de E6, donde no hay paredes para corregir.

### 4.4 Trabado (agregado el 29/10)

Un obstáculo que los sensores no vieron (un pilar a 4 cm del borde de la huella que el ultrasonido
lateral recién ve cuando la esquina ya pasó) frena el robot. Con los motores contra algo, los
encoders casi no cuentan. Regla (`_vigilar_trabado`): **se pide moverse (|v| ≥ 0,02 m/s o
|ω| ≥ 0,08 rad/s) y en 0,8 s la odometría no avanzó 1 cm ni giró 0,03 rad** → trabado:

- avanzando o retrocediendo: vuelve 5 cm, encarece ese paso 60 s y lo vuelve a barrer; a la 3.ª vez
  en el mismo paso, y solo si lleva a una celda **donde nunca entró**, el paso se cierra (si ya pasó
  por ahí, el paso existe y lo que falló es la pose);
- girando: vuelve al eje más cercano (girar para el otro lado); si también se traba, se mueve 3 cm;
- en la maniobra de un giro: a la 2.ª vez ese giro se marca imposible y se busca otro camino.

Si en algún momento no queda camino, antes de rendirse **olvida los giros marcados imposibles** y
vuelve a probar (hasta 2 veces): esos giros fallaron por cómo llegó el robot esa vez, no por el
mapa.

### 4.5 Cámara

**Modelo estenopeico** sin inclinación (Hartley y Zisserman 2004, cap. 6), sobre el servo de giro:

```
f_x = (W/2) / tan(HFOV/2) = 320 / tan(30°) = 554 px        (W = 640, HFOV = 60°, C)
píxel (u, v) → rayo:  d = adelante + ((u − c_x)/f_x)·derecha − ((v − c_y)/f_y)·arriba
```

con el origen en el centro óptico (7 mm delante del eje del servo, a 138,5 mm del piso) y
adelante = (cos ψ, sin ψ), ψ = rumbo del robot + ángulo del servo.

**Cubos y canastos: distancia por el borde de abajo.** El objeto apoya a una altura conocida z₀
(cubo 108,5 mm, base del canasto 88,5 mm, O/P), más abajo que la cámara:

```
Z = f_y · (h_cam − z₀) / (v_abajo − c_y)          (profundidad sobre el eje óptico)
```

Sensibilidad (C): dZ/dv = Z / (v − c_y). Para un cubo (h_cam − z₀ = 30 mm) a 0,5 m, v − c_y = 33 px
y **1 px = 1,5 cm**; a 1 m, 6 cm. Por eso se confirma con 2 vistas, se promedia (peso 1/n hasta
1/8) y no se usan cubos a más de 1,6 m. Si el borde está a menos de 6 px del horizonte se usa el
ancho: Z = f_x · ancho / w. El punto no puede quedar detrás de una pared conocida (se recorta al
primer bloque que cruza el rayo).

**Cubo o canasto** (`detector.py`): cubo = contorno macizo (solidez ≥ 0,9), cuadrado (lados en
razón ≤ 1,33, relleno ≥ 0,85) y con 4 a 6 vértices al aproximarlo; canasto = ancho ≥ 1,4 × alto,
solidez > 0,85. Además, un "cubo" a menos de 0,35 m de un canasto conocido se descarta (un canasto
de cerca puede parecer un cubo, y ahí van a parar los cubos entregados).

**Flechas: retroproyección sobre la pared.** Del contorno se sacan la punta, el cuello (donde
empieza la punta) y la cola. El rayo del cuello se recorre por la grilla y se anotan las caras de
bloque que cruza. Para cada cara candidata (plano vertical conocido, normal n, punto p₀) se cortan
los tres rayos con ese plano:

```
t = ((p₀ − o) · n) / (d · n),     P = o + t·d
```

y se queda la cara en la que **la punta mide 50 mm** (reglamento), con 25 % de tolerancia. En esa
cara, en milímetros reales: el cuerpo (cuello–cola) da el largo (< 45 mm → 1, < 80 mm → 2, si no
3; cuerpos de 30 / 60 / 100 mm) y la componente a lo largo de la pared da hacia dónde apunta. Esto
funciona con perspectiva y con la flecha inclinada hasta ±30° (se descartan más de 60° y caras
vistas a más de 66° de la normal).

**Zonas rojas: envolvente convexa.** Los píxeles rojos del piso (que no son flecha ni cubo ni
canasto, y no forman manchas macizas) se proyectan al piso (rayo ∩ plano z = 0). El contorno rojo
de una zona es un rectángulo: la **envolvente convexa** de lo que se ve de él queda dentro de la
zona. Los puntos del piso (hasta 1,6 m, y no detrás de una pared) se agrupan (separación 6 cm); para
cada grupo de 10 puntos o más se calcula la envolvente con la cadena monótona de Andrew (1979),
O(n log n). Si su área es de 60 cm² o más, se recorre su interior en una grilla de 2 cm y cada punto
a más de 3 cm del borde de la envolvente vota por su celda; una celda con 4 votos en la imagen
cuenta como vista roja, y con 2 imágenes queda prohibida. Una línea suelta no vota: su envolvente es
muy angosta.

Las imágenes tomadas mientras el robot gira (|ω| > 0,15 rad/s) o el servo se mueve no se usan: la
pose de ese instante no es confiable.

---

## 5. Misión: puntos guardados y decisiones

### 5.1 Puntos guardados

| Punto | Cuándo se guarda | Qué se guarda | Para qué |
|---|---|---|---|
| **Partida** | al encender | celda (0, 0), rumbo 0 = origen del marco `map` | zona de canastos; volver al final si se acaba el tiempo |
| **Entrada** | la primera celda (que no es la de partida) con los 4 lados conocidos y **pared a los dos costados** | (celda, rumbo) | volver a entrar después de entregar; separar "adentro" de "afuera" |
| **Primera bifurcación** | la primera celda **adentro** con 3 o más lados abiertos | (celda, rumbo) | alternativa de reingreso (`punto_de_reingreso: bifurcacion`) |

"Rumbo" es la **actitud** del robot en ese punto (hacia dónde miraba al pasar). Las coordenadas en
metros son (0,32·i, 0,32·j) en el marco `map`.

**Adentro** (`dentro()`): las celdas que se alcanzan desde la entrada **sin volver a salir por
donde se entró** (búsqueda en profundidad sobre los pasos transitables, sacando el paso de la
entrada hacia afuera). Las fronteras que no están adentro se postergan 60 s: así, con la salida
abierta (E6), el robot no se va a explorar el exterior mientras quede laberinto.

### 5.2 Por qué volver por el mapa y no "deshacer el camino"

El camino recorrido es una **caminata** en el grafo de celdas conocidas: tiene callejones, idas y
vueltas. Dijkstra sobre ese mismo grafo da el camino más corto entre los dos puntos, que por
definición es **igual o más corto** que cualquier caminata entre ellos, incluida la recorrida.
Además usa atajos que el robot conoció después (otra rama que se conecta). Por eso el robot no
guarda la lista de poses para repetirla: guarda el **mapa** y los **puntos**, y planifica cada vez.
(Nav2 haría lo mismo con un planificador sobre un costmap; acá no entra por lo dicho en la sección
0.)

### 5.3 Ciclo de decisión (`objetivo()`, en orden de prioridad)

1. Se acabó el tiempo (`tiempo_max`) → **volver a la partida**.
2. Con un cubo en la pinza → **su canasto**, si se conoce y hay una pose de descarga libre; si no,
   explorar (cerca del canasto si se lo vio).
3. Ya entregó los 3 → **reposo**: queda quieto junto al canasto (misión terminada).
4. Hay un cubo que falta, confirmado y alcanzable → **el más cercano**.
5. Acaba de entregar (y faltan cubos) → **volver al punto de reingreso** (entrada o bifurcación).
6. Si no → **explorar** (2.2).

Cada decisión se replanifica desde cero con el mapa del momento (el mapa cambia a cada rato y
Dijkstra es barato). Las tareas en curso se cortan si el mapa cambia lo que las justificaba (por
ejemplo, una recta frena antes si se cerró un paso por delante).

Antes de entrar a pasillos nuevos, si faltan cubos, mira 43° a cada lado con la cámara (con 60° de
campo no se ven los cubos pegados a las paredes laterales hasta estar a 2 celdas). En los cruces
nuevos mira 83° a cada lado.

### 5.4 Agarrar y soltar

- **Agarrar**: el robot queda en la celda del cubo, alineado con el pasillo, con el cubo al costado
  a la altura del eje del brazo (`agarre_dx` = −2 cm en x de `base_link`, P). Pide
  `{"accion": "agarrar", "color", "x", "y", "z"}` con la posición del cubo en `base_link`.
- **Soltar**: de espaldas al canasto, con su centro 0,19 m detrás del robot (`deposito_dist`, P),
  y pide `soltar`. Las dos poses se verifican con la huella completa contra el mapa antes de ir.
- Las dos son propuestas para acordar con MAN.

---

## 6. Supuestos (P) que cambian números si no se cumplen

| Supuesto | Dónde pega | Cómo confirmarlo |
|---|---|---|
| El robot arranca mirando hacia el laberinto | búsqueda de la entrada (2.2) | reglamento / organización |
| Pilar de 40 mm de diámetro | caja de 44 mm (1.3), validador de `gen_maze.py` | medir los de la competencia |
| Obstáculos de base cuadrada o circular | profundidad = ancho (4.1, 4.2) | reglamento |
| Cubo a r = 0,10 m del centro de giro, μ = 0,3 | límites de carga (3.3) | foto / medición con MAN |
| `offset_borde` = 2,6 cm | corrección de bordes (4.3) | medir el HC-SR04 real pasando por una abertura |
| Cono de 15° | alcances y filtros del sonar | medir el HC-SR04 real |
| Cámara sin inclinación | distancias por la cámara: 1° de inclinación corre 3 cm el punto del piso a 0,5 m (C) | medir el montaje |
| Agarre a −2 cm, descarga a 0,19 m | poses de manipulación (5.4) | MAN |

## 7. Referencias

- Andrew, A. M. (1979). Another efficient algorithm for convex hulls in two dimensions.
  *Information Processing Letters*, 9(5), 216-219.
- Borenstein, J., y Feng, L. (1996). Measurement and correction of systematic odometry errors in
  mobile robots. *IEEE Transactions on Robotics and Automation*, 12(6), 869-880.
- Dijkstra, E. W. (1959). A note on two problems in connexion with graphs. *Numerische Mathematik*,
  1, 269-271.
- Elfes, A. (1989). Using occupancy grids for mobile robot perception and navigation. *Computer*,
  22(6), 46-57.
- Ericson, C. (2004). *Real-Time Collision Detection*. Morgan Kaufmann (cap. 4 y 5: SAT).
- Gottschalk, S., Lin, M. C., y Manocha, D. (1996). OBBTree: a hierarchical structure for rapid
  interference detection. *SIGGRAPH '96*, 171-180.
- Hartley, R., y Zisserman, A. (2004). *Multiple View Geometry in Computer Vision* (2.ª ed.).
  Cambridge University Press (cap. 6: modelo de cámara).
- Lucas, É. (1882). *Récréations mathématiques*, tomo 1 (describe el método de Trémaux).
- Moravec, H. P., y Elfes, A. (1985). High resolution maps from wide angle sonar. *IEEE ICRA*,
  116-121.
- Siegwart, R., Nourbakhsh, I. R., y Scaramuzza, D. (2011). *Introduction to Autonomous Mobile
  Robots* (2.ª ed.). MIT Press (cap. 3: cinemática diferencial; cap. 5: odometría).
- Suzuki, S., y Abe, K. (1985). Topological structural analysis of digitized binary images by border
  following. *Computer Vision, Graphics, and Image Processing*, 30(1), 32-46.
- Thrun, S., Burgard, W., y Fox, D. (2005). *Probabilistic Robotics*. MIT Press (cap. 9: mapas de
  ocupación con log-odds).
- Yamauchi, B. (1997). A frontier-based approach for autonomous exploration. *IEEE CIRA*, 146-151.

## Historial

- 29/10/2026: primera versión.
