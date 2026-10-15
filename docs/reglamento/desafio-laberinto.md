# Desafío CAID 2026: "Laberinto inteligente"

> Transcripción de la página oficial del desafío (CUDAR, UTN-FRC), copiada el 12/10/2026.
> Este desafío reemplaza la descripción genérica de la pista del reglamento general
> (`CAID_2026_Reglamento.pdf`, capítulo V). El resto del reglamento general (plataforma,
> penalidades, evaluación técnica, inspección) sigue vigente.
> Si la organización publica cambios, actualizar este archivo y anotarlo en el historial al final.

## El desafío

Navegación, percepción y manipulación autónoma.

Los equipos deben resolver una misión integrada. El robot recorre de forma completamente
autónoma un laberinto, interpreta las indicaciones visuales del recorrido, localiza tres objetos
de diferentes colores, los recoge con su brazo robótico y los deposita en los recipientes
correspondientes. El desafío se realiza sobre la plataforma robótica estándar CAID 2026.

- **Percepción:** identificación de flechas direccionales y reconocimiento de los objetos por su color.
- **Navegación:** desplazamiento autónomo por el laberinto siguiendo las indicaciones visuales y evitando obstáculos.
- **Manipulación:** captura, transporte y clasificación de objetos con el brazo y la pinza del robot.

## La misión, paso a paso

La misión se completa cuando los tres cubos están bien depositados y el robot vuelve a la zona de inicio.

1. Iniciar el recorrido desde la zona de partida.
2. Ingresar al laberinto.
3. Reconocer mediante visión artificial las flechas direccionales del recorrido.
4. Interpretar las flechas para determinar la dirección de navegación.
5. Recorrer el laberinto evitando paredes y obstáculos.
6. Localizar tres cubos objetivo: rojo, verde y azul.
7. Identificar el color de cada cubo con el sistema de percepción.
8. Recoger cada cubo con el brazo robótico y la pinza.
9. Transportar cada cubo hasta su depósito.
10. Depositar cada cubo en el depósito que le corresponde.
11. Retornar a la zona de inicio.

## Cubos y depósitos

Los cubos miden 20 mm de lado. Un cubo está bien depositado cuando queda completamente dentro
del área del depósito.

| Cubo | Depósito |
|---|---|
| Rojo | Depósito 1 |
| Verde | Depósito 2 |
| Azul | Depósito 3 |

## Reglamento del desafío

### Señalización del laberinto

El recorrido cuenta con flechas direccionales de colores ubicadas en la pared del laberinto. Su
longitud indica cuántos segmentos rectos continúan: longitud 1 indica un segmento antes de la
curva, longitud 2 indica dos segmentos, y longitud 3 indica que hay por lo menos tres segmentos
rectos. Las medidas de las flechas se detallan en el anexo, al final del reglamento.

En las bifurcaciones, las flechas indican con su color el camino hacia el objetivo. No forman una
línea continua para seguir con sensores: el sistema debe detectar, identificar e interpretar una
señal visual discreta y usarla para tomar una decisión de navegación. Reconocerlas permite
resolver el desafío en menor tiempo.

La ubicación y orientación de las flechas puede cambiar entre fases, con una inclinación máxima
de ±30 grados.

### Autonomía

El desafío corresponde a la modalidad de Autonomía Completa, Categoría A. Una vez iniciada la
prueba, el robot debe completar toda la misión sin intervención humana. Ningún integrante del
equipo puede:

- controlar manualmente el movimiento del robot;
- indicar las direcciones de navegación;
- seleccionar qué objeto recoger;
- indicar qué depósito utilizar;
- controlar manualmente el brazo;
- accionar manualmente la pinza;
- dar información sobre la ubicación de los objetos durante la prueba.

La captura de los objetos también debe ser autónoma. El operador solo puede iniciar el sistema
desde la PC antes del comienzo de la prueba, según las condiciones generales del reglamento CAID 2026.

En las categorías 2 y 3, la manipulación podrá realizarse por teleoperación: se podrá controlar el
brazo robótico de forma manual, activando y desactivando la navegación cuando el caso lo amerite.

### Navegación

El robot debe desplazarse de forma autónoma usando la información de sus sensores y su sistema de
visión. La estrategia de navegación debe permitir:

- reconocer paredes y obstáculos;
- interpretar las flechas;
- determinar la dirección de desplazamiento;
- ejecutar giros;
- corregir la trayectoria;
- evitar colisiones;
- continuar el recorrido hasta localizar los objetos.

Un mapa del recorrido programado de antemano no se considera una solución válida para reemplazar
la interpretación de las señales visuales. La configuración del circuito puede cambiar entre
fases, por lo que las soluciones deben adaptarse a distintas configuraciones.

### Manipulación

La captura de los cubos se realiza con el brazo robótico y la pinza de la plataforma estándar.
El robot debe:

- detectar el objeto;
- determinar su posición relativa;
- aproximarse al objeto;
- posicionar el brazo;
- accionar la pinza;
- verificar que el objeto fue capturado;
- transportarlo hasta el depósito correspondiente;
- liberarlo dentro del área del depósito.

Los equipos pueden desarrollar distintas estrategias de control y percepción, respetando las
restricciones de hardware del reglamento.

### Variabilidad entre etapas

Para que la solución no se reduzca a una trayectoria programada de antemano, la configuración del
desafío puede cambiar entre etapas.

- **Clasificación:** la organización define la configuración del recorrido, la posición de los
  cubos, el laberinto y la disposición de las flechas.
- **Semifinal:** se usa una configuración diferente a la de la clasificación.
- **Final:** el laberinto, las flechas y la ubicación de los objetos pueden modificarse y se
  comunican a los equipos en el momento que establezca la organización.

### Finalización de la prueba

La prueba termina cuando ocurre alguna de estas condiciones:

- los tres cubos fueron depositados correctamente y el robot volvió a la zona de inicio;
- se alcanza el tiempo máximo de la ronda;
- el robot abandona el circuito;
- el equipo solicita terminar la prueba;
- el jurado determina una infracción que implica el fin de la ronda.

Si la misión no se completa, se registra el avance alcanzado según el sistema de puntuación de la
organización.

### Evaluación

Se evalúan, entre otros aspectos:

- la navegación autónoma;
- el reconocimiento e interpretación de las flechas;
- la localización de los objetos;
- la identificación correcta de los colores;
- la aproximación a los objetos;
- la captura con el brazo robótico;
- la clasificación correcta de los objetos;
- la precisión del depósito;
- la ausencia de colisiones;
- la adaptación a diferentes configuraciones del circuito;
- el tiempo de resolución.

La evaluación técnica también puede considerar la arquitectura de software, los algoritmos de
percepción y navegación, las estrategias de control y las soluciones de inteligencia artificial de
cada equipo.

## Anexo: medidas de flechas y cubos

### Flechas

Todas las flechas tienen la misma punta y el mismo ancho. Lo que cambia es el largo del cuerpo (L),
según la distancia al objetivo.

| Medida | Valor |
|---|---|
| Ancho de la punta | 70 mm (±1 mm) |
| Alto de la punta | 50 mm (±1 mm) |
| Ancho del cuerpo | 30 mm (±1 mm) |
| Largo del cuerpo (L) | 30 mm "cerca", 60 mm "normal", 100 mm "lejos" |

Los dibujos del anexo muestran las flechas en gris oscuro.

### Cubos

Cubos de 20 mm de lado, en rojo, verde y azul.

## Datos derivados (calculados por el equipo, no son parte del reglamento)

| Flecha | L (mm) | Largo total = L + 50 (mm) | Largo total / alto de punta | Largo total / ancho de punta |
|---|---|---|---|---|
| Cerca (1 tramo) | 30 | 80 | 1,6 | 1,14 |
| Normal (2 tramos) | 60 | 110 | 2,2 | 1,57 |
| Lejos (3+ tramos) | 100 | 150 | 3,0 | 2,14 |

Estas proporciones no dependen de la distancia a la cámara. "Largo total / alto de punta" tampoco
cambia si la pared se ve en ángulo, porque las dos medidas están sobre el mismo eje de la flecha.

Masa de un cubo macizo de PLA: 2 cm × 2 cm × 2 cm = 8 cm³ × 1,24 g/cm³ ≈ 9,9 g (impreso con
relleno parcial pesa menos).

## Historial

- 12/10/2026: transcripción inicial.
