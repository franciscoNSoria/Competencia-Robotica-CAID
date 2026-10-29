# Calibración de un servo del brazo: pulso contra ángulo

> Borrador para `docs/guias/calibracion-servo-base.md` (insumo de MAN-02 y MAN-05).
> Estados: **M** medido, **C** calculado, **P** propuesta o estimación sin probar, **?** dato que falta.
> Sirve para la base del brazo (el plato giratorio) y, cambiando una línea del firmware, para el hombro, el codo y la pinza.

## 1. Qué hace la prueba

El servo recibe un pulso cada 20 ms. Su **ancho** (entre 500 y 2500 µs) decide el ángulo. Lo que falta saber es qué ancho corresponde a cada ángulo del brazo, porque la ficha del servo dice "360°" y la base recorre 256° (M). El firmware mueve el servo de a poco por el monitor serie; vos medís el ángulo con el transportador y anotás los pulsos. Con eso se calcula la recta pulso → ángulo y los pulsos de los topes.

## 2. Qué necesitás

| Cosa | Para qué | Nota |
|---|---|---|
| ESP32 con USB | Corre el firmware y alimenta la lógica | La misma de siempre |
| El servo de la base (MG90S) | Es lo que se calibra | Cables: **rojo** = +5 V, **marrón** = GND, **naranja** = señal (según la ficha del vendedor) |
| El shield de baterías (5 V) | Alimenta el servo | No se alimenta el servo desde la ESP32 ni desde el USB |
| Capacitor electrolítico de 470 a 1000 µF, 10 V o más | Evita que el 5 V caiga cuando el servo arranca o se traba | **Con polaridad**: la pata larga (o la sin franja) al +5 V |
| Capacitor cerámico de 100 nF | Filtra el ruido | Sin polaridad |
| Cables dupont y un cable para el GND común | Conexiones | El GND común, soldado o en bornera (el contacto flojo ya nos dio un susto) |
| Transportador | Medir el ángulo del brazo | El que usaste para los topes |
| Tester (opcional) | Medir la corriente del servo en serie | Rango de 10 A |

Los capacitores son opcionales para el banco. Para la competencia, la propuesta v0.2 dice que hay que preguntar a la organización si los pasivos se permiten, porque el reglamento penaliza "alterar el hardware del robot estándar".

## 3. El circuito

```
 Shield de baterías                                         Servo MG90S
                    +----------- 470-1000 uF ----------+
 +5 V  ------+------|  (+ al +5 V)                     |------+-------- rojo (+5 V)
             |      +----------- 100 nF --------------+      |
             |                                                |
 GND   ------+------------------------------------------------+-------- marron (GND)
             |
             +-------- un unico cable ---------> GND de la ESP32

 ESP32 GPIO18 ------------------------------------------------------- naranja (senal)
 ESP32 USB    ------------------------------------> PC (alimenta la ESP32 y da el monitor serie)
```

Reglas:

1. **El GND de la ESP32 y el del shield se unen en un solo punto**, con un cable firme. Sin eso la señal del servo no tiene referencia y puede moverse sola.
2. **No conectar el +5 V del shield al pin 5V o VIN de la ESP32** mientras esté el USB (la propuesta v0.2 pide una sola fuente por vez).
3. Los capacitores van lo más cerca posible del conector del servo, entre el rojo y el marrón.
4. La señal sale de un pin de la ESP32 (3,3 V). La mayoría de los servos de hobby la aceptan así; si el servo no reacciona o tiembla, avisame (se resuelve con un adaptador de nivel). **?**

Pines por servo (de `hardware/electronica/propuesta-v0.2.md`):

| Servo | GPIO | `SERVO_ELEGIDO` en el firmware |
|---|---|---|
| Base (plato giratorio) | 18 | 0 |
| Hombro | 19 | 1 |
| Codo | 21 | 2 |
| Pinza | 22 | 3 |

El pan de la cámara va en la ESP32‑CAM, no acá.

## 4. Antes de energizar

1. Con todo apagado, en continuidad del tester: GND de la ESP32 ↔ GND del shield tiene que sonar siempre; mové el cable mientras medís.
2. Entre el +5 V y el GND del shield no tiene que sonar (sin cortocircuito).
3. Los demás servos del brazo (hombro, codo, pinza) **no se conectan** para esta prueba: sin señal quedan sin fuerza. El brazo tiene que estar en el reposo (retraído, apoyado sobre el 2.º piso) para que no se caiga.
4. Robot sobre la mesa, sin nada alrededor del brazo y sin cables atravesados. Una mano cerca del interruptor del shield.

## 5. Firmware

`main_calibracion_servo.c` no usa micro‑ROS ni Wi‑Fi. Al arrancar **no manda ninguna señal**: hasta que apretás `g` el servo no recibe pulsos. Mueve el pulso a lo sumo 200 µs por segundo, y arranca con límites de 1000 a 2000 µs.

| Orden | Qué hace |
|---|---|
| `g` | Empieza a mandar pulso (lleva el servo a 1500 µs, a velocidad limitada) |
| `d` / `a` | +10 / −10 µs |
| `D` / `A` | +50 / −50 µs |
| número y Enter | Ir a ese pulso (por ejemplo `1320`) |
| `c` | Volver a 1500 µs |
| `m` | **Marca**: imprime `MARCA,n,pulso` con el pulso actual |
| `s` | Soltar: deja de mandar pulsos. Al volver con `g` el servo salta al último pulso |
| `x` / `n` | Abre los límites a 500–2500 µs / vuelve a 1000–2000 µs |
| `?` | Ayuda |

Orden de encendido: primero la ESP32 por USB y esperar a que el monitor muestre el encabezado; después encender el shield.

## 6. Cómo medir

1. Con el brazo en el reposo (0°), apretá `g`. El servo va a 1500 µs. **Mirá hacia dónde se mueve el brazo** y medí el ángulo: ese es el primer dato.
2. **Si el brazo sigue girando solo con el pulso quieto**, no es un servo posicional sino de rotación continua: apretá `s` y avisame. Es lo que podría pasar si la ficha de "360°" fuera cierta.
3. Con `d` y `a` (de a 10 µs) llevá el brazo hacia un lado hasta unos 60°. Apretá `m` y anotá el ángulo medido. Repetí hacia el otro lado.
4. **Topes**: acercate de a 10 µs. En cuanto el brazo deja de avanzar o toca algo, **parás y retrocedés**. Anotá el último pulso con el que todavía avanzaba (`m`). No sigas empujando.
5. Completá la tabla (0° = reposo; izquierda positivo, derecha negativo, como en tus mediciones):

| Marca | Pulso (µs) | Ángulo medido (°) | Nota |
|---|---|---|---|
| | 1500 | | Primer movimiento al apretar `g` |
| | | +60 | |
| | | 0 (reposo) | |
| | | −60 | |
| | | +136 (tope izquierdo) | Último pulso antes del tope |
| | | −120 (tope derecho) | Último pulso antes del tope |

6. Si querés medir la corriente: tester en serie con el cable rojo, rango de 10 A, y anotá el valor con el servo quieto, moviéndose y en el tope. La propuesta v0.2 estima 0,25 A en movimiento y 0,36 a 0,75 A trabado para un SG90 (**P**); del MG90S no tenemos el dato.

## 7. Qué me tenés que mandar

- Las líneas `MARCA,…` del monitor y la tabla con los ángulos.
- Qué pasó con el primer movimiento (a qué ángulo fue con 1500 µs).
- Si en algún momento el brazo siguió girando solo, o el servo tembló o se calentó.

Con eso calculo la recta pulso → ángulo, los pulsos de los topes y los límites de software (el tope menos 5°).

## 8. Seguridad

- Nunca empujar contra un tope: el servo trabado consume mucho y puede dañar los engranajes y las piezas impresas.
- Si algo suena raro, huele a quemado o el servo se calienta, apagá el shield.
- No dejar el brazo con pulso aplicado y sin supervisión.
