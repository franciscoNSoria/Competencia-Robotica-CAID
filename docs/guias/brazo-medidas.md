# Brazo: pose de reposo y límites de giro medidos

> Borrador para `docs/guias/brazo-medidas.md` (insumo de MAN-02: poses con nombre y límites).
> Estados: **M** medido por el equipo, **C** calculado, **P** propuesta sin probar, **?** dato que falta.
> Fecha: 26/10/2026. Fotos del equipo: 2 laterales, 3 superiores y 3 frontales.

## 1. Pose de reposo con cubo

- El brazo apunta **hacia atrás** del robot, **completamente retraído y apoyado sobre el 2.º piso** (parte trasera). El cubo queda sujeto en la pinza, sobre el piso, y no puede caerse (M, fotos).
- La base gira con el plato; en el reposo el brazo está **sobre el eje del robot: 0°** (dato del equipo). Medido en la foto superior: 2,7° de desvío (C, error de la foto de varios grados), así que se toma como 0°.
- Esta es la pose "transporte con cubo". El cubo se agarra a la altura de la base del brazo (108,5 mm) y se lleva a esta pose.

## 2. Rotación de la base

Con el reposo como 0°, medido con transportador (M):

| Sentido | Tope medido | Estimado desde las fotos (C) |
|---|---|---|
| Hacia el lado izquierdo (hacia la rueda delantera izquierda) | **136°** | 135° |
| Hacia el lado derecho (hacia la rueda delantera derecha) | **120°** | 121° |

- Recorrido total: 136° + 120° = **256°**.
- Las dos estimaciones hechas a partir de las fotos (midiendo el centro de la pinza respecto del centro de la base) coinciden con el transportador dentro de 1°.
- El tope no es simétrico: 16° de diferencia. El equipo lo toma como un parámetro de calibración.

## 3. Criterio para los límites en software (P)

- No llegar a los topes: el límite de software es el tope medido **menos un margen** (propuesta: 5°), o sea **131° a la izquierda y 115° a la derecha**. Llegar al tope mecánico fuerza el servo y las piezas impresas.
- Los valores quedan como parámetros en la configuración del brazo (`arm_calibration.yaml`), no en el código.

## 4. Cómo se pueden conocer los topes sin fuerza

Los servos de este tipo no informan su posición, así que no hay un valor para leer. Opciones, de menos a más trabajo:

1. **Barrido lento con medición** (lo que ya se hizo): se manda un ángulo, se avanza de a 2° y se mide con transportador o foto hasta que la rueda deja de moverse. Después se retrocede el margen. No hace falta sentir el rebote.
2. **Corriente del servo**: medir la corriente de la alimentación del servo con un sensor (por ejemplo INA219). Al llegar a un tope la corriente sube y se queda alta. Detecta el tope, pero hay que tocarlo, aunque sea un instante.
3. **Realimentación real**: abrir el servo y sacar el cable del potenciómetro interno a un pin analógico de la ESP32. Da la posición medida, pero hay que modificar el servo.

## 5. Datos que faltan (?)

- ¿Qué servo mueve la base y con qué relación de engranajes? Un servo de hobby estándar recorre unos 180°, y acá la base recorre 256°.
- Ancho de pulso contra ángulo de la base: medir 5 puntos (−120°, −60°, 0°, 60°, 120°) y ajustar una recta. Eso da el desfase y la pendiente.
- Para los otros servos del brazo (hombro, codo, muñeca y pinza): ángulo de cada articulación en la pose de reposo, en la pose elevada de las fotos laterales y en el agarre, y sus topes.
- Alcance lateral de la pinza con el brazo girado 90° y extendido (las fotos frontales dan una idea, no una medida).
