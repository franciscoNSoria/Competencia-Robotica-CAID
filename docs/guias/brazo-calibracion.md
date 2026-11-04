# Brazo: calibración de hombro y codo, URDF y brazo en Gazebo

> Estados: **M** medido, **C** calculado, **P** supuesto sin medir. Fecha: 04/11/2026.
> Insumo: `calibracion_del_brazo.PDF` (20 páginas) y las 15 fotos de `desplazamiento_hombroYcodo.zip`.
> Código: `ros2_ws/src/caid_manipulation/caid_manipulation/cinematica.py`.

## 1. Cómo se midió

Las fotos son de frente al robot con el brazo girado al costado, así que el plano del brazo
queda en el plano de la foto. En el PDF el equipo marcó cada eslabón con un color: **azul**, el
brazo principal (hombro → codo); **rojo**, el antebrazo (codo → muñeca); **violeta**, la otra
barra del paralelogramo, paralela al rojo.

- Cada página se emparejó con su foto del zip mirando la pose. El emparejamiento automático por
  correlación dio valores de 0,2 a 0,5, con empates, y se descartó. Las páginas siguen el orden de
  cada barrido: **1 a 8 = hombro 10, 25, 40, 55, 70, 85, 100 y 115** (codo fijo en 180) y **9 a 15
  = codo 80, 100, 110, 125, 140, 160 y 180** (hombro fijo en 60).
- Los ángulos se sacaron de los trazos y no a ojo: se segmentó cada color y se le ajustó una recta
  con RANSAC. El ángulo se mide desde la horizontal hacia el lado de la pinza: 90 = vertical,
  negativo = hacia abajo.

## 2. Resultados (M)

| Página | Servo | Azul (°) | Rojo (°) | Violeta (°) |
|---|---|---|---|---|
| 1 | hombro 10 | 122,9 | 11,6 | 13,8 |
| 2 | hombro 25 | 109,2 | 11,9 | 14,0 |
| 3 | hombro 40 | 94,5 | 12,2 | 14,1 |
| 4 | hombro 55 | 78,3 | 11,6 | 13,8 |
| 5 | hombro 70 | 57,3 | 10,8 | 13,0 |
| 6 | hombro 85 | 47,7 | 9,4 | 9,5 |
| 7 | hombro 100 | 35,0 | 7,9 | 7,1 |
| 8 | hombro 115 | 29,7 | 7,1 | 0,5 |
| 9 | codo 80 | 70,8 | −72,6 | −75,4 |
| 10 | codo 100 | 70,5 | −56,0 | −62,0 |
| 11 | codo 110 | 68,0 | −47,3 | −45,0 |
| 12 | codo 125 | 69,7 | −38,3 | −32,0 |
| 13 | codo 140 | 67,2 | −19,6 | −14,1 |
| 14 | codo 160 | 69,7 | −3,0 | 1,8 |
| 15 | codo 180 | 66,9 | 8,1 | 13,7 |

Controles:
- Con el hombro fijo (páginas 9 a 15), el azul queda entre 66,9 y 70,8 (promedio 69,0). Da una
  idea del error de medición: unos ±2°.
- Con el codo fijo (páginas 1 a 8), el rojo casi no se mueve (7 a 12°). **El antebrazo depende
  solo del servo del codo**, el desacople del EEZYbotARM.
- Cruce: hombro 60 y codo 180 aparecen en los dos barridos y coinciden dentro de 3 a 4°.

## 3. Calibración (C)

| | Ajuste | Notas |
|---|---|---|
| Brazo principal | **α = 132,5° − hombro** | Con 6 puntos (10 a 85) la pendiente da −1,044 ± 0,033: transmisión directa, se fija en −1. Hombro 100 y 115 se apartan 7°: con el brazo extendido el servo no llega (ya se había visto que el hombro no levanta solo) |
| Antebrazo | **β = 0,833 · codo − 139,2°** | Pendiente 0,833 ± 0,025, rms 2,2°. No es 1: el cuadrilátero del codo no es un paralelogramo perfecto |

En el URDF (`caid_description/urdf/arm.xacro`), con el cero del ensamblado de Fusion:

- `arm_shoulder_joint` = hombro − 46,7°
- `arm_elbow_joint` = 160,0° − 0,833 · codo − hombro
- `arm_wrist_joint` = −(hombro_URDF + codo_URDF): la pinza no cambia de inclinación

Verificación: la punta calculada recorriendo el URDF generado coincide con la de
`cinematica.py` en 7 poses, con **0,000 mm** de diferencia, y la pinza queda horizontal en todas.

**El brazo trabaja hacia adelante.** Con la base en su reposo (95 nominal) la pinza queda
delante del robot. El montaje del URDF dejó de tener el giro de 180°.

## 4. Alcance y acercamiento

Con los límites de arranque del firmware (hombro 10 a 125, codo 80 a 200), el brazo agarra a la
altura del cubo (apoyado a 108,5 mm) entre **193 y 237 mm del eje de giro de la base**:

- de frente: la cara del cubo entre 22 y 66 mm del paragolpes; los **3 cm** que pidió el equipo
  entran;
- al costado: el centro del cubo entre 112 y 156 mm del costado del robot; los **16 cm** quedan en
  el máximo.

La navegación frena donde el objeto queda a **205 mm** del eje (`alcance_brazo`): de frente, el
cubo queda a 3 cm del paragolpes. En un pasillo de 40 cm, un cubo pegado a la pared lateral queda
a 175 mm del eje del robot: a la par del brazo está **demasiado cerca**. Por eso el robot entra a
la celda del cubo, se pone de frente a la pared donde está y lo agarra de frente.

## 5. Lo que falta medir (P)

1. **Una medición con regla, la más importante.** Con hombro 104 y codo 140 (la pose de las
   páginas 17 y 19), ¿a cuántos mm del eje de giro de la base queda el centro del cubo agarrado?
   El CAD da 241 mm; en la foto de costado parece estar más cerca (unos 200), pero con la
   perspectiva no se puede asegurar. De eso depende el desplazamiento muñeca → punta del modelo.
2. **Base:** ganancia y sentido. Se supuso 1,42 grados de giro por grado nominal (256° de tope a
   tope sobre 180 nominales) y que más grados nominales giran hacia la izquierda. Medir el ángulo
   con transportador para base 45, 95 y 145.
3. **Pinza:** qué valor nominal es cerrada (con un cubo de 20 mm) y cuál abierta. Se supuso 114 y
   150.

## 6. Practicar en Gazebo

```bash
ros2 launch caid_navigation mision_sim.launch.py escenario:=practica
```

El robot queda quieto con el cubo verde al alcance. El brazo de Gazebo escucha `/brazo/cmd` igual
que la ESP32, así que se mueve con la misma ventana de sliders del brazo real:

```bash
python3 tools/brazo/brazo_sliders.py
```

O pidiendo la carga entera (la posición es la del cubo en base_link):

```bash
ros2 action send_goal /cargar caid_interfaces/action/Cargar "{color: verde, x: 0.181, y: 0.0}" --feedback
```
