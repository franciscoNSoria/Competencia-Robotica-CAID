# Caracterización de los motores TT (banco, rueda al aire)

> Borrador para `docs/guias/caracterizacion-motores.md` (Refs #13 y FW-01; insumo de FW-06).
> Estados: **M** medido, **C** calculado a partir de lo medido, **P** hipótesis sin comprobar.
> Fecha de las mediciones: 2026-10-25. Todo sin carga (rueda en el aire), con una sola ESP32 por cable USB.

## 1. Qué se midió y cómo

- **Siglas de los motores** (por la posición en el chasis):
  - **FR** = *Front Right*, delantero derecho. **FL** = *Front Left*, delantero izquierdo.
  - **RR** = *Rear Right*, trasero derecho. **RL** = *Rear Left*, trasero izquierdo.
  - **RR1 / RR2** = primera y segunda corrida del mismo motor RR (se repitió para ver la variación).
- **Montaje:** un solo motor por vez en el canal B del L9110S derecho (IA = GPIO27 con PWM, IB = GPIO16 en 0) y su encoder (D0) en GPIO36. Los otros tres motores se desconectaron y se movieron los cables a mano.
- **Velocidad:** contador de pulsos (PCNT) de la ESP32 en ambos flancos, ventanas de 500 ms, 40 flancos por vuelta de rueda (20 ranuras × 2 flancos). **rpm = flancos/s × 1,5.**
- **PWM:** 5 kHz, 10 bits, `duty = linear.x / 0,5` (V_MAX = 0,5). Alimentación: shield de 2 celdas 18650, ~5 V.
- **Protocolo por motor** (`banco_cmdvel.py`): calentamiento de 120 s a duty 0,60; luego escalones de 10 s de arriba hacia abajo: 1,00 · 0,90 · 0,85 · 0,80 · 0,75 · 0,70 · 0,60 · 0,50 · 0,40 · 0,36 · 0,34 · 0,32.
- **Régimen** = promedio de las ventanas de cada escalón sin las 3 primeras (`registrar_motor.py`, tabla `RESUMEN`).
- **Verificación independiente de la escala (M):** conteo manual en video.
  - Motor delantero derecho, duty 0,40: 16,5 vueltas en 20 s = 49,5 rpm; el encoder daba 33 flancos/s = 49,5 rpm.
  - Motor delantero derecho, duty 0,50: 30⅜ vueltas en 20 s = 91,1 rpm; el encoder daba 88–100 rpm según la corrida.
  - Dos corridas del motor trasero izquierdo (RL, con la polaridad original) a duty 0,60 (25 s): 17¾ y 18 vueltas en cámara; el encoder daba 18,4 y 18,2 vueltas. Diferencia −3,4 % y −1,1 %.
  - Conclusión (M): **40 flancos por vuelta es correcto** y el encoder del RL no pierde cuentas. El del RR no se verificó con video (pendiente).

## 2. Resultados: flancos/s en régimen

| duty | FR | FL | RR1 | RR2 | RL (pol. original) | RL pol. invertida, corrida 1 | RL pol. invertida, corrida 2 (firmware v6, limpia) |
|---|---|---|---|---|---|---|---|
| 1,00 | 230,0 | 221,6 | 92,7 | 96,7 | 90,2 | 92,6 | 91,6 |
| 0,90 | 186,8 | 181,9 | 77,9 | 81,5 | 76,5 | 78,2 | 77,9 |
| 0,85 | 162,9 | 159,9 | 65,5 | 68,6 | 64,4 | 66,1 | 65,9 |
| 0,80 | 147,2 | 146,1 | 54,2 | 57,4 | 52,8 | 54,2 | 54,2 |
| 0,75 | 140,6 | 139,8 | 45,3 | 49,1 | 43,2 | 46,0 | 45,4 |
| 0,70 | 134,1 | 133,9 | 38,4 | 42,8 | 35,9 | 39,1 | 38,6 |
| 0,60 | 104,6 | 106,1 | 31,9 | 36,9 | 28,6 | — (*) | 31,5 |
| 0,50 | 69,1 | 73,2 | 24,8 | 28,4 | 24,2 | — (*) | 26,8 |
| 0,40 | 33,3 | 37,1 | 0 | 0 | 12,9 (**) | — (*) | 13,2 |
| 0,36 | 16,8 | 15,1 | 0 | 0 | 0 | — (*) | 0 (se detiene en ~1 s) |
| 0,34 | 2,0 | 0 | 0 | 0 | 0 | — (*) | 0 |
| 0,32 | 0 | 1,6 (**) | 0 | 0 | 0 | — (*) | 0 |

(\*) El motor se frenó por falta de órdenes y no volvió a arrancar con duty 0,60 o menos: escalones perdidos.
(\*\*) Celda contaminada por el empujón de arranque tras un FRENAR del watchdog.

Repetibilidad (C): las dos corridas del RL con polaridad invertida coinciden entre 0,70 y 1,00 de duty dentro de ±1,3 %.

Velocidad máxima a duty 1,00 (C): FR 345 rpm · FL 332 rpm · RR 139–145 rpm · RL 135–139 rpm.

## 3. Hallazgos

1. **Los dos delanteros son equivalentes (M/C).** FL/FR = 0,96–1,06 entre duty 0,5 y 1,0, con el mismo codo de la curva entre 0,70 y 0,80. Un factor de igualación ≈ 1 alcanza.
2. **Los dos traseros son otro motor (M).** Giran a un 39–42 % de la velocidad de los delanteros a duty 1,0, y el cociente con los delanteros varía con el duty (0,27–0,44), así que **un solo factor no los iguala**. La curva tiene otra forma: casi plana entre 0,5 y 0,7.
3. **No es el encoder (M, verificado en el RL)**: el conteo en video coincide con los flancos (−1 % a −3 %). El RR se comporta igual que el RL, pero falta verificarlo con video.
4. **El sentido de giro no explica la lentitud (M)**: invirtiendo la polaridad del RL la velocidad sigue siendo ~40 % de la de los delanteros. Entre la corrida original y las dos con polaridad invertida hay +2 % a +8 % entre duty 0,70 y 1,00 (y +10 % en 0,50–0,60 contra la corrida v6): puede haber una asimetría pequeña por sentido, que no alcanza a explicar la diferencia de 2,5 veces.
5. **Causa probable (P): los traseros son de otro lote o tipo.** Los motores se compraron en lugares distintos (los dos delanteros de una marca, los dos traseros de otra; sin etiqueta). Una caja 1:120 en lugar de 1:48 daría 48/120 = 0,40 de la velocidad, que coincide con el cociente a duty alto; una etapa extra de engranajes también aumenta la fricción y explica que el cociente baje a duty medio. Se confirma abriendo la caja y contando dientes, o con la publicación de compra.
6. **Umbral para sostener el giro, con el motor ya girando (M):** FR y FL 0,36 · RL 0,40 · RR 0,50.
7. **Arranque desde parado:** los delanteros arrancan con el empujón de 0,70 durante 100 ms. Los traseros no: RR necesitó ayuda a mano, y RL se quedó frenado más de 30 s con duty 0,60 aplicado después de un FRENAR. Con duty 1,0 arrancaron todas las veces. **El firmware actual no detecta que la rueda está parada con una orden activa.**
8. **La velocidad a duty fijo cambia entre corridas (M):** hasta ±8 % en el rango medio y ±2,5 % cerca de duty 1,0; más con el motor frío que caliente (a duty 0,60 el FR subió de 80 a 94 flancos/s en los primeros 3 minutos de marcha). Casi todo es un corrimiento aditivo, compatible con fricción que baja al calentarse (P).
9. **Codo de la curva (M):** entre duty 0,70 y 0,80 la pendiente del FR cae a la mitad (6–7 flancos/s por +0,05, contra 14–24 en el resto) y se repite en las 5 corridas del FR/FL. Verificado que no viene del código (`d = duty × 1023`). Origen desconocido.
10. **Tiempo de respuesta sin carga (M, resolución 0,5 s):** el motor llega a régimen en 1–2 s tras un escalón.

## 4. Consecuencias para FW-06 (PID)

- El lazo cerrado por rueda es obligatorio: la misma orden da velocidades distintas entre motores, entre corridas y según la temperatura.
- Cada motor necesita su propia curva de prealimentación (feedforward) y su propio umbral de arranque; un factor global no alcanza.
- Rango de velocidad que pueden seguir los cuatro a la vez, con los motores actuales: de ~37 rpm (mínimo sostenible del RR) a ~135 rpm (máximo del RL).
- El empujón de arranque debe dispararse por **velocidad medida igual a cero con orden activa**, no por `duty = 0`, y repetirse si la rueda no arranca. Hoy produce un acelerón después de cada FRENAR del watchdog (visto en FL y RL).
- Para los traseros el empujón debe ser más fuerte y más largo (duty 1,0, 200–300 ms, a confirmar), y configurable por motor.
- Polaridad por motor en configuración: los traseros giran hacia atrás con la polaridad original del cableado.
- Decisión de hardware pendiente: ¿conviene igualar los cuatro motores? Depende de la prueba con carga en el piso.

## 5. Enlace durante las pruebas (M)

| Corrida | HUECOS > 200 ms | Máximo | FRENAR espurios |
|---|---|---|---|
| FR | 23 | 321 ms | 0 |
| RR1 / RR2 | 35 / 46 | 551 / 392 ms | 0 / 0 |
| FL | 39 | 748 ms | 14 |
| RL | 33 | 622 ms | 10 |
| RL con firmware v6 (timeout 1000 ms) | 9 huecos ≥ 600 ms en 240 s | 741 ms | 0 |

El watchdog de 600 ms fue suficiente en la primera mitad de la sesión y insuficiente en la segunda. Para el banco se subió a 1000 ms (firmware v6) y desapareció el problema: en 240 s de corrida del RL el hueco más largo fue de 741 ms y no hubo ningún FRENAR espurio (el del inicio del log pertenece a la corrida anterior, y la pausa de 2968 ms es el tiempo entre una corrida y la siguiente). En la competencia la protección no debe depender del enlace (freno local con el sonar delantero, FW-04).

## 6. Pendientes

- Verificar con video el encoder del RR.
- Averiguar por qué, después de la corrida v6 del RL, `banco_cmdvel.py 0.50:3 0.30:25` dejó la rueda sin girar (sin log todavía).
- Abrir la caja de un motor trasero y de uno delantero y comparar los engranajes (o revisar la publicación de compra).
- Prueba con carga en el piso (el robot con peso), porque los umbrales y las velocidades van a cambiar.
- Origen del codo entre 0,70 y 0,80: medir con el tester la tensión en bornes del motor en esos duty.
- Repetir con el firmware v6 (sin FRENAR espurios ni empujones falsos) al menos RR y RL.
