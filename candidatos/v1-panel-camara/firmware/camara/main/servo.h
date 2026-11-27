#pragma once

/* Servo en GPIO13 de la ESP32-CAM (AI-Thinker), con LEDC timer 1 / canal 1 a 50 Hz.
 * La cámara usa el timer 0 / canal 0 (XCLK): no se pisan.
 * GPIO13 es uno de los pines de la tarjeta SD (no se usa). No va en GPIO4 porque ahí está el
 * LED flash: se prendía con cada pulso. El GPIO4 queda en bajo (LED apagado).
 *
 * Calibración: el ancho de pulso de 0, 90 y 180 grados. Entre esos puntos se interpola en dos
 * tramos (0-90 y 90-180), así sirve para servos que no son simétricos. Se guarda en NVS y
 * sobrevive a los reinicios. */

#include <stdbool.h>

#define SERVO_GPIO        13
#define LED_FLASH_GPIO    4      // se deja en bajo: LED flash apagado
#define SERVO_US_LIMITE_MIN 400    // nunca menos (la mayoría de los servos traban antes)
#define SERVO_US_LIMITE_MAX 2600

void servo_init(void);

/* Mueve a un ancho de pulso en microsegundos (se recorta a los límites). */
void servo_us(int us);

/* Mueve a un ángulo 0..180 usando la calibración. */
void servo_grados(float grados);

/* Guarda la calibración (0, 90 y 180 grados en us) en NVS. med_us <= 0: punto medio. */
bool servo_calibrar(int min_us, int med_us, int max_us);

int servo_us_actual(void);
float servo_grados_actual(void);
int servo_cal_min(void);
int servo_cal_med(void);
int servo_cal_max(void);
