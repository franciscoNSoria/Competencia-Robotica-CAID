/* Prueba 50: servo de giro de la camara (pan, MG90S) en la ESP32-CAM (AI-Thinker). SE GRABA EN LA ESP32-CAM.
 * Conexion: senal (naranja) al GPIO13 de la ESP32-CAM; rojo al +5 V del riel de servos; marron a GND. La ESP32-CAM se
 * alimenta por su pin 5V y comparte GND con el riel de servos.
 * Por que GPIO13: en la AI-Thinker el GPIO4 enciende el LED de flash, el GPIO16 es de la PSRAM, 0/2/12/15 son de arranque y
 * los de la camara estan ocupados; 13 y 14 quedan libres si no se usa la tarjeta SD.
 * PWM en el grupo de alta velocidad (timer 1): la camara usa el timer 0 de baja velocidad para su reloj (XCLK), asi esto
 * sirve igual cuando se pase al firmware de la camara.
 * En esta prueba la camara NO se enciende (es solo el servo). Usa otros topicos (/prueba_cam/...) para poder estar
 * conectada al mismo tiempo que la ESP32 de movimiento. LED rojo de la placa (GPIO33): parpadea buscando al agente.
 *
 * Ordenes (/prueba_cam/cmd = [50, orden, args]):
 *   1 us activar (0 = centro 1500)   2 us ir a us   3 delta mover   4 soltar   5 v velocidad us/s   6 lo hi limites
 *   7 x  barrido continuo entre los limites (1 empezar, 0 parar)
 * Estado:  [50, contador, activo, pulso_us, objetivo_us, min_us, max_us, vel_us_s, barriendo] */
#define PRUEBA_ID 50
#define PRUEBA_NOMBRE "servo pan de la camara (ESP32-CAM)"
#define PRUEBA_EN_CAM 1
#define PRUEBA_TOPICO "prueba_cam"
#define PRUEBA_NODO "esp32cam_prueba"
#define PIN_LED_ESTADO GPIO_NUM_33
#define LED_ACTIVO_BAJO 1
#define SERVO_MODO LEDC_HIGH_SPEED_MODE
#define SERVO_TIMER LEDC_TIMER_1
#include "prueba_uros.h"
#include "servos.h"

static servo_t S = {.nombre = "pan", .pin = GPIO_NUM_13, .canal = LEDC_CHANNEL_2, .min_us = 1000, .max_us = 2000,
                    .reposo_us = 1500};
static volatile int barriendo = 0;

static void prueba_init(void)
{
	servos_init(&S, 1);
}

static void prueba_ciclo(void)
{
	if (barriendo && S.activo && fabsf(S.pulso_us - (float)S.objetivo_us) < 1.0f) {
		servo_ir(&S, S.objetivo_us >= S.max_us ? S.min_us : S.max_us);
	}
	servo_ciclo(&S, 0.010f);
}

static void prueba_comando(const float *a, int n)
{
	if ((int)lroundf(a[0]) == 7) {
		barriendo = (n > 1 && a[1] > 0.5f);
		if (barriendo) {
			servo_ir(&S, S.max_us);
		}
		return;
	}
	barriendo = 0;
	servo_orden(&S, a, n);
}

static int prueba_estado(float *e)
{
	int k = servo_estado(&S, e, 2);
	e[k++] = (float)barriendo;
	return k;
}

static void prueba_parar(void)
{
	barriendo = 0;
	S.objetivo_us = (int)lroundf(S.pulso_us);
}
