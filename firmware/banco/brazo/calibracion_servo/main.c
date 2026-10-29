#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

/* Calibracion de UN servo del brazo: ancho de pulso contra angulo (sin micro-ROS, sin Wi-Fi).
 * Vos mandas el pulso por el monitor serie y medis el angulo con el transportador; el firmware solo mueve el servo
 * despacio y anota el pulso de cada marca. Pines: hardware/electronica/propuesta-v0.2.md (SERVO_BASE = GPIO18, etc.). */

#define SERVO_ELEGIDO 0              /* 0 = base (el plato giratorio), 1 = hombro, 2 = codo, 3 = pinza */

typedef struct {
	const char *nombre;
	gpio_num_t pin;
} servo_t;

static const servo_t SERVOS[] = {
	{"base", GPIO_NUM_18},
	{"hombro", GPIO_NUM_19},
	{"codo", GPIO_NUM_21},
	{"pinza", GPIO_NUM_22},
};

#define PWM_HZ 50
#define PERIODO_US 20000
#define CUENTAS 16384                /* 14 bits: 1,22 us por cuenta */

#define PULSO_INICIAL_US 1500
#define PULSO_MIN_SEGURO_US 1000     /* limites con los que arranca; la orden 'x' los abre a 500..2500 */
#define PULSO_MAX_SEGURO_US 2000
#define PULSO_MIN_ABS_US 500
#define PULSO_MAX_ABS_US 2500
#define PASO_CHICO_US 10
#define PASO_GRANDE_US 50
#define VEL_MAX_US_POR_S 200         /* el pulso avanza como maximo esto por segundo */
#define PERIODO_CTRL_MS 20

static int pulso_actual = PULSO_INICIAL_US;      /* lo que se esta mandando */
static int pulso_objetivo = PULSO_INICIAL_US;    /* adonde se quiere llegar */
static int limite_min = PULSO_MIN_SEGURO_US;
static int limite_max = PULSO_MAX_SEGURO_US;
static int pulsando = 0;                         /* 0: sin senal (el servo no recibe pulsos) */
static int marcas = 0;

static void pwm_init(void)
{
	ledc_timer_config_t timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_14_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = PWM_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer));

	ledc_channel_config_t canal = {
		.gpio_num = SERVOS[SERVO_ELEGIDO].pin,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,                               /* sin pulsos hasta que lo pidas */
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&canal));
}

static void pwm_us(int us)
{
	uint32_t duty = ((uint32_t)us * CUENTAS + PERIODO_US / 2) / PERIODO_US;

	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void pwm_apagar(void)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void ayuda(void)
{
	printf("\nServo '%s' en GPIO%d. Ordenes (una tecla; los numeros terminan con Enter):\n",
	       SERVOS[SERVO_ELEGIDO].nombre, (int)SERVOS[SERVO_ELEGIDO].pin);
	printf("  g       empezar a mandar pulso (lleva el servo a %d us). Antes, el brazo en el reposo\n", PULSO_INICIAL_US);
	printf("  d / a   +%d / -%d us        D / A   +%d / -%d us\n", PASO_CHICO_US, PASO_CHICO_US, PASO_GRANDE_US, PASO_GRANDE_US);
	printf("  1234 y Enter   ir a ese pulso en us        c   volver a %d us\n", PULSO_INICIAL_US);
	printf("  m       marca: anota el pulso actual (despues escribi el angulo que mediste)\n");
	printf("  s       soltar: dejar de mandar pulsos. Al volver con 'g' el servo salta al ultimo pulso\n");
	printf("  x / n   abrir los limites a %d..%d us / volver a %d..%d us\n", PULSO_MIN_ABS_US, PULSO_MAX_ABS_US,
	       PULSO_MIN_SEGURO_US, PULSO_MAX_SEGURO_US);
	printf("  ?       esta ayuda\n");
	printf("Velocidad maxima %d us/s. Si el brazo sigue girando con el pulso quieto, apreta 's' y avisame.\n\n",
	       VEL_MAX_US_POR_S);
}

static void ir_a(int us)
{
	if (us < limite_min) {
		printf("limite: %d us es menor que el minimo (%d)\n", us, limite_min);
		us = limite_min;
	} else if (us > limite_max) {
		printf("limite: %d us es mayor que el maximo (%d)\n", us, limite_max);
		us = limite_max;
	}
	pulso_objetivo = us;
	printf("objetivo %d us (desde %d)\n", pulso_objetivo, pulso_actual);
}

static void orden(char c)
{
	switch (c) {
	case 'g':
		pulsando = 1;
		pulso_objetivo = pulso_actual;
		printf("mandando %d us\n", pulso_actual);
		break;
	case 'd': ir_a(pulso_objetivo + PASO_CHICO_US); break;
	case 'a': ir_a(pulso_objetivo - PASO_CHICO_US); break;
	case 'D': ir_a(pulso_objetivo + PASO_GRANDE_US); break;
	case 'A': ir_a(pulso_objetivo - PASO_GRANDE_US); break;
	case 'c': ir_a(PULSO_INICIAL_US); break;
	case 'm':
		marcas++;
		printf("MARCA,%d,%d   (objetivo %d us)\n", marcas, pulso_actual, pulso_objetivo);
		break;
	case 's':
		pulsando = 0;
		pwm_apagar();
		printf("sin senal. Ultimo pulso mandado: %d us\n", pulso_actual);
		break;
	case 'x':
		limite_min = PULSO_MIN_ABS_US;
		limite_max = PULSO_MAX_ABS_US;
		printf("limites abiertos a %d..%d us: cuidado con los topes del brazo\n", limite_min, limite_max);
		break;
	case 'n':
		limite_min = PULSO_MIN_SEGURO_US;
		limite_max = PULSO_MAX_SEGURO_US;
		printf("limites %d..%d us\n", limite_min, limite_max);
		if (pulso_objetivo < limite_min || pulso_objetivo > limite_max) {
			ir_a(pulso_objetivo);
		}
		break;
	case '?': ayuda(); break;
	default: break;
	}
}

void app_main(void)
{
	char numero[8];
	int n_num = 0;

	pwm_init();                                  /* sin pulsos: el servo no recibe senal */
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

	printf("\n=== CALIBRACION DE SERVO (pulso contra angulo) ===\n");
	ayuda();
	printf("Todavia no se manda ninguna senal. Con el brazo en el reposo, apreta 'g'.\n");

	for (;;) {
		uint8_t c;

		while (uart_read_bytes(UART_NUM_0, &c, 1, 0) > 0) {
			if (c >= '0' && c <= '9') {
				if (n_num < (int)sizeof(numero) - 1) {
					numero[n_num++] = (char)c;
				}
			} else if (c == '\r' || c == '\n') {
				if (n_num > 0) {
					numero[n_num] = '\0';
					ir_a(atoi(numero));
					n_num = 0;
				}
			} else {
				n_num = 0;
				orden((char)c);
			}
		}

		if (pulsando) {
			int paso = VEL_MAX_US_POR_S * PERIODO_CTRL_MS / 1000;
			int antes = pulso_actual;

			if (pulso_actual < pulso_objetivo) {
				pulso_actual += (pulso_objetivo - pulso_actual < paso) ? pulso_objetivo - pulso_actual : paso;
			} else if (pulso_actual > pulso_objetivo) {
				pulso_actual -= (pulso_actual - pulso_objetivo < paso) ? pulso_actual - pulso_objetivo : paso;
			}
			pwm_us(pulso_actual);
			if (antes != pulso_actual && pulso_actual == pulso_objetivo) {
				printf("llego: %d us\n", pulso_actual);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(PERIODO_CTRL_MS));
	}
}
