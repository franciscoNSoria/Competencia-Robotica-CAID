#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

/* Prueba minima del PWM de los servos del brazo, SIN teclado ni monitor: apenas arranca manda pulsos a GPIO19 (hombro) y GPIO21 (codo)
 * y hace parpadear el LED de la placa (GPIO2) una vez por segundo. Sirve para saber si la placa corre el programa y si los servos se mueven.
 * Ciclo de 10 s (los servos saltan de golpe, sin rampa: es el movimiento que mas fuerza hace):
 *   - segundo 0 a 3: los dos en 1700 us (reposo: hombro abajo, codo retraido).
 *   - segundo 3 a 5: el codo a 1500 us (se estira unos 23 grados).
 *   - segundo 5 a 8: los dos vuelven a 1700 us.
 *   - segundo 8 a 10: el hombro a 1500 us (sube unos 23 grados).
 * Con el brazo en el reposo, una mano cerca del interruptor. Para dejar uno quieto, poneles su constante igual a REPOSO_US. */

#define PWM_HZ 50
#define PERIODO_US 20000
#define CUENTAS 16384                /* 14 bits: 1,22 us por cuenta */
#define PIN_HOMBRO GPIO_NUM_19
#define PIN_CODO GPIO_NUM_21
#define PIN_LED GPIO_NUM_2
#define REPOSO_US 1700
#define HOMBRO_ARRIBA_US 1500
#define CODO_ESTIRADO_US 1500

static void pwm_us(ledc_channel_t canal, int us)
{
	uint32_t duty = ((uint32_t)us * CUENTAS + PERIODO_US / 2) / PERIODO_US;

	ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, canal);
}

void app_main(void)
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
		.gpio_num = PIN_HOMBRO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_0,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&canal));
	canal.gpio_num = PIN_CODO;
	canal.channel = LEDC_CHANNEL_1;
	ESP_ERROR_CHECK(ledc_channel_config(&canal));

	gpio_config_t led = {
		.pin_bit_mask = 1ULL << PIN_LED,
		.mode = GPIO_MODE_OUTPUT,
	};
	ESP_ERROR_CHECK(gpio_config(&led));

	printf("\n=== PRUEBA DE PWM: GPIO19 (hombro) y GPIO21 (codo) a %d us ===\n", REPOSO_US);
	pwm_us(LEDC_CHANNEL_0, REPOSO_US);
	pwm_us(LEDC_CHANNEL_1, REPOSO_US);

	for (int s = 0;; s++) {
		gpio_set_level(PIN_LED, s % 2);                  /* parpadeo de 1 Hz: el programa esta corriendo */
		pwm_us(LEDC_CHANNEL_0, s % 10 >= 8 ? HOMBRO_ARRIBA_US : REPOSO_US);
		pwm_us(LEDC_CHANNEL_1, (s % 10 >= 3 && s % 10 < 5) ? CODO_ESTIRADO_US : REPOSO_US);
		printf("vivo %d s\n", s);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
