/* Motores (L9110S, dos PWM por rueda) y encoders (FC-03, un canal) para las pruebas.
 *
 * Pines (los mismos de firmware/banco/robot_4ruedas/main.c y del cableado actual):
 *   rueda  IA (adelante)  IB (atras)  encoder D0
 *   FL     GPIO25         GPIO26      GPIO34
 *   FR     GPIO17         GPIO33      GPIO36
 *   RL     GPIO27         GPIO16      GPIO35
 *   RR     GPIO23         GPIO32      GPIO39
 * Si una rueda gira al reves con duty positivo, cambiar su polaridad a -1 en la tabla R (o dar vuelta los cables del motor).
 *
 * PWM de 5 kHz y 10 bits (LEDC alta velocidad, timer 0). La senal que no lleva PWM queda en 0.
 * Encoders: PCNT cuenta los dos flancos, con filtro de 5 us. 40 flancos = 1 vuelta de rueda (20 ranuras, M).
 * El encoder no sabe el sentido: las cuentas son siempre positivas. */
#ifndef RUEDAS_H
#define RUEDAS_H

#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#define N_RUEDAS 4
#define RUEDA_PWM_HZ 5000
#define RUEDA_PWM_MAX 1023
#define FLANCOS_POR_VUELTA 40
#define VENTANA_CICLOS 50                   /* velocidad = flancos de los ultimos 50 x 10 ms = 0,5 s */
#define PASO_RAMPA 0.02f                    /* duty por ciclo de 10 ms: de 0 a 1 en 0,5 s */

enum { FL, FR, RL, RR };

typedef struct {
	const char *nombre;
	gpio_num_t ia, ib, enc;
	ledc_channel_t ca, cb;
	int polaridad;
	pcnt_unit_handle_t unidad;
	int cuenta_ant;
	volatile long total;                    /* flancos acumulados desde el ultimo reinicio */
	int ventana[VENTANA_CICLOS];
	int idx, suma;
	volatile float fps;                     /* flancos por segundo (sin signo) */
	volatile float duty_obj;                /* lo pedido, con signo */
	float duty;                             /* lo que sale, con rampa */
} rueda_t;

static rueda_t R[N_RUEDAS] = {
	{.nombre = "FL", .ia = GPIO_NUM_25, .ib = GPIO_NUM_26, .enc = GPIO_NUM_34, .ca = LEDC_CHANNEL_0, .cb = LEDC_CHANNEL_1, .polaridad = +1},
	{.nombre = "FR", .ia = GPIO_NUM_17, .ib = GPIO_NUM_33, .enc = GPIO_NUM_36, .ca = LEDC_CHANNEL_2, .cb = LEDC_CHANNEL_3, .polaridad = +1},
	{.nombre = "RL", .ia = GPIO_NUM_27, .ib = GPIO_NUM_16, .enc = GPIO_NUM_35, .ca = LEDC_CHANNEL_4, .cb = LEDC_CHANNEL_5, .polaridad = +1},
	{.nombre = "RR", .ia = GPIO_NUM_23, .ib = GPIO_NUM_32, .enc = GPIO_NUM_39, .ca = LEDC_CHANNEL_6, .cb = LEDC_CHANNEL_7, .polaridad = +1},
};

static void rueda_pwm(ledc_channel_t canal, uint32_t d)
{
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, canal, d);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, canal);
}

/* duty en [-1, 1]; positivo = adelante (con la polaridad de la rueda) */
static void rueda_aplicar(rueda_t *r, float duty)
{
	uint32_t d = (uint32_t)(fabsf(duty) * RUEDA_PWM_MAX + 0.5f);
	int sentido = (duty > 0.0f) ? r->polaridad : (duty < 0.0f) ? -r->polaridad : 0;

	rueda_pwm(r->ca, sentido > 0 ? d : 0);
	rueda_pwm(r->cb, sentido < 0 ? d : 0);
}

static void ruedas_init(void)
{
	ledc_timer_config_t timer = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = RUEDA_PWM_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer));

	for (int i = 0; i < N_RUEDAS; i++) {
		rueda_t *r = &R[i];
		ledc_channel_config_t canal = {
			.gpio_num = r->ia,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel = r->ca,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = LEDC_TIMER_0,
			.duty = 0,
			.hpoint = 0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
		canal.gpio_num = r->ib;
		canal.channel = r->cb;
		ESP_ERROR_CHECK(ledc_channel_config(&canal));

		pcnt_unit_config_t unidad_cfg = {.low_limit = -32768, .high_limit = 32767};
		ESP_ERROR_CHECK(pcnt_new_unit(&unidad_cfg, &r->unidad));
		pcnt_glitch_filter_config_t filtro = {.max_glitch_ns = 5000};
		ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(r->unidad, &filtro));
		pcnt_chan_config_t canal_cfg = {.edge_gpio_num = r->enc, .level_gpio_num = -1};
		pcnt_channel_handle_t ch = NULL;
		ESP_ERROR_CHECK(pcnt_new_channel(r->unidad, &canal_cfg, &ch));
		ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
		                                             PCNT_CHANNEL_EDGE_ACTION_INCREASE));
		ESP_ERROR_CHECK(pcnt_unit_enable(r->unidad));
		ESP_ERROR_CHECK(pcnt_unit_clear_count(r->unidad));
		ESP_ERROR_CHECK(pcnt_unit_start(r->unidad));
		rueda_aplicar(r, 0.0f);
	}
}

/* Cada 10 ms: encoder (flancos y velocidad) y rampa del duty (al invertir pasa por 0) */
static void rueda_ciclo(rueda_t *r)
{
	int cuenta = 0;
	pcnt_unit_get_count(r->unidad, &cuenta);
	int delta = cuenta - r->cuenta_ant;
	if (delta < 0) {
		delta = 0;
	}
	r->cuenta_ant = cuenta;
	if (cuenta > 20000) {
		pcnt_unit_clear_count(r->unidad);
		r->cuenta_ant = 0;
	}
	r->total += delta;
	r->suma += delta - r->ventana[r->idx];
	r->ventana[r->idx] = delta;
	r->idx = (r->idx + 1) % VENTANA_CICLOS;
	r->fps = r->suma / (VENTANA_CICLOS * 0.010f);

	float obj = r->duty_obj;
	if (r->duty != 0.0f && obj != 0.0f && (r->duty > 0.0f) != (obj > 0.0f)) {
		obj = 0.0f;                         /* inversion: primero a 0 */
	}
	if (obj > r->duty + PASO_RAMPA) {
		r->duty += PASO_RAMPA;
	} else if (obj < r->duty - PASO_RAMPA) {
		r->duty -= PASO_RAMPA;
	} else {
		r->duty = obj;
	}
	rueda_aplicar(r, r->duty);
}

static void ruedas_parar(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		R[i].duty_obj = 0.0f;
		R[i].duty = 0.0f;
		rueda_aplicar(&R[i], 0.0f);
	}
}

static void ruedas_reiniciar_cuentas(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		R[i].total = 0;
	}
}

#endif /* RUEDAS_H */
