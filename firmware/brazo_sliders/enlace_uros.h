/* servos_brazo.h: los 4 servos del brazo, en grados.
 *
 * Cada servo recibe un pulso de 50 Hz. El ancho del pulso es la posicion: 500 us = 0 grados, 1500 us = 90, 2500 us = 180
 * (el mismo mapeo del codigo de referencia que movio todos los servos con fuerza).
 * Los pulsos los genera el periferico LEDC del ESP32 (hardware: no ocupa la CPU).
 *
 * Una tarea llama servos_ciclo() cada 10 ms. Ahi cada servo se acerca a su objetivo con una velocidad maxima en grados
 * por segundo (0 = salta directo, que es lo que da mas fuerza: el servo ve un error grande). */
#ifndef SERVOS_BRAZO_H
#define SERVOS_BRAZO_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define SERVO_HZ 50
#define SERVO_PERIODO_US 20000
#define SERVO_CUENTAS_MAX 65535          /* 16 bits */
#define SERVO_PULSO_0_US 500             /* pulso para 0 grados */
#define SERVO_PULSO_180_US 2500          /* pulso para 180 grados */
#define SERVO_DEG_MIN 0.0f               /* tope absoluto: la ESP32 nunca manda menos de 500 us, pida lo que pida la PC */
#define SERVO_ANCHO_MIN_DEG 1.0f         /* un rango de limites tiene que medir al menos 1 grado */
/* El tope de arriba es de cada servo (campo tope_max_deg): 180 grados = 2500 us, salvo el que se haya dado de mas en main.c */

typedef struct {
	const char *nombre;
	gpio_num_t pin;
	ledc_channel_t canal;
	float tope_max_deg;                  /* tope absoluto de ESTE servo (fijo): los limites de la PC nunca lo pasan */
	volatile float min_deg, max_deg;     /* limites del servo: arrancan con los de main.c y la PC los puede cambiar */
	volatile float objetivo_deg;         /* lo que pide la PC (lo escribe el callback de micro-ROS) */
	volatile float meta_deg;             /* adonde va de verdad: es el objetivo, salvo que el reposo seguro lo retenga */
	volatile bool activo;                /* false = sin pulso, el servo queda libre (sin fuerza) */
	volatile float actual_deg;           /* lo escribe solo la tarea de control */
	bool estaba_activo;                  /* para detectar el momento en que se activa */
} servo_t;

static float limitar_deg(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

/* cuentas del LEDC (0..65535) que dan un pulso de `us` microsegundos en un periodo de 20000 us */
static uint32_t cuentas_de_us(uint32_t us)
{
	return us * SERVO_CUENTAS_MAX / SERVO_PERIODO_US;
}

static uint32_t us_de_grados(float deg)
{
	return SERVO_PULSO_0_US + (uint32_t)((SERVO_PULSO_180_US - SERVO_PULSO_0_US) * deg / 180.0f + 0.5f);
}

static void servo_escribir(const servo_t *s, uint32_t cuentas)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, s->canal, cuentas);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, s->canal);
}

/* Un solo temporizador (50 Hz, 16 bits) compartido por los 4 canales. Todos arrancan SIN pulso. */
static void servos_init(servo_t *s, int n)
{
	ledc_timer_config_t timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_num = LEDC_TIMER_0,
		.duty_resolution = LEDC_TIMER_16_BIT,
		.freq_hz = SERVO_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer));

	for (int i = 0; i < n; i++) {
		ledc_channel_config_t canal = {
			.gpio_num = s[i].pin,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = s[i].canal,
			.timer_sel = LEDC_TIMER_0,
			.intr_type = LEDC_INTR_DISABLE,
			.duty = 0,
			.hpoint = 0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
		s[i].activo = false;
		s[i].estaba_activo = false;
		s[i].meta_deg = s[i].objetivo_deg;
	}
}

/* Cambia los limites del servo (los manda la PC). La ESP32 no confia en la PC: los deja entre 0 grados y el tope de este servo
 * y descarta un rango invalido (con menos de 1 grado de ancho, o con un NaN). Si el rango es invalido, quedan los anteriores. */
static void servo_poner_limites(servo_t *s, float lo, float hi)
{
	if (isnan(lo) || isnan(hi)) {
		return;
	}
	lo = limitar_deg(lo, SERVO_DEG_MIN, s->tope_max_deg);
	hi = limitar_deg(hi, SERVO_DEG_MIN, s->tope_max_deg);
	if (hi - lo < SERVO_ANCHO_MIN_DEG) {
		return;
	}
	s->min_deg = lo;
	s->max_deg = hi;
}

/* Cambia el objetivo (siempre dentro de los limites del servo). Un NaN se ignora. */
static void servo_ir_a(servo_t *s, float deg)
{
	if (isnan(deg)) {
		return;
	}
	s->objetivo_deg = limitar_deg(deg, s->min_deg, s->max_deg);
}

/* Una pasada de control: cada servo activo se acerca a su meta. dt_s = tiempo desde la pasada anterior.
 * vel_dps = grados por segundo (0 = salto directo). */
static void servos_ciclo(servo_t *s, int n, float vel_dps, float dt_s)
{
	for (int i = 0; i < n; i++) {
		servo_t *v = &s[i];

		if (!v->activo) {
			if (v->estaba_activo) {
				servo_escribir(v, 0);            /* duty 0: sin pulso, el servo se afloja */
				v->estaba_activo = false;
			}
			continue;
		}
		if (!v->estaba_activo) {
			v->actual_deg = v->meta_deg;         /* al activar, salta a la meta (no se puede leer la posicion real) */
			v->estaba_activo = true;
		}
		float error = v->meta_deg - v->actual_deg;
		float paso = vel_dps * dt_s;
		if (vel_dps <= 0.0f || fabsf(error) <= paso) {
			v->actual_deg = v->meta_deg;
		} else {
			v->actual_deg += error > 0.0f ? paso : -paso;
		}
		servo_escribir(v, cuentas_de_us(us_de_grados(v->actual_deg)));
	}
}

#endif /* SERVOS_BRAZO_H */