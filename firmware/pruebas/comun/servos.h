/* Servos (MG90S) para las pruebas: pulso de 50 Hz con velocidad limitada.
 *
 * Pines del brazo en la ESP32 de movimiento (los que quedan libres con el cableado actual, y los mismos de los
 * firmwares de banco del brazo):  base GPIO18, hombro GPIO19, codo GPIO21, pinza GPIO22.
 * Servo de la camara (pan): en la ESP32-CAM, GPIO13 (prueba 50).
 * Cada servo: marron = GND, rojo = +5 V (riel de servos, NO el pin 5V de la ESP32), naranja = senal. GND comun con la ESP32.
 *
 * Pulsos conocidos (docs/guias/brazo-medidas.md, M): hombro 1300 us sube / 1700 us baja (reposo); codo 1700 us retrae
 * (reposo) / 1000 us estira. Base y pinza: sin medir en us (P). Los limites de arranque solo dejan ir a lo ya probado;
 * se abren desde la PC (orden "limites"), nunca mas alla de 500..2500 us.
 *
 * "Soltar" deja de mandar pulsos: el servo queda sin fuerza (el brazo cae por su peso). */
#ifndef SERVOS_H
#define SERVOS_H

#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifndef SERVO_MODO
#define SERVO_MODO LEDC_LOW_SPEED_MODE
#endif
#ifndef SERVO_TIMER
#define SERVO_TIMER LEDC_TIMER_0
#endif
#define SERVO_HZ 50
#define SERVO_PERIODO_US 20000
#define SERVO_CUENTAS 16384                 /* 14 bits: 1,22 us por cuenta */
#define SERVO_ABS_MIN_US 500
#define SERVO_ABS_MAX_US 2500
#define SERVO_VEL_DEFECTO 200.0f            /* us por segundo (unos 20 grados por segundo) */

typedef struct {
	const char *nombre;
	gpio_num_t pin;
	ledc_channel_t canal;
	int min_us, max_us;                     /* limites actuales */
	int reposo_us;                          /* donde arranca al activar */
	volatile int objetivo_us;
	volatile float pulso_us;                /* lo que sale ahora */
	volatile int activo;                    /* 0: sin pulsos */
	volatile float vel;                     /* us/s */
} servo_t;

static int servo_limitar(const servo_t *s, int us)
{
	if (us < s->min_us) {
		return s->min_us;
	}
	if (us > s->max_us) {
		return s->max_us;
	}
	return us;
}

static void servo_escribir(const servo_t *s, float us)
{
	uint32_t duty = s->activo ? (uint32_t)(us * SERVO_CUENTAS / SERVO_PERIODO_US + 0.5f) : 0;
	ledc_set_duty(SERVO_MODO, s->canal, duty);
	ledc_update_duty(SERVO_MODO, s->canal);
}

static void servos_init(servo_t *s, int n)
{
	ledc_timer_config_t timer = {
		.speed_mode = SERVO_MODO,
		.duty_resolution = LEDC_TIMER_14_BIT,
		.timer_num = SERVO_TIMER,
		.freq_hz = SERVO_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer));
	for (int i = 0; i < n; i++) {
		ledc_channel_config_t canal = {
			.gpio_num = s[i].pin,
			.speed_mode = SERVO_MODO,
			.channel = s[i].canal,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = SERVO_TIMER,
			.duty = 0,                      /* sin pulsos hasta activar */
			.hpoint = 0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
		s[i].activo = 0;
		s[i].vel = SERVO_VEL_DEFECTO;
		s[i].objetivo_us = s[i].reposo_us;
		s[i].pulso_us = (float)s[i].reposo_us;
	}
}

/* Empieza a mandar pulsos en us (0 = en el reposo). El servo salta ahi: el brazo tiene que estar en esa pose. */
static void servo_activar(servo_t *s, int us)
{
	int p = servo_limitar(s, us > 0 ? us : s->reposo_us);
	s->objetivo_us = p;
	s->pulso_us = (float)p;
	s->activo = 1;
	servo_escribir(s, s->pulso_us);
}

static void servo_soltar(servo_t *s)
{
	s->activo = 0;
	servo_escribir(s, 0.0f);
}

static void servo_ir(servo_t *s, int us)
{
	s->objetivo_us = servo_limitar(s, us);
}

/* Cada ciclo: el pulso avanza hacia el objetivo a vel us/s como maximo */
static void servo_ciclo(servo_t *s, float dt_s)
{
	if (!s->activo) {
		return;
	}
	float paso = s->vel * dt_s;
	float err = (float)s->objetivo_us - s->pulso_us;
	if (fabsf(err) <= paso) {
		s->pulso_us = (float)s->objetivo_us;
	} else {
		s->pulso_us += err > 0 ? paso : -paso;
	}
	servo_escribir(s, s->pulso_us);
}

static void servo_set_limites(servo_t *s, int lo, int hi)
{
	if (lo < SERVO_ABS_MIN_US) {
		lo = SERVO_ABS_MIN_US;
	}
	if (hi > SERVO_ABS_MAX_US) {
		hi = SERVO_ABS_MAX_US;
	}
	if (lo >= hi) {
		return;
	}
	s->min_us = lo;
	s->max_us = hi;
	s->objetivo_us = servo_limitar(s, s->objetivo_us);
}

/* Estado de un servo en e[k..k+5]: activo, pulso, objetivo, min, max, vel. Devuelve el proximo indice. */
static inline int servo_estado(const servo_t *s, float *e, int k)
{
	e[k++] = (float)s->activo;
	e[k++] = s->pulso_us;
	e[k++] = (float)s->objetivo_us;
	e[k++] = (float)s->min_us;
	e[k++] = (float)s->max_us;
	e[k++] = s->vel;
	return k;
}

/* Ordenes comunes a un servo. Devuelve 1 si la reconocio.
 *   1 us      activar (empezar a mandar pulsos) en us; 0 = en el reposo
 *   2 us      ir a us (con velocidad limitada)
 *   3 delta   mover delta us
 *   4         soltar (sin pulsos)
 *   5 vel     velocidad en us/s (20..2000)
 *   6 lo hi   limites en us (dentro de 500..2500) */
static inline int servo_orden(servo_t *s, const float *a, int n)
{
	int orden = (int)lroundf(a[0]);
	switch (orden) {
	case 1:
		servo_activar(s, n > 1 ? (int)lroundf(a[1]) : 0);
		return 1;
	case 2:
		if (n > 1) {
			servo_ir(s, (int)lroundf(a[1]));
		}
		return 1;
	case 3:
		if (n > 1) {
			servo_ir(s, s->objetivo_us + (int)lroundf(a[1]));
		}
		return 1;
	case 4:
		servo_soltar(s);
		return 1;
	case 5:
		if (n > 1) {
			s->vel = a[1] < 20.0f ? 20.0f : (a[1] > 2000.0f ? 2000.0f : a[1]);
		}
		return 1;
	case 6:
		if (n > 2) {
			servo_set_limites(s, (int)lroundf(a[1]), (int)lroundf(a[2]));
		}
		return 1;
	default:
		return 0;
	}
}

#endif /* SERVOS_H */
