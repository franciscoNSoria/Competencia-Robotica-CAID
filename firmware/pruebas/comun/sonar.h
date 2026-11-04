/* Ultrasonidos (HC-SR04 o compatibles) para las pruebas.
 *
 * Pines (cableado actual del robot, ver firmware/pruebas/README.md):
 *   frontal   TRIG GPIO5   ECHO GPIO13
 *   derecho   TRIG GPIO15  ECHO GPIO4
 *   izquierdo TRIG GPIO14  ECHO GPIO12
 * Si el sensor se alimenta con 5 V, el ECHO va con divisor (2,2 k en serie y 3,3 k a GND: 3,0 V en el pin).
 *
 * Medicion: pulso de 10 us en TRIG; el ancho del pulso de ECHO se mide con una interrupcion en los dos flancos y el
 * reloj de 1 us (esp_timer). distancia = ancho * 343 m/s / 2 (aire a 20 grados C). Un disparo cada 60 ms por sensor
 * (el HC-SR04 pide al menos 60 ms entre mediciones). Sin eco en 40 ms la medicion se cuenta como perdida. */
#ifndef SONAR_H
#define SONAR_H

#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define SONAR_US_A_M 0.0001715f             /* 343 m/s / 2 / 1e6 */
#define SONAR_TIMEOUT_US 40000
#define SONAR_MIN_M 0.02f
#define SONAR_MAX_M 4.0f
#define SONAR_HIST 5

typedef struct {
	const char *nombre;
	gpio_num_t trig;
	gpio_num_t echo;
	volatile int64_t t_sube;                /* flanco de subida del eco (us) */
	volatile int32_t ancho_us;              /* ancho del ultimo eco, -1 si no llego */
	int64_t t_disparo;
	int esperando;
	float dist_m;                           /* ultima distancia valida, NAN si la ultima se perdio */
	float hist[SONAR_HIST];
	int n_hist, i_hist;
	uint32_t validas, perdidas;
	int32_t ultimo_ancho_us;
} sonar_t;

static void IRAM_ATTR sonar_isr(void *arg)
{
	sonar_t *s = (sonar_t *)arg;
	int64_t t = esp_timer_get_time();

	if (gpio_get_level(s->echo)) {
		s->t_sube = t;
	} else if (s->t_sube != 0) {
		s->ancho_us = (int32_t)(t - s->t_sube);
		s->t_sube = 0;
	}
}

static void sonar_init(sonar_t *s)
{
	static bool servicio = false;

	if (!servicio) {
		ESP_ERROR_CHECK(gpio_install_isr_service(0));
		servicio = true;
	}
	gpio_reset_pin(s->trig);
	gpio_set_direction(s->trig, GPIO_MODE_OUTPUT);
	gpio_set_level(s->trig, 0);

	gpio_config_t cfg = {
		.pin_bit_mask = 1ULL << s->echo,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,   /* sin sensor, el pin no queda flotando (GPIO12 tiene que estar en bajo al arrancar) */
		.intr_type = GPIO_INTR_ANYEDGE,
	};
	ESP_ERROR_CHECK(gpio_config(&cfg));
	ESP_ERROR_CHECK(gpio_isr_handler_add(s->echo, sonar_isr, s));
	s->ancho_us = -1;
	s->dist_m = NAN;
}

static void sonar_disparar(sonar_t *s)
{
	s->t_sube = 0;
	s->ancho_us = -1;
	gpio_set_level(s->trig, 1);
	esp_rom_delay_us(10);
	gpio_set_level(s->trig, 0);
	s->t_disparo = esp_timer_get_time();
	s->esperando = 1;
}

/* Llamar despues de disparar. Devuelve 1 cuando la medicion termino (con o sin eco). */
static int sonar_revisar(sonar_t *s)
{
	if (!s->esperando) {
		return 0;
	}
	int32_t ancho = s->ancho_us;
	if (ancho < 0 && esp_timer_get_time() - s->t_disparo < SONAR_TIMEOUT_US) {
		return 0;
	}
	s->esperando = 0;
	s->ultimo_ancho_us = ancho;
	float d = ancho > 0 ? ancho * SONAR_US_A_M : NAN;
	if (ancho > 0 && d >= SONAR_MIN_M && d <= SONAR_MAX_M) {
		s->dist_m = d;
		s->hist[s->i_hist] = d;
		s->i_hist = (s->i_hist + 1) % SONAR_HIST;
		if (s->n_hist < SONAR_HIST) {
			s->n_hist++;
		}
		s->validas++;
	} else {
		s->dist_m = NAN;
		s->perdidas++;
	}
	return 1;
}

/* Mediana de las ultimas mediciones validas (NAN si no hay) */
static float sonar_mediana(const sonar_t *s)
{
	float v[SONAR_HIST];
	int n = s->n_hist;

	if (n == 0) {
		return NAN;
	}
	for (int i = 0; i < n; i++) {
		v[i] = s->hist[i];
	}
	for (int i = 1; i < n; i++) {
		float x = v[i];
		int j = i - 1;
		while (j >= 0 && v[j] > x) {
			v[j + 1] = v[j];
			j--;
		}
		v[j + 1] = x;
	}
	return v[n / 2];
}

static void sonar_reiniciar_cuentas(sonar_t *s)
{
	s->validas = 0;
	s->perdidas = 0;
	s->n_hist = 0;
	s->i_hist = 0;
}

#endif /* SONAR_H */
