#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_timer.h"

/* Prueba de las 4 ruedas con PI por rueda, SIN la PC (sin micro-ROS, sin Wi-Fi). Resultados por el LED (GPIO2) y por el puerto serie.
 *   Fase A (ruedas en el aire): cada rueda sola, adelante y atras, a V_TEST_FLANCOS; despues las 4 juntas. Compara la velocidad
 *            medida por el encoder con la pedida.
 *   Fase B (robot en el piso): las 4 juntas a la misma velocidad; despues avanza 1 m calculado con los encoders y se detiene.
 *            Vos medis cuanto avanzo de verdad y la relacion real / calculado corrige el diametro de la rueda.
 * El boton BOOT pasa de una etapa a la siguiente. Pines: hardware/electronica/propuesta-v0.2.md, mapa B-rueda (RR_IB en el 32).
 * Control: el de main_motor_v8_pi.c (prealimentacion con la curva medida + PI + empujon de arranque), una instancia por rueda. */

#define N_RUEDAS 4
#define PWM_HZ 5000
#define PWM_MAX 1023                 /* 10 bits */
#define PIN_LED GPIO_NUM_2
#define PIN_BOOT GPIO_NUM_0          /* boton BOOT de la placa: se lee solo despues del arranque */

#define V_TEST_FLANCOS 40.0f         /* velocidad de la prueba: 1 vuelta/s = 40 flancos/s = 0,21 m/s con rueda de 67 mm */
#define TOLERANCIA 0.10f             /* velocidad medida dentro de +/-10 % de la pedida */
#define DIAMETRO_RUEDA_M 0.067f      /* CAD; despues de la prueba de 1 m se corrige con la medida real */
#define DISTANCIA_M 1.0f
#define FLANCOS_POR_VUELTA 40        /* 20 ranuras x 2 flancos */
#define ESCALONADO_MS 300            /* las ruedas arrancan una tras otra para no hundir el 5 V con 4 empujones juntos */

#define VEL_MIN_FLANCOS 25.0f
#define VENTANA_VEL_CICLOS 30        /* velocidad = flancos de los ultimos 30 x 10 ms = 300 ms */
#define PI_CICLOS 5                  /* el PI corre cada 5 x 10 ms = 50 ms */
#define KP 0.0020f                   /* duty por cada flanco/s de error */
#define KI 0.0030f                   /* duty por cada flanco/s de error y por segundo */
#define INTEGRAL_MAX 0.40f
#define PASO_DUTY_PI 0.15f
#define SIN_GIRO_PI 6
#define MAX_INTENTOS_KICK 6

/* Curva medida (sin carga): flancos/s -> duty. Solo prealimentacion; el PI corrige el resto. */
typedef struct { float v; float duty; } punto_t;
static const punto_t CURVA_DEL[] = {
	{0.0f, 0.34f}, {16.8f, 0.36f}, {33.3f, 0.40f}, {69.1f, 0.50f}, {104.6f, 0.60f},
};
static const punto_t CURVA_RL[] = {
	{0.0f, 0.36f}, {13.2f, 0.40f}, {26.8f, 0.50f}, {31.5f, 0.60f}, {38.6f, 0.70f},
	{45.4f, 0.75f}, {54.2f, 0.80f}, {65.9f, 0.85f}, {77.9f, 0.90f}, {91.6f, 1.00f},
};
static const punto_t CURVA_RR[] = {                  /* promedio de RR1 y RR2 de caracterizacion-motores.md */
	{0.0f, 0.50f}, {26.6f, 0.50f}, {34.4f, 0.60f}, {40.6f, 0.70f}, {47.2f, 0.75f},
	{55.8f, 0.80f}, {67.0f, 0.85f}, {79.7f, 0.90f}, {94.7f, 1.00f},
};
#define N_CURVA(c) ((int)(sizeof(c) / sizeof((c)[0])))

/* Empujon progresivo para vencer la friccion estatica */
static const float KICK_DUTY_NIVEL[] = {0.70f, 0.85f, 1.00f, 1.00f};
static const int KICK_CICLOS_NIVEL[] = {10, 15, 25, 40};     /* x 10 ms */
#define N_NIVELES_KICK 4

typedef struct {
	const char *nombre;
	gpio_num_t pin_ia;               /* adelante (PWM) */
	gpio_num_t pin_ib;               /* atras (PWM)    */
	gpio_num_t pin_enc;              /* D0 del encoder */
	ledc_channel_t canal_ia;
	ledc_channel_t canal_ib;
	int polaridad;                   /* +1: IA adelante | -1: la rueda gira al reves con IA, se invierte. Se fija tras la fase 2 de la prueba de cableado */
	const punto_t *curva;
	int n_curva;
	float d_min;                     /* duty minimo que sostiene el giro (medido) */
	int nivel_kick_ini;
	pcnt_unit_handle_t unidad;
	/* compartidas con la secuencia (32 bits: lectura atomica) */
	volatile float objetivo;         /* flancos/s con signo: + es avanzar */
	volatile long total;             /* flancos acumulados */
	volatile float suma_duty;
	volatile long n_duty;
	volatile bool bloqueado;
	/* estado del control */
	float duty, integral;
	int kick, nivel_kick, intentos, sin_giro, ciclos_pi, gracia;
	int ventana[VENTANA_VEL_CICLOS], idx, suma, cuenta_ant;
} rueda_t;

/* Polaridad: dejala en +1 hasta tener el resultado de la prueba de cableado (sentido real de cada rueda) */
static rueda_t W[N_RUEDAS] = {
	{.nombre = "FL", .pin_ia = GPIO_NUM_25, .pin_ib = GPIO_NUM_26, .pin_enc = GPIO_NUM_34, .canal_ia = LEDC_CHANNEL_0,
	 .canal_ib = LEDC_CHANNEL_1, .polaridad = +1, .curva = CURVA_DEL, .n_curva = N_CURVA(CURVA_DEL), .d_min = 0.36f,
	 .nivel_kick_ini = 0},
	{.nombre = "FR", .pin_ia = GPIO_NUM_17, .pin_ib = GPIO_NUM_33, .pin_enc = GPIO_NUM_36, .canal_ia = LEDC_CHANNEL_2,
	 .canal_ib = LEDC_CHANNEL_3, .polaridad = +1, .curva = CURVA_DEL, .n_curva = N_CURVA(CURVA_DEL), .d_min = 0.36f,
	 .nivel_kick_ini = 0},
	{.nombre = "RL", .pin_ia = GPIO_NUM_27, .pin_ib = GPIO_NUM_16, .pin_enc = GPIO_NUM_35, .canal_ia = LEDC_CHANNEL_4,
	 .canal_ib = LEDC_CHANNEL_5, .polaridad = +1, .curva = CURVA_RL, .n_curva = N_CURVA(CURVA_RL), .d_min = 0.40f,
	 .nivel_kick_ini = 2},
	{.nombre = "RR", .pin_ia = GPIO_NUM_23, .pin_ib = GPIO_NUM_32, .pin_enc = GPIO_NUM_39, .canal_ia = LEDC_CHANNEL_6,
	 .canal_ib = LEDC_CHANNEL_7, .polaridad = +1, .curva = CURVA_RR, .n_curva = N_CURVA(CURVA_RR), .d_min = 0.50f,
	 .nivel_kick_ini = 2},
};

static void pausa_ms(int ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

static void led(int encendido)
{
	gpio_set_level(PIN_LED, encendido);
}

static void destellos(int n, int ms_on, int ms_off)
{
	for (int k = 0; k < n; k++) {
		led(1);
		pausa_ms(ms_on);
		led(0);
		pausa_ms(ms_off);
	}
}

/* Un resultado: un destello largo = OK, tres cortos y rapidos = MAL */
static void senal_resultado(int ok)
{
	if (ok) {
		destellos(1, 900, 700);
	} else {
		destellos(3, 120, 120);
		pausa_ms(580);
	}
}

/* Numero por el LED: tres destellos rapidos de aviso y despues cada cifra (cifra N = N destellos; el 0 es un destello muy largo) */
static void mostrar_numero(int n)
{
	char txt[12];

	snprintf(txt, sizeof(txt), "%d", n);
	destellos(3, 100, 100);
	pausa_ms(1500);
	for (const char *c = txt; *c; c++) {
		int d = *c - '0';

		if (d == 0) {
			destellos(1, 1500, 400);
		} else {
			destellos(d, 250, 300);
		}
		pausa_ms(1500);
	}
}

/* Espera BOOT (LED con pulso lento) y devuelve cuantos ms se mantuvo apretado */
static int esperar_boot_ms(void)
{
	int apretado_ms = 0;

	for (int t = 0; gpio_get_level(PIN_BOOT) != 0; t += 20) {
		led((t / 500) % 2 == 0);
		pausa_ms(20);
	}
	led(0);
	while (gpio_get_level(PIN_BOOT) == 0) {
		pausa_ms(20);
		apretado_ms += 20;
	}
	pausa_ms(200);
	return apretado_ms;
}

static void esperar_boot(void)
{
	(void)esperar_boot_ms();
}

static void cuenta_regresiva(void)
{
	for (int k = 0; k < 25; k++) {                                /* 5 s de destellos rapidos */
		destellos(1, 100, 100);
	}
}

static void pwm_set(ledc_channel_t canal, uint32_t duty)
{
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, canal, duty);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, canal);
}

static void pwm_init(void)
{
	ledc_timer_config_t timer = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = PWM_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer));

	for (int i = 0; i < N_RUEDAS; i++) {
		ledc_channel_config_t canal = {
			.gpio_num = W[i].pin_ia,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel = W[i].canal_ia,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = LEDC_TIMER_0,
			.duty = 0,
			.hpoint = 0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
		canal.gpio_num = W[i].pin_ib;
		canal.channel = W[i].canal_ib;
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
	}
}

/* duty > 0: avanza (con la polaridad de la rueda). La senal fija queda siempre en 0. */
static void motor_aplicar(const rueda_t *w, float duty)
{
	uint32_t d = (uint32_t)(fabsf(duty) * PWM_MAX + 0.5f);
	int sentido = (duty > 0.0f) ? w->polaridad : (duty < 0.0f) ? -w->polaridad : 0;

	pwm_set(w->canal_ia, sentido > 0 ? d : 0);
	pwm_set(w->canal_ib, sentido < 0 ? d : 0);
}

static void encoders_init(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		pcnt_unit_config_t unidad_cfg = {
			.low_limit = -32768,
			.high_limit = 32767,
		};
		ESP_ERROR_CHECK(pcnt_new_unit(&unidad_cfg, &W[i].unidad));

		pcnt_glitch_filter_config_t filtro_cfg = {
			.max_glitch_ns = 5000,
		};
		ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(W[i].unidad, &filtro_cfg));

		pcnt_chan_config_t canal_cfg = {
			.edge_gpio_num = W[i].pin_enc,
			.level_gpio_num = -1,
		};
		pcnt_channel_handle_t canal = NULL;
		ESP_ERROR_CHECK(pcnt_new_channel(W[i].unidad, &canal_cfg, &canal));
		ESP_ERROR_CHECK(pcnt_channel_set_edge_action(canal,
			PCNT_CHANNEL_EDGE_ACTION_INCREASE,
			PCNT_CHANNEL_EDGE_ACTION_INCREASE));

		ESP_ERROR_CHECK(pcnt_unit_enable(W[i].unidad));
		ESP_ERROR_CHECK(pcnt_unit_clear_count(W[i].unidad));
		ESP_ERROR_CHECK(pcnt_unit_start(W[i].unidad));
	}
}

/* Prealimentacion: duty que dio esa velocidad en la curva medida (interpolacion lineal) */
static float duty_ff(const rueda_t *w, float v)
{
	const punto_t *c = w->curva;

	if (v <= c[0].v) {
		return c[0].duty;
	}
	for (int i = 1; i < w->n_curva; i++) {
		if (v <= c[i].v) {
			return c[i - 1].duty + (c[i].duty - c[i - 1].duty) * (v - c[i - 1].v) / (c[i].v - c[i - 1].v);
		}
	}
	return c[w->n_curva - 1].duty;
}

static void reiniciar_estado(rueda_t *w)
{
	w->duty = 0.0f;
	w->integral = 0.0f;
	w->kick = 0;
	w->nivel_kick = w->nivel_kick_ini;
	w->intentos = 0;
	w->bloqueado = false;
	w->sin_giro = 0;
}

/* Un ciclo de 10 ms de una rueda: lectura del encoder y control (misma logica que main_motor_v8_pi.c) */
static void rueda_ciclo(rueda_t *w)
{
	float objetivo = w->objetivo;
	int cuenta = 0;
	int delta;

	pcnt_unit_get_count(w->unidad, &cuenta);
	delta = cuenta - w->cuenta_ant;
	if (delta < 0) {
		delta = 0;
	}
	w->cuenta_ant = cuenta;
	if (cuenta > 20000) {
		pcnt_unit_clear_count(w->unidad);
		w->cuenta_ant = 0;
	}
	w->suma += delta - w->ventana[w->idx];
	w->ventana[w->idx] = delta;
	w->idx = (w->idx + 1) % VENTANA_VEL_CICLOS;
	w->total += delta;

	if (objetivo == 0.0f) {
		reiniciar_estado(w);
	} else if (w->duty != 0.0f && (w->duty > 0.0f) != (objetivo > 0.0f)) {
		reiniciar_estado(w);                /* inversion: pasar por 0/0 un ciclo */
	} else if (w->bloqueado) {
		w->duty = 0.0f;                     /* sin flancos pese a los empujones: no seguir a ciegas */
		w->kick = 0;
	} else if (w->kick > 0) {
		w->kick--;                          /* durante el empujon el duty queda fijo */
		if (w->kick == 0) {
			w->gracia = VENTANA_VEL_CICLOS / PI_CICLOS;
		}
	} else if (++w->ciclos_pi >= PI_CICLOS) {
		float signo = (objetivo > 0.0f) ? 1.0f : -1.0f;
		float mag = fabsf(w->duty);
		float vref = fmaxf(fabsf(objetivo), VEL_MIN_FLANCOS);
		float v_med = w->suma / (VENTANA_VEL_CICLOS * 0.010f);

		w->ciclos_pi = 0;
		w->sin_giro = (w->suma == 0) ? w->sin_giro + 1 : 0;
		if (w->suma > 0) {
			w->nivel_kick = w->nivel_kick_ini;
			w->intentos = 0;
		}
		if ((mag == 0.0f && w->suma == 0) || w->sin_giro >= SIN_GIRO_PI) {
			/* parado con orden activa: empujon */
			if (++w->intentos > MAX_INTENTOS_KICK) {
				w->bloqueado = true;
				w->duty = 0.0f;
				printf("ERROR %s: sin flancos tras %d empujones, rueda detenida (encoder o rueda trabada)\n", w->nombre,
				       MAX_INTENTOS_KICK);
			} else {
				w->kick = KICK_CICLOS_NIVEL[w->nivel_kick];
				w->duty = signo * KICK_DUTY_NIVEL[w->nivel_kick];
				if (w->nivel_kick < N_NIVELES_KICK - 1) {
					w->nivel_kick++;
				}
			}
			w->sin_giro = 0;
		} else {
			float err = vref - v_med;
			float ff = duty_ff(w, vref);
			float d;

			if (w->gracia > 0) {
				/* la ventana de 300 ms todavia contiene el empujon: mantener la prealimentacion */
				w->gracia--;
				d = ff + w->integral;
			} else {
				float integral_nueva = w->integral + KI * err * (PI_CICLOS * 0.010f);

				d = ff + KP * err + integral_nueva;
				/* anti-windup: no integrar si el duty ya esta saturado hacia el mismo lado */
				if (!((d >= 1.0f && err > 0.0f) || (d <= w->d_min && err < 0.0f))) {
					w->integral = fminf(INTEGRAL_MAX, fmaxf(-INTEGRAL_MAX, integral_nueva));
				}
				d = ff + KP * err + w->integral;
			}
			d = fminf(1.0f, fmaxf(w->d_min, d));
			d = fminf(mag + PASO_DUTY_PI, fmaxf(mag - PASO_DUTY_PI, d));
			w->duty = signo * d;
		}
	}

	motor_aplicar(w, w->duty);
	w->suma_duty += fabsf(w->duty);
	w->n_duty++;
}

/* Tarea de control a 100 Hz para las 4 ruedas */
static void control_task(void *arg)
{
	TickType_t ultimo = xTaskGetTickCount();

	(void)arg;
	for (int i = 0; i < N_RUEDAS; i++) {
		reiniciar_estado(&W[i]);
	}
	while (1) {
		for (int i = 0; i < N_RUEDAS; i++) {
			rueda_ciclo(&W[i]);
		}
		vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(10));
	}
}

/* ---- Secuencia de prueba ---- */

typedef struct {
	long total[N_RUEDAS];
	float suma_duty[N_RUEDAS];
	long n_duty[N_RUEDAS];
	int64_t t_us;
} foto_t;

static void foto(foto_t *f)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		f->total[i] = W[i].total;
		f->suma_duty[i] = W[i].suma_duty;
		f->n_duty[i] = W[i].n_duty;
	}
	f->t_us = esp_timer_get_time();
}

/* Velocidad media (flancos/s) y duty medio entre dos fotos */
static void promedios(const foto_t *a, const foto_t *b, float v[N_RUEDAS], float duty[N_RUEDAS])
{
	float seg = (b->t_us - a->t_us) / 1e6f;

	for (int i = 0; i < N_RUEDAS; i++) {
		v[i] = (b->total[i] - a->total[i]) / seg;
		duty[i] = (b->n_duty[i] > a->n_duty[i]) ? (b->suma_duty[i] - a->suma_duty[i]) / (b->n_duty[i] - a->n_duty[i]) : 0.0f;
	}
}

static int dentro_de_tolerancia(int i, float v)
{
	return !W[i].bloqueado && fabsf(v - V_TEST_FLANCOS) <= TOLERANCIA * V_TEST_FLANCOS;
}

static void parar_todas(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		W[i].objetivo = 0.0f;
	}
	pausa_ms(800);
}

/* Mueve las ruedas de `mascara`, escalonadas; espera `asentar_ms`, mide `medir_ms` y frena. Deja en `ok` si cada rueda cumple la tolerancia. */
static void tramo(int mascara, float vel, int asentar_ms, int medir_ms, int ok[N_RUEDAS], float v[N_RUEDAS], float duty[N_RUEDAS])
{
	foto_t a, b;

	for (int i = 0; i < N_RUEDAS; i++) {
		if (mascara & (1 << i)) {
			W[i].objetivo = vel;
			pausa_ms(ESCALONADO_MS);
		}
	}
	pausa_ms(asentar_ms);
	foto(&a);
	pausa_ms(medir_ms);
	foto(&b);
	promedios(&a, &b, v, duty);
	for (int i = 0; i < N_RUEDAS; i++) {
		if (mascara & (1 << i)) {
			ok[i] = dentro_de_tolerancia(i, v[i]);
			printf("  %s: %.1f flancos/s (pedido %.1f, %+.0f %%) duty medio %.2f -> %s\n", W[i].nombre, v[i], V_TEST_FLANCOS,
			       100.0f * (v[i] - V_TEST_FLANCOS) / V_TEST_FLANCOS, duty[i], ok[i] ? "OK" : "MAL");
		}
	}
	parar_todas();
}

/* Resumen por LED: fijo 5 s = todo OK; destellos muy rapidos = alguno fallo. Despues, por rueda, `n_res` resultados */
static void resumen_led(int n_res, int res[N_RUEDAS][3])
{
	int todo_ok = 1;

	for (int i = 0; i < N_RUEDAS; i++) {
		for (int k = 0; k < n_res; k++) {
			todo_ok = todo_ok && res[i][k];
		}
	}
	if (todo_ok) {
		led(1);
		pausa_ms(5000);
		led(0);
	} else {
		destellos(25, 100, 100);
	}
	pausa_ms(1500);
	for (int i = 0; i < N_RUEDAS; i++) {
		destellos(i + 1, 300, 300);
		pausa_ms(1200);
		for (int k = 0; k < n_res; k++) {
			senal_resultado(res[i][k]);
		}
		pausa_ms(1500);
	}
}

static void secuencia_task(void *arg)
{
	int ok[N_RUEDAS];
	float v[N_RUEDAS], duty[N_RUEDAS];
	int res[N_RUEDAS][3];
	const int TODAS = (1 << N_RUEDAS) - 1;

	(void)arg;
	led(1);                                      /* LED fijo 2 s: arranco bien */
	pausa_ms(2000);
	led(0);

	printf("\n=== PRUEBA DE 4 RUEDAS CON PI (V_TEST %.0f flancos/s = %.2f m/s) ===\n", V_TEST_FLANCOS,
	       V_TEST_FLANCOS / FLANCOS_POR_VUELTA * 3.14159265f * DIAMETRO_RUEDA_M);
	printf("Fase A: ruedas EN EL AIRE. Apreta BOOT para empezar.\n");
	esperar_boot();
	cuenta_regresiva();

	/* Fase A1: cada rueda sola, adelante y atras */
	for (int i = 0; i < N_RUEDAS; i++) {
		printf("A1 %s adelante:\n", W[i].nombre);
		tramo(1 << i, +V_TEST_FLANCOS, 4000, 3000, ok, v, duty);
		res[i][0] = ok[i];
		printf("A1 %s atras:\n", W[i].nombre);
		tramo(1 << i, -V_TEST_FLANCOS, 3500, 2500, ok, v, duty);
		res[i][1] = ok[i];
		pausa_ms(500);
	}

	/* Fase A2: las 4 juntas */
	printf("A2 las 4 juntas adelante:\n");
	tramo(TODAS, +V_TEST_FLANCOS, 4000, 3000, ok, v, duty);
	for (int i = 0; i < N_RUEDAS; i++) {
		res[i][2] = ok[i];
	}

	printf("\n=== RESUMEN FASE A (a mano alzada: por rueda adelante | atras | juntas) ===\n");
	for (int i = 0; i < N_RUEDAS; i++) {
		printf("  %s: %s | %s | %s\n", W[i].nombre, res[i][0] ? "OK" : "MAL", res[i][1] ? "OK" : "MAL", res[i][2] ? "OK" : "MAL");
	}
	resumen_led(3, res);

	printf("\nFase B: poner el robot EN EL PISO, con 1,5 m libres adelante. Apreta BOOT.\n");
	esperar_boot();
	cuenta_regresiva();

	/* Fase B1: las 4 juntas en el piso, misma velocidad */
	printf("B1 las 4 juntas en el piso:\n");
	tramo(TODAS, +V_TEST_FLANCOS, 4000, 3000, ok, v, duty);
	for (int i = 0; i < N_RUEDAS; i++) {
		res[i][0] = ok[i];
	}
	resumen_led(1, res);

	/* Fase B2: 1 m calculado con los encoders, las veces que quieras */
	const float flancos_objetivo = DISTANCIA_M / (3.14159265f * DIAMETRO_RUEDA_M) * FLANCOS_POR_VUELTA;
	int primera = 1;

	for (;;) {
		foto_t a, b;
		float suma = 0.0f;

		if (primera) {
			printf("\nB2: avanzar %.2f m = %.1f flancos por rueda. Poner el robot en la marca y apretar BOOT.\n", DISTANCIA_M,
			       flancos_objetivo);
			esperar_boot();
			primera = 0;
		}
		cuenta_regresiva();
		foto(&a);
		for (int i = 0; i < N_RUEDAS; i++) {
			W[i].objetivo = +V_TEST_FLANCOS;
			pausa_ms(ESCALONADO_MS);
		}
		for (;;) {
			float media = 0.0f;

			for (int i = 0; i < N_RUEDAS; i++) {
				media += (W[i].total - a.total[i]) / (float)N_RUEDAS;
			}
			if (media >= flancos_objetivo) {
				break;
			}
			if (esp_timer_get_time() - a.t_us > 15000000) {
				printf("B2: 15 s sin llegar a la distancia: se corta\n");
				break;
			}
			pausa_ms(10);
		}
		parar_todas();
		pausa_ms(1500);                          /* que termine de rodar */
		foto(&b);
		for (int i = 0; i < N_RUEDAS; i++) {
			float f = (float)(b.total[i] - a.total[i]);

			suma += f;
			printf("  %s: %ld flancos = %.3f m\n", W[i].nombre, b.total[i] - a.total[i],
			       f / FLANCOS_POR_VUELTA * 3.14159265f * DIAMETRO_RUEDA_M);
		}
		printf("B2: media %.1f flancos = %.3f m calculados. Medi cuanto avanzo de verdad.\n", suma / N_RUEDAS,
		       suma / N_RUEDAS / FLANCOS_POR_VUELTA * 3.14159265f * DIAMETRO_RUEDA_M);

		/* Flancos medios por rueda por el LED. BOOT corto: repetir el numero. BOOT largo (1,5 s): otra corrida de 1 m */
		do {
			mostrar_numero((int)(suma / N_RUEDAS + 0.5f));
		} while (esperar_boot_ms() < 1500);
	}
}

void app_main(void)
{
	/* Lo primero: todas las entradas de los drivers en 0 */
	pwm_init();
	for (int i = 0; i < N_RUEDAS; i++) {
		motor_aplicar(&W[i], 0.0f);
	}
	encoders_init();

	gpio_config_t led_cfg = {
		.pin_bit_mask = 1ULL << PIN_LED,
		.mode = GPIO_MODE_OUTPUT,
	};
	ESP_ERROR_CHECK(gpio_config(&led_cfg));
	gpio_config_t boot_cfg = {
		.pin_bit_mask = 1ULL << PIN_BOOT,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&boot_cfg));

	xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
	xTaskCreate(secuencia_task, "secuencia", 4096, NULL, 5, NULL);
}
