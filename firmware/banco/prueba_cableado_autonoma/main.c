#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

/* Prueba de cableado de las 4 ruedas SIN la PC conectada (sin micro-ROS, sin Wi-Fi). El resultado se ve en el LED de la placa
 * (GPIO2) y en las propias ruedas; si hay un monitor conectado, tambien sale por el puerto serie.
 * Fase 1: el LED te dice que rueda girar a mano (N destellos = rueda N) y avisa si conto SU encoder.
 * Fase 2: con las ruedas en el aire apretas BOOT; el firmware mueve cada rueda un segundo, adelante y atras, y compara los 4 encoders.
 * Despues repite el resumen con el LED. Pines: hardware/electronica/propuesta-v0.2.md, mapa B-rueda (RR_IB pasa del 2 al 32). */

#define N_RUEDAS 4
#define PWM_HZ 5000
#define PWM_MAX 1023                 /* 10 bits */
#define DUTY_PRUEBA 1.0f             /* los traseros solo arrancan seguro con duty 1,0 */
#define MS_GIRO 1000
#define MS_PAUSA 1500
#define MS_ESPERA_MANO 20000         /* por rueda, en la fase 1 */
#define FLANCOS_MANO 20              /* media vuelta: 40 flancos por vuelta */
#define FLANCOS_MIN 20               /* a 1 s y duty 1,0 el trasero mas lento da ~90 */
#define FLANCOS_RUIDO 2
#define PIN_LED GPIO_NUM_2
#define PIN_BOOT GPIO_NUM_0          /* boton BOOT de la placa: se lee solo despues del arranque */

typedef struct {
	const char *nombre;
	gpio_num_t pin_ia;               /* adelante (PWM) */
	gpio_num_t pin_ib;               /* atras (PWM)    */
	gpio_num_t pin_enc;              /* D0 del encoder */
	ledc_channel_t canal_ia;
	ledc_channel_t canal_ib;
	pcnt_unit_handle_t unidad;
} rueda_t;

static rueda_t R[N_RUEDAS] = {
	{"FL", GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_34, LEDC_CHANNEL_0, LEDC_CHANNEL_1, NULL},
	{"FR", GPIO_NUM_17, GPIO_NUM_33, GPIO_NUM_36, LEDC_CHANNEL_2, LEDC_CHANNEL_3, NULL},
	{"RL", GPIO_NUM_27, GPIO_NUM_16, GPIO_NUM_35, LEDC_CHANNEL_4, LEDC_CHANNEL_5, NULL},
	{"RR", GPIO_NUM_23, GPIO_NUM_32, GPIO_NUM_39, LEDC_CHANNEL_6, LEDC_CHANNEL_7, NULL},
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
			.gpio_num = R[i].pin_ia,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel = R[i].canal_ia,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = LEDC_TIMER_0,
			.duty = 0,
			.hpoint = 0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
		canal.gpio_num = R[i].pin_ib;
		canal.channel = R[i].canal_ib;
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
	}
}

/* sentido > 0: adelante (IA = PWM, IB = 0). < 0: atras. 0: las dos en 0 */
static void rueda_aplicar(int i, int sentido, float duty)
{
	uint32_t d = (uint32_t)(duty * PWM_MAX + 0.5f);

	pwm_set(R[i].canal_ia, sentido > 0 ? d : 0);
	pwm_set(R[i].canal_ib, sentido < 0 ? d : 0);
}

static void encoders_init(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		pcnt_unit_config_t unidad_cfg = {
			.low_limit = -32768,
			.high_limit = 32767,
		};
		ESP_ERROR_CHECK(pcnt_new_unit(&unidad_cfg, &R[i].unidad));

		pcnt_glitch_filter_config_t filtro_cfg = {
			.max_glitch_ns = 5000,
		};
		ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(R[i].unidad, &filtro_cfg));

		pcnt_chan_config_t canal_cfg = {
			.edge_gpio_num = R[i].pin_enc,
			.level_gpio_num = -1,
		};
		pcnt_channel_handle_t canal = NULL;
		ESP_ERROR_CHECK(pcnt_new_channel(R[i].unidad, &canal_cfg, &canal));
		ESP_ERROR_CHECK(pcnt_channel_set_edge_action(canal,
			PCNT_CHANNEL_EDGE_ACTION_INCREASE,
			PCNT_CHANNEL_EDGE_ACTION_INCREASE));

		ESP_ERROR_CHECK(pcnt_unit_enable(R[i].unidad));
		ESP_ERROR_CHECK(pcnt_unit_clear_count(R[i].unidad));
		ESP_ERROR_CHECK(pcnt_unit_start(R[i].unidad));
	}
}

static void encoders_borrar(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		pcnt_unit_clear_count(R[i].unidad);
	}
}

static void encoders_leer(int cuentas[N_RUEDAS])
{
	for (int i = 0; i < N_RUEDAS; i++) {
		cuentas[i] = 0;
		pcnt_unit_get_count(R[i].unidad, &cuentas[i]);
	}
}

/* Mayor cuenta entre las ruedas que NO son la i; devuelve cual en *cual */
static int otro_maximo(int i, const int cuentas[N_RUEDAS], int *cual)
{
	int otro_max = 0;

	*cual = -1;
	for (int j = 0; j < N_RUEDAS; j++) {
		if (j != i && cuentas[j] > otro_max) {
			otro_max = cuentas[j];
			*cual = j;
		}
	}
	return otro_max;
}

/* Fase 1: espera que giren a mano la rueda i; 1 = conto su encoder y los otros no */
static int esperar_giro_a_mano(int i)
{
	int cuentas[N_RUEDAS];
	int cual;

	encoders_borrar();
	led(1);
	for (int t = 0; t < MS_ESPERA_MANO; t += 50) {
		pausa_ms(50);
		encoders_leer(cuentas);
		if (otro_maximo(i, cuentas, &cual) >= FLANCOS_MANO) {
			printf("  -> MAL: giro %s y conto el encoder de %s (%d): encoders cruzados\n", R[i].nombre, R[cual].nombre,
			       cuentas[cual]);
			led(0);
			return 0;
		}
		if (cuentas[i] >= FLANCOS_MANO) {
			pausa_ms(300);                       /* por si otro encoder empieza a contar tambien */
			encoders_leer(cuentas);
			led(0);
			if (otro_maximo(i, cuentas, &cual) > FLANCOS_RUIDO) {
				printf("  -> DUDOSO: %s cuenta %d pero %s tambien (%d)\n", R[i].nombre, cuentas[i], R[cual].nombre,
				       cuentas[cual]);
				return 0;
			}
			printf("  -> OK: %s cuenta %d\n", R[i].nombre, cuentas[i]);
			return 1;
		}
	}
	led(0);
	printf("  -> MAL: %s no conto nada en %d s: revisar D0, VCC y GND de su encoder\n", R[i].nombre, MS_ESPERA_MANO / 1000);
	return 0;
}

/* Fase 2: compara las 4 cuentas con la rueda que se movio; devuelve 1 si esta bien */
static int evaluar(int i, const int cuentas[N_RUEDAS])
{
	int cual;
	int otro_max = otro_maximo(i, cuentas, &cual);

	if (cuentas[i] >= FLANCOS_MIN && otro_max <= FLANCOS_RUIDO) {
		printf("  -> OK: cuenta su encoder (%d) y los otros no\n", cuentas[i]);
		return 1;
	}
	if (cuentas[i] >= FLANCOS_MIN) {
		printf("  -> DUDOSO: su encoder cuenta %d pero el de %s tambien (%d)\n", cuentas[i], R[cual].nombre, otro_max);
		return 0;
	}
	if (otro_max >= FLANCOS_MIN) {
		printf("  -> MAL: el encoder que cuenta es el de %s (%d), no el de %s (%d): encoders cruzados\n",
		       R[cual].nombre, otro_max, R[i].nombre, cuentas[i]);
		return 0;
	}
	printf("  -> MAL: su encoder cuenta solo %d. Se movio la rueda? Si giro, revisar su encoder; si no giro, revisar IA/IB y la alimentacion del driver\n",
	       cuentas[i]);
	return 0;
}

void app_main(void)
{
	int cuentas[N_RUEDAS];
	int fase1[N_RUEDAS];
	int mov[N_RUEDAS][2];

	/* Lo primero: todas las entradas de los drivers en 0 */
	pwm_init();
	for (int i = 0; i < N_RUEDAS; i++) {
		rueda_aplicar(i, 0, 0.0f);
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

	printf("\n=== PRUEBA DE CABLEADO AUTONOMA (4 ruedas) ===\n");
	led(1);                                      /* LED fijo 2 s: arranco bien */
	pausa_ms(2000);
	led(0);
	pausa_ms(3000);

	printf("\n--- Fase 1: gira a MANO la rueda que indica el LED (N destellos = rueda N: 1 FL, 2 FR, 3 RL, 4 RR) ---\n");
	for (int i = 0; i < N_RUEDAS; i++) {
		printf("Gira %s\n", R[i].nombre);
		destellos(i + 1, 300, 300);
		pausa_ms(1000);
		fase1[i] = esperar_giro_a_mano(i);       /* LED fijo mientras espera */
		senal_resultado(fase1[i]);
		pausa_ms(1000);
	}

	printf("\nPoner las ruedas EN EL AIRE. Cuando estes listo, apreta BOOT.\n");
	for (int t = 0; gpio_get_level(PIN_BOOT) != 0; t += 20) {    /* LED con pulso lento: espera el boton */
		led((t / 500) % 2 == 0);
		pausa_ms(20);
	}
	led(0);
	for (int k = 0; k < 25; k++) {               /* cuenta regresiva de 5 s con destellos rapidos */
		destellos(1, 100, 100);
	}

	printf("\n--- Fase 2: el firmware mueve cada rueda %d ms, adelante y atras (duty %.1f) ---\n", MS_GIRO, DUTY_PRUEBA);
	printf("Mira cada rueda: 'adelante' debe girar en el sentido en que avanzaria el robot.\n");
	for (int i = 0; i < N_RUEDAS; i++) {
		for (int s = 0; s < 2; s++) {
			int sentido = (s == 0) ? 1 : -1;

			printf("Rueda %s, %s (pines %d/%d, encoder %d):\n", R[i].nombre, sentido > 0 ? "ADELANTE" : "ATRAS",
			       (int)R[i].pin_ia, (int)R[i].pin_ib, (int)R[i].pin_enc);
			encoders_borrar();
			rueda_aplicar(i, sentido, DUTY_PRUEBA);
			pausa_ms(MS_GIRO);
			rueda_aplicar(i, 0, 0.0f);
			pausa_ms(200);
			encoders_leer(cuentas);
			printf("  cuentas: FL=%d FR=%d RL=%d RR=%d\n", cuentas[0], cuentas[1], cuentas[2], cuentas[3]);
			mov[i][s] = evaluar(i, cuentas);
			pausa_ms(MS_PAUSA);
		}
	}

	int todo_ok = 1;

	printf("\n=== RESUMEN (anota tambien el sentido real en que giro cada una) ===\n");
	for (int i = 0; i < N_RUEDAS; i++) {
		printf("  %s: a mano %s | adelante %s | atras %s\n", R[i].nombre, fase1[i] ? "OK" : "MAL",
		       mov[i][0] ? "OK" : "MAL", mov[i][1] ? "OK" : "MAL");
		todo_ok = todo_ok && fase1[i] && mov[i][0] && mov[i][1];
	}

	/* El resumen se repite con el LED hasta que apagues:
	 * LED fijo 5 s = las 12 pruebas OK; destellos muy rapidos 5 s = alguna fallo.
	 * Despues, por rueda: N destellos (rueda N) y tres resultados: a mano, adelante, atras (1 largo = OK, 3 cortos = MAL). */
	for (;;) {
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
			senal_resultado(fase1[i]);
			senal_resultado(mov[i][0]);
			senal_resultado(mov[i][1]);
			pausa_ms(1500);
		}
		pausa_ms(4000);
	}
}
