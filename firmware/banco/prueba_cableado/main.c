#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

/* Prueba de cableado de las 4 ruedas (sin micro-ROS, sin Wi-Fi).
 * Fase 1: giras cada rueda a mano y mira que cuente SU encoder.
 * Fase 2: el firmware mueve cada rueda un segundo, adelante y atras, y compara con los 4 encoders.
 * Pines: hardware/electronica/propuesta-v0.2.md, mapa B-rueda (RR_IB pasa del 2 al 32). */

#define N_RUEDAS 4
#define PWM_HZ 5000
#define PWM_MAX 1023                 /* 10 bits */
#define DUTY_PRUEBA 1.0f             /* los traseros solo arrancan seguro con duty 1,0 */
#define MS_GIRO 1000
#define MS_PAUSA 1500
#define MS_FASE_MANO 20000
#define FLANCOS_MIN 20               /* a 1 s y duty 1,0 el trasero mas lento da ~90 */
#define FLANCOS_RUIDO 2

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

static void pausa_ms(int ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Compara las 4 cuentas con la rueda que se movio; devuelve 1 si esta bien */
static int evaluar(int i, const int cuentas[N_RUEDAS])
{
	int otro_max = 0;
	int cual = -1;

	for (int j = 0; j < N_RUEDAS; j++) {
		if (j != i && cuentas[j] > otro_max) {
			otro_max = cuentas[j];
			cual = j;
		}
	}
	if (cuentas[i] >= FLANCOS_MIN && otro_max <= FLANCOS_RUIDO) {
		printf("  -> OK: cuenta su encoder (%d) y los otros no\n", cuentas[i]);
		return 1;
	}
	if (cuentas[i] >= FLANCOS_MIN) {
		printf("  -> DUDOSO: su encoder cuenta %d pero el de %s tambien (%d): cables de encoder en contacto o el mismo D0 en dos pines\n",
		       cuentas[i], R[cual].nombre, otro_max);
		return 0;
	}
	if (otro_max >= FLANCOS_MIN) {
		printf("  -> MAL: el encoder que cuenta es el de %s (%d), no el de %s (%d): encoders cruzados\n",
		       R[cual].nombre, otro_max, R[i].nombre, cuentas[i]);
		return 0;
	}
	printf("  -> MAL: su encoder cuenta solo %d. Se movio la rueda? Si giro, revisar el cable D0/VCC/GND del encoder; si no giro, revisar IA/IB y la alimentacion del driver\n",
	       cuentas[i]);
	return 0;
}

void app_main(void)
{
	int cuentas[N_RUEDAS];
	int resultado[N_RUEDAS][2];

	/* Lo primero: todas las entradas de los drivers en 0 */
	pwm_init();
	for (int i = 0; i < N_RUEDAS; i++) {
		rueda_aplicar(i, 0, 0.0f);
	}
	encoders_init();

	printf("\n=== PRUEBA DE CABLEADO (4 ruedas) ===\n");
	printf("Ruedas EN EL AIRE. Empieza la fase 1 en 5 s.\n");
	pausa_ms(5000);

	printf("\n--- Fase 1: gira cada rueda A MANO (%d s). Cuentas acumuladas, 40 por vuelta ---\n", MS_FASE_MANO / 1000);
	encoders_borrar();
	for (int t = 0; t < MS_FASE_MANO; t += 500) {
		pausa_ms(500);
		encoders_leer(cuentas);
		printf("  %5.1f s  FL=%5d  FR=%5d  RL=%5d  RR=%5d\n", (t + 500) / 1000.0f,
		       cuentas[0], cuentas[1], cuentas[2], cuentas[3]);
	}

	printf("\nSi todavia no encendiste la alimentacion de los motores (5 V del shield), hacelo ahora.\n");
	printf("La fase 2 empieza en 10 s.\n");
	pausa_ms(10000);

	printf("\n--- Fase 2: el firmware mueve cada rueda %d ms, adelante y atras (duty %.1f) ---\n",
	       MS_GIRO, DUTY_PRUEBA);
	printf("Mira cada rueda: 'adelante' debe girar en el sentido en que avanzaria el robot.\n");
	for (int i = 0; i < N_RUEDAS; i++) {
		for (int s = 0; s < 2; s++) {
			int sentido = (s == 0) ? 1 : -1;

			printf("Rueda %s, %s (pines %d/%d, encoder %d):\n", R[i].nombre,
			       sentido > 0 ? "ADELANTE" : "ATRAS",
			       (int)R[i].pin_ia, (int)R[i].pin_ib, (int)R[i].pin_enc);
			encoders_borrar();
			rueda_aplicar(i, sentido, DUTY_PRUEBA);
			pausa_ms(MS_GIRO);
			rueda_aplicar(i, 0, 0.0f);
			pausa_ms(200);
			encoders_leer(cuentas);
			printf("  cuentas: FL=%d FR=%d RL=%d RR=%d\n", cuentas[0], cuentas[1], cuentas[2], cuentas[3]);
			resultado[i][s] = evaluar(i, cuentas);
			pausa_ms(MS_PAUSA);
		}
	}

	printf("\n=== RESUMEN (anota tambien el sentido real en que giro cada una) ===\n");
	for (int i = 0; i < N_RUEDAS; i++) {
		printf("  %s: adelante %s | atras %s\n", R[i].nombre,
		       resultado[i][0] ? "OK" : "MAL", resultado[i][1] ? "OK" : "MAL");
	}
	printf("Fin. Para repetir, apreta EN.\n");
	for (;;) {
		pausa_ms(1000);
	}
}
