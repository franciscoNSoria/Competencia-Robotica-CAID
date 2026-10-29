#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

/* Prueba del hombro y el codo moviendose juntos (sin micro-ROS, sin Wi-Fi).
 * Levanta y baja el brazo con los dos servos coordinados para repartir el esfuerzo: el codo retraido acorta la palanca
 * que carga el hombro. Pines: hardware/electronica/propuesta-v0.2.md (hombro = GPIO19, codo = GPIO21).
 * Pulsos medidos por el equipo: hombro 1300 sube / 1700 baja; codo 1700 retrae / 1000 estira. */

#define PWM_HZ 50
#define PERIODO_US 20000
#define CUENTAS 16384                /* 14 bits: 1,22 us por cuenta */

#define HOMBRO_ABAJO 1700            /* reposo */
#define HOMBRO_ARRIBA 1300
#define CODO_RETRAIDO 1700           /* reposo */
#define CODO_FINAL 1300              /* adonde llega el codo al levantar (1000 = estirado del todo) */

#define PASO_MANUAL_US 20
#define VEL_MANUAL_US_POR_S 400      /* en las ordenes manuales el pulso avanza como maximo esto por segundo */
#define PERIODO_CTRL_MS 20
#define DURACION_INICIAL_MS 4000
#define DURACION_MIN_MS 1000
#define DURACION_MAX_MS 15000
#define ADELANTO_INICIAL_US 100      /* el pulso va esto por delante de la trayectoria: mas error, mas torque */
#define ADELANTO_MAX_US 200
#define PASO_ADELANTO_US 20
#define RETARDO_CODO_PM 600          /* 'L'/'B': el codo espera a que el hombro llegue al 60 % de su recorrido */

enum { HOMBRO, CODO, N_EJES };

typedef struct {
	const char *nombre;
	gpio_num_t pin;
	ledc_channel_t canal;
	int min_us, max_us;              /* limites: solo lo que ya se probo a mano */
	int reposo_us;
	int pulso;                       /* lo que se esta mandando */
	int objetivo;
} eje_t;

static eje_t ejes[N_EJES] = {
	{"hombro", GPIO_NUM_19, LEDC_CHANNEL_0, HOMBRO_ARRIBA, HOMBRO_ABAJO, HOMBRO_ABAJO, HOMBRO_ABAJO, HOMBRO_ABAJO},
	{"codo", GPIO_NUM_21, LEDC_CHANNEL_1, 1000, CODO_RETRAIDO, CODO_RETRAIDO, CODO_RETRAIDO, CODO_RETRAIDO},
};

static int pulsando = 0;                         /* 0: sin senal (los servos no reciben pulsos) */
static int progreso_pm = -1;                     /* 0 = reposo, 1000 = levantado, -1 = se perdio la cuenta */
static int sentido = 0;                          /* +1 levantando, -1 bajando, 0 quieto */
static int t_ms = 0;                             /* tiempo recorrido dentro del movimiento */
static int retardo_pm = 0;
static int duracion_ms = DURACION_INICIAL_MS;
static int adelanto_us = ADELANTO_INICIAL_US;
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

	for (int i = 0; i < N_EJES; i++) {
		ledc_channel_config_t canal = {
			.gpio_num = ejes[i].pin,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = ejes[i].canal,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = LEDC_TIMER_0,
			.duty = 0,                           /* sin pulsos hasta que lo pidas */
			.hpoint = 0,
		};
		ESP_ERROR_CHECK(ledc_channel_config(&canal));
	}
}

static void pwm_us(const eje_t *e, int us)
{
	uint32_t duty = ((uint32_t)us * CUENTAS + PERIODO_US / 2) / PERIODO_US;

	ledc_set_duty(LEDC_LOW_SPEED_MODE, e->canal, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, e->canal);
}

static void pwm_apagar(void)
{
	for (int i = 0; i < N_EJES; i++) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, ejes[i].canal, 0);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, ejes[i].canal);
	}
}

static void ayuda(void)
{
	printf("\nHombro (GPIO%d) y codo (GPIO%d). Ordenes (una tecla):\n", (int)ejes[HOMBRO].pin, (int)ejes[CODO].pin);
	printf("  g       empezar a mandar pulso. ANTES, el brazo en el reposo (hombro abajo %d us, codo retraido %d us)\n",
	       HOMBRO_ABAJO, CODO_RETRAIDO);
	printf("  l / b   levantar / bajar con los dos servos moviendose juntos\n");
	printf("  L / B   levantar / bajar con el codo retraido hasta que el hombro lleva el %d %% del recorrido\n",
	       RETARDO_CODO_PM / 10);
	printf("  + / -   mas lento / mas rapido (ahora %d s por recorrido)\n", duracion_ms / 1000);
	printf("  > / <   mas / menos adelanto del pulso en los movimientos (ahora %d us)\n", adelanto_us);
	printf("  u / j   hombro sube / baja %d us        i / k   codo estira / retrae %d us\n", PASO_MANUAL_US, PASO_MANUAL_US);
	printf("  r       volver al reposo        p   mostrar los pulsos        m   marca (anota como salio)\n");
	printf("  s       soltar: dejar de mandar pulsos (el brazo cae por su peso)\n");
	printf("  ?       esta ayuda\n");
	printf("Si un servo zumba sin mover el brazo mas de 2 s, apreta 's' y avisame.\n\n");
}

static void mostrar(void)
{
	printf("hombro %d us   codo %d us   progreso %d %%\n", ejes[HOMBRO].pulso, ejes[CODO].pulso,
	       progreso_pm < 0 ? -1 : progreso_pm / 10);
}

static int mezcla(int a, int b, int pm)
{
	return a + (b - a) * pm / 1000;
}

static int acotar(const eje_t *e, int us)
{
	if (us < e->min_us) {
		printf("limite: %s no pasa de %d us\n", e->nombre, e->min_us);
		return e->min_us;
	}
	if (us > e->max_us) {
		printf("limite: %s no pasa de %d us\n", e->nombre, e->max_us);
		return e->max_us;
	}
	return us;
}

static int limitar(const eje_t *e, int us)
{
	return us < e->min_us ? e->min_us : us > e->max_us ? e->max_us : us;
}

static void aplicar_progreso(void)
{
	int pm_codo = (progreso_pm - retardo_pm) * 1000 / (1000 - retardo_pm);

	if (pm_codo < 0) {
		pm_codo = 0;
	}
	ejes[HOMBRO].objetivo = mezcla(HOMBRO_ABAJO, HOMBRO_ARRIBA, progreso_pm);
	ejes[CODO].objetivo = mezcla(CODO_RETRAIDO, CODO_FINAL, pm_codo);
	ejes[HOMBRO].pulso = ejes[HOMBRO].objetivo;
	ejes[CODO].pulso = ejes[CODO].objetivo;

	/* El servo empuja en proporcion al error: se le manda un pulso adelantado en el sentido del movimiento.
	 * Al llegar a los extremos el adelanto se acota con los limites y desaparece. */
	if (progreso_pm > 0 && progreso_pm < 1000) {
		ejes[HOMBRO].pulso = limitar(&ejes[HOMBRO], ejes[HOMBRO].objetivo - sentido * adelanto_us);
	}
	if (pm_codo > 0 && pm_codo < 1000) {
		ejes[CODO].pulso = limitar(&ejes[CODO], ejes[CODO].objetivo - sentido * adelanto_us);
	}
}

static void arrancar(int dir, int retardo)
{
	if (!pulsando) {
		printf("primero 'g'\n");
		return;
	}
	if (sentido != 0) {
		printf("ya se esta moviendo\n");
		return;
	}
	if ((dir > 0 && progreso_pm != 0) || (dir < 0 && progreso_pm != 1000)) {
		printf("no estoy en el %s: mandalo primero con '%c'\n", dir > 0 ? "reposo" : "extremo levantado",
		       dir > 0 ? 'r' : 'l');
		return;
	}
	retardo_pm = retardo;
	sentido = dir;
	t_ms = dir > 0 ? 0 : duracion_ms;
	printf("%s en %d s%s\n", dir > 0 ? "levantando" : "bajando", duracion_ms / 1000, retardo ? ", codo retraido primero" : "");
}

static void manual(int e, int delta)
{
	if (!pulsando) {
		printf("primero 'g'\n");
		return;
	}
	if (sentido != 0) {
		return;
	}
	progreso_pm = -1;
	ejes[e].objetivo = acotar(&ejes[e], ejes[e].objetivo + delta);
	printf("%s objetivo %d us (desde %d)\n", ejes[e].nombre, ejes[e].objetivo, ejes[e].pulso);
}

static void orden(char c)
{
	switch (c) {
	case 'g':
		if (pulsando) {
			printf("ya se esta mandando pulso\n");
			break;
		}
		pulsando = 1;
		for (int i = 0; i < N_EJES; i++) {
			ejes[i].pulso = ejes[i].reposo_us;
			ejes[i].objetivo = ejes[i].reposo_us;
		}
		progreso_pm = 0;
		sentido = 0;
		printf("mandando hombro %d us y codo %d us\n", ejes[HOMBRO].pulso, ejes[CODO].pulso);
		break;
	case 'l': arrancar(+1, 0); break;
	case 'b': arrancar(-1, 0); break;
	case 'L': arrancar(+1, RETARDO_CODO_PM); break;
	case 'B': arrancar(-1, RETARDO_CODO_PM); break;
	case '+':
		if (sentido != 0) {
			break;
		}
		duracion_ms = duracion_ms + 1000 > DURACION_MAX_MS ? DURACION_MAX_MS : duracion_ms + 1000;
		printf("%d s por recorrido\n", duracion_ms / 1000);
		break;
	case '-':
		if (sentido != 0) {
			break;
		}
		duracion_ms = duracion_ms - 1000 < DURACION_MIN_MS ? DURACION_MIN_MS : duracion_ms - 1000;
		printf("%d s por recorrido\n", duracion_ms / 1000);
		break;
	case '>':
	case '<':
		if (sentido != 0) {
			break;
		}
		adelanto_us += c == '>' ? PASO_ADELANTO_US : -PASO_ADELANTO_US;
		adelanto_us = adelanto_us < 0 ? 0 : adelanto_us > ADELANTO_MAX_US ? ADELANTO_MAX_US : adelanto_us;
		printf("adelanto %d us (equivale a unos %d grados)\n", adelanto_us, adelanto_us * 115 / 1000);
		break;
	case 'u': manual(HOMBRO, -PASO_MANUAL_US); break;
	case 'j': manual(HOMBRO, +PASO_MANUAL_US); break;
	case 'i': manual(CODO, -PASO_MANUAL_US); break;
	case 'k': manual(CODO, +PASO_MANUAL_US); break;
	case 'r':
		if (!pulsando || sentido != 0) {
			printf("no se puede ahora\n");
			break;
		}
		for (int i = 0; i < N_EJES; i++) {
			ejes[i].objetivo = ejes[i].reposo_us;
		}
		progreso_pm = -1;
		printf("volviendo al reposo\n");
		break;
	case 'p': mostrar(); break;
	case 'm':
		marcas++;
		printf("MARCA,%d,%d,%d,%d,%d,%d\n", marcas, ejes[HOMBRO].pulso, ejes[CODO].pulso, duracion_ms, retardo_pm, adelanto_us);
		break;
	case 's':
		pulsando = 0;
		sentido = 0;
		progreso_pm = -1;
		pwm_apagar();
		printf("sin senal. Ultimos pulsos: hombro %d us, codo %d us\n", ejes[HOMBRO].pulso, ejes[CODO].pulso);
		break;
	case '?': ayuda(); break;
	default: break;
	}
}

static void paso_manual(eje_t *e)
{
	int paso = VEL_MANUAL_US_POR_S * PERIODO_CTRL_MS / 1000;

	if (e->pulso < e->objetivo) {
		e->pulso += (e->objetivo - e->pulso < paso) ? e->objetivo - e->pulso : paso;
	} else if (e->pulso > e->objetivo) {
		e->pulso -= (e->pulso - e->objetivo < paso) ? e->pulso - e->objetivo : paso;
	}
}

void app_main(void)
{
	int ticks = 0;

	pwm_init();                                  /* sin pulsos: los servos no reciben senal */
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

	printf("\n=== HOMBRO Y CODO COORDINADOS ===\n");
	ayuda();
	printf("Todavia no se manda ninguna senal. Con el brazo en el reposo, apreta 'g'.\n");

	for (;;) {
		uint8_t c;

		while (uart_read_bytes(UART_NUM_0, &c, 1, 0) > 0) {
			orden((char)c);
		}

		if (pulsando) {
			if (sentido != 0) {
				t_ms += sentido * PERIODO_CTRL_MS;
				progreso_pm = t_ms * 1000 / duracion_ms;
				if (progreso_pm >= 1000 || progreso_pm <= 0) {
					progreso_pm = progreso_pm >= 1000 ? 1000 : 0;
					sentido = 0;
					printf("llego: ");
					mostrar();
				}
				aplicar_progreso();
				if (++ticks % 10 == 0) {
					mostrar();
				}
			} else {
				int antes_h = ejes[HOMBRO].pulso, antes_c = ejes[CODO].pulso;

				paso_manual(&ejes[HOMBRO]);
				paso_manual(&ejes[CODO]);
				if (progreso_pm < 0 && ejes[HOMBRO].pulso == ejes[HOMBRO].reposo_us &&
				    ejes[CODO].pulso == ejes[CODO].reposo_us) {
					progreso_pm = 0;
				}
				if ((antes_h != ejes[HOMBRO].pulso || antes_c != ejes[CODO].pulso) &&
				    ejes[HOMBRO].pulso == ejes[HOMBRO].objetivo && ejes[CODO].pulso == ejes[CODO].objetivo) {
					printf("llego: ");
					mostrar();
				}
			}
			pwm_us(&ejes[HOMBRO], ejes[HOMBRO].pulso);
			pwm_us(&ejes[CODO], ejes[CODO].pulso);
		}
		vTaskDelay(pdMS_TO_TICKS(PERIODO_CTRL_MS));
	}
}
