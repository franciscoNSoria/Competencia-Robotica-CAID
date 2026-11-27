#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

/* Robot de 4 ruedas con micro-ROS y comandos desde la PC (robot_teclado.py). Un PI de velocidad por rueda (el de
 * main_motor_v8_pi.c, una instancia por rueda).
 * Entra:   /cmd_vel      geometry_msgs/Twist, linear.x en m/s y angular.z en rad/s (cinematica diferencial)
 *          /robot_cmd    std_msgs/Float32MultiArray [id, args...]:
 *                          1 d0 d1 d2 d3      duty directo por rueda, lazo abierto (calibracion); 0 = esa rueda libre
 *                          2 v0 v1 v2 v3      velocidad por rueda en m/s, con PI (FL, FR, RL, RR)
 *                          3 dist_m vel       mover dist_m (con signo) con rampas; vel en flancos/s (0 = por defecto)
 *                          4                  parar todo
 *                          5 diam trocha fren config en metros: diametro efectivo, trocha efectiva, adelanto de frenado
 *                          6 rueda dmin n v1 d1 v2 d2 ...   curva de prealimentacion de una rueda (n <= 10 puntos)
 * Sale:    /wheel_ticks   std_msgs/Int32MultiArray, 4 flancos acumulados con signo [FL, FR, RL, RR], 20 Hz
 *          /wheel_state   std_msgs/Float32MultiArray, 24 valores, 10 Hz:
 *                          [0..3] velocidad medida (flancos/s con signo)  [4..7] velocidad pedida (flancos/s)
 *                          [8..11] duty con signo   [12..15] rueda bloqueada (1) o no (0)
 *                          [16] movimiento activo   [17] recorrido actual (m, con signo)   [18] recorrido final (m)
 *                          [19] contador de movimientos terminados   [20] diametro efectivo (mm)   [21] adelanto de frenado (mm)
 *                          [22] contador de configuraciones y curvas aceptadas   [23] trocha efectiva (mm)
 * linear.x es m/s de verdad (en v8d, 0,5 equivalia a 75 flancos/s = 0,395 m/s).
 * Pines: hardware/electronica/propuesta-v0.2.md, mapa B-rueda (RR_IB en el 32). */

#define N_RUEDAS 4
#define PIN_LED GPIO_NUM_2
#define PWM_HZ 5000
#define PWM_MAX 1023                /* 10 bits */

/* Calibrar con la prueba del metro (docs de calibracion): el diametro efectivo y la trocha efectiva salen de medir */
#define DIAMETRO_RUEDA_M 0.067f     /* CAD (dimensions.yaml); el valor en uso se cambia con /robot_cmd id 5 */
#define TROCHA_M 0.131f             /* centro a centro (dimensions.yaml); en skid-steer la efectiva es mayor: calibrar con un giro */
#define FRENADO_M 0.030f            /* se corta esto antes de llegar para que la inercia complete el recorrido; la PC lo ajusta sola */
#define ACEL_FLANCOS_S2 100.0f      /* rampa de arranque y de frenado en los movimientos de distancia (0,53 m/s2) */
#define VEL_MOV_DEFECTO 40.0f       /* flancos/s (0,21 m/s) */
#define MOV_ESPERA_CICLOS 150       /* 1,5 s para que termine de rodar antes de dar el recorrido final */
#define MOV_MAX_CICLOS 2000         /* 20 s: tope de un movimiento */
#define MAX_PUNTOS 10
#define FLANCOS_POR_VUELTA 40       /* 20 ranuras x 2 flancos */
#define VEL_MAX_FLANCOS 75.0f       /* limite por rueda: lo que alcanzan los 4 motores con margen (0,395 m/s) */
#define VEL_MIN_FLANCOS 25.0f       /* menor velocidad que sostienen los 4 motores */
#define TIMEOUT_CMD_MS 1000         /* sin orden: frenar (banco; en la competencia lo cubre el freno local) */
#define ESCALONADO_CICLOS 10        /* cada rueda arranca 10 ciclos (100 ms) despues de la anterior */
#define VENTANA_INFORME_CICLOS 50   /* informe por el puerto serie cada 500 ms */
#define VENTANA_VEL_CICLOS 30       /* velocidad = flancos de los ultimos 30 x 10 ms = 300 ms */
#define PI_CICLOS 5                 /* el PI corre cada 5 x 10 ms = 50 ms */
#define KP 0.0020f                  /* duty por cada flanco/s de error */
#define KI 0.0030f                  /* duty por cada flanco/s de error y por segundo */
#define INTEGRAL_MAX 0.40f
#define PASO_DUTY_PI 0.15f
#define SIN_GIRO_PI 6
#define MAX_INTENTOS_KICK 6

#define PERIODO_TICKS_US 50000      /* /wheel_ticks a 20 Hz */
#define PERIODO_ESTADO_US 100000    /* /wheel_state a 10 Hz */

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
	int polaridad;                   /* +1: IA adelante | -1: la rueda gira al reves con IA (resultado de la prueba de cableado) */
	const punto_t *curva;
	int n_curva;
	punto_t curva_cal[MAX_PUNTOS];   /* curva cargada desde la PC (id 6) */
	volatile float d_min;            /* duty minimo que sostiene el giro (medido) */
	int nivel_kick_ini;
	pcnt_unit_handle_t unidad;
	/* compartidas con las tareas de micro-ROS (32 bits: lectura atomica) */
	volatile float objetivo_cmd;     /* flancos/s con signo pedidos por la orden (+ es avanzar) */
	volatile float objetivo;         /* el que usa el control, ya escalonado */
	volatile long ticks;             /* flancos acumulados con signo */
	volatile float v_med;            /* flancos/s con signo */
	volatile float duty_sal;         /* duty con signo que sale al motor */
	volatile bool bloqueado;
	volatile bool modo_duty;         /* lazo abierto: duty_directo sale tal cual */
	volatile float duty_directo;
	volatile long total;             /* flancos acumulados sin signo */
	/* estado del control */
	float duty, integral;
	int signo_ult;                   /* sentido del ultimo duty distinto de 0 */
	int kick, nivel_kick, intentos, sin_giro, ciclos_pi, gracia, espera;
	int ventana[VENTANA_VEL_CICLOS], idx, suma, cuenta_ant;
} rueda_t;

/* Polaridad: dejala en +1 hasta tener el sentido real de cada rueda */
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

#define TIMEOUT_CREAR_MS 5000       /* espera por entidad al crear (el default es 1000: una rafaga de Wi-Fi la vence) */
#define PING_ESPERA_MS 200
#define PING_CONECTADO_MS 300
#define PING_FALLOS_MAX 6

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_robot_cmd;
static rcl_publisher_t pub_ticks;
static rcl_publisher_t pub_estado;
static geometry_msgs__msg__Twist recv_twist;
static std_msgs__msg__Float32MultiArray recv_robot;
static std_msgs__msg__Int32MultiArray msg_ticks;
static std_msgs__msg__Float32MultiArray msg_estado;
static float recv_robot_buf[24];
static int32_t ticks_buf[N_RUEDAS];
static float estado_buf[24];

static volatile uint32_t ultimo_cmd_ms = 0;
static volatile bool agente_conectado = false;

static volatile float diametro_m = DIAMETRO_RUEDA_M;
static volatile float trocha_m = TROCHA_M;
static volatile float frenado_m = FRENADO_M;
static volatile int cfg_seq = 0;                 /* configuraciones y curvas aceptadas */

/* Movimiento de distancia: lo pide el callback y lo ejecuta la tarea de control */
static volatile int mov_pedido = 0;
static volatile int mov_abortar = 0;
static volatile float mov_dist_ped_m = 0.0f;
static volatile float mov_vel_ped = 0.0f;
static volatile int mov_activo = 0;
static volatile float mov_recorrido_m = 0.0f;
static volatile float mov_final_m = 0.0f;
static volatile int mov_seq = 0;

static float flancos_por_m(void)
{
	return FLANCOS_POR_VUELTA / (3.14159265f * diametro_m);
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
	/* el encoder de un solo canal no da el sentido: se toma el del ultimo duty distinto de 0 */
	w->ticks += w->signo_ult * delta;
	w->total += delta;
	w->v_med = w->signo_ult * w->suma / (VENTANA_VEL_CICLOS * 0.010f);

	if (w->modo_duty) {
		reiniciar_estado(w);                /* calibracion: el duty sale directo, sin PI */
		w->duty = w->duty_directo;
	} else if (objetivo == 0.0f) {
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
		float v_abs = w->suma / (VENTANA_VEL_CICLOS * 0.010f);

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
			float err = vref - v_abs;
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

	if (w->duty != 0.0f) {
		w->signo_ult = (w->duty > 0.0f) ? 1 : -1;
	}
	w->duty_sal = w->duty;
	motor_aplicar(w, w->duty);
}

static float flancos_recorridos(const long base[N_RUEDAS])
{
	float suma = 0.0f;

	for (int i = 0; i < N_RUEDAS; i++) {
		suma += (float)(W[i].total - base[i]);
	}
	return suma / N_RUEDAS;
}

static void ordenar_todas(float flancos)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		W[i].objetivo_cmd = flancos;
	}
}

/* Un ciclo de 10 ms de todo el control: movimiento de distancia, ordenes y las 4 ruedas */
static void control_paso(void)
{
	static int ciclo = 0;
	static int mov_fase = 0;             /* 0 libre, 1 andando, 2 asentando */
	static int mov_dir = 1;
	static float mov_obj_fl = 0.0f;
	static float mov_vel = 0.0f;
	static int mov_ciclos = 0;
	static long mov_base[N_RUEDAS] = {0};

	{
		uint32_t ahora_ms = (uint32_t)(esp_timer_get_time() / 1000);

		/* ---- movimiento de distancia ---- */
		if (mov_abortar) {
			mov_abortar = 0;
			mov_pedido = 0;
			mov_fase = 0;
			mov_activo = 0;
			ordenar_todas(0.0f);
		}
		if (mov_pedido && mov_fase == 0) {
			float dist = mov_dist_ped_m;
			float vel = mov_vel_ped > 0.0f ? mov_vel_ped : VEL_MOV_DEFECTO;

			mov_pedido = 0;
			mov_dir = dist >= 0.0f ? 1 : -1;
			mov_obj_fl = fabsf(dist) * flancos_por_m();
			mov_vel = fminf(VEL_MAX_FLANCOS, fmaxf(VEL_MIN_FLANCOS, vel));
			for (int i = 0; i < N_RUEDAS; i++) {
				mov_base[i] = W[i].total;
				W[i].modo_duty = false;
			}
			mov_ciclos = 0;
			mov_fase = 1;
			mov_activo = 1;
			printf("mover %.3f m = %.1f flancos a %.0f flancos/s\n", dist, mov_obj_fl, mov_vel);
		}
		if (mov_fase == 1) {
			float rec = flancos_recorridos(mov_base);
			float resta = mov_obj_fl - frenado_m * flancos_por_m() - rec;

			ultimo_cmd_ms = ahora_ms;
			mov_recorrido_m = mov_dir * rec / flancos_por_m();
			if (resta <= 0.0f || ++mov_ciclos > MOV_MAX_CICLOS) {
				ordenar_todas(0.0f);
				mov_fase = 2;
				mov_ciclos = 0;
			} else {
				float v_acc = VEL_MIN_FLANCOS + ACEL_FLANCOS_S2 * mov_ciclos * 0.010f;
				float v_dec = sqrtf(2.0f * ACEL_FLANCOS_S2 * resta);
				float v = fmaxf(VEL_MIN_FLANCOS, fminf(mov_vel, fminf(v_acc, v_dec)));

				ordenar_todas(mov_dir * v);
			}
		} else if (mov_fase == 2) {
			ultimo_cmd_ms = ahora_ms;
			ordenar_todas(0.0f);
			if (++mov_ciclos >= MOV_ESPERA_CICLOS) {
				mov_final_m = mov_dir * flancos_recorridos(mov_base) / flancos_por_m();
				mov_recorrido_m = mov_final_m;
				mov_seq++;
				mov_fase = 0;
				mov_activo = 0;
				printf("movimiento terminado: %.3f m calculados\n", mov_final_m);
			}
		}

		bool vencido = (uint32_t)(ahora_ms - ultimo_cmd_ms) > TIMEOUT_CMD_MS;

		for (int i = 0; i < N_RUEDAS; i++) {
			float cmd = vencido ? 0.0f : W[i].objetivo_cmd;

			if (vencido && W[i].modo_duty) {
				W[i].modo_duty = false;
				W[i].duty_directo = 0.0f;
			}
			if (W[i].modo_duty) {
				cmd = 0.0f;
			}
			if (cmd == 0.0f) {
				if (W[i].objetivo != 0.0f && vencido) {
					printf("sin orden hace mas de %d ms: FRENAR\n", TIMEOUT_CMD_MS);
				}
				W[i].objetivo = 0.0f;
				W[i].espera = 0;
			} else if (W[i].objetivo == 0.0f) {
				if (++W[i].espera > i * ESCALONADO_CICLOS) {   /* arranque escalonado: no hundir el 5 V */
					W[i].objetivo = cmd;
				}
			} else {
				W[i].objetivo = cmd;
			}
			rueda_ciclo(&W[i]);
		}

		if (++ciclo >= VENTANA_INFORME_CICLOS) {
			ciclo = 0;
			if (W[0].duty != 0.0f || W[1].duty != 0.0f || W[2].duty != 0.0f || W[3].duty != 0.0f) {
				printf("v: FL=%.0f FR=%.0f RL=%.0f RR=%.0f | ref %.0f %.0f %.0f %.0f | duty %.2f %.2f %.2f %.2f\n",
				       W[0].v_med, W[1].v_med, W[2].v_med, W[3].v_med, W[0].objetivo, W[1].objetivo, W[2].objetivo,
				       W[3].objetivo, W[0].duty, W[1].duty, W[2].duty, W[3].duty);
			}
		}
	}
}

/* Tarea de control a 100 Hz, independiente de micro-ROS: el freno no depende de que el enlace este vivo. */
static void control_task(void *arg)
{
	TickType_t ultimo = xTaskGetTickCount();

	(void)arg;
	for (int i = 0; i < N_RUEDAS; i++) {
		reiniciar_estado(&W[i]);
		W[i].signo_ult = 1;
	}
	while (1) {
		control_paso();
		vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(10));
	}
}

/* m/s de una rueda -> flancos/s, con zona muerta bajo la mitad del minimo y tope por rueda */
static float ms_a_flancos(float v)
{
	float f = v * flancos_por_m();

	if (fabsf(f) < VEL_MIN_FLANCOS * 0.5f) {
		return 0.0f;
	}
	return fminf(VEL_MAX_FLANCOS, fmaxf(-VEL_MAX_FLANCOS, f));
}

static void callback_cmd_vel(const void *msgin)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
	float vl = (float)msg->linear.x - (float)msg->angular.z * trocha_m / 2.0f;
	float vr = (float)msg->linear.x + (float)msg->angular.z * trocha_m / 2.0f;
	float mayor = fmaxf(fabsf(vl), fabsf(vr)) * flancos_por_m();

	if (mayor > VEL_MAX_FLANCOS) {                 /* mantener la curvatura: escalar las dos ruedas */
		vl *= VEL_MAX_FLANCOS / mayor;
		vr *= VEL_MAX_FLANCOS / mayor;
	}
	if (mov_activo) {
		return;                                    /* un movimiento de distancia en curso manda sobre /cmd_vel */
	}
	for (int i = 0; i < N_RUEDAS; i++) {
		W[i].modo_duty = false;
	}
	W[0].objetivo_cmd = W[2].objetivo_cmd = ms_a_flancos(vl);
	W[1].objetivo_cmd = W[3].objetivo_cmd = ms_a_flancos(vr);
	ultimo_cmd_ms = (uint32_t)(esp_timer_get_time() / 1000);
	gpio_set_level(PIN_LED, (vl != 0.0f || vr != 0.0f) ? 1 : 0);
}

static void callback_robot_cmd(const void *msgin)
{
	const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
	const float *d = msg->data.data;
	int n = (int)msg->data.size;

	if (n < 1) {
		return;
	}
	switch ((int)d[0]) {
	case 1:                                        /* duty directo (calibracion) */
		if (n >= 1 + N_RUEDAS && !mov_activo) {
			for (int i = 0; i < N_RUEDAS; i++) {
				float v = fminf(1.0f, fmaxf(-1.0f, d[1 + i]));

				W[i].objetivo_cmd = 0.0f;
				W[i].duty_directo = v;
				W[i].modo_duty = (v != 0.0f);
			}
			ultimo_cmd_ms = (uint32_t)(esp_timer_get_time() / 1000);
		}
		break;
	case 2:                                        /* velocidad por rueda, m/s */
		if (n >= 1 + N_RUEDAS && !mov_activo) {
			for (int i = 0; i < N_RUEDAS; i++) {
				W[i].modo_duty = false;
				W[i].objetivo_cmd = ms_a_flancos(d[1 + i]);
			}
			ultimo_cmd_ms = (uint32_t)(esp_timer_get_time() / 1000);
		}
		break;
	case 3:                                        /* mover una distancia */
		if (n >= 2 && !mov_activo && !mov_pedido) {
			mov_dist_ped_m = d[1];
			mov_vel_ped = n >= 3 ? d[2] : 0.0f;
			mov_pedido = 1;
		}
		break;
	case 4:                                        /* parar todo */
		mov_abortar = 1;
		for (int i = 0; i < N_RUEDAS; i++) {
			W[i].modo_duty = false;
			W[i].duty_directo = 0.0f;
			W[i].objetivo_cmd = 0.0f;
		}
		break;
	case 5:                                        /* configuracion en metros */
		if (n >= 4) {
			if (d[1] > 0.03f && d[1] < 0.15f) {
				diametro_m = d[1];
			}
			if (d[2] > 0.05f && d[2] < 0.5f) {
				trocha_m = d[2];
			}
			if (d[3] >= 0.0f && d[3] < 0.1f) {
				frenado_m = d[3];
			}
			cfg_seq++;
		}
		break;
	case 6:                                        /* curva de prealimentacion de una rueda */
		if (n >= 4) {
			int r = (int)d[1];
			int np = (int)d[3];

			if (r >= 0 && r < N_RUEDAS && np >= 1 && np <= MAX_PUNTOS && n >= 4 + 2 * np) {
				for (int k = 0; k < np; k++) {
					W[r].curva_cal[k].v = d[4 + 2 * k];
					W[r].curva_cal[k].duty = fminf(1.0f, fmaxf(0.0f, d[5 + 2 * k]));
				}
				W[r].d_min = fminf(1.0f, fmaxf(0.0f, d[2]));
				W[r].n_curva = np;
				W[r].curva = W[r].curva_cal;
				cfg_seq++;
			}
		}
		break;
	default:
		break;
	}
}

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;

static bool crear_entidades(void)
{
	allocator = rcl_get_default_allocator();

	if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
		return false;
	}
	/* Esperar mas que el default de 1 s por entidad: una rafaga de Wi-Fi dura 1,5-3,5 s */
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_creation_session_timeout(rmw_context, TIMEOUT_CREAR_MS);
	}
	if (rclc_node_init_default(&node, "esp32_motion", "", &support) != RCL_RET_OK) {
		return false;
	}
	if (rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	                                       "cmd_vel") != RCL_RET_OK) {
		return false;
	}
	if (rclc_subscription_init_best_effort(&sub_robot_cmd, &node,
	                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                       "robot_cmd") != RCL_RET_OK) {
		return false;
	}
	if (rclc_publisher_init_best_effort(&pub_ticks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	                                    "wheel_ticks") != RCL_RET_OK) {
		return false;
	}
	if (rclc_publisher_init_best_effort(&pub_estado, &node,
	                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                    "wheel_state") != RCL_RET_OK) {
		return false;
	}
	executor = rclc_executor_get_zero_initialized_executor();
	if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
		return false;
	}
	if (rclc_executor_add_subscription(&executor, &sub_cmd_vel, &recv_twist, &callback_cmd_vel, ON_NEW_DATA) != RCL_RET_OK) {
		return false;
	}
	if (rclc_executor_add_subscription(&executor, &sub_robot_cmd, &recv_robot, &callback_robot_cmd, ON_NEW_DATA) != RCL_RET_OK) {
		return false;
	}
	return true;
}

static void destruir_entidades(void)
{
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		/* Sin esperar: si el agente no contesta, no bloquear la tarea */
		(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	}
	(void)rcl_publisher_fini(&pub_ticks, &node);
	(void)rcl_publisher_fini(&pub_estado, &node);
	(void)rcl_subscription_fini(&sub_cmd_vel, &node);
	(void)rcl_subscription_fini(&sub_robot_cmd, &node);
	(void)rclc_executor_fini(&executor);
	(void)rcl_node_fini(&node);
	(void)rclc_support_fini(&support);
}

static void publicar_ticks(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		ticks_buf[i] = (int32_t)W[i].ticks;
	}
	(void)rcl_publish(&pub_ticks, &msg_ticks, NULL);
}

static void publicar_estado(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		estado_buf[i] = W[i].v_med;
		estado_buf[N_RUEDAS + i] = W[i].objetivo;
		estado_buf[2 * N_RUEDAS + i] = W[i].duty_sal;
		estado_buf[3 * N_RUEDAS + i] = W[i].bloqueado ? 1.0f : 0.0f;
	}
	estado_buf[16] = (float)mov_activo;
	estado_buf[17] = mov_recorrido_m;
	estado_buf[18] = mov_final_m;
	estado_buf[19] = (float)mov_seq;
	estado_buf[20] = diametro_m * 1000.0f;
	estado_buf[21] = frenado_m * 1000.0f;
	estado_buf[22] = (float)cfg_seq;
	estado_buf[23] = trocha_m * 1000.0f;
	(void)rcl_publish(&pub_estado, &msg_estado, NULL);
}

/* Esperando agente -> conectado -> (agente perdido) -> esperando agente. Nunca se rinde. */
void micro_ros_task(void *arg)
{
	(void)arg;
	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
		printf("uros: no se pudieron crear las opciones\n");
		vTaskDelete(NULL);
	}
	rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options) != RCL_RET_OK) {
		printf("uros: direccion del agente invalida\n");
		vTaskDelete(NULL);
	}

	/* Clave de cliente fija (de la MAC): tras un reinicio el agente reutiliza la sesion en vez de dejar una fantasma */
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
	uint32_t clave = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
	if (rmw_uros_options_set_client_key(clave, rmw_options) != RMW_RET_OK) {
		printf("uros: clave de cliente invalida\n");
		vTaskDelete(NULL);
	}
	printf("uros: clave de cliente 0x%08X\n", (unsigned)clave);

	bool conectado = false;
	int fallos_ping = 0;
	int64_t ultimo_ping_us = 0;
	int64_t ultimo_ticks_us = 0;
	int64_t ultimo_estado_us = 0;

	while (1) {
		if (!conectado) {
			gpio_set_level(PIN_LED, (esp_timer_get_time() / 250000) % 2);   /* parpadeo: esperando al agente */
			if (rmw_uros_ping_agent_options(PING_ESPERA_MS, 1, rmw_options) == RMW_RET_OK) {
				printf("uros: agente visible, creando entidades\n");
				if (crear_entidades()) {
					conectado = true;
					agente_conectado = true;
					fallos_ping = 0;
					ultimo_ping_us = esp_timer_get_time();
					gpio_set_level(PIN_LED, 0);
					printf("uros: conectado. Escucha /cmd_vel y /robot_cmd; publica /wheel_ticks y /wheel_state\n");
				} else {
					printf("uros: fallo al crear entidades, reintento\n");
					destruir_entidades();
				}
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		} else {
			int64_t ahora_us = esp_timer_get_time();

			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
			if (ahora_us - ultimo_ticks_us >= PERIODO_TICKS_US) {
				ultimo_ticks_us = ahora_us;
				publicar_ticks();
			}
			if (ahora_us - ultimo_estado_us >= PERIODO_ESTADO_US) {
				ultimo_estado_us = ahora_us;
				publicar_estado();
			}
			if (ahora_us - ultimo_ping_us > 1000000) {
				ultimo_ping_us = ahora_us;
				if (rmw_uros_ping_agent_options(PING_CONECTADO_MS, 1, rmw_options) == RMW_RET_OK) {
					fallos_ping = 0;
				} else if (++fallos_ping >= PING_FALLOS_MAX) {
					printf("uros: agente perdido, reconectando\n");
					destruir_entidades();
					conectado = false;
					agente_conectado = false;
				}
			}
			usleep(10000);
		}
	}
}

void app_main(void)
{
	/* Lo primero: motores en 0/0 */
	pwm_init();
	for (int i = 0; i < N_RUEDAS; i++) {
		motor_aplicar(&W[i], 0.0f);
	}
	encoders_init();

	gpio_reset_pin(PIN_LED);
	gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_LED, 0);

	/* Mensajes con memoria estatica: los arreglos de los multiarray apuntan a buffers propios */
	std_msgs__msg__Int32MultiArray__init(&msg_ticks);
	msg_ticks.data.data = ticks_buf;
	msg_ticks.data.size = N_RUEDAS;
	msg_ticks.data.capacity = N_RUEDAS;
	std_msgs__msg__Float32MultiArray__init(&msg_estado);
	msg_estado.data.data = estado_buf;
	msg_estado.data.size = 24;
	msg_estado.data.capacity = 24;
	std_msgs__msg__Float32MultiArray__init(&recv_robot);
	recv_robot.data.data = recv_robot_buf;
	recv_robot.data.size = 0;
	recv_robot.data.capacity = sizeof(recv_robot_buf) / sizeof(recv_robot_buf[0]);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN
	/* Sin ahorro de energia del Wi-Fi: la ESP32 no duerme entre balizas (menor latencia) */
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif

	xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
	xTaskCreate(micro_ros_task,
			"uros_task",
			CONFIG_MICRO_ROS_APP_STACK,
			NULL,
			CONFIG_MICRO_ROS_APP_TASK_PRIO,
			NULL);
}
