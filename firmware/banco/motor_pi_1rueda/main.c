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
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

/* Pines: mapa A, lado derecho. Prueba con UN motor en el canal B del L9110S derecho. */
#define PIN_IA GPIO_NUM_27          /* B-IA: adelante (PWM) */
#define PIN_IB GPIO_NUM_16          /* B-IB: atras (PWM)    */
#define PIN_LED GPIO_NUM_2
#define PIN_ENC GPIO_NUM_36         /* ENC_FR (mapa A): D0 del encoder de la delantera derecha */

#define PWM_HZ 5000
#define PWM_MAX 1023                /* 10 bits */
#define CANAL_IA LEDC_CHANNEL_0
#define CANAL_IB LEDC_CHANNEL_1

#define MOTOR_TRASERO 0             /* 0: motor delantero (1:48, rapido) | 1: motor trasero (mas lento y duro) */

#define V_MAX 0.5f                  /* linear.x (m/s) que corresponde a VEL_MAX_FLANCOS */
#define VEL_MAX_FLANCOS 75.0f       /* flancos/s a V_MAX (112 rpm): lo que alcanzan los 4 motores, con margen */
#define VEL_MIN_FLANCOS 25.0f       /* menor velocidad que sostienen los 4 motores (medido: RR 24,8; FR en caliente ~25 a duty 0,36) */
#define TIMEOUT_CMD_MS 1000         /* sin /cmd_vel: frenar (banco; en la competencia lo cubre el freno local) */
#define FLANCOS_POR_VUELTA 40       /* 20 ranuras x 2 flancos */
#define VENTANA_CICLOS 50           /* informe por el puerto serie cada 50 x 10 ms = 500 ms */
#define VENTANA_VEL_CICLOS 30       /* velocidad = flancos de los ultimos 30 x 10 ms = 300 ms */
#define PI_CICLOS 5                 /* el PI corre cada 5 x 10 ms = 50 ms */
#define KP 0.0020f                  /* duty por cada flanco/s de error */
#define KI 0.0030f                  /* duty por cada flanco/s de error y por segundo */
#define INTEGRAL_MAX 0.40f          /* tope del termino integral (anti-windup) */
#define PASO_DUTY_PI 0.15f          /* maximo cambio de duty por actualizacion del PI */
#define SIN_GIRO_PI 6               /* 6 x 50 ms = 300 ms sin flancos con orden activa: nuevo empujon */
#define MAX_INTENTOS_KICK 6         /* empujones seguidos sin ningun flanco: encoder o rueda trabada, detener el motor */

/* Curva medida (sin carga): flancos/s -> duty. Es solo la prealimentacion: el PI corrige el resto. */
typedef struct { float v; float duty; } punto_t;
#if MOTOR_TRASERO
static const punto_t CURVA[] = {
	{0.0f, 0.36f}, {13.2f, 0.40f}, {26.8f, 0.50f}, {31.5f, 0.60f}, {38.6f, 0.70f},
	{45.4f, 0.75f}, {54.2f, 0.80f}, {65.9f, 0.85f}, {77.9f, 0.90f}, {91.6f, 1.00f},
};
#define NIVEL_KICK_INICIAL 2        /* los traseros necesitan duty 1,0: empezar por un empujon fuerte */
#else
static const punto_t CURVA[] = {
	{0.0f, 0.34f}, {16.8f, 0.36f}, {33.3f, 0.40f}, {69.1f, 0.50f}, {104.6f, 0.60f},
};
#define NIVEL_KICK_INICIAL 0
#endif
#define N_CURVA ((int)(sizeof(CURVA) / sizeof(CURVA[0])))

/* Empujon progresivo para vencer la friccion estatica: si el motor no arranca, el siguiente es mas fuerte y mas largo */
static const float KICK_DUTY_NIVEL[] = {0.70f, 0.85f, 1.00f, 1.00f};
static const int KICK_CICLOS_NIVEL[] = {10, 15, 25, 40};     /* x 10 ms */
#define N_NIVELES_KICK 4

#define TIMEOUT_CREAR_MS 5000       /* espera por entidad al crear (el default es 1000: una rafaga de Wi-Fi la vence) */
#define PING_ESPERA_MS 200          /* ping al agente mientras se espera que aparezca */
#define PING_CONECTADO_MS 300       /* ping al agente (1 por segundo) estando conectado */
#define PING_FALLOS_MAX 6           /* pings fallidos seguidos (~6 s) para dar el agente por perdido */

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

static rcl_subscription_t subscriber;
static geometry_msgs__msg__Twist recv_msg;

/* Compartidas entre la tarea de micro-ROS y la de control (32 bits: lectura atomica) */
static volatile float objetivo_vel = 0.0f;       /* flancos/s con signo */
static volatile uint32_t ultimo_cmd_ms = 0;

static pcnt_unit_handle_t unidad_enc = NULL;

static void encoder_init(void)
{
	pcnt_unit_config_t unidad_cfg = {
		.low_limit = -32768,
		.high_limit = 32767,
	};
	ESP_ERROR_CHECK(pcnt_new_unit(&unidad_cfg, &unidad_enc));

	pcnt_glitch_filter_config_t filtro_cfg = {
		.max_glitch_ns = 5000,
	};
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unidad_enc, &filtro_cfg));

	pcnt_chan_config_t canal_cfg = {
		.edge_gpio_num = PIN_ENC,
		.level_gpio_num = -1,
	};
	pcnt_channel_handle_t canal = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(unidad_enc, &canal_cfg, &canal));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(canal,
		PCNT_CHANNEL_EDGE_ACTION_INCREASE,
		PCNT_CHANNEL_EDGE_ACTION_INCREASE));

	ESP_ERROR_CHECK(pcnt_unit_enable(unidad_enc));
	ESP_ERROR_CHECK(pcnt_unit_clear_count(unidad_enc));
	ESP_ERROR_CHECK(pcnt_unit_start(unidad_enc));
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

	ledc_channel_config_t canal = {
		.gpio_num = PIN_IA,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel = CANAL_IA,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&canal));
	canal.gpio_num = PIN_IB;
	canal.channel = CANAL_IB;
	ESP_ERROR_CHECK(ledc_channel_config(&canal));
}

/* duty > 0: adelante (IA = PWM, IB = 0). duty < 0: atras. 0: las dos en 0. La senal fija queda siempre en 0. */
static void motor_aplicar(float duty)
{
	uint32_t d = (uint32_t)(fabsf(duty) * PWM_MAX + 0.5f);

	if (duty > 0.0f) {
		pwm_set(CANAL_IA, d);
		pwm_set(CANAL_IB, 0);
	} else if (duty < 0.0f) {
		pwm_set(CANAL_IA, 0);
		pwm_set(CANAL_IB, d);
	} else {
		pwm_set(CANAL_IA, 0);
		pwm_set(CANAL_IB, 0);
	}
}

/* Prealimentacion: duty que dio esa velocidad en la curva medida (interpolacion lineal) */
static float duty_ff(float v)
{
	if (v <= CURVA[0].v) {
		return CURVA[0].duty;
	}
	for (int i = 1; i < N_CURVA; i++) {
		if (v <= CURVA[i].v) {
			return CURVA[i - 1].duty + (CURVA[i].duty - CURVA[i - 1].duty) *
			       (v - CURVA[i - 1].v) / (CURVA[i].v - CURVA[i - 1].v);
		}
	}
	return CURVA[N_CURVA - 1].duty;
}

/* Tarea de control a 100 Hz, independiente de micro-ROS: el freno no depende de que el enlace este vivo. */
static void control_task(void * arg)
{
	TickType_t ultimo = xTaskGetTickCount();
	float duty = 0.0f;          /* con signo: lo que sale al motor */
	float integral = 0.0f;
	int kick = 0;
	int nivel_kick = NIVEL_KICK_INICIAL;
	int intentos = 0;           /* empujones seguidos sin que la rueda gire */
	bool bloqueado = false;     /* falla de encoder o rueda trabada: motor detenido hasta que llegue orden 0 */
	int sin_giro = 0;
	int ciclo = 0;
	int ciclos_pi = 0;
	int gracia = 0;             /* actualizaciones del PI que se saltan tras un empujon */
	int ventana[VENTANA_VEL_CICLOS] = {0};
	int idx = 0;
	int suma = 0;               /* flancos de los ultimos 300 ms */
	int acum_informe = 0;       /* flancos desde el ultimo informe */
	int cuenta_ant = 0;

	(void) arg;
	while (1) {
		float objetivo = objetivo_vel;
		uint32_t ahora_ms = (uint32_t)(esp_timer_get_time() / 1000);
		int cuenta = 0;
		int delta;

		/* flancos de este ciclo (el contador corre libre; se limpia de vez en cuando) */
		pcnt_unit_get_count(unidad_enc, &cuenta);
		delta = cuenta - cuenta_ant;
		if (delta < 0) {
			delta = 0;
		}
		cuenta_ant = cuenta;
		if (cuenta > 20000) {
			pcnt_unit_clear_count(unidad_enc);
			cuenta_ant = 0;
		}
		suma += delta - ventana[idx];
		ventana[idx] = delta;
		idx = (idx + 1) % VENTANA_VEL_CICLOS;
		acum_informe += delta;

		if ((uint32_t)(ahora_ms - ultimo_cmd_ms) > TIMEOUT_CMD_MS) {
			if (duty != 0.0f || objetivo != 0.0f) {
				printf("sin /cmd_vel hace mas de %d ms: FRENAR\n", TIMEOUT_CMD_MS);
			}
			objetivo_vel = 0.0f;
			duty = 0.0f;
			integral = 0.0f;
			kick = 0;
			nivel_kick = NIVEL_KICK_INICIAL;
			intentos = 0;
			bloqueado = false;
			sin_giro = 0;
		} else if (objetivo == 0.0f) {
			duty = 0.0f;
			integral = 0.0f;
			kick = 0;
			nivel_kick = NIVEL_KICK_INICIAL;
			intentos = 0;
			bloqueado = false;
			sin_giro = 0;
		} else if (duty != 0.0f && (duty > 0.0f) != (objetivo > 0.0f)) {
			duty = 0.0f;            /* inversion: pasar por 0/0 un ciclo */
			integral = 0.0f;
			kick = 0;
			nivel_kick = NIVEL_KICK_INICIAL;
			intentos = 0;
			bloqueado = false;
			sin_giro = 0;
		} else if (bloqueado) {
			duty = 0.0f;            /* sin flancos pese a los empujones: no seguir a ciegas */
			kick = 0;
		} else if (kick > 0) {
			kick--;                 /* durante el empujon el duty queda fijo */
			if (kick == 0) {
				gracia = VENTANA_VEL_CICLOS / PI_CICLOS;
			}
		} else if (++ciclos_pi >= PI_CICLOS) {
			float signo = (objetivo > 0.0f) ? 1.0f : -1.0f;
			float mag = fabsf(duty);
			float vref = fmaxf(fabsf(objetivo), VEL_MIN_FLANCOS);
			float v_med = suma / (VENTANA_VEL_CICLOS * 0.010f);

			ciclos_pi = 0;
			sin_giro = (suma == 0) ? sin_giro + 1 : 0;
			if (suma > 0) {
				nivel_kick = NIVEL_KICK_INICIAL;
				intentos = 0;
			}
			if ((mag == 0.0f && suma == 0) || sin_giro >= SIN_GIRO_PI) {
				/* parado con orden activa: empujon */
				if (++intentos > MAX_INTENTOS_KICK) {
					bloqueado = true;
					duty = 0.0f;
					printf("ERROR: sin flancos tras %d empujones, motor detenido (encoder o rueda trabada)\n",
					       MAX_INTENTOS_KICK);
				} else {
					kick = KICK_CICLOS_NIVEL[nivel_kick];
					duty = signo * KICK_DUTY_NIVEL[nivel_kick];
					if (nivel_kick < N_NIVELES_KICK - 1) {
						nivel_kick++;
					}
				}
				sin_giro = 0;
			} else {
				float err = vref - v_med;
				float ff = duty_ff(vref);
				float d_min = CURVA[1].duty;    /* duty minimo que sostiene el giro */
				float d;

				if (gracia > 0) {
					/* la ventana de 300 ms todavia contiene el empujon: mantener la prealimentacion */
					gracia--;
					d = ff + integral;
				} else {
					float integral_nueva = integral + KI * err * (PI_CICLOS * 0.010f);

					d = ff + KP * err + integral_nueva;
					/* anti-windup: no integrar si el duty ya esta saturado hacia el mismo lado */
					if (!((d >= 1.0f && err > 0.0f) || (d <= d_min && err < 0.0f))) {
						integral = fminf(INTEGRAL_MAX, fmaxf(-INTEGRAL_MAX, integral_nueva));
					}
					d = ff + KP * err + integral;
				}
				d = fminf(1.0f, fmaxf(d_min, d));
				d = fminf(mag + PASO_DUTY_PI, fmaxf(mag - PASO_DUTY_PI, d));
				duty = signo * d;
			}
		}

		motor_aplicar(duty);

		if (++ciclo >= VENTANA_CICLOS) {
			float flancos_s = acum_informe / (VENTANA_CICLOS * 0.010f);

			ciclo = 0;
			if (duty != 0.0f || acum_informe != 0) {
				printf("v: duty=%.2f flancos/s=%.0f rpm=%.1f ref=%.0f\n", duty, flancos_s,
				       flancos_s * 60.0f / FLANCOS_POR_VUELTA, fabsf(objetivo_vel));
			}
			acum_informe = 0;
		}
		vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(10));
	}
}

static void subscription_callback(const void * msgin)
{
	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
	uint32_t ahora_ms = (uint32_t)(esp_timer_get_time() / 1000);
	uint32_t hueco_ms = ahora_ms - ultimo_cmd_ms;
	float norm = (float)msg->linear.x / V_MAX;

	if (ultimo_cmd_ms != 0 && hueco_ms > 200 && hueco_ms < 5000) {
		printf("HUECO de %u ms entre ordenes\n", (unsigned)hueco_ms);
	}
	if (norm > 1.0f) {
		norm = 1.0f;
	} else if (norm < -1.0f) {
		norm = -1.0f;
	}
	objetivo_vel = norm * VEL_MAX_FLANCOS;
	ultimo_cmd_ms = ahora_ms;
	gpio_set_level(PIN_LED, norm != 0.0f ? 1 : 0);
}

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t * rmw_options;

static bool crear_entidades(void)
{
	allocator = rcl_get_default_allocator();

	if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
		return false;
	}
	/* Esperar mas que el default de 1 s por entidad: una rafaga de Wi-Fi dura 1,5-3,5 s */
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_creation_session_timeout(rmw_context, TIMEOUT_CREAR_MS);
	}
	if (rclc_node_init_default(&node, "esp32_motion", "", &support) != RCL_RET_OK) {
		return false;
	}
	if (rclc_subscription_init_best_effort(
			&subscriber,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"cmd_vel") != RCL_RET_OK) {
		return false;
	}
	executor = rclc_executor_get_zero_initialized_executor();
	if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
		return false;
	}
	if (rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
		return false;
	}
	return true;
}

static void destruir_entidades(void)
{
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		/* Sin esperar: si el agente no contesta, no bloquear la tarea */
		(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	}
	(void)rcl_subscription_fini(&subscriber, &node);
	(void)rclc_executor_fini(&executor);
	(void)rcl_node_fini(&node);
	(void)rclc_support_fini(&support);
}

/* Esperando agente -> conectado -> (agente perdido) -> esperando agente. Nunca se rinde. */
void micro_ros_task(void * arg)
{
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

	while (1) {
		if (!conectado) {
			if (rmw_uros_ping_agent_options(PING_ESPERA_MS, 1, rmw_options) == RMW_RET_OK) {
				printf("uros: agente visible, creando entidades\n");
				if (crear_entidades()) {
					conectado = true;
					fallos_ping = 0;
					ultimo_ping_us = esp_timer_get_time();
					printf("uros: conectado, escuchando /cmd_vel\n");
				} else {
					printf("uros: fallo al crear entidades, reintento\n");
					destruir_entidades();
				}
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		} else {
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			if (esp_timer_get_time() - ultimo_ping_us > 1000000) {
				ultimo_ping_us = esp_timer_get_time();
				if (rmw_uros_ping_agent_options(PING_CONECTADO_MS, 1, rmw_options) == RMW_RET_OK) {
					fallos_ping = 0;
				} else if (++fallos_ping >= PING_FALLOS_MAX) {
					printf("uros: agente perdido, reconectando\n");
					destruir_entidades();
					conectado = false;
				}
			}
			usleep(10000);
		}
	}
}

void app_main(void)
{
	/* Lo primero: motor en 0/0 */
	pwm_init();
	motor_aplicar(0.0f);
	encoder_init();

	gpio_reset_pin(PIN_LED);
	gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_LED, 0);

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
