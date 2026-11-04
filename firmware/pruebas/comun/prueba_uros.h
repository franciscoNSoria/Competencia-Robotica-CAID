/* Base comun de las pruebas por componente: conexion inalambrica por micro-ROS, igual que el dia de la competencia.
 *
 * La ESP32 se conecta por Wi-Fi al agente micro-ROS de la PC (red, clave e IP del agente en idf.py menuconfig, como en
 * firmware/banco/README.md) y usa dos topicos:
 *   /<PRUEBA_TOPICO>/cmd     std_msgs/Float32MultiArray  PC -> ESP32  [id_prueba, orden, argumentos...]
 *   /<PRUEBA_TOPICO>/estado  std_msgs/Float32MultiArray  ESP32 -> PC  [id_prueba, contador, datos de la prueba...]
 * El id de la prueba evita mandarle ordenes de una prueba al firmware de otra: si no coincide, la orden se ignora.
 *
 * Cada prueba (un .c en firmware/pruebas/) define PRUEBA_ID y PRUEBA_NOMBRE, incluye este archivo y escribe:
 *   static void prueba_init(void)                      hardware (se llama antes de conectar)
 *   static void prueba_ciclo(void)                     cada 10 ms, en su propia tarea (no depende del enlace)
 *   static void prueba_comando(const float *a, int n)  a[0] = orden, a[1..] = argumentos
 *   static int  prueba_estado(float *e)                completa e[2..] y devuelve cuantos valores hay en total
 *   static void prueba_parar(void)                     se perdio el agente: dejar todo seguro
 *
 * Seguridad: antes de nada se ponen en 0 las 8 entradas de los drivers de motores (GPIO 25 26 17 33 27 16 23 32), asi
 * ninguna prueba deja una rueda girando por un pin flotante. En la ESP32-CAM (PRUEBA_EN_CAM) eso no se hace.
 *
 * El LED de la placa parpadea mientras busca al agente y queda apagado cuando conecta.
 * Ojo: el componente micro-ROS imprime la clave del Wi-Fi al conectar. No pegar el log del monitor en ningun lado. */
#ifndef PRUEBA_UROS_H
#define PRUEBA_UROS_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#ifndef PRUEBA_TOPICO
#define PRUEBA_TOPICO "prueba"              /* la ESP32-CAM usa "prueba_cam" para poder estar las dos conectadas */
#endif
#ifndef PRUEBA_NODO
#define PRUEBA_NODO "esp32_prueba"
#endif
#ifndef PIN_LED_ESTADO
#define PIN_LED_ESTADO GPIO_NUM_2           /* LED azul de la DevKit (en la ESP32-CAM: GPIO33, activo en bajo) */
#endif
#ifndef LED_ACTIVO_BAJO
#define LED_ACTIVO_BAJO 0
#endif
#ifndef PERIODO_ESTADO_US
#define PERIODO_ESTADO_US 100000            /* /estado a 10 Hz */
#endif

#define N_CMD 24
#define N_ESTADO 40
#define TIMEOUT_CREAR_MS 5000               /* espera por entidad al crear (una rafaga de Wi-Fi vence el 1 s por defecto) */
#define PING_ESPERA_MS 200
#define PING_CONECTADO_MS 300
#define PING_FALLOS_MAX 6

static void prueba_init(void);
static void prueba_ciclo(void);
static void prueba_comando(const float *a, int n);
static int prueba_estado(float *e);
static void prueba_parar(void);

/* Visibles para la prueba */
static volatile uint32_t ultimo_cmd_ms = 0;  /* cuando llego la ultima orden valida */
static volatile bool agente_conectado = false;

static uint32_t ms_ahora(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline float limitar_f(float x, float lo, float hi)
{
	return x < lo ? lo : (x > hi ? hi : x);
}

static void led_estado(int encendido)
{
	gpio_set_level(PIN_LED_ESTADO, LED_ACTIVO_BAJO ? !encendido : encendido);
}

static rcl_subscription_t sub_cmd;
static rcl_publisher_t pub_estado;
static std_msgs__msg__Float32MultiArray msg_cmd;
static std_msgs__msg__Float32MultiArray msg_estado;
static float cmd_buf[N_CMD];
static float estado_buf[N_ESTADO];
static uint32_t contador_estado = 0;

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;

static void callback_cmd(const void *msgin)
{
	const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;
	int n = (int)m->data.size;

	if (n < 2) {
		return;
	}
	if ((int)lroundf(m->data.data[0]) != PRUEBA_ID) {
		printf("orden para la prueba %d ignorada: este firmware es la prueba %d (%s)\n",
		       (int)lroundf(m->data.data[0]), PRUEBA_ID, PRUEBA_NOMBRE);
		return;
	}
	ultimo_cmd_ms = ms_ahora();
	prueba_comando(&m->data.data[1], n - 1);
}

static bool crear_entidades(void)
{
	allocator = rcl_get_default_allocator();
	if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
		return false;
	}
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_creation_session_timeout(rmw_context, TIMEOUT_CREAR_MS);
	}
	if (rclc_node_init_default(&node, PRUEBA_NODO, "", &support) != RCL_RET_OK) {
		return false;
	}
	if (rclc_subscription_init_best_effort(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                       PRUEBA_TOPICO "/cmd") != RCL_RET_OK) {
		return false;
	}
	if (rclc_publisher_init_best_effort(&pub_estado, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                    PRUEBA_TOPICO "/estado") != RCL_RET_OK) {
		return false;
	}
	executor = rclc_executor_get_zero_initialized_executor();
	if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
		return false;
	}
	if (rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &callback_cmd, ON_NEW_DATA) != RCL_RET_OK) {
		return false;
	}
	return true;
}

static void destruir_entidades(void)
{
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	}
	(void)rcl_publisher_fini(&pub_estado, &node);
	(void)rcl_subscription_fini(&sub_cmd, &node);
	(void)rclc_executor_fini(&executor);
	(void)rcl_node_fini(&node);
	(void)rclc_support_fini(&support);
}

static void publicar_estado(void)
{
	memset(estado_buf, 0, sizeof(estado_buf));
	estado_buf[0] = (float)PRUEBA_ID;
	estado_buf[1] = (float)(contador_estado++);
	int n = prueba_estado(estado_buf);
	if (n < 2) {
		n = 2;
	}
	if (n > N_ESTADO) {
		n = N_ESTADO;
	}
	msg_estado.data.size = (size_t)n;
	(void)rcl_publish(&pub_estado, &msg_estado, NULL);
}

/* Esperando agente -> conectado -> (agente perdido) -> esperando agente. Nunca se rinde. */
static void micro_ros_task(void *arg)
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
	/* Clave de cliente fija (de la MAC): tras un reinicio el agente reutiliza la sesion */
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
	uint32_t clave = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
	(void)rmw_uros_options_set_client_key(clave, rmw_options);

	bool conectado = false;
	int fallos_ping = 0;
	int64_t ultimo_ping_us = 0;
	int64_t ultimo_estado_us = 0;

	while (1) {
		if (!conectado) {
			led_estado((esp_timer_get_time() / 250000) % 2);   /* parpadeo: esperando al agente */
			if (rmw_uros_ping_agent_options(PING_ESPERA_MS, 1, rmw_options) == RMW_RET_OK) {
				printf("uros: agente visible, creando entidades\n");
				if (crear_entidades()) {
					conectado = true;
					agente_conectado = true;
					fallos_ping = 0;
					ultimo_ping_us = esp_timer_get_time();
					led_estado(0);
					printf("uros: conectado. Prueba %d (%s): escucha /%s/cmd y publica /%s/estado\n",
					       PRUEBA_ID, PRUEBA_NOMBRE, PRUEBA_TOPICO, PRUEBA_TOPICO);
				} else {
					printf("uros: fallo al crear entidades, reintento\n");
					destruir_entidades();
				}
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		} else {
			int64_t ahora_us = esp_timer_get_time();

			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
			if (ahora_us - ultimo_estado_us >= PERIODO_ESTADO_US) {
				ultimo_estado_us = ahora_us;
				publicar_estado();
			}
			if (ahora_us - ultimo_ping_us > 1000000) {
				ultimo_ping_us = ahora_us;
				if (rmw_uros_ping_agent_options(PING_CONECTADO_MS, 1, rmw_options) == RMW_RET_OK) {
					fallos_ping = 0;
				} else if (++fallos_ping >= PING_FALLOS_MAX) {
					printf("uros: agente perdido, dejo todo seguro y reconecto\n");
					prueba_parar();
					destruir_entidades();
					conectado = false;
					agente_conectado = false;
				}
			}
			usleep(5000);
		}
	}
}

static void ciclo_task(void *arg)
{
	(void)arg;
	TickType_t t = xTaskGetTickCount();
	while (1) {
		prueba_ciclo();
		vTaskDelayUntil(&t, pdMS_TO_TICKS(10));
	}
}

#ifndef PRUEBA_EN_CAM
static void motores_en_cero(void)
{
	static const gpio_num_t pines[] = {GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_17, GPIO_NUM_33,
	                                   GPIO_NUM_27, GPIO_NUM_16, GPIO_NUM_23, GPIO_NUM_32};
	for (size_t i = 0; i < sizeof(pines) / sizeof(pines[0]); i++) {
		gpio_reset_pin(pines[i]);
		gpio_set_direction(pines[i], GPIO_MODE_OUTPUT);
		gpio_set_level(pines[i], 0);
	}
}
#endif

void app_main(void)
{
#ifndef PRUEBA_EN_CAM
	motores_en_cero();
#endif
	gpio_reset_pin(PIN_LED_ESTADO);
	gpio_set_direction(PIN_LED_ESTADO, GPIO_MODE_OUTPUT);
	led_estado(0);

	std_msgs__msg__Float32MultiArray__init(&msg_estado);
	msg_estado.data.data = estado_buf;
	msg_estado.data.size = 2;
	msg_estado.data.capacity = N_ESTADO;
	std_msgs__msg__Float32MultiArray__init(&msg_cmd);
	msg_cmd.data.data = cmd_buf;
	msg_cmd.data.size = 0;
	msg_cmd.data.capacity = N_CMD;

	printf("\n=== Prueba %d: %s ===\n", PRUEBA_ID, PRUEBA_NOMBRE);
	prueba_init();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* sin ahorro de energia: menos latencia */
#endif

	xTaskCreatePinnedToCore(ciclo_task, "prueba", 4096, NULL, 6, NULL, 1);
	xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

#endif /* PRUEBA_UROS_H */
