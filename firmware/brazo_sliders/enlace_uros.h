/* enlace_uros.h: la conexion inalambrica de la ESP32 con la PC, por micro-ROS.
 *
 * La ESP32 se conecta por Wi-Fi al AGENTE micro-ROS que corre en la PC (red, clave, IP y puerto del agente se
 * guardan en idf.py menuconfig, no en el codigo). Por ese agente la ESP32 aparece en ROS 2 como un nodo mas, con dos topicos:
 *   /brazo/cmd     PC -> ESP32   (std_msgs/Float32MultiArray)  la orden: lo que la PC quiere
 *   /brazo/estado  ESP32 -> PC   (std_msgs/Float32MultiArray)  lo que la ESP32 esta haciendo
 *
 * Este archivo no sabe nada de servos: cuando llega una orden llama a la funcion cb_orden que le dio main.c, y cada 100 ms
 * llama a cb_estado para que main.c llene el estado a publicar.
 *
 * Si no hay agente, la ESP32 espera (LED parpadeando) y reintenta para siempre. Si el agente se cae, vuelve a esperar. */
#ifndef ENLACE_UROS_H
#define ENLACE_UROS_H

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

#define ENLACE_N_ORDEN 8                 /* cuantos numeros caben en una orden recibida */
#define ENLACE_N_ESTADO 16               /* cuantos numeros caben en el estado publicado */
#define ENLACE_PIN_LED GPIO_NUM_2        /* LED azul de la placa: parpadea = buscando al agente, apagado = conectado */
#define ENLACE_PERIODO_ESTADO_US 100000  /* el estado sale 10 veces por segundo */

typedef void (*enlace_cb_orden_t)(const float *datos, int n);
typedef int (*enlace_cb_estado_t)(float *buf);    /* llena buf y devuelve cuantos numeros puso */

static enlace_cb_orden_t cb_orden = NULL;
static enlace_cb_estado_t cb_estado = NULL;
static volatile bool enlace_conectado = false;

static rcl_subscription_t sub_orden;
static rcl_publisher_t pub_estado;
static std_msgs__msg__Float32MultiArray msg_orden;
static std_msgs__msg__Float32MultiArray msg_estado;
static float orden_buf[ENLACE_N_ORDEN];
static float estado_buf[ENLACE_N_ESTADO];

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;

/* micro-ROS llama a esta funcion cada vez que llega un mensaje a /brazo/cmd */
static void al_llegar_orden(const void *msgin)
{
	const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;

	if (cb_orden != NULL) {
		cb_orden(m->data.data, (int)m->data.size);
	}
}

static bool crear_entidades(void)
{
	allocator = rcl_get_default_allocator();
	if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) {
		return false;
	}
	/* Esperar mas que el valor por defecto (1 s) cada vez que se crea algo: una rafaga de Wi-Fi puede tardar mas */
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_creation_session_timeout(rmw_context, 5000);
	}
	if (rclc_node_init_default(&node, "esp32_brazo", "", &support) != RCL_RET_OK) {
		return false;
	}
	/* best effort: sin reenvios. Si se pierde un mensaje, el siguiente (100 ms despues) trae lo mismo actualizado */
	if (rclc_subscription_init_best_effort(&sub_orden, &node,
	                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                       "brazo/cmd") != RCL_RET_OK) {
		return false;
	}
	if (rclc_publisher_init_best_effort(&pub_estado, &node,
	                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                    "brazo/estado") != RCL_RET_OK) {
		return false;
	}
	executor = rclc_executor_get_zero_initialized_executor();
	if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
		return false;
	}
	if (rclc_executor_add_subscription(&executor, &sub_orden, &msg_orden, &al_llegar_orden, ON_NEW_DATA) != RCL_RET_OK) {
		return false;
	}
	return true;
}

static void destruir_entidades(void)
{
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);   /* sin esperar: el agente ya no esta */
	}
	(void)rcl_publisher_fini(&pub_estado, &node);
	(void)rcl_subscription_fini(&sub_orden, &node);
	(void)rclc_executor_fini(&executor);
	(void)rcl_node_fini(&node);
	(void)rclc_support_fini(&support);
}

static void publicar_estado(void)
{
	int n = (cb_estado != NULL) ? cb_estado(estado_buf) : 0;

	if (n > ENLACE_N_ESTADO) {
		n = ENLACE_N_ESTADO;
	}
	msg_estado.data.size = (size_t)n;
	(void)rcl_publish(&pub_estado, &msg_estado, NULL);
}

/* Esperando al agente -> conectado -> (agente perdido) -> esperando al agente. Nunca se rinde. */
static void tarea_micro_ros(void *arg)
{
	(void)arg;
	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
		printf("enlace: no se pudieron crear las opciones\n");
		vTaskDelete(NULL);
	}
	rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	/* IP y puerto del agente: los puso menuconfig */
	if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options) != RCL_RET_OK) {
		printf("enlace: direccion del agente invalida\n");
		vTaskDelete(NULL);
	}
	/* Identificador fijo (sale de la MAC): si la ESP32 se reinicia, el agente reconoce la sesion anterior */
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
	uint32_t clave = ((uint32_t)mac[2] << 24) | ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
	(void)rmw_uros_options_set_client_key(clave, rmw_options);

	bool conectado = false;
	int fallos = 0;
	int64_t ultimo_ping_us = 0;
	int64_t ultimo_estado_us = 0;

	while (1) {
		if (!conectado) {
			gpio_set_level(ENLACE_PIN_LED, (esp_timer_get_time() / 250000) % 2);        /* parpadeo */
			if (rmw_uros_ping_agent_options(200, 1, rmw_options) == RMW_RET_OK) {
				printf("enlace: agente visible, creando topicos\n");
				if (crear_entidades()) {
					conectado = true;
					enlace_conectado = true;
					fallos = 0;
					ultimo_ping_us = esp_timer_get_time();
					gpio_set_level(ENLACE_PIN_LED, 0);
					printf("enlace: conectado. Escucha /brazo/cmd y publica /brazo/estado\n");
				} else {
					printf("enlace: no pude crear los topicos, reintento\n");
					destruir_entidades();
				}
			}
			vTaskDelay(pdMS_TO_TICKS(500));
		} else {
			int64_t ahora_us = esp_timer_get_time();

			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));          /* atiende los mensajes que llegaron */
			if (ahora_us - ultimo_estado_us >= ENLACE_PERIODO_ESTADO_US) {
				ultimo_estado_us = ahora_us;
				publicar_estado();
			}
			if (ahora_us - ultimo_ping_us > 1000000) {                      /* cada 1 s, comprobar si sigue el agente */
				ultimo_ping_us = ahora_us;
				if (rmw_uros_ping_agent_options(300, 1, rmw_options) == RMW_RET_OK) {
					fallos = 0;
				} else if (++fallos >= 6) {
					printf("enlace: agente perdido, reconecto\n");
					destruir_entidades();
					conectado = false;
					enlace_conectado = false;
				}
			}
			usleep(5000);
		}
	}
}

/* Se llama una vez desde app_main: prepara los mensajes, la red Wi-Fi y lanza la tarea de micro-ROS */
static void enlace_iniciar(enlace_cb_orden_t al_recibir, enlace_cb_estado_t al_publicar)
{
	cb_orden = al_recibir;
	cb_estado = al_publicar;

	gpio_reset_pin(ENLACE_PIN_LED);
	gpio_set_direction(ENLACE_PIN_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(ENLACE_PIN_LED, 0);

	/* Los mensajes usan memoria estatica: el arreglo de datos apunta a nuestros buffers */
	std_msgs__msg__Float32MultiArray__init(&msg_estado);
	msg_estado.data.data = estado_buf;
	msg_estado.data.size = 0;
	msg_estado.data.capacity = ENLACE_N_ESTADO;
	std_msgs__msg__Float32MultiArray__init(&msg_orden);
	msg_orden.data.data = orden_buf;
	msg_orden.data.size = 0;
	msg_orden.data.capacity = ENLACE_N_ORDEN;

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* Wi-Fi sin ahorro de energia: menos retardo */
#endif
	xTaskCreate(tarea_micro_ros, "uros", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}

#endif /* ENLACE_UROS_H */