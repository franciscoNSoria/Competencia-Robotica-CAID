/* main.c: brazo con 4 servos que se mueven desde la PC con sliders, por Wi-Fi (micro-ROS).
 *
 * Archivos de este programa (los tres van juntos en la carpeta main/ del proyecto ESP-IDF):
 *   servos_brazo.h   los 4 servos: grados -> pulso, y movimiento con velocidad maxima
 *   enlace_uros.h    la conexion con la PC (topicos /brazo/cmd y /brazo/estado)
 *   main.c           este archivo: une las dos cosas y define los pines y limites
 * En la PC corre tools/brazo/brazo_sliders.py (la ventana con los sliders).
 *
 * Orden que llega por /brazo/cmd (Float32MultiArray), se repite 10 veces por segundo:
 *   [0..3]  angulo objetivo en grados de: base, hombro, codo, pinza
 *   [4]     mascara de servos activos (bit 0 = base, 1 = hombro, 2 = codo, 3 = pinza); un bit en 0 = ese servo suelto
 *   [5]     velocidad maxima en grados por segundo (0 = salto directo)
 * Estado que sale por /brazo/estado, 10 veces por segundo:
 *   [0..3]  angulo actual de cada servo      [4] mascara de activos     [5] velocidad en uso
 *   [6]     contador de mensajes (si baja, la ESP32 se reinicio)       [7] ms desde la ultima orden de la PC
 *   [8..15] limites minimo y maximo de cada servo (base, hombro, codo, pinza) */
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "servos_brazo.h"
#include "enlace_uros.h"

#define N_SERVOS 4
#define PERIODO_CONTROL_MS 10
#define VELOCIDAD_INICIAL_DPS 90.0f
#define VELOCIDAD_MAX_DPS 1000.0f

/* Pines: los que quedan libres en la ESP32 con el cableado actual del robot.
 * Limites en grados: lo que el servo puede recorrer sin chocar con el brazo. Los de hombro y codo cubren lo medido en el
 * banco (hombro 1300-1700 us = 72-108 grados, codo 1000-1700 us = 45-108 grados) con margen; base y pinza son una
 * propuesta (P): ajustarlos probando. */
static servo_t servos[N_SERVOS] = {
	{.nombre = "base", .pin = GPIO_NUM_18, .canal = LEDC_CHANNEL_0, .min_deg = 0.0f, .max_deg = 180.0f, .objetivo_deg = 90.0f, .actual_deg = 90.0f},
	{.nombre = "hombro", .pin = GPIO_NUM_19, .canal = LEDC_CHANNEL_1, .min_deg = 60.0f, .max_deg = 120.0f, .objetivo_deg = 90.0f, .actual_deg = 90.0f},
	{.nombre = "codo", .pin = GPIO_NUM_21, .canal = LEDC_CHANNEL_2, .min_deg = 30.0f, .max_deg = 120.0f, .objetivo_deg = 90.0f, .actual_deg = 90.0f},
	{.nombre = "pinza", .pin = GPIO_NUM_22, .canal = LEDC_CHANNEL_3, .min_deg = 0.0f, .max_deg = 180.0f, .objetivo_deg = 90.0f, .actual_deg = 90.0f},
};

static volatile float velocidad_dps = VELOCIDAD_INICIAL_DPS;
static volatile uint32_t t_ultima_orden_ms = 0;
static uint32_t contador_estado = 0;

static uint32_t ms_ahora(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Llega una orden de la PC (la llama enlace_uros.h). No confia en la PC: cada angulo se limita en servo_ir_a(). */
static void al_recibir_orden(const float *datos, int n)
{
	if (n < 6) {
		return;                                   /* orden incompleta: se ignora */
	}
	int mascara = (int)lroundf(datos[4]);
	for (int i = 0; i < N_SERVOS; i++) {
		servo_ir_a(&servos[i], datos[i]);
		servos[i].activo = (mascara >> i) & 1;
	}
	float v = datos[5];
	velocidad_dps = (v < 0.0f) ? 0.0f : (v > VELOCIDAD_MAX_DPS ? VELOCIDAD_MAX_DPS : v);
	t_ultima_orden_ms = ms_ahora();
}

/* Hay que publicar el estado (la llama enlace_uros.h): llena buf y devuelve cuantos numeros puso */
static int armar_estado(float *buf)
{
	int mascara = 0;
	for (int i = 0; i < N_SERVOS; i++) {
		buf[i] = servos[i].actual_deg;
		mascara |= servos[i].activo ? (1 << i) : 0;
		buf[8 + 2 * i] = servos[i].min_deg;
		buf[9 + 2 * i] = servos[i].max_deg;
	}
	buf[4] = (float)mascara;
	buf[5] = velocidad_dps;
	buf[6] = (float)(contador_estado++);
	buf[7] = (float)(ms_ahora() - t_ultima_orden_ms);
	return 16;
}

/* Tarea de control: cada 10 ms mueve los servos hacia su objetivo. Corre aunque no haya Wi-Fi ni agente. */
static void tarea_control(void *arg)
{
	(void)arg;
	TickType_t t = xTaskGetTickCount();
	while (1) {
		servos_ciclo(servos, N_SERVOS, velocidad_dps, PERIODO_CONTROL_MS / 1000.0f);
		vTaskDelayUntil(&t, pdMS_TO_TICKS(PERIODO_CONTROL_MS));
	}
}

void app_main(void)
{
	printf("\n=== Brazo con sliders: 4 servos por micro-ROS ===\n");
	servos_init(servos, N_SERVOS);                /* los servos quedan SIN pulso hasta que la PC los active */
	xTaskCreatePinnedToCore(tarea_control, "control", 4096, NULL, 6, NULL, 1);
	enlace_iniciar(al_recibir_orden, armar_estado);
}