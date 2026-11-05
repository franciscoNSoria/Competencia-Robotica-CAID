/* main.c: brazo con 4 servos que se mueven desde la PC con sliders, por Wi-Fi (micro-ROS).
 * Version 3: limites ajustables, tope propio de cada servo y reposo seguro obligatorio antes de girar la base.
 *
 * Archivos de este programa (los tres van juntos en la carpeta main/ del proyecto ESP-IDF):
 *   servos_brazo.h   los 4 servos: grados -> pulso, y movimiento con velocidad maxima
 *   enlace_uros.h    la conexion con la PC (topicos /brazo/cmd y /brazo/estado)
 *   main.c           este archivo: une las dos cosas y define los pines, los limites y el reposo seguro
 * En la PC corre tools/brazo/brazo_sliders.py (la ventana con los sliders).
 *
 * Orden que llega por /brazo/cmd (Float32MultiArray), se repite 10 veces por segundo:
 *   [0..3]   angulo objetivo en grados de: base, hombro, codo, pinza
 *   [4]      mascara de servos activos (bit 0 = base, 1 = hombro, 2 = codo, 3 = pinza); un bit en 0 = ese servo suelto
 *   [5]      velocidad maxima en grados por segundo (0 = salto directo)
 *   [6..13]  (opcional) limites minimo y maximo de cada servo (base, hombro, codo, pinza). Si la orden trae menos de 14 numeros,
 *            los limites no cambian. Cada limite queda entre 0 y el tope de ese servo: ver servo_poner_limites()
 *   [14..15] (opcional) reposo seguro: angulo del hombro y del codo antes de girar la base
 * Estado que sale por /brazo/estado, 10 veces por segundo:
 *   [0..3]   angulo actual de cada servo      [4] mascara de activos     [5] velocidad en uso
 *   [6]      contador de mensajes (si baja, la ESP32 se reinicio)       [7] ms desde la ultima orden de la PC
 *   [8..15]  limites minimo y maximo de cada servo (base, hombro, codo, pinza)
 *   [16..19] tope absoluto de cada servo         [20] fase de la base: 0 libre, 1 esperando el reposo, 2 girando
 *   [21..22] reposo seguro en uso (hombro, codo), ya acotado a sus limites */
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "servos_brazo.h"
#include "enlace_uros.h"

#define N_SERVOS 4
#define IDX_BASE 0
#define IDX_HOMBRO 1
#define IDX_CODO 2
#define IDX_PINZA 3
#define PERIODO_CONTROL_MS 10
#define VELOCIDAD_INICIAL_DPS 90.0f
#define VELOCIDAD_MAX_DPS 1000.0f

/* Reposo seguro: la base solo gira con el brazo recogido y alto, para no chocar con la camara.
 * Cuando la PC pide mover la base, el hombro y el codo van primero a esta pose, la base espera, gira, y al llegar el brazo
 * vuelve a lo que pida la PC. Sin realimentacion no se sabe cuando LLEGA el brazo de verdad: se espera ESPERA_REPOSO_MS
 * despues de que el brazo llegue a la pose (P: hay que probarlo en el robot, sobre todo con el hombro cargado). */
#define BLOQUEO_BASE_ACTIVO 1            /* 0 = sin bloqueo (solo para probar la base suelta, sin brazo) */
#define POSE_SEGURA_HOMBRO_DEG 60.0f     /* valores de arranque; la PC manda los suyos en [14..15] */
#define POSE_SEGURA_CODO_DEG 180.0f
#define TOL_REPOSO_DEG 1.0f              /* cuanto puede faltar para dar por llegado al hombro y al codo */
#define ESPERA_REPOSO_MS 1000
#define BASE_UMBRAL_DEG 0.5f             /* una base que pide moverse mas que esto activa el reposo seguro */
#define BASE_LLEGO_DEG 0.05f             /* ... y se la da por llegada cuando falta menos que esto */

enum { FASE_LIBRE = 0, FASE_ESPERANDO = 1, FASE_GIRANDO = 2 };

/* Pines: los que quedan libres en la ESP32 con el cableado actual del robot.
 * Si dejas el servo del hombro enchufado en el conector de la pinza (GPIO22) y la pinza en el del hombro (GPIO19), intercambia
 * los dos numeros de .pin entre las filas "hombro" y "pinza": asi cada slider mueve el servo que dice su nombre.
 *
 * .tope_max_deg: lo maximo que la ESP32 deja pedir a cada servo (grados nominales: 500 us = 0, 2500 us = 180). El codo llega a
 *   200 (2722 us): pasa de los 2500 us del manual de los MG90S, asi que hay que subirlo de a poco y mirando (P).
 * Limites de arranque: valen solo hasta que la PC mande los suyos (los que guarda en ~/caid/config/brazo_calibracion.json):
 *   los que elegiste en el banco: base 0-180, hombro 10-125, codo 80-200, pinza 100-180 (la pinza cierra apretada en 114).
 * Objetivo inicial: el reposo seguro para el hombro y el codo, el centro (95) para la base. */
static servo_t servos[N_SERVOS] = {
	{.nombre = "base", .pin = GPIO_NUM_18, .canal = LEDC_CHANNEL_0, .tope_max_deg = 180.0f, .min_deg = 0.0f, .max_deg = 180.0f, .objetivo_deg = 95.0f, .actual_deg = 95.0f},
	{.nombre = "hombro", .pin = GPIO_NUM_19, .canal = LEDC_CHANNEL_1, .tope_max_deg = 180.0f, .min_deg = 10.0f, .max_deg = 125.0f, .objetivo_deg = 60.0f, .actual_deg = 60.0f},
	{.nombre = "codo", .pin = GPIO_NUM_21, .canal = LEDC_CHANNEL_2, .tope_max_deg = 200.0f, .min_deg = 80.0f, .max_deg = 200.0f, .objetivo_deg = 180.0f, .actual_deg = 180.0f},
	{.nombre = "pinza", .pin = GPIO_NUM_22, .canal = LEDC_CHANNEL_3, .tope_max_deg = 180.0f, .min_deg = 100.0f, .max_deg = 180.0f, .objetivo_deg = 150.0f, .actual_deg = 150.0f},
};

static volatile float velocidad_dps = VELOCIDAD_INICIAL_DPS;
static volatile uint32_t t_ultima_orden_ms = 0;
static uint32_t contador_estado = 0;
static volatile float pose_segura_hombro = POSE_SEGURA_HOMBRO_DEG;
static volatile float pose_segura_codo = POSE_SEGURA_CODO_DEG;
static volatile int fase_base = FASE_LIBRE;        /* lo escribe solo la tarea de control */

static uint32_t ms_ahora(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000);
}

/* El reposo seguro de un servo, acotado a sus limites (si la PC pide un reposo fuera de lo que el servo puede, va al limite) */
static float pose_segura_de(int idx)
{
	float pedido = (idx == IDX_HOMBRO) ? pose_segura_hombro : pose_segura_codo;

	return limitar_deg(pedido, servos[idx].min_deg, servos[idx].max_deg);
}

/* Llega una orden de la PC (la llama enlace_uros.h). No confia en la PC: los limites se dejan dentro del tope de cada servo en
 * servo_poner_limites(), cada angulo se limita en servo_ir_a() y un NaN descarta la orden. */
static void al_recibir_orden(const float *datos, int n)
{
	if (n < 6 || isnan(datos[4]) || isnan(datos[5])) {
		return;                                   /* orden incompleta o con un NaN: se ignora */
	}
	int mascara = (int)lroundf(datos[4]);
	for (int i = 0; i < N_SERVOS; i++) {
		if (n >= 14) {
			servo_poner_limites(&servos[i], datos[6 + 2 * i], datos[7 + 2 * i]);   /* primero los limites ... */
		}
		servo_ir_a(&servos[i], datos[i]);                                           /* ... y despues el angulo */
		servos[i].activo = (mascara >> i) & 1;
	}
	if (n >= 16 && !isnan(datos[14]) && !isnan(datos[15])) {
		pose_segura_hombro = datos[14];
		pose_segura_codo = datos[15];
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
		buf[16 + i] = servos[i].tope_max_deg;
	}
	buf[4] = (float)mascara;
	buf[5] = velocidad_dps;
	buf[6] = (float)(contador_estado++);
	buf[7] = (float)(ms_ahora() - t_ultima_orden_ms);
	buf[20] = (float)fase_base;
	buf[21] = pose_segura_de(IDX_HOMBRO);
	buf[22] = pose_segura_de(IDX_CODO);
	return 23;
}

/* Decide la META de cada servo en esta pasada: lo que pide la PC, salvo que la base quiera girar. Entonces:
 *   FASE_ESPERANDO: el hombro y el codo van al reposo seguro y la base se queda quieta hasta que llegan (y pasa ESPERA_REPOSO_MS)
 *   FASE_GIRANDO:   el brazo se queda en el reposo seguro y la base gira hacia lo que pide la PC
 * Al llegar la base, el brazo vuelve a seguir a la PC. Si el hombro o el codo estan sueltos, la base no gira: un brazo suelto
 * no esta en ningun lugar seguro. */
static void aplicar_reposo_seguro(void)
{
	servo_t *base = &servos[IDX_BASE];
	servo_t *hombro = &servos[IDX_HOMBRO];
	servo_t *codo = &servos[IDX_CODO];
	static int pasadas_en_reposo = 0;

	for (int i = 0; i < N_SERVOS; i++) {
		servos[i].meta_deg = servos[i].objetivo_deg;
	}
	if (!BLOQUEO_BASE_ACTIVO || !base->activo) {
		fase_base = FASE_LIBRE;
		return;
	}

	float falta_base = fabsf(base->objetivo_deg - base->actual_deg);

	if (fase_base == FASE_LIBRE) {
		if (falta_base <= BASE_UMBRAL_DEG) {
			return;                               /* la base no quiere girar: todo sigue a la PC */
		}
		fase_base = FASE_ESPERANDO;
		pasadas_en_reposo = 0;
	}
	if (falta_base <= BASE_LLEGO_DEG) {
		fase_base = FASE_LIBRE;                   /* la base llego (o la PC se arrepintio): el brazo vuelve a seguir a la PC */
		return;
	}

	float hs = pose_segura_de(IDX_HOMBRO);
	float cs = pose_segura_de(IDX_CODO);
	bool en_reposo = hombro->activo && codo->activo && fabsf(hombro->actual_deg - hs) <= TOL_REPOSO_DEG &&
	                 fabsf(codo->actual_deg - cs) <= TOL_REPOSO_DEG;

	hombro->meta_deg = hs;
	codo->meta_deg = cs;
	if (fase_base == FASE_ESPERANDO) {
		base->meta_deg = base->actual_deg;        /* la base espera */
		pasadas_en_reposo = en_reposo ? pasadas_en_reposo + 1 : 0;
		if (pasadas_en_reposo * PERIODO_CONTROL_MS >= ESPERA_REPOSO_MS) {
			fase_base = FASE_GIRANDO;
		}
	} else if (!en_reposo) {                      /* FASE_GIRANDO y el brazo se solto o se movio: la base frena y espera */
		fase_base = FASE_ESPERANDO;
		pasadas_en_reposo = 0;
		base->meta_deg = base->actual_deg;
	}
}

/* Tarea de control: cada 10 ms decide las metas y mueve los servos hacia ellas. Corre aunque no haya Wi-Fi ni agente. */
static void tarea_control(void *arg)
{
	(void)arg;
	TickType_t t = xTaskGetTickCount();
	while (1) {
		aplicar_reposo_seguro();
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