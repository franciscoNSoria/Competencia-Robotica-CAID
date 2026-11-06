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
#include "esp_event.h"
#include "esp_system.h"

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

/* sonar.h trae funciones de las pruebas que este firmware no usa: que no avisen */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "sonar.h"
#pragma GCC diagnostic pop
#include "guardia.h"
#include "servos_brazo.h"

/* Robot de 4 ruedas + brazo de 4 servos, los dos en la misma ESP32 (fusion de robot_4ruedas + brazo_sliders: antes eran
 * dos firmwares separados y la prueba de banco sin camara necesitaba reflashear entre mover el robot y mover el brazo,
 * lo que no tiene sentido para una prueba de punta a punta. Un PI de velocidad por rueda (el de main_motor_v8_pi.c, una
 * instancia por rueda) y el control de los 4 servos del brazo (servos_brazo.h, igual que brazo_sliders/main.c).
 * Entra:   /cmd_vel      geometry_msgs/Twist, linear.x en m/s y angular.z en rad/s (cinematica diferencial)
 *          /robot_cmd    std_msgs/Float32MultiArray [id, args...]:
 *                          1 d0 d1 d2 d3      duty directo por rueda, lazo abierto (calibracion); 0 = esa rueda libre
 *                          2 v0 v1 v2 v3      velocidad por rueda en m/s, con PI (FL, FR, RL, RR)
 *                          3 dist_m vel       mover dist_m (con signo) con rampas; vel en flancos/s (0 = por defecto)
 *                          4                  parar todo
 *                          5 diam trocha fren [fren_giro]  config en metros: diametro efectivo, trocha efectiva, adelanto de
 *                                                          frenado (y, opcional, el adelanto de frenado de los giros)
 *                          6 rueda dmin n v1 d1 v2 d2 ...   curva de prealimentacion de una rueda (n <= 10 puntos)
 *                          7 ang_rad vel      girar EN EL LUGAR ang_rad (con signo: + izquierda, antihorario) con rampas; vel en
 *                                             flancos/s por rueda (0 = por defecto). Usa la trocha efectiva: arco = ang x trocha / 2
 *                          9 factor [nivel]   giros: las delanteras van factor x la velocidad de las traseras (0,5 a 2) y, opcional,
 *                                             el nivel del empujon de arranque de FL y FR (0 a 3; las traseras usan 2)
 *                          8 act d_stop d_lento [ciego_s]   freno local por sonar (guardia.h). act 1 = activo (0 = apagado,
 *                                             es el estado al arrancar). Sin orden en 1 s las ruedas igual se frenan.
 *                                             Con la guardia activa, el sonar frontal topa la velocidad HACIA ADELANTE:
 *                                             d >= d_lento (m) libre; entre d_stop y d_lento 0,13 m/s; a d <= d_stop, 0.
 *                                             Girar en el lugar y retroceder siguen permitidos. Sin medicion fresca del
 *                                             frontal, o sin ningun eco hace mas de ciego_s (por defecto 5 s), tampoco avanza.
 *          /brazo/cmd    std_msgs/Float32MultiArray, se repite 10 veces por segundo (protocolo igual al de brazo_sliders):
 *                          [0..3]   angulo objetivo en grados de: base, hombro, codo, pinza
 *                          [4]      mascara de servos activos (bit 0 = base, 1 = hombro, 2 = codo, 3 = pinza); bit en 0 = suelto
 *                          [5]      velocidad maxima en grados por segundo (0 = salto directo)
 *                          [6..13]  (opcional) limites minimo y maximo de cada servo (base, hombro, codo, pinza)
 *                          [14..15] (opcional) reposo seguro: angulo del hombro y del codo antes de girar la base
 * Sale:    /wheel_ticks   std_msgs/Int32MultiArray, 4 flancos acumulados con signo [FL, FR, RL, RR], 20 Hz
 *          /wheel_state   std_msgs/Float32MultiArray, 34 valores, 10 Hz:
 *                          [0..3] velocidad medida (flancos/s con signo)  [4..7] velocidad pedida (flancos/s)
 *                          [8..11] duty con signo   [12..15] rueda bloqueada (1) o no (0)
 *                          [16] movimiento activo   [17] recorrido actual (m, con signo)   [18] recorrido final (m)
 *                          [19] contador de movimientos terminados   [20] diametro efectivo (mm)   [21] adelanto de frenado (mm)
 *                          [22] contador de configuraciones y curvas aceptadas   [23] trocha efectiva (mm)
 *                          [24] tipo del ultimo movimiento: 0 distancia, 1 giro   [25] adelanto de frenado de los giros (mm)
 *                          (en un giro, [17] y [18] son el arco que recorre cada rueda en m, con signo: angulo = arco / (trocha / 2))
 *                          [26] sonar frontal  [27] sonar izquierdo  [28] sonar derecho (m; -1 = sin eco)
 *                          [29] guardia: 0 apagada, 1 libre, 2 lenta, 3 frenada, 4 falla (sin medicion fresca o ciego)
 *                          [30] d_stop (m)  [31] d_lento (m)  [32] mediciones terminadas del frontal  [33] ciego_s
 *          /brazo/estado  std_msgs/Float32MultiArray, 10 veces por segundo (igual a brazo_sliders):
 *                          [0..3]   angulo actual de cada servo      [4] mascara de activos     [5] velocidad en uso
 *                          [6]      contador de mensajes (si baja, la ESP32 se reinicio)       [7] ms desde la ultima orden de la PC
 *                          [8..15]  limites minimo y maximo de cada servo (base, hombro, codo, pinza)
 *                          [16..19] tope absoluto de cada servo         [20] fase de la base: 0 libre, 1 esperando el reposo, 2 girando
 *                          [21..22] reposo seguro en uso (hombro, codo), ya acotado a sus limites
 * linear.x es m/s de verdad (en v8d, 0,5 equivalia a 75 flancos/s = 0,395 m/s).
 * Pines: hardware/electronica/propuesta-v0.2.md, mapa B-rueda (RR_IB en el 32).
 * Sonares (firmware/pruebas/comun/sonar.h): frontal TRIG 5 / ECHO 13, derecho 15 / 4, izquierdo 14 / 12. Un disparo cada 60 ms
 * en la secuencia frontal, derecho, frontal, izquierdo: el frontal se mide cada 120 ms y los laterales cada 240 ms.
 * Servos del brazo (servos_brazo.h, pines libres con el cableado actual): base 18, hombro 19, codo 21, pinza 22. Los canales
 * LEDC de las ruedas (LEDC_HIGH_SPEED_MODE) y los del brazo (LEDC_LOW_SPEED_MODE) son dos perifericos aparte del ESP32
 * clasico: no compiten entre si aunque los dos usen el canal 0 y el LEDC_TIMER_0 de su propio grupo. */

#define N_RUEDAS 4
#define PIN_LED GPIO_NUM_2
#define PWM_HZ 5000
#define PWM_MAX 1023                /* 10 bits */

/* Dominio ROS de este robot. El agente micro-ROS crea su participante en el dominio que le pide la ESP32 (no el ROS_DOMAIN_ID de la PC):
 * en TODAS las terminales de la PC hay que usar el mismo (export ROS_DOMAIN_ID=71) o no ven los topicos. Cambiarlo requiere regrabar. */
#define ROS_DOMINIO 71
#define WIFI_TX_POTENCIA 52   /* cuartos de dBm: 52 = 13 dBm (maximo 80 = 20 dBm). Menos potencia = picos de corriente mas chicos si la fuente se hunde */

/* Calibrar con la prueba del metro (docs de calibracion): el diametro efectivo y la trocha efectiva salen de medir */
#define DIAMETRO_RUEDA_M 0.065f     /* medido (calibracion.json); el valor en uso se cambia con /robot_cmd id 5. Es lo que vale tras un REINICIO de la ESP32: tiene que ser el calibrado */
#define TROCHA_M 0.300f             /* efectiva medida con giros (calibracion.json); el CAD dice 0,131 pero en skid-steer es mayor. Tras un reinicio vale esto */
#define FRENADO_M 0.0108f           /* (calibrado) se corta esto antes de llegar para que la inercia complete el recorrido; la PC lo ajusta sola */
#define ACEL_FLANCOS_S2 100.0f      /* rampa de arranque y de frenado en los movimientos de distancia (0,53 m/s2) */
#define VEL_MOV_DEFECTO 40.0f       /* flancos/s (0,21 m/s) */
#define FRENADO_GIRO_M 0.0122f      /* (calibrado) en los giros: arco (por rueda) que se corta antes de llegar; 12 mm = 4,7 grados con trocha 0,300 */
#define VEL_GIRO_DEFECTO 30.0f      /* flancos/s por rueda en un giro: 0,16 m/s, unos 61 grados/s con trocha 0,300 */
#define GIRO_MAX_RAD 12.6f          /* un giro pedido no pasa de 12,6 rad (721 grados) */
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
#define PERIODO_ESTADO_US 100000    /* /wheel_state y /brazo/estado a 10 Hz */
#define N_ESTADO 34                 /* valores de /wheel_state */
#define ZONA_MUERTA_FLANCOS (VEL_MIN_FLANCOS * 0.5f)   /* bajo esto una rueda no se pide (ver ms_a_flancos) */
#define GIRO_FACTOR_DELANTERAS 1.15f                   /* en los giros en el lugar las delanteras (poco peso, arrancan peor) van un 15 % mas rapido que las traseras */
#define GUARDIA_VEL_LENTA VEL_MIN_FLANCOS              /* tope de la zona lenta: lo menos que sostienen los 4 motores */
#define SONAR_RANURA_US 60000       /* un disparo cada 60 ms */

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

/* 1 mientras corre un giro en el lugar: sin rampa. Velocidad maxima de entrada, sin limite al cambio de duty del PI y con el empujon
 * de arranque al menos en el nivel 2 en las 4 ruedas (que no quede una sin arrancar). */
static volatile int giro_directo = 0;

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

/* Polaridad: +1 salvo FR (-1: el motor delantero derecho gira al reves con IA; invertida el 5/11) */
static rueda_t W[N_RUEDAS] = {
	{.nombre = "FL", .pin_ia = GPIO_NUM_25, .pin_ib = GPIO_NUM_26, .pin_enc = GPIO_NUM_34, .canal_ia = LEDC_CHANNEL_0,
	 .canal_ib = LEDC_CHANNEL_1, .polaridad = +1, .curva = CURVA_DEL, .n_curva = N_CURVA(CURVA_DEL), .d_min = 0.36f,
	 .nivel_kick_ini = 0},
	{.nombre = "FR", .pin_ia = GPIO_NUM_17, .pin_ib = GPIO_NUM_33, .pin_enc = GPIO_NUM_36, .canal_ia = LEDC_CHANNEL_2,
	 .canal_ib = LEDC_CHANNEL_3, .polaridad = -1, .curva = CURVA_DEL, .n_curva = N_CURVA(CURVA_DEL), .d_min = 0.36f,
	 .nivel_kick_ini = 0},
	{.nombre = "RL", .pin_ia = GPIO_NUM_27, .pin_ib = GPIO_NUM_16, .pin_enc = GPIO_NUM_35, .canal_ia = LEDC_CHANNEL_4,
	 .canal_ib = LEDC_CHANNEL_5, .polaridad = +1, .curva = CURVA_RL, .n_curva = N_CURVA(CURVA_RL), .d_min = 0.40f,
	 .nivel_kick_ini = 2},
	{.nombre = "RR", .pin_ia = GPIO_NUM_23, .pin_ib = GPIO_NUM_32, .pin_enc = GPIO_NUM_39, .canal_ia = LEDC_CHANNEL_6,
	 .canal_ib = LEDC_CHANNEL_7, .polaridad = +1, .curva = CURVA_RR, .n_curva = N_CURVA(CURVA_RR), .d_min = 0.50f,
	 .nivel_kick_ini = 2},
};

/* ---- Brazo (4 servos, servos_brazo.h): mismo protocolo y misma logica que firmware/brazo_sliders/main.c ---- */
#define N_SERVOS 4
#define IDX_BASE 0
#define IDX_HOMBRO 1
#define IDX_CODO 2
#define IDX_PINZA 3
#define PERIODO_BRAZO_MS 10
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

/* Pines: los que quedan libres en la ESP32 con el cableado actual del robot (no chocan con los de las ruedas/sonares/LED
 * de arriba). Si dejas el servo del hombro enchufado en el conector de la pinza (GPIO22) y la pinza en el del hombro
 * (GPIO19), intercambia los dos numeros de .pin entre las filas "hombro" y "pinza".
 *
 * .tope_max_deg: lo maximo que la ESP32 deja pedir a cada servo (grados nominales: 500 us = 0, 2500 us = 180). El codo llega a
 *   200 (2722 us): pasa de los 2500 us del manual de los MG90S, asi que hay que subirlo de a poco y mirando (P).
 * Limites de arranque: valen solo hasta que la PC mande los suyos (los que guarda en ~/caid/config/brazo_calibracion.json):
 *   los que elegiste en el banco: base 0-180, hombro 10-125, codo 80-200, pinza 100-180 (la pinza cierra apretada en 114).
 * Objetivo inicial: el reposo seguro para el hombro y el codo, el centro (95) para la base. */
static servo_t servos[N_SERVOS] = {
	{.nombre = "base", .pin = GPIO_NUM_18, .canal = LEDC_CHANNEL_0, .tope_max_deg = 180.0f, .min_deg = 0.0f, .max_deg = 180.0f, .objetivo_deg = 95.0f, .actual_deg = 95.0f},
	/* hombro y pinza con los pines intercambiados respecto al cableado de brazo_sliders original: en este robot el
	 * servo del hombro quedo enchufado en el conector de la pinza (GPIO22) y el de la pinza en el del hombro (GPIO19). */
	{.nombre = "hombro", .pin = GPIO_NUM_22, .canal = LEDC_CHANNEL_1, .tope_max_deg = 180.0f, .min_deg = 10.0f, .max_deg = 125.0f, .objetivo_deg = 60.0f, .actual_deg = 60.0f},
	{.nombre = "codo", .pin = GPIO_NUM_21, .canal = LEDC_CHANNEL_2, .tope_max_deg = 200.0f, .min_deg = 80.0f, .max_deg = 200.0f, .objetivo_deg = 180.0f, .actual_deg = 180.0f},
	{.nombre = "pinza", .pin = GPIO_NUM_19, .canal = LEDC_CHANNEL_3, .tope_max_deg = 180.0f, .min_deg = 100.0f, .max_deg = 180.0f, .objetivo_deg = 150.0f, .actual_deg = 150.0f},
};

static volatile float velocidad_brazo_dps = VELOCIDAD_INICIAL_DPS;
static volatile uint32_t t_ultima_orden_brazo_ms = 0;
static uint32_t contador_estado_brazo = 0;
static volatile float pose_segura_hombro = POSE_SEGURA_HOMBRO_DEG;
static volatile float pose_segura_codo = POSE_SEGURA_CODO_DEG;
static volatile int fase_base = FASE_LIBRE;        /* lo escribe solo la tarea de control del brazo */

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

/* Llega una orden de /brazo/cmd (la llama el callback de micro-ROS). No confia en la PC: los limites se dejan dentro del
 * tope de cada servo en servo_poner_limites(), cada angulo se limita en servo_ir_a() y un NaN descarta la orden. */
static void al_recibir_orden_brazo(const float *datos, int n)
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
	velocidad_brazo_dps = (v < 0.0f) ? 0.0f : (v > VELOCIDAD_MAX_DPS ? VELOCIDAD_MAX_DPS : v);
	t_ultima_orden_brazo_ms = ms_ahora();
}

/* Llena buf con el estado del brazo para /brazo/estado y devuelve cuantos numeros puso */
static int armar_estado_brazo(float *buf)
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
	buf[5] = velocidad_brazo_dps;
	buf[6] = (float)(contador_estado_brazo++);
	buf[7] = (float)(ms_ahora() - t_ultima_orden_brazo_ms);
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
		if (pasadas_en_reposo * PERIODO_BRAZO_MS >= ESPERA_REPOSO_MS) {
			fase_base = FASE_GIRANDO;
		}
	} else if (!en_reposo) {                      /* FASE_GIRANDO y el brazo se solto o se movio: la base frena y espera */
		fase_base = FASE_ESPERANDO;
		pasadas_en_reposo = 0;
		base->meta_deg = base->actual_deg;
	}
}

/* Tarea de control del brazo: cada 10 ms decide las metas y mueve los servos hacia ellas. Corre aunque no haya Wi-Fi ni
 * agente, igual que la tarea de control de las ruedas (control_task, mas abajo), e independiente de ella. */
static void tarea_control_brazo(void *arg)
{
	(void)arg;
	TickType_t t = xTaskGetTickCount();
	while (1) {
		aplicar_reposo_seguro();
		servos_ciclo(servos, N_SERVOS, velocidad_brazo_dps, PERIODO_BRAZO_MS / 1000.0f);
		vTaskDelayUntil(&t, pdMS_TO_TICKS(PERIODO_BRAZO_MS));
	}
}

#define TIMEOUT_CREAR_MS 5000       /* espera por entidad al crear (el default es 1000: una rafaga de Wi-Fi dura 1,5-3,5 s) */
#define PING_ESPERA_MS 200
#define PING_CONECTADO_MS 300
#define PING_FALLOS_MAX 6

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}

static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_robot_cmd;
static rcl_subscription_t sub_brazo_cmd;
static rcl_publisher_t pub_ticks;
static rcl_publisher_t pub_estado;
static rcl_publisher_t pub_brazo_estado;
static geometry_msgs__msg__Twist recv_twist;
static std_msgs__msg__Float32MultiArray recv_robot;
static std_msgs__msg__Float32MultiArray recv_brazo;
static std_msgs__msg__Int32MultiArray msg_ticks;
static std_msgs__msg__Float32MultiArray msg_estado;
static std_msgs__msg__Float32MultiArray msg_brazo_estado;
static float recv_robot_buf[24];
static float recv_brazo_buf[24];
static int32_t ticks_buf[N_RUEDAS];
static float estado_buf[N_ESTADO];
static float estado_brazo_buf[23];

static volatile uint32_t ultimo_cmd_ms = 0;
static volatile bool agente_conectado = false;

static volatile float diametro_m = DIAMETRO_RUEDA_M;
static volatile float trocha_m = TROCHA_M;
static volatile float frenado_m = FRENADO_M;
static volatile float frenado_giro_m = FRENADO_GIRO_M;
static volatile int cfg_seq = 0;                 /* configuraciones y curvas aceptadas */

/* Movimiento de distancia: lo pide el callback y lo ejecuta la tarea de control */
static volatile int mov_pedido = 0;
static volatile int mov_abortar = 0;
static volatile float mov_dist_ped_m = 0.0f;
static volatile float mov_vel_ped = 0.0f;
static volatile float mov_ang_ped_rad = 0.0f;   /* giro pedido (+ izquierda) */
static volatile int mov_tipo_ped = 0;           /* 0 distancia, 1 giro (lo fija el callback antes de mov_pedido) */
static volatile int mov_tipo = 0;               /* tipo del movimiento en curso o del ultimo */
static volatile int mov_activo = 0;
static volatile float mov_recorrido_m = 0.0f;
static volatile float mov_final_m = 0.0f;
static volatile int mov_seq = 0;

/* ---- Sonares y freno local ---- */
enum { SON_F = 0, SON_D = 1, SON_I = 2, N_SONARES = 3 };
static sonar_t S[N_SONARES] = {
	{.nombre = "frontal", .trig = GPIO_NUM_5, .echo = GPIO_NUM_13},
	{.nombre = "derecho", .trig = GPIO_NUM_15, .echo = GPIO_NUM_4},
	{.nombre = "izquierdo", .trig = GPIO_NUM_14, .echo = GPIO_NUM_12},
};
static const int SON_SECUENCIA[4] = {SON_F, SON_D, SON_F, SON_I};   /* el frontal cada 120 ms, los laterales cada 240 ms */
static volatile float son_m[N_SONARES] = {-1.0f, -1.0f, -1.0f};    /* ultima medicion en m; -1 si no hubo eco */
static volatile uint32_t son_ms[N_SONARES];                         /* instante (ms) de la ultima medicion terminada */
static volatile uint32_t son_valido_ms[N_SONARES];                  /* instante (ms) del ultimo eco valido */
static volatile uint32_t son_n[N_SONARES];                          /* mediciones terminadas, con o sin eco */
static guardia_t guardia;
static int guardia_ult = GUARDIA_APAGADA;                           /* para avisar por el puerto serie cuando cambia */

/* Un ciclo de 10 ms de los sonares: la misma logica que firmware/pruebas/13_sonares_3.c en modo secuencial. */
static void sonar_ciclo(uint32_t ahora_ms)
{
	static int turno = 0;
	static int64_t ultimo_disparo_us = 0;
	int64_t ahora_us = esp_timer_get_time();
	int ocupado = 0;

	for (int i = 0; i < N_SONARES; i++) {
		if (S[i].esperando) {
			if (sonar_revisar(&S[i])) {
				son_m[i] = isnan(S[i].dist_m) ? -1.0f : S[i].dist_m;
				son_ms[i] = ahora_ms;
				if (!isnan(S[i].dist_m)) {
					son_valido_ms[i] = ahora_ms;
				}
				son_n[i]++;
			}
			ocupado |= S[i].esperando;
		}
	}
	if (ocupado || ahora_us - ultimo_disparo_us < SONAR_RANURA_US) {
		return;
	}
	ultimo_disparo_us = ahora_us;
	sonar_disparar(&S[SON_SECUENCIA[turno]]);
	turno = (turno + 1) % 4;
}

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
	w->nivel_kick = (giro_directo && w->nivel_kick_ini < 2) ? 2 : w->nivel_kick_ini;
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
			{
				const float paso_duty = giro_directo ? 1.0f : PASO_DUTY_PI;

				d = fminf(mag + paso_duty, fmaxf(mag - paso_duty, d));
			}
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

/* Las ruedas de la izquierda (FL, RL) y las de la derecha (FR, RR) por separado: un giro en el lugar */
static void ordenar_lados(float izq, float der)
{
	W[0].objetivo_cmd = W[2].objetivo_cmd = izq;
	W[1].objetivo_cmd = W[3].objetivo_cmd = der;
}

/* Giro en el lugar: izquierda y derecha por separado, con las delanteras algo mas rapido (GIRO_FACTOR_DELANTERAS, tope de velocidad) */
static volatile float giro_factor_del = GIRO_FACTOR_DELANTERAS;   /* se cambia con la orden 9 */

static void ordenar_giro(float izq, float der)
{
	const float f = giro_factor_del;

	W[0].objetivo_cmd = fmaxf(-VEL_MAX_FLANCOS, fminf(VEL_MAX_FLANCOS, izq * f));
	W[1].objetivo_cmd = fmaxf(-VEL_MAX_FLANCOS, fminf(VEL_MAX_FLANCOS, der * f));
	W[2].objetivo_cmd = izq;
	W[3].objetivo_cmd = der;
}

/* Un ciclo de 10 ms de todo el control: movimiento de distancia o de giro, ordenes y las 4 ruedas */
static void control_paso(void)
{
	static int ciclo = 0;
	static int mov_fase = 0;             /* 0 libre, 1 andando, 2 asentando */
	static int mov_dir = 1;
	static int mov_giro = 0;             /* 1: el movimiento en curso es un giro en el lugar */
	static float mov_obj_fl = 0.0f;
	static float mov_frenado_fl = 0.0f;  /* adelanto de frenado de este movimiento, en flancos */
	static float mov_vel = 0.0f;
	static int mov_ciclos = 0;
	static long mov_base[N_RUEDAS] = {0};

	{
		uint32_t ahora_ms = (uint32_t)(esp_timer_get_time() / 1000);

		sonar_ciclo(ahora_ms);

		/* ---- movimiento de distancia ---- */
		if (mov_abortar) {
			mov_abortar = 0;
			mov_pedido = 0;
			mov_fase = 0;
			mov_activo = 0;
			ordenar_todas(0.0f);
		}
		if (mov_pedido && mov_fase == 0) {
			float vel = mov_vel_ped;
			float dist = 0.0f;                       /* metros que tiene que recorrer cada rueda (con signo) */

			mov_pedido = 0;
			mov_giro = mov_tipo_ped;
			if (mov_giro) {
				dist = mov_ang_ped_rad * trocha_m / 2.0f;      /* arco de cada rueda: angulo x radio, con radio = trocha / 2 */
				vel = vel > 0.0f ? vel : VEL_GIRO_DEFECTO;
				mov_frenado_fl = frenado_giro_m * flancos_por_m();
			} else {
				dist = mov_dist_ped_m;
				vel = vel > 0.0f ? vel : VEL_MOV_DEFECTO;
				mov_frenado_fl = frenado_m * flancos_por_m();
			}
			mov_tipo = mov_giro;
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
			if (mov_giro) {
				printf("girar %.1f grados (arco %.3f m = %.1f flancos por rueda) a %.0f flancos/s\n",
				       mov_ang_ped_rad * 57.29578f, dist, mov_obj_fl, mov_vel);
			} else {
				printf("mover %.3f m = %.1f flancos a %.0f flancos/s\n", dist, mov_obj_fl, mov_vel);
			}
		}
		if (mov_fase == 1) {
			float rec = flancos_recorridos(mov_base);
			float resta = mov_obj_fl - mov_frenado_fl - rec;

			ultimo_cmd_ms = ahora_ms;
			mov_recorrido_m = mov_dir * rec / flancos_por_m();
			if (resta <= 0.0f || ++mov_ciclos > MOV_MAX_CICLOS) {
				ordenar_todas(0.0f);
				mov_fase = 2;
				mov_ciclos = 0;
			} else {
				float v_acc = VEL_MIN_FLANCOS + ACEL_FLANCOS_S2 * mov_ciclos * 0.010f;
				float v_dec = sqrtf(2.0f * ACEL_FLANCOS_S2 * resta);
				float v = mov_giro ? mov_vel                       /* giro: de una a la velocidad pedida, sin rampa de subida ni de bajada */
				                   : fmaxf(VEL_MIN_FLANCOS, fminf(mov_vel, fminf(v_acc, v_dec)));

				if (mov_giro) {
					ordenar_giro(-mov_dir * v, mov_dir * v);   /* izquierda para atras y derecha para adelante = giro antihorario */
				} else {
					ordenar_todas(mov_dir * v);
				}
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
				if (mov_giro) {
					printf("giro terminado: arco %.3f m = %.1f grados calculados\n", mov_final_m,
					       mov_final_m / (trocha_m / 2.0f) * 57.29578f);
				} else {
					printf("movimiento terminado: %.3f m calculados\n", mov_final_m);
				}
			}
		}

		giro_directo = (mov_fase == 1 && mov_giro);

		bool vencido = (uint32_t)(ahora_ms - ultimo_cmd_ms) > TIMEOUT_CMD_MS;
		float cmd_v[N_RUEDAS];
		float tope;

		/* Freno local por sonar (solo si la PC lo activo, orden 8): topa la velocidad lineal hacia adelante. No toca los giros
		 * en el lugar ni la marcha atras, ni el duty directo (calibracion). */
		for (int i = 0; i < N_RUEDAS; i++) {
			cmd_v[i] = (vencido || W[i].modo_duty) ? 0.0f : W[i].objetivo_cmd;
		}
		tope = guardia_paso(&guardia, ahora_ms, son_m[SON_F], son_n[SON_F], son_ms[SON_F], son_valido_ms[SON_F],
							GUARDIA_VEL_LENTA);
		if (guardia.estado != guardia_ult) {
			guardia_ult = guardia.estado;
			printf("guardia: estado %d (frente %.2f m)\n", guardia.estado, (double)son_m[SON_F]);
		}
		guardia_limitar(cmd_v, N_RUEDAS, tope, ZONA_MUERTA_FLANCOS);
		if (mov_activo && mov_fase == 1 && !mov_giro && mov_dir > 0 && tope == 0.0f && !mov_abortar) {
			mov_abortar = 1;                    /* un movimiento hacia adelante no se queda esperando delante de un obstaculo */
			printf("guardia: movimiento abortado por obstaculo\n");
		}

		for (int i = 0; i < N_RUEDAS; i++) {
			float cmd = cmd_v[i];

			if (vencido && W[i].modo_duty) {
				W[i].modo_duty = false;
				W[i].duty_directo = 0.0f;
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
		if (n >= 2 && !isnan(d[1]) && !mov_activo && !mov_pedido) {
			mov_dist_ped_m = d[1];
			mov_vel_ped = (n >= 3 && !isnan(d[2])) ? d[2] : 0.0f;
			mov_tipo_ped = 0;                      /* el tipo se fija ANTES de mov_pedido: la tarea de control lo lee al verlo */
			mov_pedido = 1;
		}
		break;
	case 7:                                        /* girar en el lugar: ang_rad con signo (+ izquierda), vel en flancos/s */
		if (n >= 2 && !isnan(d[1]) && fabsf(d[1]) > 0.0f && fabsf(d[1]) <= GIRO_MAX_RAD && !mov_activo && !mov_pedido) {
			mov_ang_ped_rad = d[1];
			mov_vel_ped = (n >= 3 && !isnan(d[2])) ? d[2] : 0.0f;
			mov_tipo_ped = 1;
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
			if (n >= 5 && d[4] >= 0.0f && d[4] < 0.05f) {
				frenado_giro_m = d[4];
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
	case 8:                                        /* freno local por sonar: activa d_stop_m d_lento_m [ciego_s] */
		if (n >= 4 && !isnan(d[1])) {
			int activa = d[1] > 0.5f;
			int antes = guardia.activa;
			float ciego = (n >= 5 && !isnan(d[4])) ? d[4] : guardia.ciego_s;

			if (!guardia_configurar(&guardia, activa, d[2], d[3], ciego)) {
				printf("guardia: valores fuera de rango, no se cambia nada\n");
			} else if (activa != antes) {
				printf("guardia: %s (frena a %.2f m, lenta desde %.2f m, ciego %.0f s)\n", activa ? "ACTIVA" : "apagada",
					   (double)guardia.d_stop_m, (double)guardia.d_lento_m, (double)guardia.ciego_s);
			}
		}
		break;
	case 9:                                        /* delanteras en los giros: factor [nivel_kick_ini de FL y FR] */
		if (n >= 2 && d[1] >= 0.5f && d[1] <= 2.0f) {
			giro_factor_del = d[1];
			if (n >= 3 && d[2] >= 0.0f && d[2] < (float)N_NIVELES_KICK - 0.5f) {
				W[0].nivel_kick_ini = W[1].nivel_kick_ini = (int)(d[2] + 0.5f);
			}
			cfg_seq++;
			printf("giro: delanteras x%.2f, empujon inicial nivel %d\n", (double)giro_factor_del, W[0].nivel_kick_ini);
		}
		break;
	default:
		break;
	}
}

/* Llega una orden de /brazo/cmd (la escucha el executor junto con cmd_vel y robot_cmd) */
static void callback_brazo_cmd(const void *msgin)
{
	const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

	al_recibir_orden_brazo(msg->data.data, (int)msg->data.size);
}

static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;
static rmw_init_options_t *rmw_options;

/* Cada paso de crear_entidades() avisa su nombre y el codigo de rcl_ret_t si falla: con 3 suscripciones y 3
 * publicadores (antes 2 y 2) no sabiamos CUAL de todos estos pasos era el que fallaba. */
static bool paso_ok(rcl_ret_t rc, const char *nombre)
{
	if (rc != RCL_RET_OK) {
		printf("uros: crear_entidades: fallo en %s (rcl_ret_t %d)\n", nombre, (int)rc);
		return false;
	}
	return true;
}

static bool crear_entidades(void)
{
	allocator = rcl_get_default_allocator();

	if (!paso_ok(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator), "support_init")) {
		return false;
	}
	/* Esperar mas que el default de 1 s por entidad: una rafaga de Wi-Fi dura 1,5-3,5 s */
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
	if (rmw_context != NULL) {
		(void)rmw_uros_set_context_entity_creation_session_timeout(rmw_context, TIMEOUT_CREAR_MS);
	}
	if (!paso_ok(rclc_node_init_default(&node, "esp32_motion", "", &support), "node_init")) {
		return false;
	}
	if (!paso_ok(rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	                                       "cmd_vel"), "sub cmd_vel")) {
		return false;
	}
	if (!paso_ok(rclc_subscription_init_best_effort(&sub_robot_cmd, &node,
	                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                       "robot_cmd"), "sub robot_cmd")) {
		return false;
	}
	/* best effort, igual que las demas: sin reenvios. Si se pierde un mensaje, el siguiente (100 ms despues) trae lo mismo
	 * actualizado, igual que en firmware/brazo_sliders/enlace_uros.h */
	if (!paso_ok(rclc_subscription_init_best_effort(&sub_brazo_cmd, &node,
	                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                       "brazo/cmd"), "sub brazo_cmd")) {
		return false;
	}
	if (!paso_ok(rclc_publisher_init_best_effort(&pub_ticks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	                                    "wheel_ticks"), "pub wheel_ticks")) {
		return false;
	}
	if (!paso_ok(rclc_publisher_init_best_effort(&pub_estado, &node,
	                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                    "wheel_state"), "pub wheel_state")) {
		return false;
	}
	if (!paso_ok(rclc_publisher_init_best_effort(&pub_brazo_estado, &node,
	                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	                                    "brazo/estado"), "pub brazo_estado")) {
		return false;
	}
	executor = rclc_executor_get_zero_initialized_executor();
	if (!paso_ok(rclc_executor_init(&executor, &support.context, 3, &allocator), "executor_init")) {
		return false;
	}
	if (!paso_ok(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &recv_twist, &callback_cmd_vel, ON_NEW_DATA),
	             "executor add cmd_vel")) {
		return false;
	}
	if (!paso_ok(rclc_executor_add_subscription(&executor, &sub_robot_cmd, &recv_robot, &callback_robot_cmd, ON_NEW_DATA),
	             "executor add robot_cmd")) {
		return false;
	}
	if (!paso_ok(rclc_executor_add_subscription(&executor, &sub_brazo_cmd, &recv_brazo, &callback_brazo_cmd, ON_NEW_DATA),
	             "executor add brazo_cmd")) {
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
	(void)rcl_publisher_fini(&pub_brazo_estado, &node);
	(void)rcl_subscription_fini(&sub_cmd_vel, &node);
	(void)rcl_subscription_fini(&sub_robot_cmd, &node);
	(void)rcl_subscription_fini(&sub_brazo_cmd, &node);
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
	estado_buf[24] = (float)mov_tipo;
	estado_buf[25] = frenado_giro_m * 1000.0f;
	estado_buf[26] = son_m[SON_F];
	estado_buf[27] = son_m[SON_I];
	estado_buf[28] = son_m[SON_D];
	estado_buf[29] = (float)guardia.estado;
	estado_buf[30] = guardia.d_stop_m;
	estado_buf[31] = guardia.d_lento_m;
	estado_buf[32] = (float)son_n[SON_F];
	estado_buf[33] = guardia.ciego_s;
	(void)rcl_publish(&pub_estado, &msg_estado, NULL);
}

static void publicar_estado_brazo(void)
{
	int n = armar_estado_brazo(estado_brazo_buf);

	msg_brazo_estado.data.size = (size_t)n;
	(void)rcl_publish(&pub_brazo_estado, &msg_brazo_estado, NULL);
}

/* Esperando agente -> conectado -> (agente perdido) -> esperando agente. Nunca se rinde. */
/* Diagnostico del Wi-Fi y de los reinicios (todo sale por el puerto serie) */
static const char *causa_reinicio(esp_reset_reason_t r)
{
	switch (r) {
	case ESP_RST_POWERON:  return "POWERON (se fue y volvio la alimentacion)";
	case ESP_RST_BROWNOUT: return "BROWNOUT (la tension bajo de 2,4 V)";
	case ESP_RST_PANIC:    return "PANIC (el programa se cayo)";
	case ESP_RST_TASK_WDT: case ESP_RST_INT_WDT: case ESP_RST_WDT: return "WATCHDOG (una tarea se colgo)";
	case ESP_RST_SW:       return "SW (reinicio por software)";
	case ESP_RST_EXT:      return "EXT (pin EN)";
	default:               return "otra";
	}
}

static void wifi_evento(void *arg, esp_event_base_t base, int32_t id, void *datos)
{
	(void)arg;
	(void)base;
	if (id == WIFI_EVENT_STA_DISCONNECTED) {
		wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)datos;
		printf("wifi: DESCONECTADO del AP, razon %d (200-209 = baliza perdida / sin respuesta, 2 = autenticacion vencida, 15 = clave)\n", d->reason);
	}
}

void micro_ros_task(void *arg)
{
	(void)arg;
	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
		printf("uros: no se pudieron crear las opciones\n");
		vTaskDelete(NULL);
	}
	if (rcl_init_options_set_domain_id(&init_options, ROS_DOMINIO) != RCL_RET_OK) {
		printf("uros: dominio ROS invalido\n");
		vTaskDelete(NULL);
	}
	printf("uros: dominio ROS %d\n", ROS_DOMINIO);
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

	int64_t ultimo_rssi_us = 0;
	while (1) {
		if (esp_timer_get_time() - ultimo_rssi_us > 10000000) {
			ultimo_rssi_us = esp_timer_get_time();
			wifi_ap_record_t ap;
			if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
				printf("wifi: senal %d dBm (canal %d), agente %s\n", ap.rssi, ap.primary, conectado ? "conectado" : "buscando");
			} else {
				printf("wifi: SIN ASOCIAR al AP\n");
			}
		}
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
					printf("uros: conectado. Escucha /cmd_vel, /robot_cmd y /brazo/cmd; publica /wheel_ticks, /wheel_state y "
					       "/brazo/estado\n");
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
				publicar_estado_brazo();
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

	/* Sonares (GPIO12, el eco izquierdo, tiene que estar en bajo al arrancar) y freno local, apagado hasta que la PC lo active */
	guardia_init(&guardia);
	for (int i = 0; i < N_SONARES; i++) {
		sonar_init(&S[i]);
	}

	/* Brazo: igual que brazo_sliders/main.c, los servos quedan SIN pulso hasta que la PC los active */
	servos_init(servos, N_SERVOS);

	/* Mensajes con memoria estatica: los arreglos de los multiarray apuntan a buffers propios */
	std_msgs__msg__Int32MultiArray__init(&msg_ticks);
	msg_ticks.data.data = ticks_buf;
	msg_ticks.data.size = N_RUEDAS;
	msg_ticks.data.capacity = N_RUEDAS;
	std_msgs__msg__Float32MultiArray__init(&msg_estado);
	msg_estado.data.data = estado_buf;
	msg_estado.data.size = N_ESTADO;
	msg_estado.data.capacity = N_ESTADO;
	std_msgs__msg__Float32MultiArray__init(&recv_robot);
	recv_robot.data.data = recv_robot_buf;
	recv_robot.data.size = 0;
	recv_robot.data.capacity = sizeof(recv_robot_buf) / sizeof(recv_robot_buf[0]);
	std_msgs__msg__Float32MultiArray__init(&recv_brazo);
	recv_brazo.data.data = recv_brazo_buf;
	recv_brazo.data.size = 0;
	recv_brazo.data.capacity = sizeof(recv_brazo_buf) / sizeof(recv_brazo_buf[0]);
	std_msgs__msg__Float32MultiArray__init(&msg_brazo_estado);
	msg_brazo_estado.data.data = estado_brazo_buf;
	msg_brazo_estado.data.size = 0;
	msg_brazo_estado.data.capacity = sizeof(estado_brazo_buf) / sizeof(estado_brazo_buf[0]);

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN
	/* Sin ahorro de energia del Wi-Fi: la ESP32 no duerme entre balizas (menor latencia) */
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	esp_wifi_set_max_tx_power(WIFI_TX_POTENCIA);
	esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_evento, NULL);
#endif
	printf("uros: causa del ultimo reinicio: %s\n", causa_reinicio(esp_reset_reason()));

	xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 6, NULL, 1);
	xTaskCreatePinnedToCore(tarea_control_brazo, "brazo", 4096, NULL, 6, NULL, 1);
	xTaskCreate(micro_ros_task,
			"uros_task",
			CONFIG_MICRO_ROS_APP_STACK,
			NULL,
			CONFIG_MICRO_ROS_APP_TASK_PRIO,
			NULL);
}
