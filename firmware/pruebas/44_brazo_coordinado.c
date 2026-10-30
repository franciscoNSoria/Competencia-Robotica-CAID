/* Prueba 44: los 4 servos del brazo coordinados, con poses y la secuencia de agarre lateral.
 *   base GPIO18   hombro GPIO19   codo GPIO21   pinza GPIO22   (marron GND, rojo +5 V del riel de servos, naranja senal)
 *
 * Poses (us de base, hombro, codo, pinza). Las de hombro y codo salen de lo medido (hombro 1300 arriba / 1700 abajo,
 * codo 1700 retraido / 1000 estirado, M); base y pinza son PROPUESTAS (P): ajustarlas con las pruebas 40 y 43, y
 * guardarlas desde la PC (quedan en ~/caid/config/brazo_poses.json y se mandan solas cada vez que la ESP32 conecta).
 *   0 reposo            brazo atras, abajo y retraido (apoyado en el 2.o piso), pinza en el medio
 *   1 arriba            sube el hombro con el codo retraido                 ("subir y retraer el brazo")
 *   2 lateral           gira la base al costado izquierdo                   ("girar hacia el lateral")
 *   3 abierta           abre la pinza                                       ("abrir la mano")
 *   4 extendida         estira el codo y baja un poco el hombro              ("extender el codo")
 *   5 cerrada           cierra la pinza                                     ("cerrar la mano")
 *   6 retraida          retrae el codo                                      ("retraer el codo")
 *   7 levantada         sube el hombro                                      ("y levantar")
 *   8 atras             gira la base hacia atras con el cubo
 *   9 reposo con cubo   baja y apoya el brazo en el 2.o piso, pinza cerrada
 * Secuencia (orden 11): 1, 2, 3, 4, 5, 6, 7, 8, 9. Paso a paso con la orden 12.
 * Cada paso mueve los 4 servos juntos (todos llegan a la vez, con arranque y frenado suaves) en T ms (por defecto 2000);
 * si para eso alguno pasaria de 400 us/s, el paso dura mas.
 * Si el agente se pierde, el brazo se queda quieto donde esta.
 *
 * Ordenes (/prueba/cmd = [44, orden, args]):
 *   1           activar los 4 en la pose de reposo (EL BRAZO TIENE QUE ESTAR EN EL REPOSO)
 *   2 j us      servo j (0 base, 1 hombro, 2 codo, 3 pinza) a us       3 j delta  mover servo j delta us
 *   4           soltar los 4 (sin pulsos: el brazo cae)                 5 v        velocidad de los movimientos manuales (us/s)
 *   6 j lo hi   limites del servo j                                     10 k       ir a la pose k
 *   11          secuencia completa                                      12         siguiente paso de la secuencia
 *   13          detener (se queda donde esta y cancela la secuencia)    14 k j us  cambiar la pose k, servo j
 *   15 ms       duracion de cada paso                                   16 k       guardar los pulsos actuales en la pose k
 *   17 ms       pausa entre pasos de la secuencia                       18 k b h c p  pose k completa (us de los 4)
 * Estado:  [44, contador, (activo, pulso, objetivo) x 4 servos, paso_secuencia (-1 ninguno), secuencia_corriendo,
 *           moviendo, pose_destino, duracion_ms, pausa_ms, cambios_de_poses] */
#define PRUEBA_ID 44
#define PRUEBA_NOMBRE "brazo coordinado"
#include "prueba_uros.h"
#include "servos.h"

#define N_SERVOS 4
#define N_POSES 10
#define VEL_MAX_COORD 400.0f                 /* us/s */

static servo_t S[N_SERVOS] = {
	{.nombre = "base", .pin = GPIO_NUM_18, .canal = LEDC_CHANNEL_0, .min_us = 1000, .max_us = 2000, .reposo_us = 1500},
	{.nombre = "hombro", .pin = GPIO_NUM_19, .canal = LEDC_CHANNEL_1, .min_us = 1300, .max_us = 1700, .reposo_us = 1700},
	{.nombre = "codo", .pin = GPIO_NUM_21, .canal = LEDC_CHANNEL_2, .min_us = 1000, .max_us = 1700, .reposo_us = 1700},
	{.nombre = "pinza", .pin = GPIO_NUM_22, .canal = LEDC_CHANNEL_3, .min_us = 1000, .max_us = 2000, .reposo_us = 1500},
};

/*                               base  hombro  codo  pinza */
static int POSE[N_POSES][N_SERVOS] = {
	{1500, 1700, 1700, 1500},   /* 0 reposo */
	{1500, 1300, 1700, 1500},   /* 1 arriba */
	{1000, 1300, 1700, 1500},   /* 2 lateral (izquierdo, P) */
	{1000, 1300, 1700, 1900},   /* 3 abierta (P) */
	{1000, 1450, 1300, 1900},   /* 4 extendida (P) */
	{1000, 1450, 1300, 1100},   /* 5 cerrada (P) */
	{1000, 1450, 1700, 1100},   /* 6 retraida */
	{1000, 1300, 1700, 1100},   /* 7 levantada */
	{1500, 1300, 1700, 1100},   /* 8 atras */
	{1500, 1700, 1700, 1100},   /* 9 reposo con cubo */
};
static const int SECUENCIA[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
#define N_SEC ((int)(sizeof(SECUENCIA) / sizeof(SECUENCIA[0])))

static volatile int duracion_ms = 2000;
static volatile int pausa_ms = 500;
static volatile int cambios = 0;
static volatile int paso = -1;               /* indice en SECUENCIA del ultimo paso empezado */
static volatile int corriendo = 0;           /* secuencia completa en marcha */
static volatile int pose_destino = -1;
static volatile int pedido_pose = -1;        /* lo pide el callback, lo ejecuta el ciclo */
static volatile int pedido_detener = 0;

static volatile int moviendo = 0;
static float p0[N_SERVOS], p1[N_SERVOS];
static uint32_t t0_ms, t_mov_ms, fin_ms;

static int todos_activos(void)
{
	for (int j = 0; j < N_SERVOS; j++) {
		if (!S[j].activo) {
			return 0;
		}
	}
	return 1;
}

static void empezar_pose(int k)
{
	float tmax = 0.0f;
	if (!todos_activos()) {
		printf("brazo: primero activar los 4 servos (orden 1)\n");
		corriendo = 0;
		return;
	}
	for (int j = 0; j < N_SERVOS; j++) {
		p0[j] = S[j].pulso_us;
		p1[j] = (float)servo_limitar(&S[j], POSE[k][j]);
		float t = fabsf(p1[j] - p0[j]) / VEL_MAX_COORD * 1000.0f;
		if (t > tmax) {
			tmax = t;
		}
	}
	t_mov_ms = (uint32_t)(tmax > duracion_ms ? tmax : duracion_ms);
	t0_ms = ms_ahora();
	moviendo = 1;
	pose_destino = k;
	printf("brazo: a la pose %d en %u ms\n", k, (unsigned)t_mov_ms);
}

static void prueba_init(void)
{
	servos_init(S, N_SERVOS);
}

static void prueba_ciclo(void)
{
	if (pedido_detener) {
		pedido_detener = 0;
		moviendo = 0;
		corriendo = 0;
		for (int j = 0; j < N_SERVOS; j++) {
			S[j].objetivo_us = (int)lroundf(S[j].pulso_us);
		}
	}
	if (pedido_pose >= 0 && !moviendo) {
		int k = pedido_pose;
		pedido_pose = -1;
		empezar_pose(k);
	}
	if (moviendo) {
		float s = (float)(ms_ahora() - t0_ms) / (float)t_mov_ms;
		if (s >= 1.0f) {
			s = 1.0f;
		}
		float u = s * s * (3.0f - 2.0f * s);          /* arranque y frenado suaves */
		for (int j = 0; j < N_SERVOS; j++) {
			S[j].pulso_us = p0[j] + (p1[j] - p0[j]) * u;
			S[j].objetivo_us = (int)lroundf(S[j].pulso_us);
			servo_escribir(&S[j], S[j].pulso_us);
		}
		if (s >= 1.0f) {
			moviendo = 0;
			fin_ms = ms_ahora();
		}
	} else {
		for (int j = 0; j < N_SERVOS; j++) {
			servo_ciclo(&S[j], 0.010f);
		}
		if (corriendo && ms_ahora() - fin_ms >= (uint32_t)pausa_ms) {
			if (paso + 1 < N_SEC) {
				paso = paso + 1;
				empezar_pose(SECUENCIA[paso]);
			} else {
				corriendo = 0;
				printf("brazo: secuencia terminada\n");
			}
		}
	}
}

static int servo_valido(float x)
{
	int j = (int)lroundf(x);
	return j >= 0 && j < N_SERVOS ? j : -1;
}

static int pose_valida(float x)
{
	int k = (int)lroundf(x);
	return k >= 0 && k < N_POSES ? k : -1;
}

static void prueba_comando(const float *a, int n)
{
	int orden = (int)lroundf(a[0]);
	int j = n > 1 ? servo_valido(a[1]) : -1;
	int k = n > 1 ? pose_valida(a[1]) : -1;

	switch (orden) {
	case 1:
		for (int i = 0; i < N_SERVOS; i++) {
			S[i].reposo_us = POSE[0][i];
			servo_activar(&S[i], POSE[0][i]);
		}
		paso = -1;
		break;
	case 2:
		if (j >= 0 && n > 2 && !moviendo) {
			servo_ir(&S[j], (int)lroundf(a[2]));
		}
		break;
	case 3:
		if (j >= 0 && n > 2 && !moviendo) {
			servo_ir(&S[j], S[j].objetivo_us + (int)lroundf(a[2]));
		}
		break;
	case 4:
		pedido_detener = 1;
		for (int i = 0; i < N_SERVOS; i++) {
			servo_soltar(&S[i]);
		}
		break;
	case 5:
		if (n > 1) {
			for (int i = 0; i < N_SERVOS; i++) {
				S[i].vel = limitar_f(a[1], 20.0f, 2000.0f);
			}
		}
		break;
	case 6:
		if (j >= 0 && n > 3) {
			servo_set_limites(&S[j], (int)lroundf(a[2]), (int)lroundf(a[3]));
		}
		break;
	case 10:
		if (k >= 0) {
			corriendo = 0;
			pedido_pose = k;
		}
		break;
	case 11:
		paso = -1;
		fin_ms = ms_ahora() - (uint32_t)pausa_ms;
		corriendo = 1;
		break;
	case 12:
		if (!moviendo && !corriendo) {
			paso = (paso + 1 < N_SEC) ? paso + 1 : 0;
			pedido_pose = SECUENCIA[paso];
		}
		break;
	case 13:
		pedido_detener = 1;
		break;
	case 14:
		if (k >= 0 && n > 3) {
			int jj = servo_valido(a[2]);
			if (jj >= 0) {
				POSE[k][jj] = (int)limitar_f(a[3], SERVO_ABS_MIN_US, SERVO_ABS_MAX_US);
				cambios++;
			}
		}
		break;
	case 15:
		if (n > 1) {
			duracion_ms = (int)limitar_f(a[1], 300.0f, 15000.0f);
		}
		break;
	case 16:
		if (k >= 0) {
			for (int i = 0; i < N_SERVOS; i++) {
				POSE[k][i] = (int)lroundf(S[i].pulso_us);
			}
			cambios++;
		}
		break;
	case 17:
		if (n > 1) {
			pausa_ms = (int)limitar_f(a[1], 0.0f, 10000.0f);
		}
		break;
	case 18:
		if (k >= 0 && n > 5) {
			for (int i = 0; i < N_SERVOS; i++) {
				POSE[k][i] = (int)limitar_f(a[2 + i], SERVO_ABS_MIN_US, SERVO_ABS_MAX_US);
			}
			cambios++;
		}
		break;
	default:
		break;
	}
}

static int prueba_estado(float *e)
{
	int k = 2;
	for (int j = 0; j < N_SERVOS; j++) {
		e[k++] = (float)S[j].activo;
		e[k++] = S[j].pulso_us;
		e[k++] = (float)S[j].objetivo_us;
	}
	e[k++] = (float)paso;
	e[k++] = (float)corriendo;
	e[k++] = (float)moviendo;
	e[k++] = (float)pose_destino;
	e[k++] = (float)duracion_ms;
	e[k++] = (float)pausa_ms;
	e[k++] = (float)cambios;
	return k;
}

static void prueba_parar(void)
{
	pedido_detener = 1;
}
