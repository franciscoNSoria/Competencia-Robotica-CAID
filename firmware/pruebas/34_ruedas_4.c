/* Prueba 34: las 4 ruedas con sus encoders. Primero EN EL AIRE (robot sobre un soporte).
 *   FL IA 25 / IB 26 / enc 34    FR IA 17 / IB 33 / enc 36    RL IA 27 / IB 16 / enc 35    RR IA 23 / IB 32 / enc 39
 *
 * Manual: duty por rueda o el mismo para las 4 (si la PC deja de mandar la orden por mas de 600 ms, frena).
 * Verificacion automatica de cableado (orden 5): mueve una rueda por vez (FL, FR, RL, RR) 2,5 s al duty pedido, con 1 s
 * de pausa entre ruedas, y cuenta los flancos de LOS 4 encoders en los ultimos 2 s de cada una. Resultado: una matriz
 * 4 x 4 (fila = rueda movida, columna = encoder). Bien cableado: solo cuenta la diagonal. Si cuenta otra columna, ese
 * encoder esta en el pin de otra rueda. Si no cuenta ninguno, el motor no giro o el encoder no anda.
 * El sentido de giro hay que mirarlo: con duty positivo las 4 tienen que ir hacia adelante (si no, polaridad en ruedas.h).
 * Arranque de las 4 juntas: el elevador de 5 V puede caer; si la ESP32 se reinicia, probar con duty mas bajo.
 *
 * Ordenes (/prueba/cmd = [34, orden, args]):
 *   1 d0 d1 d2 d3   duty por rueda (FL, FR, RL, RR), de -1 a 1       2 d   el mismo duty para las 4
 *   3               cuentas en cero                                   4     parar todo (y cancelar la verificacion)
 *   5 d             verificacion automatica con duty d (0 = 0,6)
 * Estado:  [34, contador, duty FL FR RL RR, flancos FL FR RL RR, flancos/s FL FR RL RR, paso_verificacion,
 *           matriz 4x4 por filas]   paso: -1 sin verificar, 0..3 rueda en prueba, 4 terminada */
#define PRUEBA_ID 34
#define PRUEBA_NOMBRE "4 ruedas con encoders"
#include "prueba_uros.h"
#include "ruedas.h"

#define WATCHDOG_MS 600
#define AUTO_GIRO_MS 2500
#define AUTO_MEDIR_MS 2000
#define AUTO_PAUSA_MS 1000

static volatile int paso_auto = -1;          /* -1 nada, 0..3 rueda, 4 terminada */
static volatile int pedir_auto = 0;
static volatile float duty_auto = 0.6f;
static uint32_t t_paso_ms = 0;
static long base[N_RUEDAS];
static int midiendo = 0;
static float matriz[N_RUEDAS][N_RUEDAS];

static void prueba_init(void)
{
	ruedas_init();
}

static void auto_ciclo(void)
{
	uint32_t t = ms_ahora() - t_paso_ms;
	int i = paso_auto;

	for (int j = 0; j < N_RUEDAS; j++) {
		R[j].duty_obj = (j == i && t < AUTO_GIRO_MS) ? duty_auto : 0.0f;
	}
	if (!midiendo && t >= AUTO_GIRO_MS - AUTO_MEDIR_MS && t < AUTO_GIRO_MS) {
		midiendo = 1;
		for (int j = 0; j < N_RUEDAS; j++) {
			base[j] = R[j].total;
		}
	}
	if (midiendo && t >= AUTO_GIRO_MS) {
		midiendo = 0;
		for (int j = 0; j < N_RUEDAS; j++) {
			matriz[i][j] = (float)(R[j].total - base[j]);
		}
		printf("verificacion: rueda %s -> FL %.0f  FR %.0f  RL %.0f  RR %.0f flancos\n", R[i].nombre,
		       matriz[i][0], matriz[i][1], matriz[i][2], matriz[i][3]);
	}
	if (t >= AUTO_GIRO_MS + AUTO_PAUSA_MS) {
		paso_auto = i + 1;
		t_paso_ms = ms_ahora();
	}
}

static void prueba_ciclo(void)
{
	if (pedir_auto) {
		pedir_auto = 0;
		memset(matriz, 0, sizeof(matriz));
		midiendo = 0;
		paso_auto = 0;
		t_paso_ms = ms_ahora();
	}
	if (paso_auto >= 0 && paso_auto < N_RUEDAS) {
		auto_ciclo();
	} else if (ms_ahora() - ultimo_cmd_ms > WATCHDOG_MS) {
		for (int j = 0; j < N_RUEDAS; j++) {
			R[j].duty_obj = 0.0f;
		}
	}
	for (int j = 0; j < N_RUEDAS; j++) {
		rueda_ciclo(&R[j]);
	}
}

static void prueba_comando(const float *a, int n)
{
	int orden = (int)lroundf(a[0]);
	if (paso_auto >= 0 && paso_auto < N_RUEDAS && orden != 4) {
		return;                             /* durante la verificacion solo se acepta parar */
	}
	switch (orden) {
	case 1:
		for (int j = 0; j < N_RUEDAS && j + 1 < n; j++) {
			R[j].duty_obj = limitar_f(a[j + 1], -1.0f, 1.0f);
		}
		break;
	case 2:
		if (n > 1) {
			for (int j = 0; j < N_RUEDAS; j++) {
				R[j].duty_obj = limitar_f(a[1], -1.0f, 1.0f);
			}
		}
		break;
	case 3:
		ruedas_reiniciar_cuentas();
		break;
	case 4:
		paso_auto = -1;
		ruedas_parar();
		break;
	case 5:
		duty_auto = (n > 1 && fabsf(a[1]) > 0.05f) ? limitar_f(a[1], -1.0f, 1.0f) : 0.6f;
		pedir_auto = 1;
		break;
	default:
		break;
	}
}

static int prueba_estado(float *e)
{
	int k = 2;
	for (int j = 0; j < N_RUEDAS; j++) {
		e[k++] = R[j].duty;
	}
	for (int j = 0; j < N_RUEDAS; j++) {
		e[k++] = (float)R[j].total;
	}
	for (int j = 0; j < N_RUEDAS; j++) {
		e[k++] = R[j].fps;
	}
	e[k++] = (float)paso_auto;
	for (int i = 0; i < N_RUEDAS; i++) {
		for (int j = 0; j < N_RUEDAS; j++) {
			e[k++] = matriz[i][j];
		}
	}
	return k;
}

static void prueba_parar(void)
{
	paso_auto = -1;
	ruedas_parar();
}
