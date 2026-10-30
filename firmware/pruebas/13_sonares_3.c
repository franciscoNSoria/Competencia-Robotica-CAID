/* Prueba 13: los 3 sonares juntos.
 *   frontal TRIG GPIO5 / ECHO GPIO13   derecho TRIG GPIO15 / ECHO GPIO4   izquierdo TRIG GPIO14 / ECHO GPIO12
 * Modo 0 (por defecto), secuencial: dispara uno por vez cada 60 ms (frontal, derecho, izquierdo), asi un eco no le llega
 * a otro sensor. Cada sensor se actualiza cada 180 ms. Es como se van a usar en la competencia.
 * Modo 1, simultaneo: los 3 a la vez cada 60 ms. Sirve para ver si se interfieren: si en modo 1 aparecen distancias
 * raras que en modo 0 no aparecen, hay interferencia (cross-talk).
 *
 * Ordenes (/prueba/cmd = [13, orden, args]):  1  cuentas en cero     2 m  modo (0 secuencial, 1 simultaneo)
 * Estado:  [13, contador, modo, frontal: dist mediana validas perdidas, derecho: ..., izquierdo: ...]
 *          (distancias en m, -1 sin eco) */
#define PRUEBA_ID 13
#define PRUEBA_NOMBRE "3 sonares"
#include "prueba_uros.h"
#include "sonar.h"

#define PERIODO_DISPARO_US 60000

static sonar_t S[3] = {
	{.nombre = "frontal", .trig = GPIO_NUM_5, .echo = GPIO_NUM_13},
	{.nombre = "derecho", .trig = GPIO_NUM_15, .echo = GPIO_NUM_4},
	{.nombre = "izquierdo", .trig = GPIO_NUM_14, .echo = GPIO_NUM_12},
};
static volatile int modo = 0;
static int turno = 0;
static int64_t ultimo_disparo_us = 0;

static void prueba_init(void)
{
	for (int i = 0; i < 3; i++) {
		sonar_init(&S[i]);
	}
}

static void prueba_ciclo(void)
{
	int64_t ahora = esp_timer_get_time();
	int ocupado = 0;

	for (int i = 0; i < 3; i++) {
		if (S[i].esperando) {
			sonar_revisar(&S[i]);
			ocupado |= S[i].esperando;
		}
	}
	if (ocupado || ahora - ultimo_disparo_us < PERIODO_DISPARO_US) {
		return;
	}
	ultimo_disparo_us = ahora;
	if (modo == 1) {
		for (int i = 0; i < 3; i++) {
			sonar_disparar(&S[i]);
		}
	} else {
		sonar_disparar(&S[turno]);
		turno = (turno + 1) % 3;
	}
}

static void prueba_comando(const float *a, int n)
{
	int orden = (int)lroundf(a[0]);
	if (orden == 1) {
		for (int i = 0; i < 3; i++) {
			sonar_reiniciar_cuentas(&S[i]);
		}
	} else if (orden == 2 && n > 1) {
		modo = a[1] > 0.5f ? 1 : 0;
		for (int i = 0; i < 3; i++) {
			sonar_reiniciar_cuentas(&S[i]);
		}
	}
}

static int prueba_estado(float *e)
{
	int k = 2;
	e[k++] = (float)modo;
	for (int i = 0; i < 3; i++) {
		float med = sonar_mediana(&S[i]);
		e[k++] = isnan(S[i].dist_m) ? -1.0f : S[i].dist_m;
		e[k++] = isnan(med) ? -1.0f : med;
		e[k++] = (float)S[i].validas;
		e[k++] = (float)S[i].perdidas;
	}
	return k;
}

static void prueba_parar(void)
{
}
