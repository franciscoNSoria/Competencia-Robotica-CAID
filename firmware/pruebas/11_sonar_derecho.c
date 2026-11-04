/* Prueba 11: sonar derecho solo. TRIG GPIO15, ECHO GPIO4.
 * Mide cada 60 ms y publica la ultima distancia, la mediana de las ultimas 5 y las cuentas de mediciones validas y
 * perdidas. No mueve nada.
 * Conexion del sensor: VCC a 5 V (o 3,3 V si es un HC-SR04P / RCWL-9610A), GND comun, TRIG directo al GPIO15,
 * ECHO al GPIO4 a traves del divisor 2,2 k / 3,3 k si el sensor esta a 5 V.
 *
 * Ordenes (/prueba/cmd = [11, orden]):  1  poner las cuentas en cero
 * Estado:  [11, contador, distancia_m (-1 sin eco), ancho_eco_us, mediana_m (-1 si no hay), validas, perdidas] */
#define PRUEBA_ID 11
#define PRUEBA_NOMBRE "sonar derecho"
#include "prueba_uros.h"
#include "sonar.h"

#define PERIODO_DISPARO_US 60000

static sonar_t S = {.nombre = "derecho", .trig = GPIO_NUM_15, .echo = GPIO_NUM_4};
static int64_t ultimo_disparo_us = 0;

static void prueba_init(void)
{
	sonar_init(&S);
}

static void prueba_ciclo(void)
{
	int64_t ahora = esp_timer_get_time();
	if (S.esperando) {
		sonar_revisar(&S);
	} else if (ahora - ultimo_disparo_us >= PERIODO_DISPARO_US) {
		ultimo_disparo_us = ahora;
		sonar_disparar(&S);
	}
}

static void prueba_comando(const float *a, int n)
{
	(void)n;
	if ((int)lroundf(a[0]) == 1) {
		sonar_reiniciar_cuentas(&S);
	}
}

static int prueba_estado(float *e)
{
	float med = sonar_mediana(&S);
	e[2] = isnan(S.dist_m) ? -1.0f : S.dist_m;
	e[3] = (float)S.ultimo_ancho_us;
	e[4] = isnan(med) ? -1.0f : med;
	e[5] = (float)S.validas;
	e[6] = (float)S.perdidas;
	return 7;
}

static void prueba_parar(void)
{
}
