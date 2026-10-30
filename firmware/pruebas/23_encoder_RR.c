/* Prueba 23: encoder de la rueda RR solo (trasera derecha). Los motores quedan en 0: se gira la rueda A MANO.
 * Encoder RR: D0 al GPIO39; VCC del FC-03 a 3,3 V (pin 3V3 de la ESP32), GND comun. 40 flancos = 1 vuelta.
 * Tambien publica las cuentas de los otros 3 encoders: si al girar esta rueda cuenta otro, los cables estan cruzados.
 *
 * Ordenes (/prueba/cmd = [23, orden]):  1  cuentas en cero
 * Estado:  [23, contador, flancos_RR, flancos_por_s_RR, vueltas_RR, flancos FL, FR, RL, RR] */
#define PRUEBA_ID 23
#define PRUEBA_NOMBRE "encoder RR"
#include "prueba_uros.h"
#include "ruedas.h"

#define SEL RR

static void prueba_init(void)
{
	ruedas_init();
}

static void prueba_ciclo(void)
{
	for (int i = 0; i < N_RUEDAS; i++) {
		R[i].duty_obj = 0.0f;               /* los motores no se mueven en esta prueba */
		rueda_ciclo(&R[i]);
	}
}

static void prueba_comando(const float *a, int n)
{
	(void)n;
	if ((int)lroundf(a[0]) == 1) {
		ruedas_reiniciar_cuentas();
	}
}

static int prueba_estado(float *e)
{
	int k = 2;
	e[k++] = (float)R[SEL].total;
	e[k++] = R[SEL].fps;
	e[k++] = (float)R[SEL].total / FLANCOS_POR_VUELTA;
	for (int i = 0; i < N_RUEDAS; i++) {
		e[k++] = (float)R[i].total;
	}
	return k;
}

static void prueba_parar(void)
{
	ruedas_parar();
}
