/* Prueba 33: motor de la rueda RR solo (trasera derecha), con su encoder. LA RUEDA TIENE QUE ESTAR EN EL AIRE (robot sobre
 * un soporte) hasta saber que gira para el lado correcto.
 * Rueda RR: IA (adelante) GPIO23, IB (atras) GPIO32, encoder D0 GPIO39. Las otras 3 ruedas quedan en 0.
 * El duty sube y baja con rampa (0 a 100 % en 0,5 s) y al invertir pasa por 0. Si la PC deja de mandar la orden por mas
 * de 600 ms, el motor frena (la herramienta de la PC la repite 10 veces por segundo).
 * Duty de arranque medido en el banco: 0,36-0,50 segun la rueda (con la rueda en el aire).
 *
 * Ordenes (/prueba/cmd = [33, orden, args]):  1 d  duty de -1 a 1 (positivo = adelante)   2  cuentas en cero   3  parar
 * Estado:  [33, contador, duty_pedido, duty_actual, flancos_RR, flancos_por_s_RR, flancos FL FR RL RR, flancos/s FL FR RL RR] */
#define PRUEBA_ID 33
#define PRUEBA_NOMBRE "motor RR"
#include "prueba_uros.h"
#include "ruedas.h"

#define SEL RR
#define WATCHDOG_MS 600

static void prueba_init(void)
{
	ruedas_init();
}

static void prueba_ciclo(void)
{
	if (ms_ahora() - ultimo_cmd_ms > WATCHDOG_MS) {
		R[SEL].duty_obj = 0.0f;
	}
	for (int i = 0; i < N_RUEDAS; i++) {
		if (i != SEL) {
			R[i].duty_obj = 0.0f;
		}
		rueda_ciclo(&R[i]);
	}
}

static void prueba_comando(const float *a, int n)
{
	int orden = (int)lroundf(a[0]);
	if (orden == 1 && n > 1) {
		R[SEL].duty_obj = limitar_f(a[1], -1.0f, 1.0f);
	} else if (orden == 2) {
		ruedas_reiniciar_cuentas();
	} else if (orden == 3) {
		R[SEL].duty_obj = 0.0f;
	}
}

static int prueba_estado(float *e)
{
	int k = 2;
	e[k++] = R[SEL].duty_obj;
	e[k++] = R[SEL].duty;
	e[k++] = (float)R[SEL].total;
	e[k++] = R[SEL].fps;
	for (int i = 0; i < N_RUEDAS; i++) {
		e[k++] = (float)R[i].total;
	}
	for (int i = 0; i < N_RUEDAS; i++) {
		e[k++] = R[i].fps;
	}
	return k;
}

static void prueba_parar(void)
{
	ruedas_parar();
}
