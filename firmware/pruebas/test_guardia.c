/* Pruebas de guardia.h en la PC (sin ESP-IDF).
 *   gcc -Wall -Wextra -Werror -o /tmp/test_guardia firmware/pruebas/test_guardia.c -I firmware/pruebas/comun -lm && /tmp/test_guardia */
#include <stdio.h>
#include <stdlib.h>

#include "guardia.h"

static guardia_t g;
static uint32_t ahora, seq, f_ms, valido_ms;
static float f_m;
static int n_pruebas;

#define VERIFICAR(cond)                                                                                                \
	do {                                                                                                           \
		n_pruebas++;                                                                                           \
		if (!(cond)) {                                                                                         \
			printf("FALLA linea %d: %s\n", __LINE__, #cond);                                               \
			exit(1);                                                                                       \
		}                                                                                                      \
	} while (0)

static void reiniciar(int activa)
{
	guardia_init(&g);
	guardia_configurar(&g, activa, 0.15f, 0.40f, 5.0f);
	ahora = 10000;
	seq = 0;
	f_ms = 0;
	valido_ms = 0;
	f_m = -1.0f;
}

/* una medicion del frontal terminada ahora: d < 0 es "sin eco" */
static void medir(float d)
{
	seq++;
	f_m = d;
	f_ms = ahora;
	if (d >= 0.0f) {
		valido_ms = ahora;
	}
}

static float paso(void)
{
	return guardia_paso(&g, ahora, f_m, seq, f_ms, valido_ms, 25.0f);
}

int main(void)
{
	float lim;

	/* apagada: no limita nunca, aunque haya algo a 5 cm */
	reiniciar(0);
	medir(0.05f);
	VERIFICAR(isinf(paso()) && g.estado == GUARDIA_APAGADA);

	/* libre y lenta */
	reiniciar(1);
	medir(1.0f);
	VERIFICAR(isinf(paso()) && g.estado == GUARDIA_LIBRE);
	ahora += 120;
	medir(0.35f);
	VERIFICAR(paso() == 25.0f && g.estado == GUARDIA_LENTA);
	ahora += 120;
	medir(0.45f);
	VERIFICAR(isinf(paso()) && g.estado == GUARDIA_LIBRE);

	/* frena en d_stop y se libera con 2 mediciones por encima de d_stop + 3 cm */
	reiniciar(1);
	medir(0.30f);
	paso();
	ahora += 120;
	medir(0.14f);
	VERIFICAR(paso() == 0.0f && g.estado == GUARDIA_FRENADA);
	ahora += 120;
	medir(0.16f);                                   /* dentro de la histeresis: sigue frenada */
	VERIFICAR(paso() == 0.0f);
	ahora += 120;
	medir(0.20f);                                   /* primera por encima: todavia frenada */
	VERIFICAR(paso() == 0.0f && g.estado == GUARDIA_FRENADA);
	ahora += 120;
	medir(0.16f);                                   /* vuelve a la histeresis: la cuenta se reinicia */
	VERIFICAR(paso() == 0.0f);
	ahora += 120;
	medir(0.20f);
	VERIFICAR(paso() == 0.0f);
	ahora += 120;
	medir(0.21f);                                   /* segunda seguida: se libera, y a 21 cm queda en zona lenta */
	VERIFICAR(paso() == 25.0f && g.estado == GUARDIA_LENTA);

	/* una medicion sin eco no libera una guardia frenada */
	reiniciar(1);
	medir(0.10f);
	VERIFICAR(paso() == 0.0f);
	for (int i = 0; i < 6; i++) {
		ahora += 120;
		medir(-1.0f);
		VERIFICAR(paso() == 0.0f && g.estado == GUARDIA_FRENADA);
	}

	/* la ultima distancia valida se conserva 600 ms; despues, sin eco = libre */
	reiniciar(1);
	medir(0.30f);
	paso();
	ahora += 300;
	medir(-1.0f);
	VERIFICAR(paso() == 25.0f && g.estado == GUARDIA_LENTA);
	ahora += 400;
	medir(-1.0f);
	VERIFICAR(isinf(paso()) && g.estado == GUARDIA_LIBRE);

	/* sin mediciones nuevas por mas de 400 ms: falla, no avanza */
	reiniciar(1);
	medir(1.0f);
	VERIFICAR(isinf(paso()));
	ahora += 390;
	VERIFICAR(isinf(paso()));
	ahora += 20;
	VERIFICAR(paso() == 0.0f && g.estado == GUARDIA_FALLA);
	medir(1.0f);                                    /* vuelve a medir: se recupera */
	VERIFICAR(isinf(paso()) && g.estado == GUARDIA_LIBRE);

	/* ciego: mediciones sin eco por mas de ciego_s */
	reiniciar(1);
	guardia_configurar(&g, 1, 0.15f, 0.40f, 2.0f);
	medir(1.0f);
	VERIFICAR(isinf(paso()));
	for (int i = 0; i < 16; i++) {                  /* 16 x 120 ms = 1,9 s */
		ahora += 120;
		medir(-1.0f);
		VERIFICAR(isinf(paso()));
	}
	ahora += 120;
	medir(-1.0f);
	VERIFICAR(paso() == 0.0f && g.estado == GUARDIA_FALLA);
	medir(0.9f);
	VERIFICAR(isinf(paso()));

	/* la configuracion rechaza lo absurdo y no cambia nada */
	reiniciar(1);
	VERIFICAR(guardia_configurar(&g, 1, 0.20f, 0.50f, 5.0f) == 1 && g.d_stop_m == 0.20f && g.d_lento_m == 0.50f);
	VERIFICAR(guardia_configurar(&g, 1, 0.02f, 0.50f, 5.0f) == 0);
	VERIFICAR(guardia_configurar(&g, 1, 0.20f, 0.22f, 5.0f) == 0);
	VERIFICAR(guardia_configurar(&g, 1, 0.20f, 3.00f, 5.0f) == 0);
	VERIFICAR(guardia_configurar(&g, 1, NAN, 0.50f, 5.0f) == 0);
	VERIFICAR(guardia_configurar(&g, 1, 0.20f, 0.50f, 0.5f) == 0);
	VERIFICAR(g.d_stop_m == 0.20f && g.d_lento_m == 0.50f && g.ciego_s == 5.0f && g.activa == 1);
	VERIFICAR(guardia_configurar(&g, 0, 0.20f, 0.50f, 5.0f) == 1 && g.activa == 0);

	/* guardia_limitar */
	{
		float a[4] = {40, 40, 40, 40};
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == 0 && a[1] == 0 && a[2] == 0 && a[3] == 0);
	}
	{
		float a[4] = {40, 40, 40, 40};
		guardia_limitar(a, 4, 25.0f, 12.5f);
		VERIFICAR(a[0] == 25 && a[1] == 25 && a[2] == 25 && a[3] == 25);
	}
	{
		float a[4] = {-40, 40, -40, 40};                /* giro en el lugar: no se toca */
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == -40 && a[1] == 40 && a[2] == -40 && a[3] == 40);
	}
	{
		float a[4] = {30, 50, 30, 50};                  /* arco hacia el obstaculo: queda en 0 */
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == 0 && a[1] == 0 && a[2] == 0 && a[3] == 0);
	}
	{
		float a[4] = {10, 70, 10, 70};                  /* arco cerrado: se vuelve un giro en el lugar */
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == -30 && a[1] == 30 && a[2] == -30 && a[3] == 30);
	}
	{
		float a[4] = {-40, -40, -40, -40};              /* marcha atras: no se toca */
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == -40 && a[3] == -40);
	}
	{
		float a[4] = {40, 40, 40, 40};
		guardia_limitar(a, 4, INFINITY, 12.5f);
		VERIFICAR(a[0] == 40 && a[3] == 40);
	}
	{
		float a[4] = {0, 50, 0, 0};                     /* una sola rueda: tampoco avanza */
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == 0 && a[1] == 0 && a[2] == 0 && a[3] == 0);
	}
	{
		float a[4] = {39.1234f, 39.1234f, 39.1234f, 39.1234f};   /* el redondeo no deja una velocidad minuscula */
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == 0.0f && a[1] == 0.0f && a[2] == 0.0f && a[3] == 0.0f);
	}
	{
		float a[4] = {0, 0, 0, 0};
		guardia_limitar(a, 4, 0.0f, 12.5f);
		VERIFICAR(a[0] == 0 && a[3] == 0);
	}

	/* barrido: acercarse a 0,20 m/s hacia una pared (un cuadro del frontal cada 120 ms); nunca pasa de d_stop */
	reiniciar(1);
	{
		float d = 1.00f;
		int frenado = 0;
		float v = 40.0f;                                /* flancos/s pedidos */

		for (int i = 0; i < 80; i++) {
			ahora += 120;
			medir(d);
			lim = paso();
			float c[4] = {v, v, v, v};

			guardia_limitar(c, 4, lim, 12.5f);
			if (c[0] == 0.0f) {
				frenado = 1;
			}
			d -= (c[0] / 197.0f) * 0.120f;          /* 197 flancos/m con D = 64,6 mm */
			VERIFICAR(d > 0.05f);
		}
		VERIFICAR(frenado && g.estado == GUARDIA_FRENADA);
		VERIFICAR(d > 0.10f && d <= 0.16f);
	}

	printf("guardia.h: %d verificaciones OK\n", n_pruebas);
	return 0;
}
