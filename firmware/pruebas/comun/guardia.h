/* Freno local con el sonar frontal ("guardia"). C puro, sin ESP-IDF: se prueba en la PC con firmware/pruebas/test_guardia.c.
 *
 * La PC decide hacia donde ir, pero el freno no puede depender del Wi-Fi (hay huecos de 200-400 ms entre ordenes). Cada 10 ms
 * el control de las ruedas llama a guardia_paso() con lo que ve el sonar frontal y recibe el tope de la velocidad lineal
 * hacia adelante, en flancos/s (INFINITY = sin tope):
 *
 *   apagada  la PC no la activo                                                     sin tope
 *   libre    d >= d_lento                                                           sin tope
 *   lenta    d_stop < d < d_lento                                                   tope de vel_lenta
 *   frenada  d <= d_stop                                                            0: no avanza (girar y retroceder si)
 *   falla    sin medicion terminada en 400 ms, o sin ningun eco hace mas de ciego_s 0: no avanza
 *
 * Una vez frenada se libera con 2 mediciones validas seguidas por encima de d_stop + 3 cm. Una medicion sin eco no libera:
 * un obstaculo muy cerca o en angulo tambien da "sin eco". Tampoco borra de golpe la ultima distancia valida: se conserva
 * 600 ms para la zona lenta.
 *
 * guardia_limitar() aplica el tope a las 4 ruedas: resta el exceso de velocidad lineal a las ruedas que tienen orden. Un giro
 * en el lugar no se toca (su velocidad lineal es 0); un arco hacia el obstaculo se vuelve un giro o se detiene. */
#ifndef GUARDIA_H
#define GUARDIA_H

#include <math.h>
#include <stdint.h>

#define GUARDIA_HISTERESIS_M 0.03f
#define GUARDIA_FRESCO_MS 400
#define GUARDIA_RETENCION_MS 600

enum { GUARDIA_APAGADA = 0, GUARDIA_LIBRE = 1, GUARDIA_LENTA = 2, GUARDIA_FRENADA = 3, GUARDIA_FALLA = 4 };

typedef struct {
	/* configuracion: la escribe el callback de /robot_cmd (valores de 32 bits: lectura atomica en la tarea de control) */
	volatile int activa;
	volatile float d_stop_m;
	volatile float d_lento_m;
	volatile float ciego_s;
	/* estado: solo lo toca guardia_paso() */
	int estado;
	int frenada;
	int libres;
	uint32_t seq_visto;
	int hay_vigente;
	float d_vigente_m;
	uint32_t t_vigente_ms;
} guardia_t;

static void guardia_init(guardia_t *g)
{
	g->activa = 0;
	g->d_stop_m = 0.15f;
	g->d_lento_m = 0.40f;
	g->ciego_s = 5.0f;
	g->estado = GUARDIA_APAGADA;
	g->frenada = 0;
	g->libres = 0;
	g->seq_visto = 0;
	g->hay_vigente = 0;
	g->d_vigente_m = 0.0f;
	g->t_vigente_ms = 0;
}

/* Devuelve 1 si acepta los valores y 0 si alguno esta fuera de rango (en ese caso no cambia nada). */
static int guardia_configurar(guardia_t *g, int activa, float d_stop_m, float d_lento_m, float ciego_s)
{
	if (isnan(d_stop_m) || isnan(d_lento_m) || isnan(ciego_s) || d_stop_m < 0.05f || d_stop_m > 0.60f ||
	    d_lento_m < d_stop_m + 0.05f || d_lento_m > 2.0f || ciego_s < 1.0f || ciego_s > 600.0f) {
		return 0;
	}
	g->d_stop_m = d_stop_m;
	g->d_lento_m = d_lento_m;
	g->ciego_s = ciego_s;
	g->activa = activa ? 1 : 0;
	return 1;
}

/* Llamar cada ciclo de control (10 ms).
 *   f_m          ultima medicion del frontal, en m; negativa si no hubo eco
 *   f_seq        cuenta de mediciones terminadas del frontal (con o sin eco): sube 1 por medicion
 *   f_ms         instante de la ultima medicion terminada (ms, el mismo reloj que ahora_ms; nunca posterior a ahora_ms)
 *   f_valido_ms  instante del ultimo eco valido
 *   vel_lenta    tope en la zona lenta (flancos/s)
 * Devuelve el tope de la velocidad lineal hacia adelante en flancos/s, o INFINITY si no hay tope. */
static float guardia_paso(guardia_t *g, uint32_t ahora_ms, float f_m, uint32_t f_seq, uint32_t f_ms,
                          uint32_t f_valido_ms, float vel_lenta)
{
	int nueva = (f_seq != g->seq_visto);

	g->seq_visto = f_seq;
	if (!g->activa) {
		g->estado = GUARDIA_APAGADA;
		g->frenada = 0;
		g->libres = 0;
		g->hay_vigente = 0;
		return INFINITY;
	}
	if (nueva && f_m >= 0.0f) {
		g->d_vigente_m = f_m;
		g->t_vigente_ms = f_ms;
		g->hay_vigente = 1;
		if (f_m <= g->d_stop_m) {
			g->frenada = 1;
			g->libres = 0;
		} else if (g->frenada) {
			if (f_m > g->d_stop_m + GUARDIA_HISTERESIS_M) {
				if (++g->libres >= 2) {
					g->frenada = 0;
					g->libres = 0;
				}
			} else {
				g->libres = 0;
			}
		}
	}
	int viejo = (uint32_t)(ahora_ms - f_ms) > (uint32_t)GUARDIA_FRESCO_MS;
	int ciego = (uint32_t)(ahora_ms - f_valido_ms) > (uint32_t)(g->ciego_s * 1000.0f);

	if (viejo || ciego) {
		g->estado = GUARDIA_FALLA;
		return 0.0f;
	}
	if (g->frenada) {
		g->estado = GUARDIA_FRENADA;
		return 0.0f;
	}
	if (g->hay_vigente && (uint32_t)(ahora_ms - g->t_vigente_ms) <= (uint32_t)GUARDIA_RETENCION_MS &&
	    g->d_vigente_m < g->d_lento_m) {
		g->estado = GUARDIA_LENTA;
		return vel_lenta;
	}
	g->estado = GUARDIA_LIBRE;
	return INFINITY;
}

/* c[]: velocidad pedida de cada rueda en flancos/s (+ es adelante). lim: tope de la velocidad lineal (el promedio de las n
 * ruedas). zona_muerta: lo que quede por debajo de esto, en valor absoluto, pasa a 0 (el PI no sostiene menos). */
static void guardia_limitar(float *c, int n, float lim, float zona_muerta)
{
	float suma = 0.0f;
	int con_orden = 0;

	if (isinf(lim)) {
		return;
	}
	for (int i = 0; i < n; i++) {
		if (c[i] != 0.0f) {
			suma += c[i];
			con_orden++;
		}
	}
	if (con_orden == 0) {
		return;
	}
	float exceso = (suma - lim * (float)n) / (float)con_orden;

	if (exceso <= 0.0f) {
		return;
	}
	for (int i = 0; i < n; i++) {
		if (c[i] != 0.0f) {
			c[i] -= exceso;
			if (fabsf(c[i]) < zona_muerta) {
				c[i] = 0.0f;
			}
		}
	}
}

#endif /* GUARDIA_H */
