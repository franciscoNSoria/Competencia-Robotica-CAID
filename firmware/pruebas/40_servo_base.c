/* Prueba 40: servo base solo (MG90S), GPIO18. Recorrido de tope a tope 230-245 grados (M); 50 us mueven 3-6 grados (M). Bajar el pulso va al lateral IZQUIERDO, subirlo al DERECHO (M).
 * Conexion: marron = GND, rojo = +5 V del riel de servos (NO el pin 5V de la ESP32), naranja = GPIO18. GND comun.
 * Arranca SIN pulsos. Al activar, el servo salta al pulso de reposo (1500 us): el brazo apuntando hacia atras (0 grados). El pulso del reposo de la base NO esta medido (P): activar con el brazo suelto o sin armar.
 * Limites de arranque 1000..2000 us (probados a mano en banco solo en pasos; P); se abren desde la PC hasta 500..2500 us. Velocidad 200 us/s.
 * Si el agente se pierde, el servo se queda quieto donde esta (no se suelta: el brazo no cae).
 *
 * Ordenes (/prueba/cmd = [40, orden, args]):
 *   1 us  activar en us (0 = reposo)   2 us  ir a us   3 delta  mover delta us   4  soltar (sin pulsos)
 *   5 v   velocidad us/s               6 lo hi  limites
 * Estado:  [40, contador, activo, pulso_us, objetivo_us, min_us, max_us, vel_us_s] */
#define PRUEBA_ID 40
#define PRUEBA_NOMBRE "servo base"
#include "prueba_uros.h"
#include "servos.h"

static servo_t S = {.nombre = "base", .pin = GPIO_NUM_18, .canal = LEDC_CHANNEL_0, .min_us = 1000, .max_us = 2000,
                    .reposo_us = 1500};

static void prueba_init(void)
{
	servos_init(&S, 1);
}

static void prueba_ciclo(void)
{
	servo_ciclo(&S, 0.010f);
}

static void prueba_comando(const float *a, int n)
{
	servo_orden(&S, a, n);
}

static int prueba_estado(float *e)
{
	return servo_estado(&S, e, 2);
}

static void prueba_parar(void)
{
	S.objetivo_us = (int)lroundf(S.pulso_us);
}
