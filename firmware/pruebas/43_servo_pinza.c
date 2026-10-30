/* Prueba 43: servo pinza solo (MG90S), GPIO22. Subir el pulso ABRE y bajarlo CIERRA (M: en banco, 10 pasos de 50 us cerraban del todo). No apretar contra el tope: el servo se calienta.
 * Conexion: marron = GND, rojo = +5 V del riel de servos (NO el pin 5V de la ESP32), naranja = GPIO22. GND comun.
 * Arranca SIN pulsos. Al activar, el servo salta al pulso de reposo (1500 us): el pulso de reposo de la pinza NO esta medido (P).
 * Limites de arranque 1000..2000 us (P); se abren desde la PC hasta 500..2500 us. Velocidad 200 us/s.
 * Si el agente se pierde, el servo se queda quieto donde esta (no se suelta: el brazo no cae).
 *
 * Ordenes (/prueba/cmd = [43, orden, args]):
 *   1 us  activar en us (0 = reposo)   2 us  ir a us   3 delta  mover delta us   4  soltar (sin pulsos)
 *   5 v   velocidad us/s               6 lo hi  limites
 * Estado:  [43, contador, activo, pulso_us, objetivo_us, min_us, max_us, vel_us_s] */
#define PRUEBA_ID 43
#define PRUEBA_NOMBRE "servo pinza"
#include "prueba_uros.h"
#include "servos.h"

static servo_t S = {.nombre = "pinza", .pin = GPIO_NUM_22, .canal = LEDC_CHANNEL_0, .min_us = 1000, .max_us = 2000,
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
