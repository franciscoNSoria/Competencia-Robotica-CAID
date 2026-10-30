/* Prueba 41: servo hombro solo (MG90S), GPIO19. Con el brazo armado, el hombro solo no levantaba (M): probar primero sin la parte de adelante del brazo.
 * Conexion: marron = GND, rojo = +5 V del riel de servos (NO el pin 5V de la ESP32), naranja = GPIO19. GND comun.
 * Arranca SIN pulsos. Al activar, el servo salta al pulso de reposo (1700 us): brazo abajo, apoyado en el 2.o piso.
 * Limites de arranque 1300..1700 us (lo medido: 1300 sube, 1700 baja; M); se abren desde la PC hasta 500..2500 us. Velocidad 200 us/s.
 * Si el agente se pierde, el servo se queda quieto donde esta (no se suelta: el brazo no cae).
 *
 * Ordenes (/prueba/cmd = [41, orden, args]):
 *   1 us  activar en us (0 = reposo)   2 us  ir a us   3 delta  mover delta us   4  soltar (sin pulsos)
 *   5 v   velocidad us/s               6 lo hi  limites
 * Estado:  [41, contador, activo, pulso_us, objetivo_us, min_us, max_us, vel_us_s] */
#define PRUEBA_ID 41
#define PRUEBA_NOMBRE "servo hombro"
#include "prueba_uros.h"
#include "servos.h"

static servo_t S = {.nombre = "hombro", .pin = GPIO_NUM_19, .canal = LEDC_CHANNEL_0, .min_us = 1300, .max_us = 1700,
                    .reposo_us = 1700};

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
