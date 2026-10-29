#ifndef CORE_ZERO_H
#define CORE_ZERO_H

#include <stddef.h>

#include "shared_data.h"

void vTaskCore1_PID(void *pvParameters);

void leer_encoders(int32_t *ticks_izq, int32_t *ticks_der);
void leer_ultrasonicos(float *dist_frente_m, float *dist_izq_m, float *dist_der_m);
void controlar_pid(float target_v, float target_w,
                  int32_t ticks_izq, int32_t ticks_der,
                  float dist_frente_m, float dist_izq_m, float dist_der_m);
void escribir_motores(float pwm_izq, float pwm_der);
void mover_servo_camara(int16_t angulo_grados);
void ejecutar_trayectoria_brazo(const float *posiciones, size_t count);

#endif /* CORE_ZERO_H */
