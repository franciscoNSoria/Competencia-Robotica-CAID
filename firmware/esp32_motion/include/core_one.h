#ifndef CORE_ONE_H
#define CORE_ONE_H

#include <stddef.h>

#include "shared_data.h"

void vTaskCore0_MicroROS(void *pvParameters);

void iniciar_wifi(void);
void publicar_telemetria(const telemetria_t *snapshot);
void gestionar_servo_camara(int16_t angulo_grados);
void ejecutar_brazo_desde_ros(const float *posiciones, size_t count);

#endif /* CORE_ONE_H */
