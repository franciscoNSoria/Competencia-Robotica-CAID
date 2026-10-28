#ifndef SHARED_DATA_H
#define SHARED_DATA_H

// TODO: Completar con los includes
// Con ESP-IDF #include "freertos/FreeRTOS.h", etc.
#include ...

// Mutex y Queues Globales
extern SemaphoreHandle_t mutex_velocidad;
extern QueueHandle_t queue_telemetria;

// Variables de velocidad con Mutex
extern float global_target_v;
extern float global_target_w;

// Struct para enviar la telemetría (Core 0 -> Core 1)
struct TelemetriaRobot {
    long ticks_izq;
    long ticks_der;
    float dist_frente;
    float dist_izq;
    float dist_der;
};

#endif