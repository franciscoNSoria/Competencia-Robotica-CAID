#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

void vTaskCore0_MicroROS(void *pvParameters);
void vTaskCore1_PID(void *pvParameters);

/*
 * Decisión consolidada:
 * - Core 0: Wi-Fi + lwIP + micro-ROS + comunicaciones.
 * - Core 1: PID + sensores + stop + maniobras + control local.
 * - cmd_vel via mutex protegido y telemetría via cola de longitud 1 que
 *   sobrescribe el último dato.
 */
#define CORE_WIFI_MICROROS    0
#define CORE_PID              1

#define TASK_PRIORITY_COMMS      5
#define TASK_PRIORITY_CONTROL    6
#define TASK_STACK_BYTES_COMMS    8192
#define TASK_STACK_BYTES_CONTROL  4096

#define PERIODO_CONTROL_MS      20
#define VENTANA_VELOCIDAD_MS    100
#define PERIODO_TELEMETRIA_MS   100
#define SLOT_ULTRASONIDO_MS     30
#define WATCHDOG_CMD_MS         500

#define UMBRAL_STOP_FRONTAL_M    0.20f
#define UMBRAL_STOP_LATERAL_M    0.15f

/* cmd_vel: PC -> control */
typedef struct {
    float   v;
    float   w;
    int64_t stamp_us;
    bool    valida;
} cmd_vel_t;

extern SemaphoreHandle_t mutex_cmd_vel;
extern cmd_vel_t g_cmd_vel_actual;

void setpoint_escribir(float v, float w);
bool setpoint_leer(cmd_vel_t *out, TickType_t timeout_ticks);

/* Maniobras: misma política de propiedad, un único escritor dentro del control. */
typedef enum {
    MANIOBRA_AVANZAR,
    MANIOBRA_GIRAR,
    MANIOBRA_CANCELAR
} maniobra_tipo_t;

typedef struct {
    uint32_t        id;
    maniobra_tipo_t tipo;
    float           valor;
} maniobra_cmd_t;

typedef enum {
    MODO_CRUCERO,
    MODO_MANIOBRA,
    MODO_BRAZO
} modo_t;

typedef enum {
    MANIOBRA_EN_CURSO,
    MANIOBRA_TERMINADA,
    MANIOBRA_ABORTADA
} maniobra_estado_t;

typedef enum {
    MOTIVO_NINGUNO,
    MOTIVO_OBSTACULO,
    MOTIVO_BLOQUEO,
    MOTIVO_TIEMPO,
    MOTIVO_CANCELADA
} maniobra_motivo_t;

typedef struct {
    float    umbral_frente_m;
    uint32_t vence_ms;
} override_frente_t;

/* Telemetría: último estado válido, sin perder el snapshot más reciente */
typedef struct {
    uint32_t seq;
    int64_t  stamp_us;
    int32_t  ticks_izq;
    int32_t  ticks_der;
    float    dist_frente_m;
    float    dist_izq_m;
    float    dist_der_m;
    modo_t   modo;
    uint32_t maniobra_id;
    maniobra_estado_t maniobra_estado;
    maniobra_motivo_t maniobra_motivo;
    bool     brazo_ocupado;
} telemetria_t;

extern QueueHandle_t cola_telemetria;

#define BRAZO_NUM_ARTICULACIONES 4

typedef struct {
    float posicion[BRAZO_NUM_ARTICULACIONES];
} brazo_cmd_t;

#endif /* SHARED_DATA_H */