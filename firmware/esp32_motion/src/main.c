#include "hardware_init.h"
#include "ledc_driver.h"
#include "pcnt_driver.h"
#include "shared_data.h"
#include "ultrasonic_driver.h"

SemaphoreHandle_t mutex_cmd_vel = NULL;
QueueHandle_t cola_telemetria = NULL;
cmd_vel_t g_cmd_vel_actual = {0.0f, 0.0f, 0, false};

TaskHandle_t task_core_comms = NULL;
TaskHandle_t task_core_control = NULL;

static void create_shared_objects(void) {
    mutex_cmd_vel = xSemaphoreCreateMutex();
    cola_telemetria = xQueueCreate(1, sizeof(telemetria_t));
    configASSERT(mutex_cmd_vel != NULL);
    configASSERT(cola_telemetria != NULL);
}

void app_main(void) {
    create_shared_objects();
    init_hardware();
    pcnt_init();
    ledc_init();
    ultrasonic_init();

    xTaskCreatePinnedToCore(
        vTaskCore0_MicroROS,
        "microros_comm",
        TASK_STACK_BYTES_COMMS,
        NULL,
        TASK_PRIORITY_COMMS,
        &task_core_comms,
        CORE_WIFI_MICROROS
    );

    xTaskCreatePinnedToCore(
        vTaskCore1_PID,
        "control_pid",
        TASK_STACK_BYTES_CONTROL,
        NULL,
        TASK_PRIORITY_CONTROL,
        &task_core_control,
        CORE_PID
    );

    vTaskDelete(NULL);
}

void setpoint_escribir(float v, float w) {
    if (mutex_cmd_vel == NULL) {
        return;
    }

    if (xSemaphoreTake(mutex_cmd_vel, portMAX_DELAY) == pdTRUE) {
        g_cmd_vel_actual.v = v;
        g_cmd_vel_actual.w = w;
        g_cmd_vel_actual.stamp_us = esp_timer_get_time();
        g_cmd_vel_actual.valida = true;
        xSemaphoreGive(mutex_cmd_vel);
    }
}

bool setpoint_leer(cmd_vel_t *out, TickType_t timeout_ticks) {
    if (out == NULL || mutex_cmd_vel == NULL) {
        return false;
    }

    if (xSemaphoreTake(mutex_cmd_vel, timeout_ticks) != pdTRUE) {
        return false;
    }

    *out = g_cmd_vel_actual;
    xSemaphoreGive(mutex_cmd_vel);
    return true;
}
