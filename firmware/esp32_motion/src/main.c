#include "shared_data.h"
#include "core_zero.h"
#include "core_one.h"

// Instanciación de variables globales
SemaphoreHandle_t mutex_velocidad;
QueueHandle_t queue_telemetria;
float global_target_v = 0.0;
float global_target_w = 0.0;

// Handles de las tareas
TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

void setup() {
    Serial.begin(115200);

    // Mutex y Queue que se crean antes de lanzar las tareas
    mutex_velocidad = xSemaphoreCreateMutex();
    queue_telemetria = xQueueCreate(10, sizeof(TelemetriaRobot));

    // Lanzar Tarea del core_zero (control PID) en el núcleo 0
    xTaskCreatePinnedToCore(
        vTaskCore0_Control, 
        "Task_PID", 
        10000, 
        NULL, 
        1, 
        &TaskCore0, 
        0
    );

    // Lanzar Tarea del core_one (micro-ROS) en el núcleo 1
    xTaskCreatePinnedToCore(
        vTaskCore1_MicroROS, 
        "Task_uROS", 
        20000, 
        NULL, 
        1, 
        &TaskCore1, 
        1
    );
}

void loop() {
    vTaskDelete(NULL); // Elimina el loop default
}