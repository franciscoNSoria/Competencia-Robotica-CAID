#include "core_zero.h"

#include <math.h>

#include "ledc_driver.h"
#include "pcnt_driver.h"
#include "ultrasonic_driver.h"

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void aplicar_stop_local(float dist_frente_m, float dist_izq_m, float dist_der_m, cmd_vel_t *cmd_actual) {
    const bool bloqueo_frontal = dist_frente_m < UMBRAL_STOP_FRONTAL_M;
    const bool bloqueo_lateral = (dist_izq_m < UMBRAL_STOP_LATERAL_M) || (dist_der_m < UMBRAL_STOP_LATERAL_M);

    if (bloqueo_frontal || bloqueo_lateral) {
        cmd_actual->v = 0.0f;
        cmd_actual->w = 0.0f;
    }
}

static void publicar_snapshot_telemetria(int32_t ticks_izq, int32_t ticks_der,
                                       float dist_frente_m, float dist_izq_m, float dist_der_m,
                                       bool robot_bloqueado) {
    telemetria_t snapshot = {0};
    static uint32_t seq = 0;

    snapshot.seq = ++seq;
    snapshot.stamp_us = esp_timer_get_time();
    snapshot.ticks_izq = ticks_izq;
    snapshot.ticks_der = ticks_der;
    snapshot.dist_frente_m = dist_frente_m;
    snapshot.dist_izq_m = dist_izq_m;
    snapshot.dist_der_m = dist_der_m;
    snapshot.modo = MODO_CRUCERO;
    snapshot.maniobra_id = 0;
    snapshot.maniobra_estado = MANIOBRA_TERMINADA;
    snapshot.maniobra_motivo = MOTIVO_NINGUNO;
    snapshot.brazo_ocupado = robot_bloqueado;

    xQueueOverwrite(cola_telemetria, &snapshot);
}

void vTaskCore1_PID(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrecuencia = pdMS_TO_TICKS(PERIODO_CONTROL_MS);
    cmd_vel_t orden_actual = {0.0f, 0.0f, 0, false};
    int32_t ticks_izq = 0;
    int32_t ticks_der = 0;
    float dist_frente_m = INFINITY;
    float dist_izq_m = INFINITY;
    float dist_der_m = INFINITY;
    bool robot_bloqueado = false;
    static float last_pwm_izq = 0.0f;
    static float last_pwm_der = 0.0f;

    (void)pvParameters;

    while (1) {
        const int64_t ahora_us = esp_timer_get_time();

        if (setpoint_leer(&orden_actual, pdMS_TO_TICKS(1)) == false) {
            orden_actual.v = 0.0f;
            orden_actual.w = 0.0f;
        }

        if ((ahora_us - orden_actual.stamp_us) > (WATCHDOG_CMD_MS * 1000LL)) {
            orden_actual.v = 0.0f;
            orden_actual.w = 0.0f;
            orden_actual.valida = false;
        }

        leer_encoders(&ticks_izq, &ticks_der);
        leer_ultrasonicos(&dist_frente_m, &dist_izq_m, &dist_der_m);

        aplicar_stop_local(dist_frente_m, dist_izq_m, dist_der_m, &orden_actual);

        if ((dist_frente_m < UMBRAL_STOP_FRONTAL_M) ||
            (dist_izq_m < UMBRAL_STOP_LATERAL_M) ||
            (dist_der_m < UMBRAL_STOP_LATERAL_M)) {
            robot_bloqueado = true;
            orden_actual.v = 0.0f;
            orden_actual.w = 0.0f;
        } else {
            robot_bloqueado = false;
        }

        float pwm_izq = 0.0f;
        float pwm_der = 0.0f;
        controlar_pid(orden_actual.v, orden_actual.w, ticks_izq, ticks_der,
                     dist_frente_m, dist_izq_m, dist_der_m);

        /*
         * La lógica concreta del controlador real se define en el hardware real.
         * Mientras tanto, el esqueleto usa una ley simple de proporcionalidad para
         * que el Core 1 tenga una salida visible y un comportamiento coherente.
         */
        pwm_izq = clampf((orden_actual.v + orden_actual.w * 0.5f), -1.0f, 1.0f);
        pwm_der = clampf((orden_actual.v - orden_actual.w * 0.5f), -1.0f, 1.0f);
        last_pwm_izq = pwm_izq;
        last_pwm_der = pwm_der;

        escribir_motores(pwm_izq, pwm_der);
        publicar_snapshot_telemetria(ticks_izq, ticks_der, dist_frente_m, dist_izq_m, dist_der_m, robot_bloqueado);

        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}

void leer_encoders(int32_t *ticks_izq, int32_t *ticks_der) {
    if (ticks_izq == NULL || ticks_der == NULL) {
        return;
    }

    *ticks_izq = pcnt_get_left_ticks();
    *ticks_der = pcnt_get_right_ticks();
}

void leer_ultrasonicos(float *dist_frente_m, float *dist_izq_m, float *dist_der_m) {
    if (dist_frente_m == NULL || dist_izq_m == NULL || dist_der_m == NULL) {
        return;
    }

    *dist_frente_m = ultrasonic_read_front();
    *dist_izq_m = ultrasonic_read_left();
    *dist_der_m = ultrasonic_read_right();
}

void controlar_pid(float target_v, float target_w,
                  int32_t ticks_izq, int32_t ticks_der,
                  float dist_frente_m, float dist_izq_m, float dist_der_m) {
    (void)target_v;
    (void)target_w;
    (void)ticks_izq;
    (void)ticks_der;
    (void)dist_frente_m;
    (void)dist_izq_m;
    (void)dist_der_m;
}

void escribir_motores(float pwm_izq, float pwm_der) {
    ledc_set_motor_left(pwm_izq);
    ledc_set_motor_right(pwm_der);
}

void mover_servo_camara(int16_t angulo_grados) {
    ledc_set_servo_camera(angulo_grados);
}

void ejecutar_trayectoria_brazo(const float *posiciones, size_t count) {
    if (posiciones == NULL || count == 0) {
        return;
    }

    for (size_t i = 0; i < count && i < 4; ++i) {
        ledc_set_servo_arm((uint8_t)i, (int16_t)posiciones[i]);
    }
}
