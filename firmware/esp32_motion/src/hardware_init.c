#include "pins.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"

void init_hardware(void) {
    /*
     * Inicialización del pinout base del robot.
     * Este archivo centraliza la configuración de GPIO, LEDC y PCNT para que el
     * control del Core 1 llame a funciones de alto nivel sin hardcodear pines en
     * cada módulo del firmware.
     */

    gpio_reset_pin(MOTOR_IZQ_A);
    gpio_reset_pin(MOTOR_IZQ_B);
    gpio_reset_pin(MOTOR_DER_A);
    gpio_reset_pin(MOTOR_DER_B);

    gpio_set_direction(MOTOR_IZQ_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IZQ_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DER_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DER_B, GPIO_MODE_OUTPUT);

    gpio_set_level(MOTOR_IZQ_A, 0);
    gpio_set_level(MOTOR_IZQ_B, 0);
    gpio_set_level(MOTOR_DER_A, 0);
    gpio_set_level(MOTOR_DER_B, 0);

    gpio_reset_pin(US_FRONTAL_TRIG);
    gpio_reset_pin(US_IZQ_TRIG);
    gpio_reset_pin(US_DER_TRIG);
    gpio_reset_pin(US_FRONTAL_ECHO);
    gpio_reset_pin(US_IZQ_ECHO);
    gpio_reset_pin(US_DER_ECHO);

    gpio_set_direction(US_FRONTAL_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_IZQ_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_DER_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_FRONTAL_ECHO, GPIO_MODE_INPUT);
    gpio_set_direction(US_IZQ_ECHO, GPIO_MODE_INPUT);
    gpio_set_direction(US_DER_ECHO, GPIO_MODE_INPUT);

    gpio_reset_pin(SERVO_BASE);
    gpio_reset_pin(SERVO_HOMBRO);
    gpio_reset_pin(SERVO_CODO);
    gpio_reset_pin(SERVO_PINZA);
    gpio_reset_pin(SERVO_CAMARA);

    gpio_set_direction(SERVO_BASE, GPIO_MODE_OUTPUT);
    gpio_set_direction(SERVO_HOMBRO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SERVO_CODO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SERVO_PINZA, GPIO_MODE_OUTPUT);
    gpio_set_direction(SERVO_CAMARA, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LED_STATUS);
    gpio_set_direction(LED_STATUS, GPIO_MODE_OUTPUT);

    /* PCNT: una configuración base. El uso concreto se resuelve en el driver. */
    gpio_set_direction(ENC_IZQ, GPIO_MODE_INPUT);
    gpio_set_direction(ENC_DER, GPIO_MODE_INPUT);
    gpio_set_direction(ENC_IZQ_AUX, GPIO_MODE_INPUT);
    gpio_set_direction(ENC_DER_AUX, GPIO_MODE_INPUT);
}
