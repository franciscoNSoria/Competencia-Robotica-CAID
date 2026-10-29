#ifndef PINS_H
#define PINS_H

/*
 * Mapa de pines del ESP32-WROOM-32 usado como referencia para el robot.
 * Basado en la serigrafía del módulo y en la convención de hardware del repo:
 * - Motores: GPIO25, 26, 27, 16
 * - Servos: GPIO18, 19, 21, 22, 23
 * - Ultrasonidos: triggers GPIO5, 14, 15 y ecos GPIO13, 4, 17
 * - Encoders: GPIO34, 35, 36, 39
 * - ADC: GPIO32, 33
 * - LED de estado: GPIO2
 * - GPIO12 queda libre para uso futuro
 */

/* ---------- Motores ---------- */
#define MOTOR_IZQ_A     25
#define MOTOR_IZQ_B     26
#define MOTOR_DER_A     27
#define MOTOR_DER_B     16

#define MOTOR_PWM_HZ    5000

/* ---------- Servos ---------- */
#define SERVO_BASE      18
#define SERVO_HOMBRO    19
#define SERVO_CODO      21
#define SERVO_PINZA     22
#define SERVO_CAMARA    23

/* ---------- Ultrasonidos ---------- */
#define US_FRONTAL_TRIG 5
#define US_IZQ_TRIG     14
#define US_DER_TRIG     15

#define US_FRONTAL_ECHO 13
#define US_IZQ_ECHO     4
#define US_DER_ECHO     17

/* ---------- Encoders ---------- */
#define ENC_IZQ         34
#define ENC_DER         35
#define ENC_IZQ_AUX     36
#define ENC_DER_AUX     39

/* ---------- Conversores / ADC ---------- */
#define ADC_BATTERY_1   32
#define ADC_BATTERY_2   33

/* ---------- Estado / debug ---------- */
#define LED_STATUS      2

/* ---------- Reservado ---------- */
#define GPIO12_LIBRE    12

#endif /* PINS_H */
