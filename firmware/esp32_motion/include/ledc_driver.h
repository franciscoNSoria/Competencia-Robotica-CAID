#ifndef LEDC_DRIVER_H
#define LEDC_DRIVER_H

#include <stdint.h>

void ledc_init(void);
void ledc_set_motor_left(float duty_normalized);
void ledc_set_motor_right(float duty_normalized);
void ledc_set_servo_camera(int16_t angle_deg);
void ledc_set_servo_arm(uint8_t index, int16_t angle_deg);

#endif /* LEDC_DRIVER_H */
