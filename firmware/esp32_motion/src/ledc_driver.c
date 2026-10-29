#include "ledc_driver.h"

#include "driver/ledc.h"
#include "pins.h"

static void configure_channel(ledc_channel_t channel, gpio_num_t pin, ledc_timer_t timer) {
    ledc_channel_config_t cfg = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&cfg);
}

void ledc_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    configure_channel(LEDC_CHANNEL_0, MOTOR_IZQ_A, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_1, MOTOR_IZQ_B, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_2, MOTOR_DER_A, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_3, MOTOR_DER_B, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_4, SERVO_BASE, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_5, SERVO_HOMBRO, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_6, SERVO_CODO, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_7, SERVO_PINZA, LEDC_TIMER_0);
    configure_channel(LEDC_CHANNEL_8, SERVO_CAMARA, LEDC_TIMER_0);
}

void ledc_set_motor_left(float duty_normalized) {
    (void)duty_normalized;
}

void ledc_set_motor_right(float duty_normalized) {
    (void)duty_normalized;
}

void ledc_set_servo_camera(int16_t angle_deg) {
    (void)angle_deg;
}

void ledc_set_servo_arm(uint8_t index, int16_t angle_deg) {
    (void)index;
    (void)angle_deg;
}
