#include "ultrasonic_driver.h"

#include "pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"

void ultrasonic_init(void) {
    gpio_set_direction(US_FRONTAL_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_IZQ_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_DER_TRIG, GPIO_MODE_OUTPUT);
    gpio_set_direction(US_FRONTAL_ECHO, GPIO_MODE_INPUT);
    gpio_set_direction(US_IZQ_ECHO, GPIO_MODE_INPUT);
    gpio_set_direction(US_DER_ECHO, GPIO_MODE_INPUT);
}

void ultrasonic_trigger_cycle(void) {
    gpio_set_level(US_FRONTAL_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_FRONTAL_TRIG, 0);
}

float ultrasonic_read_front(void) {
    return 0.0f;
}

float ultrasonic_read_left(void) {
    return 0.0f;
}

float ultrasonic_read_right(void) {
    return 0.0f;
}
