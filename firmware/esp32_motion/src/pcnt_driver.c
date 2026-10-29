#include "pcnt_driver.h"

#include "driver/pcnt.h"
#include "pins.h"

void pcnt_init(void) {
    pcnt_config_t config = {
        .pulse_gpio_num = ENC_IZQ,
        .ctrl_gpio_num = -1,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 10000,
        .counter_l_lim = -10000,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0,
    };

    pcnt_unit_config(&config);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

void pcnt_reset_left(void) {
    pcnt_counter_clear(PCNT_UNIT_0);
}

void pcnt_reset_right(void) {
    pcnt_counter_clear(PCNT_UNIT_1);
}

int32_t pcnt_get_left_ticks(void) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    return (int32_t)count;
}

int32_t pcnt_get_right_ticks(void) {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_1, &count);
    return (int32_t)count;
}
