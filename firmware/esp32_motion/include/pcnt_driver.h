#ifndef PCNT_DRIVER_H
#define PCNT_DRIVER_H

#include <stdint.h>

void pcnt_init(void);
void pcnt_reset_left(void);
void pcnt_reset_right(void);
int32_t pcnt_get_left_ticks(void);
int32_t pcnt_get_right_ticks(void);

#endif /* PCNT_DRIVER_H */
