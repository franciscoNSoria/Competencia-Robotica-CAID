#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

void ultrasonic_init(void);
void ultrasonic_trigger_cycle(void);
float ultrasonic_read_front(void);
float ultrasonic_read_left(void);
float ultrasonic_read_right(void);

#endif /* ULTRASONIC_DRIVER_H */
