#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

struct stepper;

uint_fast8_t stepper_event(struct timer *t);
struct stepper *stepper_oid_lookup(uint8_t oid);
uint32_t stepper_get_position(struct stepper *s);
void stepper_apply_correction_step(struct stepper *s, int_fast8_t want_increase);

#endif // stepper.h
