#ifndef ROTATION_H
#define ROTATION_H

#include <string.h>
#include <stdbool.h>
#include "stm32f3xx.h"
#include "stm32f3xx_hal.h"

extern int t_set;
extern int pwm_max;
extern int pwm_val;
extern int t_sum_prev;
extern int t_sum_curr;
extern bool new_state;

extern float Ku;


void startRotate(void);

void changeRotation(void);

void stopAll(void);

void calcErr(void);

void rotate(void);

void PID(void);



#endif
