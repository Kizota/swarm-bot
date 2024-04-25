#ifndef TIMER_H
#define TIMER_H

#include "main.h"

typedef struct
{
	volatile uint32_t tick;
	uint32_t interval;
	uint8_t timeout;
}Timer;


uint8_t create_timer(Timer *timer, uint32_t interval);

uint8_t timing(Timer *timer);

void SysTick_init();

void Timer_2_init();

void timer3_init();

void timer4_init();

uint8_t set_pulse_width(volatile uint32_t *comp_val, float rate);
#endif
