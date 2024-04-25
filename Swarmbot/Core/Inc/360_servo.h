#ifndef _360_SERVO_H
#define _360_SERVO_H

#include "main.h"
#include "timer.h"

#define MAX_SPEED 150

#define STOP_SPEED 0

#define MIN_SPEED -150

#define PWM_RATE_MAX_SPEED_FORWARD 8.6
#define PWM_RATE_MIN_SPEED_FORWARD 7.6

#define PWM_RATE_MAX_SPEED_BACKWARD 7.2
#define PWM_RATE_MIN_SPEED_BACKWARD 6.2

#define PWM_RATE_STOP 7.4

typedef enum {
	FORWARD, BACKWARD
} DIRECTION;

typedef struct
{
	volatile uint32_t *compare_value;
	float pre_speed;
	float speed;
}Servo;


float map(float x, float in_min, float in_max, float out_min, float out_max);

uint8_t create_servo(Servo *servo, volatile uint32_t *compare_value);

uint8_t set_pwm_signal_servo(float *pwm_rate, float rpm, DIRECTION direction);

uint8_t check_update_speed(Servo *servo);


#endif
