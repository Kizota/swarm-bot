#include "360_servo.h"
#include <stdlib.h>
#include <stdio.h>
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


uint8_t create_servo(Servo *servo, volatile uint32_t *compare_value) {
	if (servo == NULL) {
		return 0;
	}
	servo->compare_value = compare_value;

	servo->pre_speed = -1;
	servo->speed = 0;

	return 1;
}

uint8_t set_pwm_signal_servo(float *pwm_rate, float rpm, DIRECTION direction) {
	if (pwm_rate == NULL) {
		return 0;
	}

	switch (direction) {
	case FORWARD:
		*pwm_rate = map(rpm, STOP_SPEED, MAX_SPEED, PWM_RATE_MIN_SPEED_FORWARD,       //7.6 - 8.6
	 	 PWM_RATE_MAX_SPEED_FORWARD);

		break;
	case BACKWARD:
		*pwm_rate = map(rpm, MIN_SPEED, STOP_SPEED, PWM_RATE_MIN_SPEED_BACKWARD,      //7.4 - 6.4
		PWM_RATE_MAX_SPEED_BACKWARD);

		break;
	}

	return 1;
}

uint8_t check_update_speed(Servo *servo) {
	if (servo == NULL) {
		return 0;
	}

	//check if speed is already updated
	if (servo->pre_speed == servo->speed) {

		return 1;

	} else {
		servo->pre_speed = servo->speed;
	}

	float pwm_rate = PWM_RATE_STOP;

	if (servo->speed > 0) {
		set_pwm_signal_servo(&pwm_rate, servo->speed, FORWARD);
	}

	else if (servo->speed < 0) {
		set_pwm_signal_servo(&pwm_rate, servo->speed, BACKWARD);
	}


	set_pulse_width(servo->compare_value, pwm_rate);
	return 1;
}
