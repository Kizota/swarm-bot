#include "PID_control.h"
#include "timer.h"
#include <stdio.h>
#include <stdlib.h>

uint8_t PID_init(PID *pid) {
	if (pid == NULL) {
		return 0;
	}

	//constant
	pid->kp = 0;
	pid->ki = 0;
	pid->kd = 0;

	//P - I - D value
	pid->derevative = 0;
	pid->integral = 0;
	pid->proportional = 0;

	pid->pre_error = 0;
	return 1;
}

uint8_t tuning(PID *pid, float kp, float ki, float kd) {
	if (pid == NULL) {
		return 0;
	}

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	return 1;
}

uint8_t PID_controller(PID *pid, Timer *timer, float setpoint, float input,
		float *output) {
	if (pid == NULL || timer == NULL || output == NULL) {
		return 0;
	}

	if (timer->timeout) {

		//error term
		float error = input - setpoint;
		//printf("error: %f\n", error);
		//fflush(stdout);

		//proportional
		pid->proportional = pid->kp * error;

		//integral
		//unwind - only turn on in steady-state error
		if (error < 5) {
			pid->integral += pid->ki * error;
		}

		//derevative
		pid->derevative = pid->kd * (pid->pre_error - error);

		//unwind -simply turning off when get out of the zone
		if (error == 0 || pid->integral > 100) {
			pid->integral = 0;
		}

		*output = pid->proportional + pid->derevative;// + pid->integral;


	//	printf("output %f",*output);

		pid->pre_error = error;

		timer->timeout = 0;
	}
	return 1;
}

