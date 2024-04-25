#include "Robot.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define RIGHT 1
#define LEFT 0

#define NORMAL_RPM 70

uint8_t robot_init(Robot *robot, Movement *movement, Servo *l_wheel,
		Servo *r_wheel, Distance *eye, uint8_t set_distance) {
	if (robot == NULL || l_wheel == NULL || r_wheel == NULL || eye == NULL) {
		return 0;
	}
	//robot
	robot->l_wheel = l_wheel;
	robot->r_wheel = r_wheel;
	robot->eye = eye;

	movement->pre_mode = TURNING;
	movement->mode = FORWARD;

	movement->distance = set_distance;

	movement->speed = 0;

	return 1;
}


uint8_t set_speed(Robot *robot, float speed_rpm) {
	if (robot == NULL) {
		return 0;
	}
	robot->l_wheel->speed = speed_rpm;
	robot->r_wheel->speed = -speed_rpm-35;

	return 1;
}

uint8_t set_turn_speed_random_direction(Robot *robot, float speed_rpm) {

	if (robot == NULL) {
		return 0;
	}
	uint8_t sign = rand() % 2;

	switch (sign) {
	case RIGHT:
		robot->l_wheel->speed = -speed_rpm;
		robot->r_wheel->speed = -speed_rpm;
		break;
	case LEFT:
		robot->l_wheel->speed = speed_rpm;
		robot->r_wheel->speed = speed_rpm;
		break;
	}

	return 1;
}


uint8_t drifting_robot(Robot *robot, Movement *movement,PID *pos_control,Timer *timer,Timer *pos_timer) {
    //check robot struct
	if (robot == NULL || movement == NULL || timer == NULL) {
		return 0;
	}

	//check pid control struct
	if(pos_control == NULL || pos_timer == NULL)
	{
		return 0;
	}

	switch (movement->mode) {
	case AHEAD:
		if (movement->pre_mode != movement->mode) {
			//set speed
			movement->speed = NORMAL_RPM;
			set_speed(robot, movement->speed);
			movement->pre_mode = movement->mode;

			printf("AHEAD!\n");fflush(stdout);
		}

		if (robot->eye->value <= movement->distance * 4) {
			movement->mode = SMOOTH_BREAK;
		}
		break;
	case SMOOTH_BREAK:
        if(movement->pre_mode != movement->mode)
        {
        	printf("SMOOTH BREAK!\n"); fflush(stdout);
        	movement->pre_mode = movement->mode;
        }

        //PID control - take command and procces
        PID_controller(pos_control,pos_timer,(float)movement->distance,robot->eye->value,&movement->speed);
		 set_speed(robot, movement->speed);


         if(robot->eye->value <= movement->distance)
		{
			movement->mode = TURNING;
		}

		break;
	case TURNING:
		if (movement->pre_mode != movement->mode) {
			//set speed
			movement->speed = NORMAL_RPM;
			set_turn_speed_random_direction(robot, movement->speed*0.7);
			movement->pre_mode = movement->mode;

			printf("turning!\n");fflush(stdout);

		}

		if (robot->eye->value > movement->distance *2 && timer->timeout) {
			movement->mode = AHEAD;
			timer->timeout = 0;
		}

		break;
	}

	return 1;
}

uint8_t drifting_robot_old(Robot *robot, Movement *movement,Timer *timer) {

	if (robot == NULL || movement == NULL || timer == NULL) {
		return 0;
	}

	switch (movement->mode) {
	case AHEAD:
		if (movement->pre_mode != movement->mode) {
			set_speed(robot, NORMAL_RPM);
			printf("ahead!\n");fflush(stdout);
			printf("left speed: %f - right speed: %f!\n",robot->l_wheel->speed,robot->r_wheel->speed);fflush(stdout);
			movement->pre_mode = movement->mode;
		}

		if (robot->eye->value <= movement->distance) {
			movement->mode = TURNING;
		}
		break;

	case TURNING:
		if (movement->pre_mode != movement->mode) {
			set_turn_speed_random_direction(robot, NORMAL_RPM);
			printf("turning!\n");fflush(stdout);
			printf("left speed: %f - right speed: %f!\n",robot->l_wheel->speed,robot->r_wheel->speed);fflush(stdout);
			movement->pre_mode = movement->mode;
		}

		if (robot->eye->value > movement->distance && timer->timeout) {
			movement->mode = AHEAD;
			timer->timeout = 0;
		}

		break;
	}

	return 1;
}
