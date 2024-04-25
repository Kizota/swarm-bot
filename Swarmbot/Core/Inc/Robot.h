#ifndef ROBOT_H
#define ROBOT_H

#include "Ultrasonic.h"
#include "360_servo.h"
#include "PID_control.h"


typedef enum
{
	AHEAD,
	SMOOTH_BREAK,
	TURNING,
}Mode;

typedef struct
{
	//robot component
	Servo *l_wheel;
	Servo *r_wheel;
	Distance *eye;
}Robot;


typedef struct
{
	//movement
	uint8_t distance;

	float speed;

	Mode pre_mode;
	Mode mode;
}Movement;

uint8_t robot_init(Robot *robot, Movement *movement, Servo *l_wheel,
		Servo *r_wheel, Distance *eye, uint8_t set_distance);

uint8_t set_speed(Robot *robot, float speed_rpm);

uint8_t set_turn_speed_random_direction(Robot *robot, float speed_rpm);

uint8_t drifting_robot(Robot *robot, Movement *movement,PID *pos_control,Timer *timer,Timer *pos_timer);


#endif
