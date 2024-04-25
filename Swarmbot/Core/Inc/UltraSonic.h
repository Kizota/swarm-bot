#ifndef UTRASONIC_H
#define UTRASONIC_H

#include "main.h"

typedef enum
{
	ECHO_HIGH,
	ECHO_LOW
}Echo_state;

typedef enum
{
  SAFE,
  WARNING
}Safety_state;


typedef struct
{
	volatile uint32_t *capture_val;

	volatile Echo_state state;

	volatile uint32_t high_record;
	volatile uint32_t low_record;
}Echo_record;

typedef struct
{
	volatile uint32_t value;

	volatile Safety_state safety_sta;
	Safety_state pre_safety_sta;

}Distance;

uint8_t create_ultrasonic_datas(Echo_record * record, Distance * distance, volatile uint32_t *capture_val);

uint8_t check_detecting_status(Echo_record record);

uint8_t calculate_distance(Echo_record record, Distance *distance);

uint8_t check_ultra_sonic_info(Echo_record *record,  Distance *distance);

#endif
