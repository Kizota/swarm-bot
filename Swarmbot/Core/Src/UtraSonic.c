#include "UltraSonic.h"
#include "stdio.h"
#include "stdlib.h"

#define LOW_PRE_TRIGGER_PERIOD 400 //us
#define HIGH_PRE_TRIGGER_PERIOD 500 //us

#define MAX_DETECTING_PERIOD 38000 //38ms

#define SAFE_THRESHOLD 15
#define DANGEROUS_THRESHOLD 6

#define TOLERANCE 1

uint8_t create_ultrasonic_datas(Echo_record *record, Distance *distance,
		volatile uint32_t *capture_val) {
	if (record == NULL || distance == NULL || capture_val == NULL) {
		return 0;
	}

	record->capture_val = capture_val;

	record->state = ECHO_LOW;
	record->high_record = 0;
	record->low_record = 0;

	distance->value = 0;
	distance->safety_sta = SAFE;
	distance->pre_safety_sta = SAFE;

	return 1;

}

uint8_t check_detect_echo_rise(Echo_record record) {

	return (*(record.capture_val) > LOW_PRE_TRIGGER_PERIOD
			&& *(record.capture_val) < HIGH_PRE_TRIGGER_PERIOD);
}

uint8_t check_detecting_status(Echo_record record) {

	return (*(record.capture_val) < record.low_record + MAX_DETECTING_PERIOD);
}

uint8_t calculate_distance(Echo_record record, Distance *distance) {
	if (distance == NULL) {
		return 0;
	}

	distance->value = (record.low_record - record.high_record) / 58;
   // printf("high record: %lu - low record: %lu - result distance %lu\n",record.high_record,record.low_record, distance->value);fflush(stdout);
	return 1;
}

uint8_t check_ultra_sonic_info(Echo_record *record, Distance *distance) {
	if (record == NULL || distance == NULL) {
		return 0;
	}
	switch (record->state) {
	case ECHO_LOW:

		if (check_detect_echo_rise(*record)) {
			record->high_record = *(record->capture_val);
			record->state = ECHO_HIGH;
		}
		break;
	case ECHO_HIGH:
		record->low_record = *(record->capture_val);

		if (check_detecting_status(*record)) {
			calculate_distance(*record, distance);
		}

		record->state = ECHO_LOW;
		break;

	}
	return 1;
}


