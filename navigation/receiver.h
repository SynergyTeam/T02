/*
 * receiver.h
 *
 *  Created on: 13.06.2013
 *      Author: a_cherepanov
 */

#ifndef RECEIVER_H_
#define RECEIVER_H_

#include "struct.h"

//основные состояния приемника
typedef enum Receiver_wMode {
	RECEIVER_INIT,
	RECEIVER_NAVIGATION,
	RECEIVER_UPDATE
} Receiver_wMode;

//режимы позиционирования
typedef enum Navigation_Mode {
	MODE_2D = 2,
	MODE_3D
} Navigation_Mode;

typedef struct rcv_geoid {
    uint16_t        IntervalTimer[MAX_ARCHIVES];
    uint16_t        NopTimer[MAX_ARCHIVES];
    uint16_t        rmc_build;
    char            *nmea_buf;

    Receiver_wMode  state;
    uint8_t         StDelay;
    uint8_t         indication;
    uint8_t         save_point_timer;
    uint8_t         poll_error;
    uint8_t         poll_cntr;
    uint8_t         poll_events;

    unsigned        satellite_sys:1;
    unsigned        cold_start:1;
    unsigned        restart:1;
} rcv_geoid;

extern rcv_geoid receiver;

extern void ReceiverReset(void);
extern void ReceiverColdStart(void);
extern char nmea_crc(char **pointer, int32_t size);


#endif /* RECEIVER_H_ */
