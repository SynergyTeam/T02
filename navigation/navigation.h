/*----------------------------------------------------------------------------*/
#ifndef __NAVIGATION__
#define __NAVIGATION__

#include "struct.h"
#include "navigation/receiver.h"

#define NS_bit                  0x01
#define WE_bit                  0x02
#define AV_bit                  0x04
#define Parking_bit             0x40
#define COLD_START_TIME         180

//режимы анализа(конфигурации) геозон
enum _geo_mode {
    geo_None = 0,
    geo_Inside,
    geo_Outside
} _geo_mode;

extern nav_solution nSolution;

extern void NavigationTsk_Data(rcv_geoid *RCV, uint16_t len, sys_config *cfg);

extern int GetGeozoneList(geoid *point, uint8_t apoint);
extern int CheckLocationInZone(geoid *point, geoid *zone, uint16_t amnt, uint16_t radius);
extern float CalcGeoidDistance(geoid *cur, geoid *old);
extern void CorrectFirstNavTime(nav_solution *nav, int32_t timecode);
extern void GeoidAnalyse(rcv_geoid *RCV, sys_config *cfg);

extern uint8_t ParkingCondition(void);
extern void RestartIntervalTimer(char nT, uint16_t time);
extern uint16_t filtered_rmc_msg(char *buf, struct rcv_geoid *RCV);
extern uint32_t CheckOverSpeed(void);

extern uint16_t GetGPSSatelliteSize(sys_events *E);
extern char* PutListGPSSatellite(sys_events *E, char *data, uint16_t s1Num);
extern uint16_t GetGNSSSatelliteSize(sys_events *E);
extern char* PutListGNSSSatellite(sys_events *E, char *data, uint16_t s2Num);
extern uint16_t GetAtlitudeSensorSize(sys_events *E);
extern char *PutAtlitudeSensor(sys_events *E, char *data, uint16_t s2Num);

#endif /* __NAVIGATION__ */
