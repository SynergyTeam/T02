#ifndef __RTC_H__
#define __RTC_H__

#include "struct.h"

#define SECONDS_BEFORE_BEGIN_OF_UNIX_TIME       (2006141056)
#define DELTA_UNIX_TIME                         (719163)

#define SECONDS_IN_DAY                          (86400)
#define DAYS_IN_YEAR                            (365)

extern const RTCTime FW_time;
extern const RTCTime dlTime;

extern RTCTime GetPacketTime(char *tData);
extern void SetPacketTime(char *tData, RTCTime *time);
extern uint32_t CalcDay(RTCTime *time);
extern uint32_t getsecond(RTCTime *time);
extern int32_t  CalcDeltaTime(RTCTime *time1, RTCTime *time2);
extern void TimeCorrection(RTCTime *time, int32_t delta);
extern int32_t calcSecondsTime(RTCTime *time1);
extern RTCTime UnixTimeToRTC(uint32_t time);
extern int32_t GetUnixTime(RTCTime *time);

#endif /* end __RTC_H__ */
