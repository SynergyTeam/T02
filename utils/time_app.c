#include <inttypes.h>
#include <stdlib.h>
#include "config.h"
#include "struct.h"
#include "time_app.h"

#define EPOCH_YR                    1970
#define LEAPYEAR(year)              (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year)              (LEAPYEAR(year) ? 366 : 365)

const RTCTime FW_time = {0, 0, 0, FW_DAY, FW_MON, FW_YEAR, 0, 0, 0};
const RTCTime dlTime = {0, 0, 0, 1, 1, 2000, 0, 0, 0};
const uint8_t _ytab[2][12] = {
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
    {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

//------------------------------------------------------------------------------
//получить время/дату из BCD пакета данных (сервер/приемник)
RTCTime GetPacketTime(char *tData) {
    uint8_t byte, tmp, value, error;
    RTCTime pTime;

    tmp = *tData++;                                                             //число
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_mday = value;

    tmp = *tData++;                                                             //месяц
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_mon = value;
    if (value < 1 || value > 12)
        error = 1;
    else
        error = 0;

    tmp = *tData++;                                                             //год
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_year = 2000 + value;

    byte = pTime.tm_mon;                                                        //проверка валидности даты
    value = _ytab[0][byte-1];
    if(byte == 2)
        value += LEAPYEAR(pTime.tm_year);
    byte = pTime.tm_mday;
    if(byte < 1 || byte > value)
        error = 1;

    tmp = *tData++;                                                             //часы
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_hour = value;
    if(value > 23) error = 1;
    tmp = *tData++;                                                             //минуты
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_min = value;
    if(value > 59) error = 1;
    tmp = *tData++;                                                             //секунды
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_sec = value;
    if(value > 59) error = 1;
    if(error)
        pTime = FW_time;
    return pTime;
}
//------------------------------------------------------------------------------
//установить время в серверном пакета в соответствии с локальным
void SetPacketTime(char *tData, RTCTime *time) {
    uint16_t year;
    *tData++ = ((time->tm_mday / 10) << 4) | (time->tm_mday % 10);
    *tData++ = ((time->tm_mon / 10) << 4) | (time->tm_mon % 10);
    year = time->tm_year - 2000;
    *tData++ = ((year / 10) << 4) | (year % 10);
    *tData++ = ((time->tm_hour / 10) << 4) | (time->tm_hour % 10);
    *tData++ = ((time->tm_min / 10) << 4) | (time->tm_min % 10);
    *tData++ = ((time->tm_sec / 10) << 4) | (time->tm_sec % 10);
}

//------------------------------------------------------------------------------
//вычислить порядковый номер дня с начала нашей эры
uint32_t CalcDay(RTCTime *time) {
    uint32_t dayAmount, yearDay, month;

    yearDay = (time->tm_year - 1) / 4;                                          //кол-во високосных лет
    yearDay -= (time->tm_year - 1) / 100;                                       //коррекция / 100
    yearDay += (time->tm_year - 1) / 400;                                       //коррекция / 400
    dayAmount = yearDay * 366;
    dayAmount += ((time->tm_year - 1) - yearDay) * DAYS_IN_YEAR;

    yearDay = 0;
    if(time->tm_mon > 1) {
        for(month = 1; month < time->tm_mon; ++month)
            yearDay += _ytab[0][month-1];                                       //число полных месяцев
        if(time->tm_mon > 2 )
            yearDay += LEAPYEAR(time->tm_year);                                 //коррекция текущего високосного года
    }
    yearDay += time->tm_mday;
    time->tm_yday = yearDay;
    dayAmount += yearDay;
    time->tm_wday = dayAmount % 7;
    return dayAmount;
}

//------------------------------------------------------------------------------
//вычислить порядковый номер секунды с начала дня
uint32_t getsecond(RTCTime *time) {
    uint32_t second;
    second = time->tm_hour * 3600;
    second += time->tm_min * 60;
    second += time->tm_sec;
    return second;
}

//------------------------------------------------------------------------------
//вычислить дельту времени в секундах между time1 и time2 (в пределах года!!!)
int32_t CalcDeltaTime(RTCTime *time1, RTCTime *time2) {
    int32_t Day1, Day2, Sec1, Sec2;
    Day1 = CalcDay(time1); Sec1 = getsecond(time1);
    Day2 = CalcDay(time2); Sec2 = getsecond(time2);
    Day1 -= Day2;                                                               //дельта дней
    Day2 = DAYS_IN_YEAR + LEAPYEAR(time1->tm_year + 1);                         //ограничение
    if(Day1 > Day2) {
        Day1 = Day2; Sec2 = Sec1;
    } else {
        Day2 = DAYS_IN_YEAR + LEAPYEAR(time1->tm_year - 1);
        if(Day1 < -Day2) {
            Day1 = -Day2; Sec2 = Sec1;
        }
    }
    Day1 *= SECONDS_IN_DAY;
    Day1 += (Sec1 - Sec2);                                                      //дельта секунд
    return Day1;
}

//------------------------------------------------------------------------------
//скорректировать время на дельту
void TimeCorrection(RTCTime *time, int32_t delta) {
    int32_t aDay, aSec, cDay, cSec, month;
    aDay = CalcDay(time);                                                       //число дней
    aSec = getsecond(time);                                                     //число секунд
    cDay = delta / SECONDS_IN_DAY;                                              //коррекция полных дней
    cSec = delta % SECONDS_IN_DAY;                                              //коррекция секунд
    aSec += cSec;
    if (aSec < 0) {
        cDay -= 1;
        aSec += SECONDS_IN_DAY;
    } else if (aSec >= SECONDS_IN_DAY) {
        cDay += 1;
        aSec -= SECONDS_IN_DAY;
    }
    time->tm_hour = aSec / 3600;
    aSec %= 3600;
    time->tm_min = aSec / 60;
    time->tm_sec = aSec % 60;

    aDay += cDay;
    time->tm_year = aDay / DAYS_IN_YEAR;                                        //кол-во лет
    do {
        cDay = (time->tm_year - 1) / 4;                                         //кол-во високосных лет
        cDay -= (time->tm_year - 1) / 100;                                      //коррекция / 100
        cDay += (time->tm_year - 1) / 400;                                      //коррекция / 400
        delta = cDay * (DAYS_IN_YEAR + 1);
        delta += ((time->tm_year - 1) - cDay) * DAYS_IN_YEAR;
        if (delta >= aDay)
            --time->tm_year;
    } while (delta >= aDay);
    time->tm_yday = aDay - delta;
    time->tm_wday = aDay % 7;

    cDay = 0;
    for (month = 1; month <= 12; month++) {
        char aDay = 0;
        if (month == 2)
            aDay = LEAPYEAR(time->tm_year);
        if (time->tm_yday > (cDay + _ytab[0][month - 1] + aDay))
            cDay += _ytab[0][month - 1] + aDay;
        else {
            time->tm_mon = month;
            time->tm_mday = time->tm_yday - cDay;
            break;
        }
    }
}

//------------------------------------------------------------------------------
//Вычисление дельты времени (в секундах) относительно 2000-го года
int32_t calcSecondsTime(RTCTime *time1) {
    int32_t Day;
    Day = CalcDay(time1);
    Day -= 730120;                                                              //дельта дней
    Day *= SECONDS_IN_DAY;
    Day += getsecond(time1);                                                    //дельта секунд
    return Day;
}

/**
 * Преобразование UNIX времени в человеческое
 * @param timer UNIX время (число)
 * @return человеческое время
 */
RTCTime UnixTimeToRTC(uint32_t time) {
    uint32_t vlm, dayno;
    RTCTime Time;

    vlm = (unsigned long)time % SECONDS_IN_DAY;
    dayno = (unsigned long)time / SECONDS_IN_DAY;

    Time.tm_sec = vlm % 60;
    Time.tm_min = (vlm % 3600) / 60;
    Time.tm_hour = vlm / 3600;
    Time.tm_wday = (dayno + 4) % 7;                                             /* day 0 was a thursday */
    Time.tm_year = EPOCH_YR;
    Time.tm_mon = 0;
    while (dayno >= (vlm = YEARSIZE(Time.tm_year))) {
        dayno -= vlm;
        Time.tm_year++;
    }
    Time.tm_yday = dayno;
    vlm = LEAPYEAR(Time.tm_year);
    while (dayno >= _ytab[vlm][Time.tm_mon]) {
        dayno -= _ytab[vlm][Time.tm_mon];
        Time.tm_mon++;
    }
    Time.tm_mon += 1;
    Time.tm_mday = dayno + 1;
    return Time;
}

/*------------------------------------------------------------------------------
 * Вычислить время в формате UNIX time из времени time
 * Возвращает число секунд
 */
int32_t GetUnixTime(RTCTime *time) {
    uint32_t unix;

    unix = CalcDay(time);
    unix -= DELTA_UNIX_TIME;                                                    //дельта дней (1985 лет - 1 день)
    unix *= SECONDS_IN_DAY;
    unix += getsecond(time);                                                    //дельта секунд
    return unix;
}

