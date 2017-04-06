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
//�������� �����/���� �� BCD ������ ������ (������/��������)
RTCTime GetPacketTime(char *tData) {
    uint8_t byte, tmp, value, error;
    RTCTime pTime;

    tmp = *tData++;                                                             //�����
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_mday = value;

    tmp = *tData++;                                                             //�����
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_mon = value;
    if (value < 1 || value > 12)
        error = 1;
    else
        error = 0;

    tmp = *tData++;                                                             //���
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_year = 2000 + value;

    byte = pTime.tm_mon;                                                        //�������� ���������� ����
    value = _ytab[0][byte-1];
    if(byte == 2)
        value += LEAPYEAR(pTime.tm_year);
    byte = pTime.tm_mday;
    if(byte < 1 || byte > value)
        error = 1;

    tmp = *tData++;                                                             //����
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_hour = value;
    if(value > 23) error = 1;
    tmp = *tData++;                                                             //������
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_min = value;
    if(value > 59) error = 1;
    tmp = *tData++;                                                             //�������
    value = ((tmp >> 4) * 10) + (tmp & 0x0F);
    pTime.tm_sec = value;
    if(value > 59) error = 1;
    if(error)
        pTime = FW_time;
    return pTime;
}
//------------------------------------------------------------------------------
//���������� ����� � ��������� ������ � ������������ � ���������
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
//��������� ���������� ����� ��� � ������ ����� ���
uint32_t CalcDay(RTCTime *time) {
    uint32_t dayAmount, yearDay, month;

    yearDay = (time->tm_year - 1) / 4;                                          //���-�� ���������� ���
    yearDay -= (time->tm_year - 1) / 100;                                       //��������� / 100
    yearDay += (time->tm_year - 1) / 400;                                       //��������� / 400
    dayAmount = yearDay * 366;
    dayAmount += ((time->tm_year - 1) - yearDay) * DAYS_IN_YEAR;

    yearDay = 0;
    if(time->tm_mon > 1) {
        for(month = 1; month < time->tm_mon; ++month)
            yearDay += _ytab[0][month-1];                                       //����� ������ �������
        if(time->tm_mon > 2 )
            yearDay += LEAPYEAR(time->tm_year);                                 //��������� �������� ����������� ����
    }
    yearDay += time->tm_mday;
    time->tm_yday = yearDay;
    dayAmount += yearDay;
    time->tm_wday = dayAmount % 7;
    return dayAmount;
}

//------------------------------------------------------------------------------
//��������� ���������� ����� ������� � ������ ���
uint32_t getsecond(RTCTime *time) {
    uint32_t second;
    second = time->tm_hour * 3600;
    second += time->tm_min * 60;
    second += time->tm_sec;
    return second;
}

//------------------------------------------------------------------------------
//��������� ������ ������� � �������� ����� time1 � time2 (� �������� ����!!!)
int32_t CalcDeltaTime(RTCTime *time1, RTCTime *time2) {
    int32_t Day1, Day2, Sec1, Sec2;
    Day1 = CalcDay(time1); Sec1 = getsecond(time1);
    Day2 = CalcDay(time2); Sec2 = getsecond(time2);
    Day1 -= Day2;                                                               //������ ����
    Day2 = DAYS_IN_YEAR + LEAPYEAR(time1->tm_year + 1);                         //�����������
    if(Day1 > Day2) {
        Day1 = Day2; Sec2 = Sec1;
    } else {
        Day2 = DAYS_IN_YEAR + LEAPYEAR(time1->tm_year - 1);
        if(Day1 < -Day2) {
            Day1 = -Day2; Sec2 = Sec1;
        }
    }
    Day1 *= SECONDS_IN_DAY;
    Day1 += (Sec1 - Sec2);                                                      //������ ������
    return Day1;
}

//------------------------------------------------------------------------------
//��������������� ����� �� ������
void TimeCorrection(RTCTime *time, int32_t delta) {
    int32_t aDay, aSec, cDay, cSec, month;
    aDay = CalcDay(time);                                                       //����� ����
    aSec = getsecond(time);                                                     //����� ������
    cDay = delta / SECONDS_IN_DAY;                                              //��������� ������ ����
    cSec = delta % SECONDS_IN_DAY;                                              //��������� ������
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
    time->tm_year = aDay / DAYS_IN_YEAR;                                        //���-�� ���
    do {
        cDay = (time->tm_year - 1) / 4;                                         //���-�� ���������� ���
        cDay -= (time->tm_year - 1) / 100;                                      //��������� / 100
        cDay += (time->tm_year - 1) / 400;                                      //��������� / 400
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
//���������� ������ ������� (� ��������) ������������ 2000-�� ����
int32_t calcSecondsTime(RTCTime *time1) {
    int32_t Day;
    Day = CalcDay(time1);
    Day -= 730120;                                                              //������ ����
    Day *= SECONDS_IN_DAY;
    Day += getsecond(time1);                                                    //������ ������
    return Day;
}

/**
 * �������������� UNIX ������� � ������������
 * @param timer UNIX ����� (�����)
 * @return ������������ �����
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
 * ��������� ����� � ������� UNIX time �� ������� time
 * ���������� ����� ������
 */
int32_t GetUnixTime(RTCTime *time) {
    uint32_t unix;

    unix = CalcDay(time);
    unix -= DELTA_UNIX_TIME;                                                    //������ ���� (1985 ��� - 1 ����)
    unix *= SECONDS_IN_DAY;
    unix += getsecond(time);                                                    //������ ������
    return unix;
}

