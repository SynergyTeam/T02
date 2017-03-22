/******************************************************************************/
//

#ifndef __AT_CONFIG__
#define __AT_CONFIG__

//�������� ������� �����
#define PrjSignature                0x32303153

//���� �������� ��������
#define FW_YEAR                     2016
#define FW_MON                      11
#define FW_DAY                      11

//������� ������
#define SYSTEM_VER                  904

//����������� ���������� ������
#define HW_UINP                     2
#define HW_DINP                     0
#define HW_PLSINP                   0
#define HW_DOUT                     1
#define HW_AINP                     0

//������ ������ ������������� ������
//#define UINP_AD                                                                 //UINP: ����������, ����������
#define UINP_ADP                                                                //UINP: ����������, ����������, �������

#define UINP_GROUP                  2                                           //���-�� ����� � UINP ������

//���������� ���������������� ������ � �� ��������
#define SERIAL_PORTS                1                                           //���-��
#define SERIAL_485                  0                                           //����� �����

//#define HW_TAMPER                                                               //������ �������� �������

//���������� CAN ������
//#define CAN_BUS                     2

//CAN LOG �� RS232
#define CAN_LOG_BUS

// ������������ ������ ��������� ����� CAN
//#define TACHO_SUPPORT

//�������������� ���� ����������
//#define ST8088                      37
#define MTK3333                     38

//������� ����� ��������� �������
//#define BACKUP_BATTERY

//��������� �������������
//#define ACCELEROMETER
//#define ACCELEROMETER_DEBUG

//���������� �����������
//#define PWR_BATTERY

//����� HIBERNATE
//#define HIBERNATE

//����� �������� � ����������������� �����
#define StandbyTime                 300

//����������/������ ���������� �������
#define UPDATE

//����� ����������� ����
#ifdef DEBUG
#define debuglog                    ConsoleMes
#define debugarray(...)
#else
#define debuglog(...)
#define debugarray(...)
#endif

//����� ������������ ���� ������ ������ ������
//#define MODEM_ERR_REPORT
//����� ���������� ������ � �������
//#define SHOW_BUILT_PACKAGE
//#define MODEM_DEBUG
//����� ��������� ���� � �������
//#define MODEM_CONSOLE_ECHO

//����� SIM ����
#define SIM_HOLDERS                 2
/* ��� SIM ����� */
#define CHIP_SIM                    1
#define STANDART_SIM                2
#define SIM_TYPE                    STANDART_SIM
/*����� ����������� ��������� �� ������� ��������*/
//#define LBS_ENABLE
/*���������� �������� ���������*/
#define VOICE_AMPLIFIER

//������ �����
#define GPRS_communication          2

/***** ���������� ������� (� ��������) ******/
#ifdef WiFi_communication
#ifdef SAT_communication
#define MAX_ARCHIVES                3
#else
#define MAX_ARCHIVES                2
#endif /* SAT_communication */
#else
#ifdef SAT_communication
#define MAX_ARCHIVES                2
#else
#define MAX_ARCHIVES                1
#endif /* SAT_communication */
#endif /* WiFi_communication */

//������������ ������ ������ ������
#define MIN_IN_PACKET_SIZE          9
#define MAX_IN_PACKET_SIZE          0x400
//������ �������� ������ � ������ x16 ��������->������
#define X16_PACKET_SIZE             768

#define MIN_OUT_PACKET_SIZE         28
#define STD_OUT_PACKET_SIZE         36
#define MAX_OUT_PACKET_SIZE         768
#define MAX_PACKET_QUEUE            4


//����������� ������ ��������
#define LLS_RANGE                   10                                          //������ � ���
#define LLS_TERM_RANGE              34                                          //����������� � ���
#define ANALOG_RANGE                42
#define RFID_RANGE                  50
#define LIST_GPS_SATELLITES         58
#define LIST_GNSS_SATELLITES        59
#define COUNTER_RANGE               90
#define TACHO_RANGE                 106
#define NEW_DINP_RANGE              122
#define CAN_BIT_RANGE               218
#define CAN_DATA_RANGE              346
#define WIREID_RANGE                730

// ������ �������� ��� �������������
#define ACCEL_ANALOG_RANGE          60
// ������ �������� ��� �������������
#define ACCEL_DIGITAL_RANGE         154

/*********************** ��������� ������� ���� Omnicom ***********************/

//������������ ���������� ������������
#define MAX_LIQUID_LEVEL_SENSOR     8

/*************************����������� RFID ����� ******************************/

//������������ ���������� ������������
#define MAX_RFID_READERS            4

//������� ����� ������� �����������
#define NET_ADR_RFID_READERS        33

/*************************** ����� WireID *************************************/

//������������ ���������� ������������
#define MAX_WIREID_READERS          4

//������� ����� ������� �����������
#define NET_ADR_WIREID_READERS      37

#endif /* __AT_CONFIG__ */
