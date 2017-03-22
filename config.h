/******************************************************************************/
//

#ifndef __AT_CONFIG__
#define __AT_CONFIG__

//цифровая подпись файла
#define PrjSignature                0x32303153

//дата создания прошивки
#define FW_YEAR                     2016
#define FW_MON                      11
#define FW_DAY                      11

//текущая версия
#define SYSTEM_VER                  904

//определение аппаратной версии
#define HW_UINP                     2
#define HW_DINP                     0
#define HW_PLSINP                   0
#define HW_DOUT                     1
#define HW_AINP                     0

//режимы работы универсальных входов
//#define UINP_AD                                                                 //UINP: аналоговые, дискретные
#define UINP_ADP                                                                //UINP: аналоговые, дискретные, счетные

#define UINP_GROUP                  2                                           //Кол-во групп в UINP входах

//количество последовательных портов и их описание
#define SERIAL_PORTS                1                                           //кол-во
#define SERIAL_485                  0                                           //номер порта

//#define HW_TAMPER                                                               //Датчик открытия корпуса

//количество CAN портов
//#define CAN_BUS                     2

//CAN LOG на RS232
#define CAN_LOG_BUS

// обрабатывать данные тахографа через CAN
//#define TACHO_SUPPORT

//поддерживаемые типы приемников
//#define ST8088                      37
#define MTK3333                     38

//батарея часов реального времени
//#define BACKUP_BATTERY

//обработка акселерометра
//#define ACCELEROMETER
//#define ACCELEROMETER_DEBUG

//встроенный аккумулятор
//#define PWR_BATTERY

//режим HIBERNATE
//#define HIBERNATE

//времы перехода в энергосберегающий режим
#define StandbyTime                 300

//обновление/полная компиляция проекта
#define UPDATE

//вывод отладочного лога
#ifdef DEBUG
#define debuglog                    ConsoleMes
#define debugarray(...)
#else
#define debuglog(...)
#define debugarray(...)
#endif

//вывод расширенного кода ошибок ответа модема
//#define MODEM_ERR_REPORT
//вывод собранного пакета в консоль
//#define SHOW_BUILT_PACKAGE
//#define MODEM_DEBUG
//вывод модемного лога в консоль
//#define MODEM_CONSOLE_ECHO

//число SIM карт
#define SIM_HOLDERS                 2
/* Тип SIM карты */
#define CHIP_SIM                    1
#define STANDART_SIM                2
#define SIM_TYPE                    STANDART_SIM
/*опция определения координат по базовым станциям*/
//#define LBS_ENABLE
/*аппаратный звуковой усилитель*/
#define VOICE_AMPLIFIER

//каналы связи
#define GPRS_communication          2

/***** количество архивов (и серверов) ******/
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

//максимальный размер пакета данных
#define MIN_IN_PACKET_SIZE          9
#define MAX_IN_PACKET_SIZE          0x400
//Размер полезных данных в пакете x16 терминал->сервер
#define X16_PACKET_SIZE             768

#define MIN_OUT_PACKET_SIZE         28
#define STD_OUT_PACKET_SIZE         36
#define MAX_OUT_PACKET_SIZE         768
#define MAX_PACKET_QUEUE            4


//назначенные адреса датчиков
#define LLS_RANGE                   10                                          //данные с ДУЖ
#define LLS_TERM_RANGE              34                                          //температура с ДУЖ
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

// Номера датчиков для акселерометра
#define ACCEL_ANALOG_RANGE          60
// Номера даткиков для акселерометра
#define ACCEL_DIGITAL_RANGE         154

/*********************** топливные датчики типа Omnicom ***********************/

//максимальное количество считывателей
#define MAX_LIQUID_LEVEL_SENSOR     8

/*************************Считыватели RFID меток ******************************/

//максимальное количество считывателей
#define MAX_RFID_READERS            4

//сетевой адрес первого считывателя
#define NET_ADR_RFID_READERS        33

/*************************** метки WireID *************************************/

//максимальное количество считывателей
#define MAX_WIREID_READERS          4

//сетевой адрес первого считывателя
#define NET_ADR_WIREID_READERS      37

#endif /* __AT_CONFIG__ */
