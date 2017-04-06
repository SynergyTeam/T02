/*----------------------------------------------------------------------------*/
//������������ ��������� ������
/*----------------------------------------------------------------------------*/
#ifndef _STRUCT_
#define _STRUCT_

#include <inttypes.h>
#include <time.h>
#include "config.h"

#define PACKED __attribute__ ((packed))

//*****************************************************************************
//���� ��������� �������
typedef struct tm RTCTime;

//�������������� ���������� ����� (������ + �������)
typedef struct geoid_t {
    float       Long;
    float       Lat;
} geoid;

//�������� ������� ����� (�����, ����������, ��������, ���� � ��.)
typedef struct navpos_t {
    RTCTime     time;
    geoid       loc;
    float       altitude;
    float       velocity;
    float       distance;
    uint16_t    course;
    uint8_t     sat_info;                   // ����� ��������� ����������� � ������������� �������
    uint8_t     hDop;                       // ������ �������������� �������� * 10
    uint8_t     vDop;                       // ������ ������������ �������� * 10
    uint8_t     valid_flag;
} nav_pos;

//��������� ��������� �����
typedef struct laststate_t {
    uint16_t    gd_len;
    char            name[4];
    uint16_t        idFile;
    nav_pos         prvSolution;
    uint32_t        ptrArchive[MAX_ARCHIVES];
    uint8_t         lastOutput;
    uint16_t    gd_crc;
} last_state;

//������������ ����� �������
typedef enum nav_time {
    Firmware = 1,                           // ����� �������� ��������
    LastConfig,                             // ����� ��������� ������������ �������
    RealTimeClock,                          // ����� �� ����� ��������� �������
    RCV_HotStart,                           // ����� �� ��������� (���������)
    CMD_Server,                             // ����� � �������
    RCV_FirstFix                            // ����� �� ��������� ��� �������� �����������
} nav_time;

//��������� ������������� ������� (�������� ������� ����� + ���������� ����������)
typedef struct nav_solution {
    nav_pos     *solution;

    uint16_t    fs_timer;
    uint16_t    mf_timer;
    uint16_t    mf_deep_timer;
    uint16_t    delay_parking;
    uint16_t    thCourse;                   // �������� ���������� ��������� �����

    geoid       park_pos;
    geoid       dSdt_pos;
    float       cVelocity;
    float       dDistance;

    unsigned    sync_geoid :1;
    unsigned    sync_hDop :1;
    unsigned    sync_vDop :1;
    unsigned    sync_mfilter :1;
    unsigned    sync_park_point:1;
    unsigned    show_current_point:1;
    unsigned    MotionFilter:4;
    unsigned    Parking:1;
    unsigned    valid_solution:1;
    unsigned    zone_compare:1;             // ���� ���������� ������ ����� ������� (�������� ����������)
    uint8_t     lost_valid_solution;

    char        delay_solution;
    nav_time    type_time;
    char        sat_number[33];
} nav_solution;

//��������� ������������ ������� ������
typedef struct {
    uint16_t    m_inp;
    uint16_t    s_inp;
} master_slave;

//��������� ������� � "��������" ����� ��� "����������" ��������
typedef struct {
    uint8_t     inp;
    uint8_t     num;
    int16_t     vlm[5];
} thresholds;

//�������� SIM �����
typedef struct {
    char        pin[4];
    char        apn[48];
    char        login[24];
    char        pass[24];
} sim_data;

//�������� �����
typedef struct gsm_info {
    unsigned    sim_num:8;
    unsigned    mcc_code:12;
    unsigned    mnc_code:12;
} gsm_info;

//�������� �������
typedef struct {
    uint8_t     ip[4];
    uint16_t    port;
//    uint8_t     skey[16];
} server_data;

//��������� ����� UART
typedef struct uartcfg_t {
    uint32_t    speed;
    uint8_t     word_length;
    uint8_t     XON;
    uint8_t     XOFF;
    unsigned    parity:4;
    unsigned    stop_length:2;
    unsigned    flow_control:2;
    unsigned    port_use:2;
    unsigned    echo_mode:2;
    unsigned    device:4;
} uartcfg;

//��������� ������������ �������� ������ ��������
typedef struct {
    uint8_t     avg;
    uint8_t     n_adr;
    uint8_t     delay_on;
    uint8_t     delay_off;
    uint16_t    volume;
    uint16_t    volume_adc;
} lls_cfg;

//��������� ���������������� ����������
typedef struct {
    uint16_t id_len;
    char        id[16];
    sim_data    sim_cadr[SIM_HOLDERS];
    char        M_sms_center[14];
    char        M_cmd_number[14];
    server_data server[MAX_ARCHIVES];
    uint16_t    server_events[MAX_ARCHIVES];
    uint8_t     STR_percent[MAX_ARCHIVES];
    unsigned    COM_channel_gsm:MAX_ARCHIVES;
    uint16_t    version;
    uint16_t id_crc;

    uint16_t bd_len;
    char            RCV_type;
    char            RCV_satellite_mode;
    char            RCV_valid_delay;
    char            RCV_hdop;
    char            RCV_vdop;

    uint8_t         NAV_gz_relation;        // ����� ������
//    struct geoid    NAV_geo_zona[10];
    char            NAV_start_motion;
    char            NAV_null_speed;
    char            NAV_max_speed;
    char            NAV_dCourse;
    char            NAV_dDistance;
    uint8_t         null_byte1_1;
    uint16_t        NAV_mInterval[MAX_ARCHIVES];
    uint16_t        NAV_pInterval[MAX_ARCHIVES];
    uint16_t        NAV_parking_evaluation;
    uint16_t        NAV_rmc_build;

    uint32_t        M_priority_net[10];
    gsm_info        M_forbidden_net[10];
    char            M_call_number[14];
    uint32_t        M_data_interval;
    uint8_t         M_data_package;
    uint8_t         M_hconnections;
    uint8_t         M_rconnections;
    uint8_t         null_byte2_1;
    uint16_t        M_check_int;
    uint16_t        M_ack_time;

    char            M_mic_level;
    char            M_spk_level;
    char            BT_Name[16];
    char            BT_Addr[18];

    char            INP_ignition_key;
    char            OUT_amplifier;
    char            OUT_ring;
    char            OUT_speed_lim;

    uartcfg         UART[SERIAL_PORTS];

    uint16_t        DINP_inversion;
    uint16_t        DINP_tremble[(HW_DINP + HW_UINP)];
    master_slave    DINP_dependence[(HW_DINP + HW_UINP)];
    uint16_t        DINP_pulse;
    char            DINP_pls_per_turn[HW_PLSINP + HW_UINP];
    char            DINP_pls_time[HW_PLSINP + HW_UINP];
    thresholds      DINP_d_thld[HW_PLSINP + HW_UINP];

    uint8_t         UINP_work_mode;
    char            UINP_cfg;
    char            AINP_used;
    char            AINP_norm_inp;
    char            AINP_pr_delay[HW_AINP + HW_UINP];
    char            AINP_averaging[HW_AINP + HW_UINP];
    uint16_t        AINP_events_timer;
    uint16_t        AINP_ref_U;
    thresholds      AINP_a_thld[4];

#if MAX_LIQUID_LEVEL_SENSOR > 0
    lls_cfg         LLS[MAX_LIQUID_LEVEL_SENSOR];
    uint16_t        LLS_rInt;
    uint32_t        LLS_send;
    char            LLS_pInt;
    char            LLS_Sum1[MAX_LIQUID_LEVEL_SENSOR];
    char            LLS_Sum2[MAX_LIQUID_LEVEL_SENSOR / 2];
#endif /* MAX_LIQUID_LEVEL_SENSOR */

#if MAX_RFID_READERS > 0
    char            RFID_n_adr[MAX_RFID_READERS];
    char            ID_events;
#endif /* MAX_RFID_READERS */

#if MAX_WIREID_READERS > 0
    char            WIRE_n_adr[MAX_WIREID_READERS];
#endif /* MAX_WIREID_READERS */

    uint16_t        NAV_zInterval;          //�������� ������� ������ �������
//    EnsParType      Accel;

#ifdef HIBERNATE
    uint16_t        HIB_wakeup_int;         //�������� �����������
    uint16_t        HIB_on_duty_time;       //����������������� ��������������
    char            HIB_on;                 //��������� ������ HIBERNATE
    unsigned        HIB_on_reset:6;         //��������� ����������� ������ HIBERNATE
    unsigned        HIB_on_source:2;        //��������� ���������
    unsigned        HIB_wakeup_source:6;    //��������� ��������� �����������
    uint32_t        HIB_dinp_source;        //��������� �� ������
#endif

    uint8_t         BAT_modem_off;          //����� ���������� ������
    uint8_t         PWR_battery_chr;        //����� ���������� ������� ����������� ������������
    uint8_t         PWR_suspend_mode;       //����� �������� � ����� ����������� �����������������
    unsigned        NAV_2D_solution:1;
    unsigned        NAV_true_position:1;
    unsigned        NAV_rCourse:1;

    unsigned        DVC_suspend_mode:2;
    unsigned        M_comm_mode:2;          //����� ����� � �������� COMMUNICATION_MODE
    unsigned        DVC_console_echo:1;
    unsigned        NAV_sat_list:1;
    unsigned        M_auto_answer:1;        //����� ���������� ������ �� �������� ������ (1 - ���, 0- ����.)

    unsigned        M_voice_interface:1;    //��� ���������� ��� ������� �����
    unsigned        Tacho_hide_info:2;      //�������� - �������� ������������ ������

    unsigned        SlowMotionThld:8;       //���������������� �������������
    uint16_t bd_crc;
 } sys_config;

//��������� �������
typedef struct sys_events {
    uint32_t pcktSource;
    uint8_t  saving_tmr;
    unsigned restart_device:1;
    unsigned device_cfg:1;
    unsigned con_echo:1;
    unsigned usb_bridge:3;
    volatile unsigned uart_echo:1;

    unsigned OneTenth:1;
    unsigned NewSecond:1;

    unsigned file_type:4;
    unsigned file_dwn:4;                    //��������� state ������ �������� �����

    volatile unsigned init_serial[SERIAL_PORTS];
    unsigned lls_data:1;
    unsigned lls_debug:1;
    unsigned rfid_data:1;
    unsigned rfid_debug:1;
    unsigned wireid_data:1;
    unsigned wireid_debug:1;

    unsigned accel_ready:1;
    unsigned OverMaxSpeed:1;
    unsigned geo_info:1;                    //���������� � �������
    unsigned StoreNavData:1;
    unsigned WrongNavData:1;                //������� �������� ������� � ������ ���������� ���������
    unsigned IncVoiceCall:1;
    unsigned AmplifierState:1;
    unsigned can_debug:4;
    unsigned EnterHib:1;
    unsigned EnterHibLowPwr:1;
    volatile unsigned DialNumberChanged:1;
} sys_events;

#endif
