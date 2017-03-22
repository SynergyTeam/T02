//------------------------------------------------------------------------------
/* Header files */
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
//#include "driverlib/debug.h"

#include "config.h"
#include "struct.h"
#include "console/console.h"
//#include "cpu_resource/hardware.h"
//#include "cpu_resource/watchdog.h"
//#include "navigation/navigation.h"
//#include "communication/modem.h"
//#include "communication/protocol_descr.h"
//#include "power_mng.h"
#include "sflash/archive.h"
//#include "peripherals/accel/ensure/EnsTelematics.h"
#include "console/data_opr.h"
//#include "time_app.h"
#include "settings.h"

#ifdef TARGET_IS_BLIZZARD
//������� ����������
extern sys_events Events;

sys_config *config = (sys_config*)FLASH_CONFIG;
char *tanks_ptr = (char*)(FLASH_CONFIG - (MAX_LIQUID_LEVEL_SENSOR * 1024));
#endif
#ifdef TARGET_IS_SNOWFLAKE
char dTanksTbl[MAX_LIQUID_LEVEL_SENSOR * 1024], *tanks_ptr = dTanksTbl;
sys_config dConfig, *config = (sys_config*)&dConfig;
#endif

last_state LGD;
uint32_t AdrA, AdrB;

//��������� ��� ������ last_state
const char lPnt_str[] = {"lPnt"};

static uint32_t GetPrvPoint(last_state *lgd, uint32_t Adr);
static settings_err check_lst_struct (last_state *rPnt);
static char CheckSettings(sys_config *cfg);
static void DefaultCFGx00(sys_config *cfg);
static void DefaultCFGx01(sys_config *cfg);
static char ChangeSettings(sys_config *cfg);


/*------------------------------------------------------------------------------
 * ������ ��������� ��������.
 * ���������� true ��� �������� ������, false ��� ������
 */
char ReadSettings(nav_solution *nSolution) {
    char err_a, err_b, cfgStat;
    sys_config *cfg;
    last_state *lgd = &LGD;
    uint32_t att;

    cfg = (sys_config*)malloc(sizeof(sys_config));
    // ������������� ���������� ����������� (��� ������� CRC)
    memset(lgd, 0, sizeof(*lgd));
    lgd->prvSolution.time = FW_time;
    memcpy(lgd->name, lPnt_str, sizeof(lPnt_str)-1);
    // ����� ���������� ������������ ���������
    AdrA = GetPrvPoint(lgd, GeoidAdrA);
    AdrB = GetPrvPoint(lgd, GeoidAdrB);
    // ����������� ����� �� ��������� - ������
    cfgStat = check_lst_struct(lgd);
    nSolution->solution = &lgd->prvSolution;
    nSolution->park_pos = nSolution->solution->loc;
    nSolution->Parking = 1;														//���� �������
    nSolution->sync_geoid = 0;
    nSolution->type_time = LastConfig;											//����� ��������� ������������
    if (cfgStat == sett_ok)
        UpdateNavigationData(nSolution->solution, 1);
    for(err_a = 0xFF, att = 0; err_a && att < 5; ++att) {						//���� ������ � ��������
        flash_read((char*)config, SettingsCopy, sizeof(*config), flash);		//��������� ���������� �� FLASH
        err_a = CheckSettings(config);											//�������� CRC
    }
    for(err_b = 0xFF, att = 0; err_b && att < 5; ++att) {						//���� ������ � ��������
        flash_read((uint8_t*)cfg, SettingsAdr, sizeof(*cfg));                   //��������� ���������� �� FLASH
        err_b = CheckSettings(cfg);												//�������� CRC
    }
    if(err_a || err_b) {
        if(err_a < err_b) {														//� ������ ����� ������ ������ - ���������� ��
            memcpy(cfg, config, sizeof(*config));
            err_b = err_a;
        }
        DefaultSettings(cfg, err_b);											//�������� �� ���������
        SaveSettings(cfg, 0);
    }
    else {
        if(ChangeSettings(cfg)) SaveSettings(cfg, 0);
    }
    err_b |= cfgStat;

    if(err_b & based)
        memset(tanks_ptr, 0xFF, LLS_TABLE_SIZE * MAX_LIQUID_LEVEL_SENSOR);
    else
        flash_read(tanks_ptr, FLASH_LLS_TABLE, LLS_TABLE_SIZE * MAX_LIQUID_LEVEL_SENSOR, flash);
    att = Task_disable();
    memcpy(config, cfg, sizeof(*config));
    Task_restore(att);
    free(cfg);
    return (err_b);
}
//------------------------------------------------------------------------------
//���������� ��������� ��������
void SaveSettings(sys_config *cfg, uint8_t delay) {
    uint16_t len;
    uint32_t key;
    char *ptr;

    //���������� ����������� �����
    ptr = (char*) &cfg->id_len + sizeof(cfg->id_len);
    cfg->id_len = (char*) &cfg->id_crc - ptr;
    cfg->id_crc = CRCx1021(ptr, cfg->id_len);

    ptr = (char*) &cfg->bd_len + sizeof(cfg->bd_len);
    cfg->bd_len = (char*) &cfg->bd_crc - ptr;
    cfg->bd_crc = CRCx1021(ptr, cfg->bd_len);

    //���������� �������� �� FLASH ����������
    len = sizeof(*cfg);
    if (!delay) {
        //���������� �������� �� ������� FLASH
        mfile_write((char*)cfg, SettingsAdr, sizeof(*cfg), flash);
        mfile_write((char*)cfg, SettingsCopy, sizeof(*cfg), flash);
    }
    key = Task_disable();
    memmove(config, cfg, len);
    Task_restore(key);
}
//------------------------------------------------------------------------------
//���������� ������������� ���������� � ��������
void SaveLastState(void) {
	last_state *lgd;															//��������� ����������
	circ_buf sett;
	char *ptr;
	uint16_t len;

	lgd = &LGD;																	//���������� ����������� �����
	CalcDay(&lgd->prvSolution.time);
	ptr = (char*)&lgd->gd_len + sizeof(lgd->gd_len);
	lgd->gd_len = (char*)&lgd->gd_crc - ptr;
	lgd->gd_crc = CRCx1021(ptr, lgd->gd_len);
	len = sizeof(*lgd);

	debuglog("Store settings at address %X (copy A)\r\n", AdrA);
	sett.start = GeoidAdrA;
	sett.end = GeoidAdrA + ST_GEOID_SIZE;
	AdrA = write_circ_memory(&sett, (char*)lgd, AdrA, len);						//������ ������ (����� 1)

	debuglog("Store settings at address %X (copy B)\r\n", AdrB);
	sett.start = GeoidAdrB;
	sett.end = GeoidAdrB + ST_GEOID_SIZE;
	AdrB = write_circ_memory(&sett, (char*)lgd, AdrB, len);						//������ ������ (����� 2)
}
/*------------------------------------------------------------------------------
 * �������� ��������� ������
 */
static settings_err check_lst_struct (last_state *rPnt) {
	uint16_t len, crc;
	char *ptr;
	ptr = (char*)&rPnt->gd_len + sizeof(rPnt->gd_len);
	len = (char*)&rPnt->gd_crc - ptr;
	if(len != rPnt->gd_len)
		return (shorttime);
	crc = CRCx1021(ptr, len);
	ptr += len;
	len = ptr[0] | (ptr[1] << 8);
	if(len != crc)
		return (shorttime);
	return (sett_ok);
}

//------------------------------------------------------------------------------
//������ ���������� ��������� �������
static uint32_t GetPrvPoint(last_state *lgd, uint32_t Adr) {
    uint32_t nAdr, rAdr, bufSize;
	circ_buf sett;
	last_state *rgd;
	int bLen, tLen;
    char *ptr, *mBuf;

	sett.start = rAdr = Adr;
	sett.end = Adr + ST_GEOID_SIZE;
    bufSize = 0x200;
    mBuf = malloc(bufSize);

	for(tLen = 0, bLen = 0; tLen <= ST_GEOID_SIZE; tLen += bLen) {
        read_circ_memory(&sett, mBuf, Adr, bufSize);
        ptr = mBuf;
		nAdr = Adr;

        while ((bLen = bufSize - (ptr - mBuf)) > 0) {
			if(bLen < sizeof(*lgd)) break;										//���� ������ ������ ������ ������� �������� - �����
			ptr = SearchStr(ptr, bLen, (char*)lPnt_str);
            if(ptr == NULL) {
                ptr = mBuf + bufSize;
				if(bLen > sizeof(*lgd))
					nAdr = shift_in_circ_buf(&sett, nAdr, bLen);
				continue;
			}
			ptr -= ((char*)&lgd->name - (char*)&lgd->gd_len);					//��������� �� ������ ���������
			rgd = (last_state*)ptr;
			if(check_lst_struct(rgd) == sett_ok) {
				ptr += sizeof(*lgd);
                nAdr = shift_in_circ_buf(&sett, Adr, (ptr - mBuf));             //���������� ����� � ������
                if(rgd->prvSolution.time.RTC_Year != 2080)                      //������ �� 80-�� ����
                    if(CalcDeltaTime(&rgd->prvSolution.time, &lgd->prvSolution.time) >= 0) {
                        memmove(lgd, rgd, sizeof(*lgd));
                        rAdr = nAdr;
                    }
			}
			else
				ptr += (sizeof(lPnt_str) - 1);
		}
        if(bLen < bufSize) {
            bLen = bufSize - bLen;
        }
		Adr = nAdr;
		wdog_Clear();
	}
    free(mBuf);
	debuglog("Used settings from address %X\r\n", rAdr - sizeof(*lgd));

	Adr = check_cln_circ_buf(&sett, rAdr);
    if(Adr > 0) {
        debuglog("Warning! Recovery struct of data page! Address %X\r\n", rAdr);
        nAdr = rAdr & ~FLASH_BLOCK_MASK;
        flash_copy(BBoxRecovery, nAdr, Adr);
        flash_copy(nAdr, BBoxRecovery, Adr);
    }

	return rAdr;
}

//------------------------------------------------------------------------------
//�������� ����������� �����
static char CheckSettings(sys_config *cfg) {
	uint16_t len, crc;  														//��������� ����������
	char err, *ptr;
	err = 0x00;

	/* �������� �������� �������� ID, IP, APN � �� */
	ptr = (char*) &cfg->id_len + sizeof(cfg->id_len);
	len = (char*) &cfg->id_crc - ptr;
	if (len != cfg->id_len)
		err |= primary;
	else {
		crc = CRCx1021(ptr, len);
		ptr += len;
		len = ptr[0] | (ptr[1] << 8);
		if (len != crc)
			err |= primary;
	}
	/* �������� ����������� �������� */
	ptr = (char*) &cfg->bd_len + sizeof(cfg->bd_len);
	len = (char*) &cfg->bd_crc - ptr;
	if (len != cfg->bd_len)
		err |= based;
	else {
		crc = CRCx1021(ptr, len);
		ptr += len;
		len = ptr[0] | (ptr[1] << 8);
		if (len != crc)
			err |= based;
	}

	return err;
}
//------------------------------------------------------------------------------  
void DefaultSettings(sys_config *cfg, char error) {
    char aN;
    if(error & primary) {
        DefaultCFGx00(cfg);                                                     //��� �������� ��������
        for(aN = 1; aN < MAX_ARCHIVES; ++aN)
            LGD.ptrArchive[aN] = StartBBoxData;
    }
    if(error & based) DefaultCFGx01(cfg);
    ChangeSettings(cfg);
}

//------------------------------------------------------------------------------
//�������� ��������� �� ���������
static void DefaultCFGx00(sys_config *cfg) {
	uint16_t len;
	char *ptr;
	ptr = (char*)&cfg->id_len + sizeof(cfg->id_len);
	len = (char*)&cfg->id_crc - ptr;
	memset(ptr, 0, len);														//��������� �������
	memset(cfg->id, '0', sizeof(cfg->id));
	memset(cfg->M_cmd_number, '0', 7);											//�������������� ����� SMS ����������
	memset(cfg->M_call_number, 0x00, sizeof(cfg->M_call_number));               //����� ������ ����������. ��-���������, ������
	cfg->server[0].ip[0] = 195;													//������ Space Team
	cfg->server[0].ip[1] = 133;
	cfg->server[0].ip[2] = 87;
	cfg->server[0].ip[3] = 90;
	cfg->server[0].port = 7000;
	cfg->server_events[0] = 0xFFFF;
	cfg->STR_percent[0] = 100;
}
  
//------------------------------------------------------------------------------
//������� ��������� �� ���������
static void DefaultCFGx01(sys_config *cfg) {
    int16_t amnt;
    char *ptr;
    ptr = (char*)&cfg->bd_len + sizeof(cfg->bd_len);
    amnt = (char*)&cfg->bd_crc - ptr;
    memset(ptr, 0, amnt);														//��������� �������

    cfg->RCV_valid_delay = 4;
    cfg->RCV_hdop = 40;
    cfg->RCV_vdop = 13;
    cfg->NAV_start_motion = 4;
    cfg->NAV_null_speed = 37;
    cfg->NAV_max_speed = 110;
    cfg->NAV_dCourse = 19;
    cfg->NAV_mInterval[0] = 60;													//������������ ������� ������ ��� 1-�� �������
    cfg->NAV_pInterval[0] = 300;
    cfg->NAV_parking_evaluation = 300;

    cfg->NAV_2D_solution = 1;													//���������������� 2D �� ���������
    cfg->NAV_rCourse = 0;														//������� �� ��������� ����� � ��������

    cfg->M_data_package = 32;                                                   //����������� ������ ����� ������
    cfg->M_hconnections = 5;													//��������� ����������
    cfg->M_rconnections = 4;													//����� �������� � ��������
//    cfg->M_check_int = 150;
    cfg->M_ack_time = 3;
    cfg->M_mic_level = 5;
    cfg->M_spk_level = 10;
    
    cfg->COM_channel_gsm = 1;													//������� ����� GSM
    
    cfg->BT_Name[0] = '\0';                                                     //BT-��� �� ������
    cfg->BT_Addr[0] = '\0';                                                     //BT-����� �� �����
    cfg->M_auto_answer  = 0;                                                    //��������� �������� �� ���������
    cfg->M_voice_interface = VOICE_BUILDIN_HW;                                  //���������� ����������
    cfg->UINP_cfg = 0x36;                                                       //UINP1,2: 0-40V, ��� ��������. UINP3,4: 0-40V, ��� ��������
    cfg->UINP_work_mode = 0x1111;                                               //UINP1-4 ����������
//    cfg->DINP_pull_up = 1;
    cfg->DINP_inversion = 0x1EFF0;                                              //��������� �����
    memset(cfg->DINP_pls_time, 1, (HW_PLSINP + HW_UINP));                       //����� �������� ���������
#ifdef UINP_AD
    memset(cfg->DINP_pls_time, 1, HW_PLSINP);                                   //����� �������� ���������
#endif
    for(amnt = 0; amnt < SERIAL_PORTS; ++amnt) {
        cfg->UART[amnt].speed = 19200;                                          //��������� ����������������� �����
        cfg->UART[amnt].word_length = 8;
        cfg->UART[amnt].parity = 0;
        cfg->UART[amnt].stop_length = 1;
        cfg->UART[amnt].flow_control = 0;
        cfg->UART[amnt].XON = 17;
        cfg->UART[amnt].XOFF = 19;
        cfg->UART[amnt].port_use = 0;
    }

#if MAX_LIQUID_LEVEL_SENSOR > 0
    cfg->LLS_pInt = 30;
    cfg->LLS_rInt = 300;
    for(amnt = 0; amnt < MAX_LIQUID_LEVEL_SENSOR; ++amnt) {
        cfg->LLS[amnt].n_adr = 1 + amnt;
        cfg->LLS[amnt].delay_on = 15;
        cfg->LLS[amnt].delay_off = 30;
    }
#endif /* MAX_LIQUID_LEVEL_SENSOR */
#if CAN_BUS > 0
    /* CAN ���� */
    //��������
    cfg->CAN.bitrate = 250;
    // ��������� �����
    cfg->CAN.requests_mode = 0;
    //����� ������� ��� �������� ����� ����� �������� (���)
    cfg->CAN.period[0] = 60;
    cfg->CAN.period[1] = 300;
#endif
#ifdef ACCELEROMETER
    //������������� ������������� "��-���������"
    cfg->Accel = def_Accel;
    cfg->SlowMotionThld = 4;
#endif
    cfg->PWR_battery_chr = 60;                                                  //����� ������ ������������
    cfg->BAT_modem_off = (BATTERY_POOR_LEVEL - 0.1) * 10;

    cfg->DVC_console_echo = 0;
#ifdef HIBERNATE
    cfg->HIB_wakeup_int = 2;
    cfg->HIB_on_duty_time = 1;
    cfg->HIB_on = 0;
    cfg->HIB_on_reset = 1;
    cfg->HIB_on_source = 0;
    cfg->HIB_wakeup_source = 1;
    cfg->HIB_dinp_source = 0;
#endif /* HIBERNATE */
}
//------------------------------------------------------------------------------
//����������� ��������
static char ChangeSettings(sys_config *cfg) {
    char change = 0x00;                                                         //���� ��������� ��������
    if(cfg->version != SYSTEM_VER) {
        change = 0x01;
    }
    //��������� � ������ 9.02
    if(cfg->version < 902) {
        cfg->M_data_package = 32;                                               //����������� ������ ����� ������
        memset(cfg->M_call_number, 0, sizeof(cfg->M_call_number));
    }
    //��������� � ������ 9.04
    if (cfg->version < 904) {
        if (cfg->SlowMotionThld == 2)
            cfg->SlowMotionThld = 4;                                            //���������������� �������������
    }
    if(change) {
        cfg->version = SYSTEM_VER;                                              //��������� ���������
    }
    return change;
}

/*------------------------------------------------------------------------------
 * ���������� ��������� ������������ (��������) �������
 * dest - ��������� �� ������ ������ ��� ���������
 * src - ��������� �� ���� � �����������, size - ��� ������
 */
void ChangeCfg(char *dest, char *src, int size) {
    sys_config *cfg;
    ASSERT(dest >= (char*)config && dest < ((char*)config + sizeof(*config)));
    cfg = (sys_config*)malloc(sizeof(sys_config));
    dest = (char*)cfg + (dest - (char*)config);
    memcpy(cfg, config, sizeof(*config));
    memcpy(dest, src, size);
    SaveSettings(cfg, 0);
    free(cfg);
}

/*------------------------------------------------------------------------------
 * ���������� ������������ ������� ������� �������
 * ��� TM4C123 �������� ����������� �� FLASH, ��� TM4C129 �� ������� ������
 * ptr - ����� ��� ����������, table - ����� �������
 */
void SaveTankTable(uint32_t ptr, char *table, int32_t len) {
#ifdef TARGET_IS_BLIZZARD
    ptr &= (~0x3FF);
    MAP_FlashProtectSet(ptr, FlashReadWrite);
    MAP_FlashErase(ptr);														//������� ��������
    if(len > 0)
        MAP_FlashProgram((uint32_t*)table, ptr, len);
#endif
#ifdef TARGET_IS_SNOWFLAKE
    uint32_t tNum, offset, idx;
    tNum = (ptr - (uint32_t)tanks_ptr) / LLS_TABLE_SIZE;						//����� �������
    offset = tNum / (FLASH_BLOCK_SIZE / LLS_TABLE_SIZE);						//����� �������� � ������� ��� ��������
    offset = FLASH_LLS_TABLE + (FLASH_BLOCK_SIZE * offset);
    tNum = tNum % (FLASH_BLOCK_SIZE / LLS_TABLE_SIZE);
    memset((void*)ptr, 0xFF, LLS_TABLE_SIZE);
    if(len > 0)
        memcpy((void*)ptr, table, len);
    flash_copy(BBoxRecovery, offset, FLASH_BLOCK_SIZE);							//�������� �������� ���������
    for (idx = 0; idx < (FLASH_BLOCK_SIZE / LLS_TABLE_SIZE); ++idx) {
        ptr = idx * LLS_TABLE_SIZE;
        if(idx == tNum) {
            if(len > 0)
                mfile_write(table, offset + ptr, LLS_TABLE_SIZE, flash);
            continue;
        }
        flash_copy(offset + ptr, BBoxRecovery + ptr, LLS_TABLE_SIZE);			//�������� �������
    }
#endif
}
/*------------------------------------------------------------------------------
 * �������� ��������� �� ������ �������� "type" �������� ��������� "code" �� ���������
 */
const char *Opr25001[def_ncfg] = {"internet.mts.ru", "mts", "mts"};
const char *Opr25002[def_ncfg] = {"internet", "", ""};
const char *Opr25007[def_ncfg] = {"internet.smarts.ru", "", ""};
const char *Opr25017[def_ncfg] = {"internet.usi.ru", "", ""};
const char *Opr25020[def_ncfg] = {"internet.tele2.ru", "tele2", "tele2"};
const char *Opr25039[def_ncfg] = {"internet.usi.ru", "", ""};
const char *Opr25099[def_ncfg] = {"internet.beeline.ru", "beeline", "beeline"};
const char *OprJasper[def_ncfg] = {"m2m.beeline.ru", "beeline", "beeline"};
//const char sms_CenterJasper[]={"+79037030621"};
const char *DefOpr[def_ncfg] = {"internet", "", ""};

char *default_nCfg(uint32_t code, net_cfg type) {
	switch(code) {
	case 25001:
		return (char*)Opr25001[type];
	case 25002:
		return (char*)Opr25002[type];
	case 25007:
		return (char*)Opr25007[type];
	case 25017:
		return (char*)Opr25017[type];
	case 25020:
		return (char*)Opr25020[type];
	case 25039:
		return (char*)Opr25039[type];
	case 25099:
		return (char*)Opr25099[type];
	default:
		return (char*)DefOpr[type];
	}
}
