#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include "struct.h"

// Размер структур данных
#define SETTINGS_SIZE           0x1000	//0x04B0	/* sizeof(sys_config) */
#define ST_GEOID_SIZE           0x2000	//0x004A	/* sizeof(last_state) */
#define LLS_TABLE_SIZE          0x400

// Адреса данных во внешней памяти
#define SettingsAdr             0x0000
#define SettingsCopy            (SettingsAdr + SETTINGS_SIZE)
#define GeoidAdrA               (SettingsCopy + SETTINGS_SIZE)
#define GeoidAdrB               (GeoidAdrA + ST_GEOID_SIZE)
#define FLASH_LLS_TABLE         (GeoidAdrB + ST_GEOID_SIZE)
#define BBoxRecovery            (FLASH_LLS_TABLE + 0x2000)
#define StartBBoxData           (BBoxRecovery + ST_GEOID_SIZE)

//Карта используемой FLASH памяти процессора
#define FLASH_FW_LOADER         0x000FC000

//весовая оценка ошибок
typedef enum settings_err {
	sett_ok = 0,
	shorttime = 0x08,
	based = 0x20,
	primary = 0x40,
} settings_err;

//настройки оператора по умолчанию
typedef enum net_cfg {
	def_apn = 0,
	def_login,
	def_pass,

	def_ncfg
} net_cfg;

extern sys_config  *config;
extern uint8_t     *tanks_ptr;
extern last_state   LGD;

extern uint8_t ReadSettings(nav_solution *nSolution);
extern void DefaultSettings(sys_config *cfg, uint8_t error);
extern void SaveSettings(sys_config *cfg, uint8_t delay);
extern void SaveLastState(void);
extern void ChangeCfg(uint8_t *dest, uint8_t *src, uint16_t size);
extern void SaveTankTable(uint32_t ptr, uint8_t *table, int32_t len);
extern char *default_nCfg(uint32_t code, net_cfg type);


#endif /* __SETTINGS_H__ */

