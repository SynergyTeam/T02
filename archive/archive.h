#ifndef __ARCHIVE__
#define __ARCHIVE__

#include "struct.h"
#include "Drivers/spi_flash/flash_25xxx.h"
#include <archive/circ_memory.h>

//Информация о состоянии архива
typedef enum a_info {
	FULL_SIZE = 0,                  //подсчет размера архива
	FREE_SIZE,                      //свободного пространства в архиве
	DATA_SIZE,                      //размер данных в архиве
	AMNT_PACKETS,                   //число пакетов
	DATA_POINTER                    //указатель на начало данных
} a_info;

//определяем структуру описания архива
typedef struct archive_str {
    circ_buf         memory;
    uint32_t         bData;
    uint32_t         tData;
    uint32_t         lData;
    uint32_t         packets;
    uint32_t         timecode;
    uint16_t         pLen;
} PACKED archive_str;

// структура архивной записи
typedef struct rec_header {
    uint16_t    pLen;                                                           //размер записи (архивного пакета)
    uint16_t    pCRC;                                                           //контрольная сумма
    uint16_t    pNum;                                                           //номер записи
    uint32_t    pTime;                                                          //время пакета
} PACKED ARec;

extern archive_str Archive[MAX_ARCHIVES];

/* функции */
extern void ArchiveChipInit(void);
extern uint8_t ArchiveGetBorder(uint8_t *percent, uint32_t *bData);
extern void ArchiveSetBorder(sys_config *cfg);
extern uint32_t ArchiveCalcSize(uint8_t aNum, uint32_t start, uint16_t *PacketNum);
extern uint32_t ArchiveClean(uint8_t anum);
extern void ArchiveTrDelay(uint16_t *timer, uint8_t aNum, uint8_t traffic_ctrl);
extern uint32_t ArchiveGetInfo(uint8_t aNum, a_info used);

extern int32_t SavePacket(uint8_t aNum, uint8_t *pData, uint16_t pLen);
extern int32_t ReadArchivePacket(uint8_t aNum, uint8_t *pData, uint16_t blen);
extern int32_t DelArchivePacket(uint8_t aNum, uint16_t packet);

extern int32_t ArchiveTxtInfo(uint8_t anum, char *msg);

#endif /* __ARCHIVE__ */

