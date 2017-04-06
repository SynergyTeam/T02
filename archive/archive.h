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
void ArchiveChipInit(void);
char ArchiveGetBorder(char *percent, uint32_t *bData);
void ArchiveSetBorder(sys_config *cfg);
uint32_t ArchiveCalcSize(char aNum, uint32_t start, uint16_t *PacketNum);
uint32_t ArchiveClean(char anum);
void ArchiveTrDelay(uint16_t *timer, char aNum, char traffic_ctrl);
uint32_t ArchiveGetInfo(char aNum, a_info used);

int SavePacket(char aNum, char *pData, uint16_t pLen);
int ReadArchivePacket(uint16_t aNum, char *pData, uint16_t pLen);
int DelArchivePacket(char aNum, uint16_t packet);

int ArchiveTxtInfo(char anum, char *msg);

#endif /* __ARCHIVE__ */

