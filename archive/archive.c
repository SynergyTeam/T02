/*------------------------------------------------------------------------------
 * ������������ �����
 */
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "struct.h"
#include "settings.h"
//#include "cpu_resource/watchdog.h"
#include "Drivers/spi_bus.h"
#include "archive/archive.h"
//#include "sflash/queue_mng.h"
//#include "communication/protocol_descr.h"
//#include "navigation/navigation.h"
//#include "dataoperation.h"
//#include "time_app.h"
#include "console/console.h"

//����������� ������� ������� ���������� � ������
#define MIN_ARCH_SIZE				(MIN_OUT_PACKET_SIZE + sizeof(ARec))
#define MAX_ARCH_SIZE				(MAX_OUT_PACKET_SIZE + sizeof(ARec))

//���������-��������� �������
#define GET_BUFFER                  ((char*)malloc(MAX_OUT_PACKET_SIZE))
#define FREE_BUFFER(x)              free(x)
#define USEFUL_DATA                 MAX_OUT_PACKET_SIZE

//�������� ����������
extern sys_events Events;

archive_str Archive[MAX_ARCHIVES];

/* ��������� ������� */
static uint32_t checking_sequence(uint16_t *PacketNum, archive_str *arch, char *buf, int bLen);
static int32_t checking_border(archive_str *arch, int nSize);

static uint32_t check_pckPacket(ARec *rec);
static uint16_t get_pckLen(uint16_t pLen);
static uint16_t get_unpckLen(uint16_t pLen);

/*------------------------------------------------------------------------------
 * ������������� ������.
 * �������������� ������������� ���� ������� � ������� ������, ��������� ������ �������
 */
void ArchiveChipInit(void) {
	uint32_t rsize;
	uint8_t arch;

	rsize = flash_init(ssi_FLASH);
	for (arch = 0; arch < MAX_ARCHIVES; ++arch) {
		Archive[arch].memory.start = (rsize > 0) ? StartBBoxData : rsize;
		Archive[arch].memory.end = rsize;
	}
}

/*------------------------------------------------------------------------------
 * ���������� ������ ������(��) � �������� ���������� ���������� ������ ������
 * ���� ��� ��������� �������� � ��������������� ���� ������ ��������� 0,
 * ���� �������������� ��������� ��������� ��������� 1
 */
char ArchiveGetBorder(uint8_t *percent, uint32_t *bData) {
	uint32_t  arcSize[MAX_ARCHIVES], maxSize, indx;
	archive_str *volatile arc;
	char aNum;

	//���������� ������� ������� ��������
	maxSize = 0;
	for (aNum = 0; aNum < MAX_ARCHIVES; ++aNum) {
		arc = &Archive[aNum];
		arcSize[aNum] = arc->memory.end - arc->memory.start;					//������ ��������� ������
		arcSize[aNum] *= (float)((float)percent[aNum] / 100.0);					//������ ��������
		if (percent[aNum] > 0) {												//��������� ����������� ������
			arcSize[aNum] = (arcSize[aNum] & ~FLASH_BLOCK_MASK) + FLASH_BLOCK_SIZE;
		}
		if(arcSize[aNum] > maxSize) {											//����� ������������� ��������
			maxSize = arcSize[aNum];
			indx = aNum;
		}
	}
	//��������� ������� ������������� �����
	maxSize = arc->memory.end - arc->memory.start;
	for (aNum = 0; aNum < MAX_ARCHIVES; ++aNum) {
		if(aNum == indx) continue;
		maxSize -= arcSize[aNum];
	}
	arcSize[indx] = maxSize;

	//������������� ������� ������� � ��������� ���������
	indx = 0;
	maxSize = 0;
	for (aNum = 0; aNum < MAX_ARCHIVES; ++aNum) {
		arc = &Archive[aNum];
		arc->memory.start += indx;
		arc->memory.end = arc->memory.start;
		if (arcSize[aNum] > 0)
			arc->memory.end += (arcSize[aNum] - 1);
		indx += arcSize[aNum];
		if (bData[aNum] < arc->memory.start || bData[aNum] > arc->memory.end) {
			bData[aNum] = arc->memory.start;
			maxSize = 1;
		}
	}
	return maxSize;
}

//------------------------------------------------------------------------------
// ��������� ����� ������ ������.
// ����������� �������� ������, �������� ������ � ������������ ���� ���������� ��������
void ArchiveSetBorder(sys_config *cfg) {
	uint32_t fSize, aN;

	//���������� ������� ������ ������
	for (fSize = 0, aN = 0; aN < MAX_ARCHIVES; ++aN) {
        QUEUE_LOCK(aN);                                                         //��������� �������� � ��������
		if(Archive[aN].memory.end > fSize)
			fSize = Archive[aN].memory.end;
	}
	//��������� ������������� ������
	for (aN = 0; aN < MAX_ARCHIVES; ++aN) {
		Archive[aN].memory.start = StartBBoxData;
		Archive[aN].memory.end = fSize;
	}
	//��������� ����� ������ � ������� ������
    ArchiveGetBorder(cfg->STR_percent, LGD.ptrArchive);
    for(aN = 0; aN < MAX_ARCHIVES; ++aN) {
        Archive[aN].bData = LGD.ptrArchive[aN];
        LGD.ptrArchive[aN] = ArchiveClean(aN);
        QUEUE_UNLOCK(aN);                                                       //��������� �������� � ���������
    }
}

/*------------------------------------------------------------------------------
 * ������� ������� ����������� � ������
 * ���������� ����� �������� ���������� ������ (� �������� ������������ 2000-�� ����)
 * ���� 0 ���� ��� �������.
 */
uint32_t ArchiveCalcSize(uint8_t aNum, uint32_t start, uint16_t *PacketNum) {
	archive_str *volatile str;
	uint32_t time;
	char *mBuf;

	str = &Archive[aNum];
    mBuf = GET_BUFFER;
	str->lData = str->bData = start;
	debuglog("Archive: Scan from address: %08X\r\n", start);
    time = checking_sequence(PacketNum, str, mBuf, USEFUL_DATA);
	start = check_cln_circ_buf(&str->memory, str->lData);
    if(start > 0) {
        uint32_t pntr;
        debuglog("Warning! Recovery struct of data page! Address 0x%X\r\n", str->lData);
        pntr = str->lData & ~FLASH_BLOCK_MASK;
        flash_copy(BBoxRecovery, pntr, start);
        flash_copy(pntr, BBoxRecovery, start);
    }
	str->tData = str->bData;
    FREE_BUFFER(mBuf);
#ifdef DEBUG
	ArchiveTxtInfo(aNum, mstr);
	debuglog(mstr);
#endif
	return time;
}

/* ������ � �������� ������ */
static uint32_t checking_sequence(uint16_t *PacketNum, archive_str *arch, char *buf, int bLen) {
    uint16_t plen, dlen;
    uint32_t tLen, timecode;
    ARec *rec;

    arch->timecode = 0;
    arch->packets = 0;
    arch->pLen = 0;
    if (arch->memory.start >= arch->memory.end)
        return (0);

	for(tLen = arch->memory.end - arch->memory.start; tLen > 0; ) {
		read_circ_memory(&arch->memory, buf, arch->lData, bLen);

		for (dlen = bLen; dlen > 0; ) {
			rec = (ARec*)(buf + (bLen - dlen));
			plen = rec->pLen;                                                   //������ ������
			// ����� ������ �� ��������� - ������ ���� ������� � �������� ������
			if(plen > dlen &&
					(plen >= MIN_ARCH_SIZE && plen <= MAX_ARCH_SIZE)) {
				dlen = 0; continue;
			}
            timecode = check_pckPacket(rec);                                    //����� �������� ������
            // ���� ����� "�����" - �����, �.�. ��������� ������ ������ ���
            if(!timecode) {
				debuglog("Archive: Incorrect timecode!\r\n");
				tLen = 0; break;
			}
            // ����� ���������� ������ ������ ���� ������ ��� ����� ������� ����������� ������
            if (arch->timecode != NULL) {
                if (timecode < arch->timecode) {
                    debuglog("Archive: Incorrect sequence the time of packets!\r\n");
                    tLen = 0; break;
                }
            }
			// ��������� ����������� �����
			arch->packets += 1;
			arch->timecode = timecode;
			arch->pLen = plen;
			*PacketNum = rec->pNum;
			arch->lData = shift_in_circ_buf(&arch->memory, arch->lData, plen);

			dlen -= plen;
			tLen -= plen;
		}
		wdog_Clear();
	}
	return arch->timecode;
}

//------------------------------------------------------------------------------
// C������� ������ �� ������� ������. anum - ����� ������
// ���������� ��������� �� ������ ������
#pragma FUNCTION_OPTIONS (ArchiveClean, "--opt_level=0")
uint32_t ArchiveClean(uint8_t aNum) {
	archive_str *volatile str;													//��������� ����������
	uint32_t space;

    QUEUE_LOCK(aNum);
    str = &Archive[aNum];
    space = ArchiveGetInfo(aNum, FULL_SIZE);
    if (space > 0) {
        str->lData = shift_in_circ_buf(&str->memory, str->bData, FLASH_BLOCK_SIZE);
        str->tData = flash_sect_erase(str->lData);
        str->lData = str->bData = str->tData;									//������������� ����������
        str->packets = str->pLen = 0;
        Events.StoreNavData = 1;												//���������� ���� ���������� ��������
    }
    QUEUE_UNLOCK(aNum);
    return str->bData;
}
/*------------------------------------------------------------------------------
 �������� ��������� ������ (�������������� �������) � ����� �������
------------------------------------------------------------------------------*/
void ArchiveTrDelay(uint16_t *timer, uint8_t aNum, uint8_t traffic_ctrl) {
	archive_str *volatile str;

	str = &Archive[aNum];
	if (str->tData == str->bData) {
		*timer = 0;
		if (traffic_ctrl != 0)
            *timer = 1;                                                         //TODO ������� �� ������������
	}
}

/*------------------------------------------------------------------------------
 * ���������� � ��������� ������. type - ��� ����������
 */
uint32_t ArchiveGetInfo(uint8_t aNum, a_info type) {
	archive_str *volatile str;
	int32_t size, fill_mem;

	str = &Archive[aNum];
	if(type == AMNT_PACKETS)
		return str->packets;
    if(type == DATA_POINTER)
        return str->memory.start;
	size = str->memory.end - str->memory.start;
	if (size > 0)
		++size;
	if(type == FULL_SIZE)
		return (size);
	fill_mem = str->lData - str->bData;
	if(fill_mem < 0)
		fill_mem = size - (str->bData - str->lData);
	if (type == DATA_SIZE)
		return fill_mem;
	return (size - fill_mem);
}

/*------------------------------------------------------------------------------
 ������ ������ ������ � �����.
 ���������� 0 ���� �������� ������ ������
 ���� ��������� > 0 ������ ���������� ������
 ���� ��������� < 0 ����� ������� �������
------------------------------------------------------------------------------*/
int SavePacket(uint8_t aNum, char *pData, uint16_t pLen) {
	archive_str *volatile str;													//��������� �� ���������
	int32_t result;
	char *mBuf;

    pLen = get_pckLen(pLen);                                                    //������ ��������� ������

    str = &Archive[aNum];
    //�������� ����������� ���������� ������
    if ((pLen >= MIN_ARCH_SIZE && pLen <= MAX_ARCH_SIZE) &&
            (pLen <= ArchiveGetInfo(aNum, FULL_SIZE))) {
        mBuf = GET_BUFFER;
        //�������� � ��������� ���������� ������ ������
#ifdef DEBUG
        ArchiveTxtInfo(aNum, mstr);
#endif
        result = checking_border(str, pLen);
        if (result < 0) {
#ifdef DEBUG
            debuglog(mstr);
#endif
            LGD.ptrArchive[aNum] = str->bData;									//������ n-�� ������
            SaveLastState();
        }
        //������ � ������ ������
        str->pLen = zipPacket(mBuf, pData);
        str->lData = write_circ_memory(&str->memory, mBuf, str->lData, str->pLen);
        ++str->packets;
        if(!result) result = str->pLen;
#ifdef DEBUG
        else {
            ArchiveTxtInfo(aNum, mstr);
            debuglog(mstr);
        }
#endif
        FREE_BUFFER(mBuf);
    }
	return result;
}

/*------------------------------------------------------------------------------
 * �������� � ��������� ������ ������ �������� ������
 * ���������� 0 ���� ��� ������ ����; ���� ������������� ����� ���������� �������
 */
static int32_t checking_border(archive_str *arch, int nSize) {
	uint32_t nLData;
    int32_t fill_mem;
	uint16_t dlen, plen;
	char *mBuf;

	if (!arch->packets || nSize <= 0)											//����� ����, ������������� ������ ������ - �����
		return (0);
	nLData = shift_in_circ_buf(&arch->memory, arch->lData, nSize);				//����� ������� ������
    // �������� ����� ������ ����� � �� ������ ������� ������ - �����
    if((arch->lData > arch->bData) && (arch->lData < nLData))
        return (0);
    /* �������� ������� 100%-�� ���������� ������ - ���� ����� �������� �����
     * �������� � ��������� ������ ������. � ���� ������ ��� �� �����������
     * ��������� ������ ������ � ������� ��������.
     */
    nSize = arch->memory.end - arch->memory.start;
    fill_mem = nLData - arch->bData;
    if(fill_mem < 0)
        fill_mem = nSize - (nLData - arch->lData);
    nSize -= fill_mem;
    if (nSize == 0) {
        nLData = arch->bData;
        fill_mem = arch->bData;
    } else fill_mem = 0;
	/* ���� �������� ����� ������ ������������ � ����������
	 * ��������� ����� ����� �� ��������� � ���� ���� � ���������.
	 * ���� ���������� ����������� ��������� ������ ������ ����� ����� �� ������ �����
	 */
	nSize = 0;
	nLData &= ~FLASH_BLOCK_MASK;
    mBuf = GET_BUFFER;
	while (nLData == (arch->bData & ~FLASH_BLOCK_MASK)) {
        read_circ_memory(&arch->memory, mBuf, arch->bData, USEFUL_DATA);
        for (dlen = USEFUL_DATA; dlen > 0; ) {
            plen = ((ARec*)(mBuf + (USEFUL_DATA - dlen)))->pLen;                //������ ������
            // �������� ������� ������. TODO �������� ���� ��������� ����������� ���������
            if (plen < MIN_ARCH_SIZE || plen > MAX_ARCH_SIZE) {
                nLData += FLASH_BLOCK_SIZE;
                nSize = 0;
            }
            // ����� ������ �� ��������� - ������ ���� ������� � �������� ������
			if(plen > dlen) {
				dlen = 0; continue;
			}
			arch->bData = shift_in_circ_buf(&arch->memory, arch->bData, plen);
			arch->packets -= 1;
			nSize += 1;
			dlen -= plen;
			if(nLData != (arch->bData & ~FLASH_BLOCK_MASK))
				break;
		}
		wdog_Clear();
	}
    if (fill_mem != NULL)
        flash_sect_erase(fill_mem);
    FREE_BUFFER(mBuf);
	return (-nSize);
}

/*------------------------------------------------------------------------------
 * ������ ������ �� ������ aLen � ������� pData, �������� �� ����� blen
 * ���������� ������ ������ � ������
 */
int ReadArchivePacket(uint8_t aNum, char *pData, uint16_t blen) {
	uint16_t plen, rlen, aLen;													//������������ ����������
	archive_str *volatile arch;
    ARec *rec;
	char *mBuf;

	arch = &Archive[aNum];
	if(!arch->packets) return (0);												//��� ������� - �����

    mBuf = GET_BUFFER;
    for(arch->tData = arch->bData, rlen = 0; blen > 0; ) {
        read_circ_memory(&arch->memory, mBuf, arch->bData, USEFUL_DATA);
        for (aLen = USEFUL_DATA; aLen > 0; ) {
            rec = (ARec*)(mBuf + (USEFUL_DATA - aLen));
            plen = rec->pLen;                                                   //������ ������
			// ����� ������ �� ��������� - ������ ���� ������� � �������� ������
			if(plen > aLen &&
					(plen >= MIN_ARCH_SIZE && plen <= MAX_ARCH_SIZE)) {
				aLen = 0; continue;
			}
            // ��� ����� � ������ - �����
            if ((blen - rlen) < get_unpckLen(plen)) {
                blen = 0; break;
            }
            // ������ ��� �������� ������
            if (check_pckPacket(rec) == NULL) {
                if (rlen == NULL) {
                    debuglog("Archive: skip %d bytes\r\n", plen);
                    arch->bData = shift_in_circ_buf(&arch->memory, arch->bData, plen);
                    arch->packets -= 1;
                }
                blen = 0; break;
            }
			// ��������� ���������, � ������� ��������� �����
			arch->tData = shift_in_circ_buf(&arch->memory, arch->tData, plen);
			aLen -= plen;
			// ��������� ��������� � ������� �������� �����
			plen = unzipPacket(pData, (char*)rec);
			pData += plen;
			rlen += plen;
		}
	}
    FREE_BUFFER(mBuf);
	return rlen;
}

/*------------------------------------------------------------------------------
 * �������� ������ �� ������ aNum � ������� packet
 * ��������� ������ ���������� ������, ���� -1 ���� ������ � ������ ���
 */
int DelArchivePacket(uint8_t aNum, uint16_t packet) {
	archive_str *volatile arch;													//������������ ����������
    ARec *rec;
	char *mBuf;

	arch = &Archive[aNum];
	if(!arch->packets) return (-1);												//��� �������, ������ �������

	//������� � �������� ������
    mBuf = GET_BUFFER;
    read_circ_memory(&arch->memory, mBuf, arch->bData, MAX_OUT_PACKET_SIZE);
    rec = (ARec*)mBuf;
    if(check_pckPacket(rec) && (packet == rec->pNum)) {                         //�������� ������ � ��� ������
        arch->packets -= 1;                                                     //������� �������
        packet = rec->pLen;                                                     //������ ������
        //������ � ��������� ���������� � ���������� �������
        arch->bData = shift_in_circ_buf(&arch->memory, arch->bData, packet);
        /* TODO
         * ����� ������������� �������� �������� ������� ����������������� ���������
         * ������ ������. ������ ���������� ����������� ���� ������ 100 ������,
         * ���� ������ ��������, ���� ������ 100 ������ ���� ������ �����
         */
        if(arch->tData == arch->bData) {
            if(!arch->packets)
                ArchiveClean(aNum);                                             //������� ������
            LGD.ptrArchive[aNum] = arch->bData;
        }
    } else packet = 0;
    FREE_BUFFER(mBuf);
    return ((packet > 0) ? packet : -1);
}
//------------------------------------------------------------------------------
//�������� ����������� ������.
//���� ����� ���������� ������������ ����� ��� �������� � �������� (������������ 2000 ����)
static uint32_t check_pckPacket(ARec *rec) {
    uint16_t aCRC, cCRC;
    uint32_t aTime;

    if ((rec->pLen < MIN_ARCH_SIZE) || (rec->pLen > MAX_ARCH_SIZE)) {
        debuglog("Archive: Incorrect packet size: %d byte(s)\r\n", rec->pLen);
        return (0);
    }

    aCRC = rec->pCRC;
    rec->pCRC = 0x0000;
    cCRC = CRCx1021((char*)rec, rec->pLen);
    aTime = (aCRC == cCRC) ? rec->pTime : 0;
    if(!aTime) {
        debuglog("Archive: Incorrect packet CRC %04X, expected %04X\r\n", cCRC, aCRC);
    }
    rec->pCRC = aCRC;
    return aTime;
}
//------------------------------------------------------------------------------
//���������� ������� ��������� ������
//������� ������: �������� ������ pLen
static uint16_t get_pckLen(uint16_t pLen) {
    return (pLen + sizeof(ARec));
}
//------------------------------------------------------------------------------
//���������� ������� �������� ������ �� ���������
static uint16_t get_unpckLen(uint16_t len) {
    return (len - sizeof(ARec));
}

/*------------------------------------------------------------------------------
 * ���������� ������ � ����������� � ������� � ������
 */
int ArchiveTxtInfo(char anum, char *msg) {
	archive_str *volatile str;
	int len;

	str = &Archive[anum];
	len = usprintf(msg, "\r\nARCHIVE #%d\r\nQUEUE:\r\n", 1 + anum);
//FIXME	len += usprintf(msg + len, "PACKETS:%d\r\n", Queue[anum].size);
	len += usprintf(msg + len, "FLASH:\r\nADR:0x%06X-0x%06X\r\n",str->memory.start,str->memory.end);
	len += usprintf(msg + len, "USE:0x%06X-0x%06X\r\n", str->bData, str->lData);
	len += usprintf(msg + len, "PACKETS:%d\r\n", str->packets);
	return len;
}
