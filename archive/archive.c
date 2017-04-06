/*------------------------------------------------------------------------------
 * Подключаемые файлы
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

//стандартные размеры пакетов хранящийся в архиве
#define MIN_ARCH_SIZE				(MIN_OUT_PACKET_SIZE + sizeof(ARec))
#define MAX_ARCH_SIZE				(MAX_OUT_PACKET_SIZE + sizeof(ARec))

//платформо-зависимые макросы
#define GET_BUFFER                  ((char*)malloc(MAX_OUT_PACKET_SIZE))
#define FREE_BUFFER(x)              free(x)
#define USEFUL_DATA                 MAX_OUT_PACKET_SIZE

//внешнние переменные
extern sys_events Events;

archive_str Archive[MAX_ARCHIVES];

/* локальные функции */
static uint32_t checking_sequence(uint16_t *PacketNum, archive_str *arch, char *buf, int bLen);
static int32_t checking_border(archive_str *arch, int nSize);

static uint32_t check_pckPacket(ARec *rec);
static uint16_t get_pckLen(uint16_t pLen);
static uint16_t get_unpckLen(uint16_t pLen);

/*------------------------------------------------------------------------------
 * Инициализация архива.
 * Осуществляется инициализация шины доступа к внешней памяти, установка границ архивов
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
 * Вычисление границ архива(ов) и проверка валидности указателей начала данных
 * Если все указатели попадают в соответствующие окна архива возращает 0,
 * если осуществлялась коррекция указателя возращает 1
 */
char ArchiveGetBorder(uint8_t *percent, uint32_t *bData) {
	uint32_t  arcSize[MAX_ARCHIVES], maxSize, indx;
	archive_str *volatile arc;
	char aNum;

	//вычисление размера каждого сегмента
	maxSize = 0;
	for (aNum = 0; aNum < MAX_ARCHIVES; ++aNum) {
		arc = &Archive[aNum];
		arcSize[aNum] = arc->memory.end - arc->memory.start;					//размер доступной памяти
		arcSize[aNum] *= (float)((float)percent[aNum] / 100.0);					//размер сегмента
		if (percent[aNum] > 0) {												//учитываем архитектуру памяти
			arcSize[aNum] = (arcSize[aNum] & ~FLASH_BLOCK_MASK) + FLASH_BLOCK_SIZE;
		}
		if(arcSize[aNum] > maxSize) {											//поиск максимального сегмента
			maxSize = arcSize[aNum];
			indx = aNum;
		}
	}
	//коррекция размера максимального блока
	maxSize = arc->memory.end - arc->memory.start;
	for (aNum = 0; aNum < MAX_ARCHIVES; ++aNum) {
		if(aNum == indx) continue;
		maxSize -= arcSize[aNum];
	}
	arcSize[indx] = maxSize;

	//устанавливаем границы архивов и проверяем указатель
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
// Установка новых границ архива.
// Выполняется разметка архива, стирание архива и выставляется флаг сохранения настроек
void ArchiveSetBorder(sys_config *cfg) {
	uint32_t fSize, aN;

	//вычисление полного объема памяти
	for (fSize = 0, aN = 0; aN < MAX_ARCHIVES; ++aN) {
        QUEUE_LOCK(aN);                                                         //блокируем операции с очередью
		if(Archive[aN].memory.end > fSize)
			fSize = Archive[aN].memory.end;
	}
	//начальная инициализация границ
	for (aN = 0; aN < MAX_ARCHIVES; ++aN) {
		Archive[aN].memory.start = StartBBoxData;
		Archive[aN].memory.end = fSize;
	}
	//установка новых границ и очистка памяти
    ArchiveGetBorder(cfg->STR_percent, LGD.ptrArchive);
    for(aN = 0; aN < MAX_ARCHIVES; ++aN) {
        Archive[aN].bData = LGD.ptrArchive[aN];
        LGD.ptrArchive[aN] = ArchiveClean(aN);
        QUEUE_UNLOCK(aN);                                                       //разрешаем операции с оччередью
    }
}

/*------------------------------------------------------------------------------
 * Подсчет пакетов сохраненных в архиве
 * Возвращает время создания последнего пакета (в секундах относительно 2000-го года)
 * либо 0 если нет пакетов.
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

/* чтение и проверка пактов */
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
			plen = rec->pLen;                                                   //размер пакета
			// пакет считан не полностью - читаем блок начиная с текущего пакета
			if(plen > dlen &&
					(plen >= MIN_ARCH_SIZE && plen <= MAX_ARCH_SIZE)) {
				dlen = 0; continue;
			}
            timecode = check_pckPacket(rec);                                    //время создания пакета
            // Если пакет "битый" - выход, т.к. проверять дальше смысла нет
            if(!timecode) {
				debuglog("Archive: Incorrect timecode!\r\n");
				tLen = 0; break;
			}
            // Время следующего пакета должно быть больше или равно времени предыдущего пакета
            if (arch->timecode != NULL) {
                if (timecode < arch->timecode) {
                    debuglog("Archive: Incorrect sequence the time of packets!\r\n");
                    tLen = 0; break;
                }
            }
			// учитываем проверенный пакет
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
// Cтирание архива во внешней памяти. anum - номер архива
// Возвращает указатель на начало архива
#pragma FUNCTION_OPTIONS (ArchiveClean, "--opt_level=0")
uint32_t ArchiveClean(uint8_t aNum) {
	archive_str *volatile str;													//временные переменные
	uint32_t space;

    QUEUE_LOCK(aNum);
    str = &Archive[aNum];
    space = ArchiveGetInfo(aNum, FULL_SIZE);
    if (space > 0) {
        str->lData = shift_in_circ_buf(&str->memory, str->bData, FLASH_BLOCK_SIZE);
        str->tData = flash_sect_erase(str->lData);
        str->lData = str->bData = str->tData;									//переустановка указателей
        str->packets = str->pLen = 0;
        Events.StoreNavData = 1;												//выставляем флаг сохранения настроек
    }
    QUEUE_UNLOCK(aNum);
    return str->bData;
}
/*------------------------------------------------------------------------------
 Проверка состояния архива (подтвержденных пакетов) и сброс таймера
------------------------------------------------------------------------------*/
void ArchiveTrDelay(uint16_t *timer, uint8_t aNum, uint8_t traffic_ctrl) {
	archive_str *volatile str;

	str = &Archive[aNum];
	if (str->tData == str->bData) {
		*timer = 0;
		if (traffic_ctrl != 0)
            *timer = 1;                                                         //TODO функция не используется
	}
}

/*------------------------------------------------------------------------------
 * Информация о состоянии архива. type - тип информации
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
 Запись пакета данных в архив.
 возвращает 0 если неверный размер пакета
 если результат > 0 размер записанных данных
 если результат < 0 число стертых пакетов
------------------------------------------------------------------------------*/
int SavePacket(uint8_t aNum, char *pData, uint16_t pLen) {
	archive_str *volatile str;													//указатель на хранилище
	int32_t result;
	char *mBuf;

    pLen = get_pckLen(pLen);                                                    //размер архивного пакета

    str = &Archive[aNum];
    //проверка возможности сохранения пакета
    if ((pLen >= MIN_ARCH_SIZE && pLen <= MAX_ARCH_SIZE) &&
            (pLen <= ArchiveGetInfo(aNum, FULL_SIZE))) {
        mBuf = GET_BUFFER;
        //проверка и коррекция укаазателя начала архива
#ifdef DEBUG
        ArchiveTxtInfo(aNum, mstr);
#endif
        result = checking_border(str, pLen);
        if (result < 0) {
#ifdef DEBUG
            debuglog(mstr);
#endif
            LGD.ptrArchive[aNum] = str->bData;									//начало n-го архива
            SaveLastState();
        }
        //сжатие и запись пакета
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
 * Проверка и коррекция границ начала архивных данных
 * Возвращает 0 если все данные целы; либо отрицательное число потерянных пакетов
 */
static int32_t checking_border(archive_str *arch, int nSize) {
	uint32_t nLData;
    int32_t fill_mem;
	uint16_t dlen, plen;
	char *mBuf;

	if (!arch->packets || nSize <= 0)											//архив пуст, отрицательный размер пакета - выход
		return (0);
	nLData = shift_in_circ_buf(&arch->memory, arch->lData, nSize);				//новая граница данных
    // Конечный адрес растет вверх и не достиг границы архива - выход
    if((arch->lData > arch->bData) && (arch->lData < nLData))
        return (0);
    /* Проверка условия 100%-го заполнения архива - если новый конечный адрес
     * совпадет с последним байтом архива. В этом случае так же увеличиваем
     * указатель начала архива и стираем страницу.
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
	/* Если конечный адрес данных приближается к начальному
	 * проверяем новый адрес на попадание в один блок с начальным.
	 * Если необходимо увеличиваем указатель начала архива чтобы выйти из общего блока
	 */
	nSize = 0;
	nLData &= ~FLASH_BLOCK_MASK;
    mBuf = GET_BUFFER;
	while (nLData == (arch->bData & ~FLASH_BLOCK_MASK)) {
        read_circ_memory(&arch->memory, mBuf, arch->bData, USEFUL_DATA);
        for (dlen = USEFUL_DATA; dlen > 0; ) {
            plen = ((ARec*)(mBuf + (USEFUL_DATA - dlen)))->pLen;                //размер пакета
            // Проверка размера пакета. TODO Возможно надо проверять целостность структуры
            if (plen < MIN_ARCH_SIZE || plen > MAX_ARCH_SIZE) {
                nLData += FLASH_BLOCK_SIZE;
                nSize = 0;
            }
            // Пакет считан не полностью - читаем блок начиная с текущего пакета
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
 * Чтение данных из архива aLen в область pData, размером не более blen
 * возвращает размер данных в буфере
 */
int ReadArchivePacket(uint8_t aNum, char *pData, uint16_t blen) {
	uint16_t plen, rlen, aLen;													//используемые переменные
	archive_str *volatile arch;
    ARec *rec;
	char *mBuf;

	arch = &Archive[aNum];
	if(!arch->packets) return (0);												//нет пакетов - выход

    mBuf = GET_BUFFER;
    for(arch->tData = arch->bData, rlen = 0; blen > 0; ) {
        read_circ_memory(&arch->memory, mBuf, arch->bData, USEFUL_DATA);
        for (aLen = USEFUL_DATA; aLen > 0; ) {
            rec = (ARec*)(mBuf + (USEFUL_DATA - aLen));
            plen = rec->pLen;                                                   //размер пакета
			// Пакет считан не полностью - читаем блок начиная с текущего пакета
			if(plen > aLen &&
					(plen >= MIN_ARCH_SIZE && plen <= MAX_ARCH_SIZE)) {
				aLen = 0; continue;
			}
            // нет места в буфере - выход
            if ((blen - rlen) < get_unpckLen(plen)) {
                blen = 0; break;
            }
            // ошибка при проверке пакета
            if (check_pckPacket(rec) == NULL) {
                if (rlen == NULL) {
                    debuglog("Archive: skip %d bytes\r\n", plen);
                    arch->bData = shift_in_circ_buf(&arch->memory, arch->bData, plen);
                    arch->packets -= 1;
                }
                blen = 0; break;
            }
			// Коррекция указателя, и размера архивного блока
			arch->tData = shift_in_circ_buf(&arch->memory, arch->tData, plen);
			aLen -= plen;
			// Коррекция указателя и размера эфирного блока
			plen = unzipPacket(pData, (char*)rec);
			pData += plen;
			rlen += plen;
		}
	}
    FREE_BUFFER(mBuf);
	return rlen;
}

/*------------------------------------------------------------------------------
 * Удаление пакета из архива aNum с номером packet
 * Возращает размер удаленного пакета, либо -1 если пакета в архиве нет
 */
int DelArchivePacket(uint8_t aNum, uint16_t packet) {
	archive_str *volatile arch;													//используемые переменные
    ARec *rec;
	char *mBuf;

	arch = &Archive[aNum];
	if(!arch->packets) return (-1);												//нет пакетов, нечего удалять

	//чтениие и проверка пакета
    mBuf = GET_BUFFER;
    read_circ_memory(&arch->memory, mBuf, arch->bData, MAX_OUT_PACKET_SIZE);
    rec = (ARec*)mBuf;
    if(check_pckPacket(rec) && (packet == rec->pNum)) {                         //проверка пакета и его номера
        arch->packets -= 1;                                                     //счетчик пакетов
        packet = rec->pLen;                                                     //размер пакета
        //чтение и коррекция информации о оставшихся пакетах
        arch->bData = shift_in_circ_buf(&arch->memory, arch->bData, packet);
        /* TODO
         * После подтверждения доставки архивных пакетов переустанавливаем указатель
         * начала архива. Данная информация сохраняется либо каждые 100 метров,
         * если объект движется, либо каждые 100 секунд если объект стоит
         */
        if(arch->tData == arch->bData) {
            if(!arch->packets)
                ArchiveClean(aNum);                                             //очистка архива
            LGD.ptrArchive[aNum] = arch->bData;
        }
    } else packet = 0;
    FREE_BUFFER(mBuf);
    return ((packet > 0) ? packet : -1);
}
//------------------------------------------------------------------------------
//Проверка целостности пакета.
//Если пакет нормальный возвращается время его создания в секундах (относительно 2000 года)
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
//Вычисление размера архивного пакета
//входные данные: исходный размер pLen
static uint16_t get_pckLen(uint16_t pLen) {
    return (pLen + sizeof(ARec));
}
//------------------------------------------------------------------------------
//Вычисление размера эфирного пакета из архивного
static uint16_t get_unpckLen(uint16_t len) {
    return (len - sizeof(ARec));
}

/*------------------------------------------------------------------------------
 * Подготовка строки с информацией о очереди и архиве
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
