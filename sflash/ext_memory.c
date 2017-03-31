/*----------------------------------------------------------------------------*/
/* Работа с последовательной памятью */
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
//#include <xdc/std.h>
//#include <xdc/runtime/System.h>

//#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/gates/GateMutex.h>

#include <Drivers/spi/SPI.h>
#include <Drivers/spi/SPITivaDMA.h>

//#include "cpu_resource/hardware.h"
//#include "cpu_resource/spi_bus.h"
#include "ext_memory.h"

//------------------------------------------------------------------------------
//Поддерживаемые производители
#define MACRONIX                    (0xC2)
#define MXSMIO                      (0x20)

#define WINBOND                     (0xEF)
#define BUS_MODE                    (0x40)

//макросы доступа
#define MEMORY_LOCK                 (xSemaphoreTake(Flash.gate, portMAX_DELAY))
#define MEMORY_UNLOCK               (xSemaphoreGive(Flash.gate))

//структура данных драйвера памяти
typedef struct {
    SPI_HandleTypeDef*  handle;         // SPI шина
    SemaphoreHandle_t   gate;           // mutex доступа
    uint8_t             cmdbuf[5];      // буфер команды
    uint8_t             *rdData;        // указатель на буфер чтения данных
    uint8_t             *wrData;        // указатель на буфер записи данных
    uint32_t            size;           // размер данных для записи/чтения
} t_FlTrans;

//------------------------------------------------------------------------------
/* буфер обмена данными */

/* локальные переменные */
static t_FlTrans Flash;

/* локальные функции */
static void wr_enable(t_FlTrans *flash);
static uint32_t flash_info(t_FlTrans *flash);

/*------------------------------------------------------------------------------
 * CallBack функция-драйвер доступа к внешней памяти
 * Вызывается из драйвера после окончания транзакции на шине
 * В первой транзакции передается команда, в последующих осуществляется обмен данными
 */
void flash_state_maсhine(SPI_Handle handle, SPI_Transaction *transaction) {
    SPITivaDMA_Object *object = handle->object;
    uint16_t len;

    if (Flash.size == NULL) {
        xSemaphoreGive(object->transferComplete);
    }
    else {
        object->transaction = transaction;
        len = (Flash.size > 1024) ? 1024 : Flash.size;
        transaction->count = len;
        if (Flash.wrData != NULL) {
            transaction->txBuf = Flash.wrData;
            Flash.wrData += len;
        }
        if (Flash.rdData != NULL) {
            transaction->rxBuf = Flash.rdData;
            Flash.rdData += len;
        }
        Flash.size -= len;
        SPI_DMA_transfer(Flash.handle, transaction);
    }
}

/*------------------------------------------------------------------------------
 * Инициализация внешней памяти
 * Выполняется настройка SSI интерфейса, чтение JEDEC кода и тд.
 * Возвращает размер памяти
 */
uint32_t flash_init(uint32_t *rsize) {

    Flash.handle = pvPortMalloc(sizeof(SPI_HandleTypeDef));
    Flash.handle->hdmatx = pvPortMalloc(sizeof(DMA_HandleTypeDef));
    Flash.handle->hdmatx->Parent = Flash.handle;
    Flash.handle->hdmarx = pvPortMalloc(sizeof(DMA_HandleTypeDef));
    Flash.handle->hdmarx->Parent = Flash.handle;
    Flash.gate = xSemaphoreCreateRecursiveMutex();

    Flash.handle->Instance = SPI5;
    Flash.handle->Init.Mode = SPI_MODE_MASTER;
    Flash.handle->Init.Direction = SPI_DIRECTION_2LINES;
    Flash.handle->Init.DataSize = SPI_DATASIZE_8BIT;
    Flash.handle->Init.CLKPolarity = SPI_POLARITY_LOW;
    Flash.handle->Init.CLKPhase = SPI_PHASE_1EDGE;
    Flash.handle->Init.NSS = SPI_NSS_HARD_OUTPUT;
    Flash.handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    Flash.handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
    Flash.handle->Init.TIMode = SPI_TIMODE_DISABLE;
    Flash.handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    Flash.handle->Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(Flash.handle) != HAL_OK) {
        Error_Handler();
    }

	SPI_Params_init(&memory);													//начальные значения
	memory.frameFormat = SPI_POL1_PHA1;											//режим шины
	memory.bitRate = 25000000;													//скорость шины
    memory.transferMode = SPI_MODE_CALLBACK;
    memory.transferCallbackFxn = flash_state_maсhine;

    Flash.handle = SPI_open(ssi_FLASH, &memory);
    if (Flash.handle == NULL) {
        System_abort("Can't open Flash bus!");
    }
	*rsize = flash_info(&Flash);

	return (*rsize);
}

//------------------------------------------------------------------------------
//разрешение записи
static void wr_enable(t_FlTrans *flash) {
    // команда
    flash->cmdbuf[0] = WREN;
    flash->ssi.txBuf = flash->cmdbuf;
    flash->ssi.rxBuf = NULL;
    flash->ssi.count = 1;
    // ответ
    flash->size = 0;

    SPI_transfer(flash->handle, &flash->ssi);
    xSemaphoreTake(&(((SPITivaDMA_Object*)flash->handle->object)->transferComplete), portMAX_DELAY);
}
//------------------------------------------------------------------------------
//Выяснение размера и типа используемой памяти (flash)
static uint32_t flash_info(t_FlTrans *flash) {
	uint32_t fsize = 0;

    // команда
    flash->cmdbuf[0] = JEDEC_Code;
    flash->ssi.txBuf = flash->cmdbuf;
    flash->ssi.rxBuf = flash->cmdbuf;
    flash->ssi.count = 4;
    // ответ
    flash->size = 0;

    SPI_transfer(flash->handle, &flash->ssi);
    xSemaphoreTake(((SPITivaDMA_Object*)flash->handle->object)->transferComplete, portMAX_DELAY);


    /* Проверка типа поддерживаемой памяти:
     * Macronix MX25xxx35/36
     * Winbond W25Q128FV
     */
    if ((flash->cmdbuf[1] == MACRONIX && flash->cmdbuf[2] == MXSMIO) ||
            (flash->cmdbuf[1] == WINBOND && flash->cmdbuf[2] == BUS_MODE)) {
        switch (flash->cmdbuf[3]) {                                             //объем доступной памяти
		case 0x18:																//128Mb - (page = 4K)
			fsize = 0x1000000;
			break;
		default:
			break;
		}
    }
    return (fsize);
}
/*------------------------------------------------------------------------------
 * Ожидание готовности памяти (окончания записи)
 * Возращает значение RDSR
 */
static uint8_t flash_ready(t_FlTrans *flash) {
    // команда
    flash->cmdbuf[0] = RDSR;
    flash->ssi.txBuf = &flash->cmdbuf[0];
    flash->ssi.rxBuf = &flash->cmdbuf[2];
    flash->ssi.count = 2;
    // ответ
    flash->size = 0;
    do {
        SPI_transfer(flash->handle, &flash->ssi);
        xSemaphoreTake(((SPITivaDMA_Object*)flash->handle->object)->transferComplete, portMAX_DELAY);
        flash->size = 0;
    } while (flash->cmdbuf[3] & WIP_bit);

    return (flash->cmdbuf[3]);
}
/*------------------------------------------------------------------------------
 * Cтирание сектора данных с адреса addr
 */
uint32_t flash_sect_erase(uint32_t addr) {

	MEMORY_LOCK;
    flash_ready(&Flash);                                                        //ждем готовности
    wr_enable(&Flash);                                                          //разрешаем запись

	// команда
	addr &= ~FLASH_BLOCK_MASK;
    Flash.cmdbuf[0] = SectorErase;
    Flash.cmdbuf[1] = addr >> 16;
    Flash.cmdbuf[2] = addr >> 8;
    Flash.cmdbuf[3] = addr;
    Flash.ssi.txBuf = Flash.cmdbuf;
    Flash.ssi.rxBuf = NULL;
    Flash.ssi.count = 4;
    Flash.size = 0;

	// Initiate SPI transfer
    SPI_transfer(Flash.handle, &Flash.ssi);
    xSemaphoreTake(((SPITivaDMA_Object*)Flash.handle->object)->transferComplete, portMAX_DELAY);
	MEMORY_UNLOCK;
	return addr;
}

/*------------------------------------------------------------------------------
 * Cтирание всей памяти
 * В реальности, из-за времени выполнения команды (50 сек), используется стирание секторов
 */
void flash_erase(void) {
	MEMORY_LOCK;														        //блокируем доступ к ресурсу
    flash_ready(&Flash);                                                        //ждем готовности
    wr_enable(&Flash);                                                          //разрешаем запись

    Flash.cmdbuf[0] = ChipErase;
    Flash.ssi.txBuf = Flash.cmdbuf;
    Flash.ssi.rxBuf = NULL;
    Flash.ssi.count = 1;
    Flash.size = 0;

    // Initiate SPI transfer
    SPI_transfer(Flash.handle, &Flash.ssi);
    xSemaphoreTake(((SPITivaDMA_Object*)Flash.handle->object)->transferComplete, portMAX_DELAY);
	MEMORY_UNLOCK;
}
/*------------------------------------------------------------------------------
 * Чтение данных из внешний памяти с адреса Adr в буфер *data размером len байт
 * Возращает число считанных байт или код ошибки (число  < 0)
 */
int flash_read(uint8_t *data, uint32_t Adr, int len) {
    if (len <= 0) return (0);
    MEMORY_LOCK;
    flash_ready(&Flash);                                                        //ожидание готовности

    // команда
    Flash.cmdbuf[0] = READ;
    Flash.cmdbuf[1] = (uint8_t)(Adr >> 16);
    Flash.cmdbuf[2] = (uint8_t)(Adr >> 8);
    Flash.cmdbuf[3] = (uint8_t)Adr;
    Flash.ssi.txBuf = Flash.cmdbuf;
    Flash.ssi.rxBuf = NULL;
    Flash.ssi.count = 4;
    // чтение данных
    Flash.wrData = NULL;
    Flash.rdData = (uint8_t*)data;
    Flash.size = len;

    // транзакция
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, MR_CS);
//    MAP_GPIOPinWrite(GPIO_PORTH_BASE, MR_CS, 0);

    // Initiate SPI transfer
    SPI_transfer(Flash.handle, &Flash.ssi);
    xSemaphoreTake(((SPITivaDMA_Object*)Flash.handle->object)->transferComplete, portMAX_DELAY);

//    MAP_GPIOPinConfigure(GPIO_PH5_SSI2FSS);
//    MAP_GPIOPinTypeSSI(GPIO_PORTH_BASE, MR_CS);

    MEMORY_UNLOCK;
    return (len);
}

/*------------------------------------------------------------------------------
 * Запись данных во внешнюю память по адресу Adr из буфера *data. Размер блока len байт.
 * Сектор, в который производится запись, должен быть чистым.
 * Стирание сектора производится автоматически перед записью первого байта
 */
int flash_write(uint8_t *data, uint32_t Adr, int len) {
	uint32_t blockSize, wrlen;
	if(len <= 0) return (0);
    MEMORY_LOCK;														        //блокируем доступ к ресурсу
    wrlen = len;

	do {
		if((Adr & FLASH_BLOCK_MASK) == 0x0000) {								//стираем сектор
			flash_sect_erase(Adr);
		}
        flash_ready(&Flash);                                                    //ждем готовности
        wr_enable(&Flash);                                                      //разрешаем запись
		/* Вычисление размера блока данных */
		blockSize = (Adr + 0x100) & 0xFFFFFF00;
		blockSize -= Adr;
		if(len < blockSize) blockSize = len;

        // команда
        Flash.cmdbuf[0] = WRITE;
        Flash.cmdbuf[1] = (uint8_t)(Adr >> 16);
        Flash.cmdbuf[2] = (uint8_t)(Adr >> 8);
        Flash.cmdbuf[3] = (uint8_t)Adr;
        Flash.ssi.txBuf = Flash.cmdbuf;
        Flash.ssi.rxBuf = NULL;
        Flash.ssi.count = 4;
        // запись данных
        Flash.wrData = (uint8_t*)data;
        Flash.rdData = NULL;
        Flash.size = blockSize;

        // команда
//        MAP_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, MR_CS);
//        MAP_GPIOPinWrite(GPIO_PORTH_BASE, MR_CS, 0);

        // Initiate SPI transfer
        SPI_transfer(Flash.handle, &Flash.ssi);
        xSemaphoreTake(((SPITivaDMA_Object*)Flash.handle->object)->transferComplete, portMAX_DELAY);

//        MAP_GPIOPinConfigure(GPIO_PH5_SSI2FSS);
//        MAP_GPIOPinTypeSSI(GPIO_PORTH_BASE, MR_CS);

        Adr += blockSize;
        data += blockSize;
        len -= blockSize;
	} while (len > 0);

    MEMORY_UNLOCK;														        //восстанавливаем доступ к ресурсу
	return (wrlen);
}

/*------------------------------------------------------------------------------
 * Копирование блока данных размером size из адреса addr1 в addr2
 * XXX при записи блока не контролируется его чистота!
 * для стирания блока его адрес должен быть выровнен по началу страницы
 */
void flash_copy(uint32_t addr2, uint32_t addr1, int size) {
	uint32_t blen = 0x200;
	uint8_t *mbuf;

	MEMORY_LOCK;														        //блокируем доступ к ресурсу
	mbuf = malloc(blen);
	while (size > 0) {
		if(size < blen) blen = size;
		flash_read(mbuf, addr1, blen);
		flash_write(mbuf, addr2, blen);
		addr1 += blen;
		addr2 += blen;
		size -= blen;
	}
	free(mbuf);
	MEMORY_UNLOCK;														        //восстанавливаем доступ к ресурсу
}

