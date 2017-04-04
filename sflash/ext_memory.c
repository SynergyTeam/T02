/*----------------------------------------------------------------------------*/
/* Работа с последовательной памятью */
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "hardware.h"
#include "ext_memory.h"

extern void Error_Handler(void);
//------------------------------------------------------------------------------
//Поддерживаемые производители
#define MACRONIX                    (0xC2)
#define MXSMIO                      (0x20)

#define WINBOND                     (0xEF)
#define BUS_MODE                    (0x40)

//макросы доступа
#define MEMORY_LOCK                 (xSemaphoreTake(Flash.gate, portMAX_DELAY))
#define MEMORY_UNLOCK               (xSemaphoreGive(Flash.gate))

//------------------------------------------------------------------------------
/* буфер обмена данными */

/* локальные переменные */
t_FlTrans Flash;

/* локальные функции */
static void wr_enable(t_FlTrans *flash);
static uint32_t flash_info(t_FlTrans *flash);

/*------------------------------------------------------------------------------
 * CallBack функция-драйвер доступа к внешней памяти
 * Вызывается из драйвера после окончания транзакции на шине
 * В первой транзакции передается команда, в последующих осуществляется обмен данными
 */
void flash_state_machine_callback(void) {
    BaseType_t flashWake = pdFALSE;

    if (Flash.size == 0) {
        xSemaphoreGiveFromISR(Flash.done, &flashWake);
    }
    else {
        if (Flash.rdData != NULL) {
            HAL_SPI_TransmitReceive_DMA(Flash.handle, Flash.rdData, Flash.rdData, Flash.size);
            Flash.rdData = NULL;
        }
        else {
            if (Flash.wrData != NULL) {
                HAL_SPI_Transmit_DMA(Flash.handle, Flash.wrData, Flash.size);
                Flash.wrData = NULL;
            }
        }

        Flash.size = 0;
    }
}

/*------------------------------------------------------------------------------
 * Инициализация внешней памяти
 * Выполняется настройка SSI интерфейса, чтение JEDEC кода и тд.
 * Возвращает размер памяти
 */
uint32_t flash_init(uint32_t *rsize) {
    SPI_Handle sflash;

    sflash = SPI_open(ssi_FLASH, NULL);
    UNUSED(sflash);

    Flash.handle = pvPortMalloc(sizeof(SPI_HandleTypeDef));
    memset(Flash.handle, 0x00, sizeof(SPI_HandleTypeDef));
    Flash.handle->hdmatx = pvPortMalloc(sizeof(DMA_HandleTypeDef));
    Flash.handle->hdmatx->Parent = Flash.handle;
    Flash.handle->hdmarx = pvPortMalloc(sizeof(DMA_HandleTypeDef));
    Flash.handle->hdmarx->Parent = Flash.handle;
    Flash.gate = xSemaphoreCreateRecursiveMutex();
    Flash.done = xSemaphoreCreateBinary();

    Flash.handle->Instance = SPI5;
    Flash.handle->Init.Mode              = SPI_MODE_MASTER;
    Flash.handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    Flash.handle->Init.Direction         = SPI_DIRECTION_2LINES;
    Flash.handle->Init.CLKPhase          = SPI_PHASE_1EDGE;
    Flash.handle->Init.CLKPolarity       = SPI_POLARITY_LOW;
    Flash.handle->Init.DataSize          = SPI_DATASIZE_8BIT;
    Flash.handle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    Flash.handle->Init.TIMode            = SPI_TIMODE_DISABLE;
    Flash.handle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    Flash.handle->Init.CRCPolynomial     = 7;
    Flash.handle->Init.NSS               = SPI_NSS_HARD_OUTPUT;

    if (HAL_SPI_Init(Flash.handle) != HAL_OK) {
        Error_Handler();
//        System_abort("Can't open Flash bus!");
    }
    *rsize = flash_info(&Flash);

    return (*rsize);
}

//------------------------------------------------------------------------------
//разрешение записи
static void wr_enable(t_FlTrans *flash) {
    // команда
    flash->cmdbuf[0] = WREN;
    // ответ
    flash->size = 0;

    HAL_SPI_Transmit_DMA(flash->handle, flash->cmdbuf, 1);
    xSemaphoreTake(flash->done, portMAX_DELAY);
}
//------------------------------------------------------------------------------
//Выяснение размера и типа используемой памяти (flash)
static uint32_t flash_info(t_FlTrans *flash) {
	uint32_t fsize = 0;

    // команда
    flash->cmdbuf[0] = JEDEC_Code;
    // ответ
    flash->size = 0;

    HAL_SPI_TransmitReceive_DMA(flash->handle, flash->cmdbuf, flash->cmdbuf, 4);
    xSemaphoreTake(flash->done, portMAX_DELAY);


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
    do {
        flash->size = 0;
        HAL_SPI_TransmitReceive_DMA(flash->handle, &flash->cmdbuf[0], &flash->cmdbuf[2], 2);
        xSemaphoreTake(flash->done, portMAX_DELAY);
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
    // ответ
    Flash.size = 0;

	// Initiate SPI transfer
    HAL_SPI_Transmit_DMA(Flash.handle, Flash.cmdbuf, 4);
    xSemaphoreTake(Flash.done, portMAX_DELAY);
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
    Flash.size = 0;

    // Initiate SPI transfer
    HAL_SPI_Transmit_DMA(Flash.handle, Flash.cmdbuf, 1);
    xSemaphoreTake(Flash.done, portMAX_DELAY);
	MEMORY_UNLOCK;
}
/*------------------------------------------------------------------------------
 * Чтение данных из внешний памяти с адреса Adr в буфер *data размером len байт
 * Возращает число считанных байт или код ошибки (число  < 0)
 */
int flash_read(uint8_t *data, uint32_t Adr, int len) {
    GPIO_InitTypeDef GPIO_InitStruct;

    if (len <= 0) return (0);
    MEMORY_LOCK;
    flash_ready(&Flash);                                                        //ожидание готовности

    // команда
    Flash.cmdbuf[0] = READ;
    Flash.cmdbuf[1] = (uint8_t)(Adr >> 16);
    Flash.cmdbuf[2] = (uint8_t)(Adr >> 8);
    Flash.cmdbuf[3] = (uint8_t)Adr;
    // чтение данных
    Flash.wrData = NULL;
    Flash.rdData = (uint8_t*)data;
    Flash.size = len;

    // транзакция
    GPIO_InitStruct.Pin = FLASH_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);

    // Initiate SPI transfer
    HAL_SPI_Transmit_DMA(Flash.handle, Flash.cmdbuf, 4);
    xSemaphoreTake(Flash.done, portMAX_DELAY);

    GPIO_InitStruct.Pin = FLASH_NSS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI5;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    MEMORY_UNLOCK;
    return (len);
}

/*------------------------------------------------------------------------------
 * Запись данных во внешнюю память по адресу Adr из буфера *data. Размер блока len байт.
 * Сектор, в который производится запись, должен быть чистым.
 * Стирание сектора производится автоматически перед записью первого байта
 */
int flash_write(uint8_t *data, uint32_t Adr, int len) {
    GPIO_InitTypeDef GPIO_InitStruct;
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
        // запись данных
        Flash.wrData = (uint8_t*)data;
        Flash.rdData = NULL;
        Flash.size = blockSize;

        // команда
        GPIO_InitStruct.Pin = FLASH_NSS_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);

        // Initiate SPI transfer
        HAL_SPI_Transmit_DMA(Flash.handle, Flash.cmdbuf, 4);
        xSemaphoreTake(Flash.done, portMAX_DELAY);

        GPIO_InitStruct.Pin = FLASH_NSS_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI5;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

