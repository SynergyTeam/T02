/*----------------------------------------------------------------------------*/
/* ������ � ���������������� ������� */

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "hardware.h"
#include "Drivers/spi_bus.h"
#include "Drivers/spi/SPITivaDMA.h"
#include "Drivers/spi_flash/flash_25xxx.h"
#include "console/console.h"

//------------------------------------------------------------------------------
//����� �������� ���������� ������
#define VERIFY_WRITTEN_DATA         (0)

//������� �������
#define MEMORY_LOCK                 (xSemaphoreTakeRecursive(Flash.gate, portMAX_DELAY))
#define MEMORY_UNLOCK               (xSemaphoreGiveRecursive(Flash.gate))
#define drvOBJ(handler)             ((SPIDMA_Object*)handler->object)

//�������������� �������������
#define MACRONIX                    (0xC2)
#define MXSMIO                      (0x20)

#define WINBOND                     (0xEF)
#define BUS_MODE                    (0x40)

#define debuglog(...)

//------------------------------------------------------------------------------
t_FlTrans Flash;

/* ��������� ������� */
static void wr_enable(t_FlTrans *flash);
static uint32_t flash_size(t_FlTrans *flash);

/*------------------------------------------------------------------------------
 * CallBack �������-������� ������� � ������� ������
 * ���������� �� �������� ����� ��������� ���������� �� ����
 * � ������ ���������� ���������� �������, � ����������� �������������� ����� �������
 */
void flash_state_machine (SPI_Handle handle, SPI_Transaction *transaction) {
    SPIDMA_Object *object = handle->object;

    if (Flash.size == 0x0000) {
        BaseType_t h_pr_TaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(object->transferComplete, &h_pr_TaskWoken);
        portYIELD_FROM_ISR(h_pr_TaskWoken);
    }
    else {
        object->transaction = transaction;
        transaction->count = Flash.size;
        if (Flash.wrData != NULL) {
            transaction->txBuf = Flash.wrData;
        }
        else {
            if (Flash.rdData != NULL) {
                transaction->rxBuf = Flash.rdData;
            }
        }
        Flash.size = 0;
        SPI_DMA_transfer(Flash.handle, transaction);
    }
}

/*------------------------------------------------------------------------------
 * ������������� ������� ������
 * ����������� ��������� SSI ����������, ������ JEDEC ���� � ��.
 * ���������� ������ ������
 */
uint32_t flash_init(uint32_t ssiBus) {
    SPI_Params      memory;														//������������ ����������

    Flash.gate = xSemaphoreCreateRecursiveMutex();
    assert_param(Flash.gate != NULL);
    SPI_Params_init(&memory);													//��������� ��������
    memory.frameFormat = SPI_POL1_PHA1;											//����� ����
//    memory.bitRate = SFLASH_WIFI_CLK;											//�������� ����
//    memory.nss = SPI_SOFT;
    memory.transferMode = SPI_MODE_CALLBACK;
    memory.transferCallbackFxn = flash_state_machine;

    Flash.handle = SPI_open(ssiBus, &memory);
    if (Flash.handle == NULL) {
//        System_abort("Can't open Flash bus!");
    }
    SSI[ssiBus] = Flash.handle;
    return (flash_size(&Flash));
}

/*------------------------------------------------------------------------------
 * ��������� ����
 */
void flash_close(uint32_t ssiBus) {
    vSemaphoreDelete(Flash.gate);
    SPI_close(Flash.handle);
    SSI[ssiBus] = NULL;
}

//------------------------------------------------------------------------------
//���������� ������
static void wr_enable(t_FlTrans *flash) {
    // �������
    flash->cmdbuf[0] = WREN;
    // ����������
    flash->ssi.txBuf = flash->cmdbuf;
    flash->ssi.rxBuf = NULL;
    flash->ssi.count = 1;
    flash->size = 0;
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
    SPI_transfer(flash->handle, &flash->ssi);
    xSemaphoreTake((drvOBJ(flash->handle)->transferComplete), portMAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);
}
//------------------------------------------------------------------------------
//��������� ������� � ���� ������������ ������ (flash)
static uint32_t flash_size(t_FlTrans *flash) {
	uint32_t fsize = 0;

    // �������
    flash->cmdbuf[0] = JEDEC_Code;
    flash->ssi.txBuf = flash->cmdbuf;
    flash->ssi.rxBuf = flash->cmdbuf;
    flash->ssi.count = 4;
    // �����
    flash->size = 0;
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
    SPI_transfer(flash->handle, &flash->ssi);
    xSemaphoreTake((drvOBJ(flash->handle)->transferComplete), portMAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);

    /* �������� ���� �������������� ������:
     * Macronix MX25xxx35/36
     * Winbond W25Q128FV
     */
    if (flash->cmdbuf[1] == MACRONIX && flash->cmdbuf[2] == MXSMIO) {
        debuglog("Flash: Macronix MX25xxx series\r\n");
        fsize = 1;
    }
    else
    if (flash->cmdbuf[1] == WINBOND && flash->cmdbuf[2] == BUS_MODE) {
        debuglog("Flash: Winbond W25xxx series\r\n");
        fsize = 1;
    }
    else {
        debuglog("Flash: unknown vendor!\r\n");
    }

    if (fsize) {
        switch (flash->cmdbuf[3]) {                                             //����� ��������� ������
        case 0x18:                                                              //128Mb - (page = 4K)
            fsize = 0x1000000;
            debuglog("Flash: size 0x%X bytes\r\n", fsize);
            break;
        default:
            debuglog("Flash: unknown size!\r\n", fsize);
            break;
        }
    }
    return (fsize);
}
/*------------------------------------------------------------------------------
 * �������� ���������� ������ (��������� ������)
 * ��������� �������� RDSR
 */
static uint8_t flash_ready(t_FlTrans *flash) {
    register uint32_t rdData;

    // �������
    flash->cmdbuf[0] = RDSR;
    flash->cmdbuf[1] = 0xFF;
    // ����������
    flash->ssi.txBuf = &flash->cmdbuf[0];
    flash->ssi.rxBuf = &flash->cmdbuf[2];
    flash->ssi.count = 2;
    // �����
    do {
        flash->size = 0;
        HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
        SPI_transfer(flash->handle, &flash->ssi);
        xSemaphoreTake((drvOBJ(flash->handle)->transferComplete), portMAX_DELAY);
        HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);
        rdData = flash->cmdbuf[3];
    } while (rdData & WIP_bit);

    return (uint8_t)rdData;
}
/*------------------------------------------------------------------------------
 * C������� ������� ������ � ������ addr
 */
uint32_t flash_sect_erase(uint32_t addr) {
    t_FlTrans *flash;

	MEMORY_LOCK;
    flash = &Flash;

    wr_enable(flash);                                                           //��������� ������

    // �������
    flash->cmdbuf[0] = SectorErase;
    flash->cmdbuf[1] = (uint8_t)(addr >> 16);
    flash->cmdbuf[2] = (uint8_t)(addr >> 8);
    flash->cmdbuf[3] = (uint8_t)addr;
    // ����������
    flash->ssi.txBuf = flash->cmdbuf;
    flash->ssi.rxBuf = NULL;
    flash->ssi.count = 4;
    flash->size = 0;
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
    SPI_transfer(flash->handle, &flash->ssi);
    xSemaphoreTake((drvOBJ(flash->handle)->transferComplete), portMAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);

    flash_ready(flash);                                                         //���� ����������
	MEMORY_UNLOCK;
	return addr;
}

/*------------------------------------------------------------------------------
 * C������� ���� ������
 * � ����������, ��-�� ������� ���������� ������� (50 ���), ������������ �������� ��������
 */
void flash_erase(void) {
    MEMORY_LOCK;                                                                //��������� ������ � �������
    wr_enable(&Flash);                                                          //��������� ������

    // �������
    Flash.cmdbuf[0] = ChipErase;
    // ����������
    Flash.ssi.txBuf = Flash.cmdbuf;
    Flash.ssi.rxBuf = NULL;
    Flash.ssi.count = 1;
    Flash.size = 0;
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
    SPI_transfer(Flash.handle, &Flash.ssi);
    xSemaphoreTake((drvOBJ(Flash.handle)->transferComplete), portMAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);

    flash_ready(&Flash);                                                        //���� ����������
	MEMORY_UNLOCK;
}
/*------------------------------------------------------------------------------
 * ������ ������ �� ������� ������ � ������ Adr � ����� *data �������� len ����
 * ��������� ����� ��������� ���� ��� ��� ������ (�����  < 0)
 */
int flash_read(uint8_t *data, uint32_t Adr, int len) {
    if (len <= 0) return (0);
    MEMORY_LOCK;

    // �������
    Flash.cmdbuf[0] = READ;
    Flash.cmdbuf[1] = (uint8_t)(Adr >> 16);
    Flash.cmdbuf[2] = (uint8_t)(Adr >> 8);
    Flash.cmdbuf[3] = (uint8_t)Adr;
    Flash.ssi.txBuf = Flash.cmdbuf;
    Flash.ssi.rxBuf = NULL;
    Flash.ssi.count = 4;
    // ������ ������
    Flash.wrData = NULL;
    Flash.rdData = (uint8_t*)data;
    Flash.size = len;

    // Initiate SPI transfer
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
    SPI_transfer(Flash.handle, &Flash.ssi);
    xSemaphoreTake((drvOBJ(Flash.handle)->transferComplete), portMAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);

    MEMORY_UNLOCK;
    return (len);
}

/*------------------------------------------------------------------------------
 * ������ ������ �� ������� ������ �� ������ Adr �� ������ *data. ������ ����� len ����.
 * ������, � ������� ������������ ������, ������ ���� ������.
 * �������� ������� ������������ ������������� ����� ������� ������� �����
 */
int flash_write(uint8_t *data, uint32_t Adr, int len) {
    uint32_t blockSize, size;
    uint8_t *wrBuf;

    if(len <= 0)
        return (0);

    MEMORY_LOCK;                                                                //��������� ������ � �������
    size = len;
    wrBuf = pvPortMalloc(0x104);                                                //����. ������ ������� � ����� ������

    do {
        //������� ������
        if((Adr & FLASH_BLOCK_MASK) == 0x0000) {
            flash_sect_erase(Adr);
        }
        //��������� ������
        wr_enable(&Flash);

        // ���������� ������� ����� ������
        blockSize = (Adr + 0x100) & 0xFFFFFF00;
        blockSize -= Adr;
        if(len < blockSize)
            blockSize = len;
        // �������
        wrBuf[0] = WRITE;
        wrBuf[1] = (uint8_t)(Adr >> 16);
        wrBuf[2] = (uint8_t)(Adr >> 8);
        wrBuf[3] = (uint8_t)Adr;
        // ������
        memcpy(&wrBuf[4], data, blockSize);
        Flash.ssi.txBuf = wrBuf;
        Flash.ssi.rxBuf = NULL;
        Flash.ssi.count = 4 + blockSize;
        Flash.size = 0;
        // ����������
        HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_RESET);
        SPI_transfer(Flash.handle, &Flash.ssi);
        xSemaphoreTake((drvOBJ(Flash.handle)->transferComplete), portMAX_DELAY);
        HAL_GPIO_WritePin(GPIOE, FLASH_NSS_PIN, GPIO_PIN_SET);
        //���� ����������
        flash_ready(&Flash);

        //��������, ���� ���������
#if VERIFY_WRITTEN_DATA
        flash_read(wrBuf, Adr, blockSize);
        if (memcmp(wrBuf, data, blockSize)) {
            debuglog("Flash: Couldn't write data!\r\n");
            blockSize = 0;
        }
#endif

        Adr += blockSize;
        data += blockSize;
        len -= blockSize;
    } while (len > 0);

    vPortFree(wrBuf);
    MEMORY_UNLOCK;                                                              //��������������� ������ � �������
    return (size);
}

/*------------------------------------------------------------------------------
 * ����������� ����� ������ �������� size �� ������ addr1 � addr2
 * XXX ��� ������ ����� �� �������������� ��� �������!
 * ��� �������� ����� ��� ����� ������ ���� �������� �� ������ ��������
 */
void flash_copy(uint32_t addr2, uint32_t addr1, int size) {
	uint32_t blen = 0x200;
	uint8_t *mbuf;

    MEMORY_LOCK;                                                                //��������� ������ � �������
	mbuf = pvPortMalloc(blen);
	while (size > 0) {
		if(size < blen) blen = size;
		flash_read(mbuf, addr1, blen);
		flash_write(mbuf, addr2, blen);
		addr1 += blen;
		addr2 += blen;
		size -= blen;
	}
	vPortFree(mbuf);
    MEMORY_UNLOCK;                                                              //��������������� ������ � �������
}

