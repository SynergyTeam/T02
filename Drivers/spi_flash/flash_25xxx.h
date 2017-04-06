/*
 * flash_25xxx.h
 *
 *  Created on: 29 ����. 2015 �.
 *      Author: a_cherepanov
 */

#ifndef CPU_DRIVERS_FLASH_FLASH_25XXX_H_
#define CPU_DRIVERS_FLASH_FLASH_25XXX_H_

#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "Drivers/spi_bus.h"

// ������� ����� ������
#define FLASH_BLOCK_SIZE        (0x1000)
#define FLASH_BLOCK_MASK        (FLASH_BLOCK_SIZE - 1)

/*
 * ������ �������������� ������
 */
#define WREN                    0x06
#define WRDI                    0x04
#define RDSR                    0x05
#define RDCR                    0x15
#define WRSR                    0x01
#define READ                    0x03
#define WRITE                   0x02
#define SLEEP                   0xB9
#define WAKE                    0xAB

#define JEDEC_Code              0x9F
#define SectorErase             0x20
#define Block32Erase            0x52
#define Block64Erase            0xD8
#define ChipErase               0xC7
#define EnableWSR               0x50

/*
 * �������� ������� ����� �������� RDSR
 */
#define WIP_bit                 0x01
#define WEL_bit                 0x02
#define QE_bit                  0x40
#define SRWD_bit                0x80

//��������� ������ �������� ������
typedef struct {
    SPI_Handle          handle;         // SPI ����
    SPI_Transaction     ssi;            // �������� ������� ����������
    SemaphoreHandle_t   gate;           // mutex �������
    uint8_t             cmdbuf[5];      // ����� �������
    uint8_t             *rdData;        // ��������� �� ����� ������ ������
    uint8_t             *wrData;        // ��������� �� ����� ������ ������
    uint32_t            size;           // ������ ������ ��� ������/������
} t_FlTrans;

/* ��������� ���������� */
extern t_FlTrans Flash;

/* ��������� */
extern uint32_t flash_init(uint32_t ssiBus);
extern void     flash_close(uint32_t ssiBus);
extern int      flash_read(uint8_t *buf, uint32_t adr, int len);
extern int      flash_write(uint8_t *buf, uint32_t adr, int len);
extern uint32_t flash_sect_erase(uint32_t addr);
extern void     flash_erase(void);
extern void     flash_copy(uint32_t buff, uint32_t addr, int size);

#endif /* CPU_DRIVERS_FLASH_FLASH_25XXX_H_ */
