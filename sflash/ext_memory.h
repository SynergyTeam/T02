//------------------------------------------------------------------------------
#ifndef __S_MEMORY__
#define __S_MEMORY__

#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "Drivers/spi_bus.h"

// Размеры блока данных
#define FLASH_BLOCK_SIZE        (0x1000)
#define FLASH_BLOCK_MASK        (FLASH_BLOCK_SIZE - 1)

/*
 * Список поддерживаемых команд
 */
#define WREN 					0x06
#define WRDI 					0x04
#define RDSR					0x05
#define RDCR					0x15
#define WRSR					0x01
#define	READ					0x03
#define WRITE 					0x02
#define SLEEP					0xB9
#define WAKE					0xAB

#define JEDEC_Code				0x9F
#define SectorErase				0x20
#define Block32Erase            0x52
#define Block64Erase            0xD8
#define ChipErase				0xC7
#define EnableWSR				0x50

/*
 * Описание битовой маски регистра RDSR
 */
#define WIP_bit					0x01
#define WEL_bit					0x02
#define QE_bit					0x40
#define SRWD_bit				0x80

//структура данных драйвера памяти
typedef struct t_FlTrans {
    SPI_HandleTypeDef*  handle;         // SPI шина
    SPI_Handle          ssi_bus;
    SPI_Transaction     ssi;
    SemaphoreHandle_t   gate;           // mutex доступа
    SemaphoreHandle_t   done;           // семафор завершения операции
    uint8_t             cmdbuf[5];      // буфер команды
    uint8_t             *rdData;        // указатель на буфер чтения данных
    uint8_t             *wrData;        // указатель на буфер записи данных
    uint32_t            size;           // размер данных для записи/чтения
} t_FlTrans;

//типы микросхем используемой памяти
enum ram_type {
    flash = 0,

    all_types
} ram_type;

/* переменные */
extern t_FlTrans Flash;

/* заголовки */
extern uint32_t flash_init(uint32_t *rsize);
extern int      flash_read(uint8_t *buf, uint32_t adr, int len);
extern int      flash_write(uint8_t *buf, uint32_t adr, int len);
extern uint32_t flash_sect_erase(uint32_t addr);
extern void     flash_erase(void);
extern void     flash_copy(uint32_t buff, uint32_t addr, int size);
extern void     flash_state_machine_callback(void);

#endif /* __S_MEMORY__ */
