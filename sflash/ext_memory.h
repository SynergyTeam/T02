//------------------------------------------------------------------------------
#ifndef __S_MEMORY__
#define __S_MEMORY__

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

//типы микросхем используемой памяти
enum ram_type {
    flash = 0,

    all_types
} ram_type;

/* заголовки */
extern uint32_t flash_init(uint32_t *rsize);
extern int      flash_read(uint8_t *buf, uint32_t adr, int len);
extern int      flash_write(uint8_t *buf, uint32_t adr, int len);
extern uint32_t flash_sect_erase(uint32_t addr);
extern void     flash_erase(void);
extern void     flash_copy(uint32_t buff, uint32_t addr, int size);

#endif /* __S_MEMORY__ */
