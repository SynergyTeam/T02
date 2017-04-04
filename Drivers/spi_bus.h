/*----------------------------------------------------------------------------*/
#ifndef _SPI_INIT_
#define _SPI_INIT_

// подключаемые модули
#include "Drivers/spi/SPI.h"

/*!
 *  @def    Device_SPIName
 *  @brief  Перечисление SPI шин
 */
typedef enum Device_SPIName {
    ssi_FLASH,
    ssi_EXT,

    SSI_BUS
} Device_SPIName;

//-----------------------------------------------------------------------------
//внешние переменные
extern SPI_Handle SSI[SSI_BUS];

//внешние функции
extern void HW_initSPI(uint32_t clock);
extern void uDMAInit(void);

#endif /* _SPI_INIT_ */
