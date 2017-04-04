//------------------------------------------------------------------------------
//подключаемые файлы
#include <inttypes.h>
#include <stdbool.h>

#include "hardware.h"
#include <Drivers/spi/SPI.h>
#include <Drivers/spi/SPITivaDMA.h>
#include "Drivers/spi_bus.h"

//------------------------------------------------------------------------------
/* SPI objects */
SPIDMA_Object spiDMAobjects[SSI_BUS];
__attribute__ ((aligned (32)))
uint32_t spiScratchBuf[SSI_BUS];

//Описание API последовательных шин
SPI_Handle SSI[SSI_BUS] = { 0 };

/* SPI configuration structure, describing which pins are to be used */
const SPIDMA_HWAttrs spiTivaDMAHWAttrs[SSI_BUS] = {
    {
        SPI5,
        SPI5_IRQn,
        &spiScratchBuf[0],
        0xFF,
        DMA2_Stream6_BASE,
        DMA_CHANNEL_7,
        DMA2_Stream6_IRQn,
        DMA2_Stream5_BASE,
        DMA_CHANNEL_7,
        DMA2_Stream5_IRQn,
    },
    {
        SPI3,
        SPI3_IRQn,
        &spiScratchBuf[1],
        0x00,
        DMA1_Stream5_BASE,
        DMA_CHANNEL_0,
        DMA1_Stream5_IRQn,
        DMA1_Stream2_BASE,
        DMA_CHANNEL_0,
        DMA1_Stream2_IRQn
    },
};

const SPI_Config SPI_config[] = {
    {&SPIDMA_fxnTable, &spiDMAobjects[0], &spiTivaDMAHWAttrs[0]},
    {&SPIDMA_fxnTable, &spiDMAobjects[1], &spiTivaDMAHWAttrs[1]},
    {NULL, NULL, NULL},
};

/*------------------------------------------------------------------------------
 * Инициализация всех SPI интерфейсов
 */
void HW_initSPI(uint32_t clock) {
    // SSI0 - FRAM
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
//    MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
//    MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
//    MAP_GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
//    MAP_GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
//    MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, FRAM_MISO | FRAM_MOSI | FRAM_FSS | FRAM_SCLK);
//
//    // SSI2 - память / WiFi
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
//    MAP_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
//    MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
//    MAP_GPIOPinConfigure(GPIO_PD2_SSI2FSS);
//    MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
//    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, SPI_SCLK | SPI_FSS | SPI_MOSI | SPI_MISO);
//
//    // SSI3 - PRINTER
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
//    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
//    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
//    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
//    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, PRINTER_SDOUT | PRINTER_SDIN | PRINTER_SCLK);
//
//    uDMAInit();
    SPI_init();
}

/*------------------------------------------------------------------------------
 * Обработчик прерывания
 */
void SSI0IntHandler(void) {
//    SPI_serviceISR(SSI[ssi_FRAM]);
}

/*------------------------------------------------------------------------------
 * Обработчик прерывания
 */
void SSI2IntHandler (void) {
//    SPI_serviceISR(SSI[ssi_FLASH_WiFi]);
}

/*------------------------------------------------------------------------------
 * Обработчик прерывания SSI2 шины
 */
void SSI3IntHandler(void) {
//    SPI_serviceISR(SSI[ssi_PRINTER]);
}

