//------------------------------------------------------------------------------
//подключаемые файлы
#include <inttypes.h>
#include <stdbool.h>

#include "hardware.h"
#include <Drivers/spi/SPI.h>
#include <Drivers/spi/SPITivaDMA.h>
#include "Drivers/spi_bus.h"
#include "Drivers/spi_flash/flash_25xxx.h"

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
    GPIO_InitTypeDef  GPIO_InitStruct;

    // SPI5 - FLASH
    // Peripheral clock enable
    FLASH_BUS_ENABLE();
    FLASH_CLK_ENABLE();
    FLASH_DMAx_ENABLE();

    GPIO_InitStruct.Pin = FLASH_NSS_PIN | FLASH_SCK_PIN | FLASH_MISO_PIN | FLASH_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI5;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


    // SSI3 - PRINTER
    // Peripheral clock enable
    __HAL_RCC_SPI4_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    SPI_init();
}


/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void FLASH_IRQHandler(void) {
    HAL_SPI_IRQHandler(Flash.handle);
}

/**
  * @brief  This function handles DMA Rx interrupt request.
  * @param  None
  * @retval None
  */
void FLASH_DMA_RX_IRQHandler(void) {
    SPIDMA_Object *object = Flash.handle->object;
    HAL_DMA_IRQHandler(object->hdmarx);
}

/**
  * @brief  This function handles DMA Tx interrupt request.
  * @param  None
  * @retval None
  */
void FLASH_DMA_TX_IRQHandler(void) {
    SPIDMA_Object *object = Flash.handle->object;
    HAL_DMA_IRQHandler(object->hdmatx);
}


