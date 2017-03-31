/*
 * hardware.h
 *
 *  Created on: 31 марта 2017 г.
 *      Author: a_cherepanov
 */

#ifndef INC_HARDWARE_H_
#define INC_HARDWARE_H_

#define FLASH_BUS                   SPI5
#define FLASH_BUS_ENABLE()          __HAL_RCC_GPIOE_CLK_ENABLE()
#define FLASH_CLK_ENABLE()          __HAL_RCC_SPI5_CLK_ENABLE()
#define FLASH_DMAx_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define FLASH_FORCE_RESET()         __HAL_RCC_SPI5_FORCE_RESET()
#define FLASH_RELEASE_RESET()       __HAL_RCC_SPI5_RELEASE_RESET()

/* Definition for SPIx Pins */
#define FLASH_NSS_PIN               GPIO_PIN_11
#define FLASH_SCK_PIN               GPIO_PIN_12
#define FLASH_MISO_PIN              GPIO_PIN_13
#define FLASH_MOSI_PIN              GPIO_PIN_14

/* Definition for SPIx's DMA */
#define FLASH_TX_DMA_STREAM          DMA2_Stream6
#define FLASH_RX_DMA_STREAM          DMA2_Stream5
#define FLASH_TX_DMA_CHANNEL         DMA_CHANNEL_7
#define FLASH_RX_DMA_CHANNEL         DMA_CHANNEL_7

/* Definition for SPIx's NVIC */
#define FLASH_IRQn                   SPI5_IRQn
#define FLASH_IRQHandler             SPI5_IRQHandler

/* Definition for SPIx's NVIC */
#define FLASH_DMA_TX_IRQn            DMA2_Stream6_IRQn
#define FLASH_DMA_RX_IRQn            DMA2_Stream5_IRQn

#define FLASH_DMA_TX_IRQHandler      DMA2_Stream6_IRQHandler
#define FLASH_DMA_RX_IRQHandler      DMA2_Stream5_IRQHandler

#endif /* INC_HARDWARE_H_ */
