/*
 * spi_hal_msp.c
 *
 *  Created on: 31 марта 2017 г.
 *      Author: a_cherepanov
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "hardware.h"

extern void flash_state_machine_callback(void);

/**
  * @brief SPI MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  *           - NVIC configuration for SPI interrupt request enable
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    GPIO_InitTypeDef  GPIO_InitStruct;

    if (hspi->Instance == SPI4) {
        /* USER CODE BEGIN SPI4_MspInit 0 */

        /* USER CODE END SPI4_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_SPI4_CLK_ENABLE();

        /**SPI4 GPIO Configuration
        PE2     ------> SPI4_SCK
        PE4     ------> SPI4_NSS
        PE6     ------> SPI4_MOSI
         */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /* USER CODE BEGIN SPI4_MspInit 1 */

        /* USER CODE END SPI4_MspInit 1 */
    }
    if (hspi->Instance == FLASH_BUS) {
        /* Peripheral clock enable */
        FLASH_BUS_ENABLE();
        FLASH_CLK_ENABLE();
        FLASH_DMAx_ENABLE();

        /**SPI5 GPIO Configuration
        PE11     ------> SPI5_NSS
        PE12     ------> SPI5_SCK
        PE13     ------> SPI5_MISO
        PE14     ------> SPI5_MOSI
        */
        GPIO_InitStruct.Pin = FLASH_NSS_PIN | FLASH_SCK_PIN | FLASH_MISO_PIN | FLASH_MOSI_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI5;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        /*##-3- Configure the DMA ############################################*/
        /* Configure the DMA handler for Transmission process */
        hspi->hdmatx->Instance = FLASH_TX_DMA_STREAM;
        hspi->hdmatx->Init.Channel = FLASH_TX_DMA_CHANNEL;
        hspi->hdmatx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hspi->hdmatx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hspi->hdmatx->Init.MemBurst = DMA_MBURST_INC4;
        hspi->hdmatx->Init.PeriphBurst = DMA_PBURST_INC4;
        hspi->hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;
        hspi->hdmatx->Init.PeriphInc = DMA_PINC_DISABLE;
        hspi->hdmatx->Init.MemInc = DMA_MINC_ENABLE;
        hspi->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hspi->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hspi->hdmatx->Init.Mode = DMA_NORMAL;
        hspi->hdmatx->Init.Priority = DMA_PRIORITY_LOW;
        HAL_DMA_Init(hspi->hdmatx);

//        /* Associate the initialized DMA handle to the the SPI handle */
//        __HAL_LINKDMA(hspi, hdmatx, hdma_tx);

        /* Configure the DMA handler for Transmission process */
        hspi->hdmarx->Instance = FLASH_RX_DMA_STREAM;
        hspi->hdmarx->Init.Channel = FLASH_RX_DMA_CHANNEL;
        hspi->hdmarx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hspi->hdmarx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hspi->hdmarx->Init.MemBurst = DMA_MBURST_INC4;
        hspi->hdmarx->Init.PeriphBurst = DMA_PBURST_INC4;
        hspi->hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;
        hspi->hdmarx->Init.PeriphInc = DMA_PINC_DISABLE;
        hspi->hdmarx->Init.MemInc = DMA_MINC_ENABLE;
        hspi->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hspi->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hspi->hdmarx->Init.Mode = DMA_NORMAL;
        hspi->hdmarx->Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(hspi->hdmarx);

//        /* Associate the initialized DMA handle to the the SPI handle */
//        __HAL_LINKDMA(hspi, hdmarx, hdma_rx);

        /*##-3- Configure the NVIC for SPI ###################################*/
        /* NVIC for SPI */
        HAL_NVIC_SetPriority(ExtFLASH_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(ExtFLASH_IRQn);

        /*##-4- Configure the NVIC for DMA ###################################*/
        /* NVIC configuration for DMA transfer complete interrupt (SPI2_TX) */
        HAL_NVIC_SetPriority(FLASH_DMA_TX_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(FLASH_DMA_TX_IRQn);

        /* NVIC configuration for DMA transfer complete interrupt (SPI2_RX) */
        HAL_NVIC_SetPriority(FLASH_DMA_RX_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(FLASH_DMA_RX_IRQn);

        /*##-5- Configure the NVIC for SPI ###################################*/
        HAL_NVIC_SetPriority(ExtFLASH_IRQn, 1, 2);
        HAL_NVIC_EnableIRQ(ExtFLASH_IRQn);
    }
}

/**
  * @brief SPI MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI4) {
    /* USER CODE BEGIN SPI4_MspDeInit 0 */

    /* USER CODE END SPI4_MspDeInit 0 */
      /* Peripheral clock disable */
      __HAL_RCC_SPI4_CLK_DISABLE();

      /**SPI4 GPIO Configuration
      PE2     ------> SPI4_SCK
      PE4     ------> SPI4_NSS
      PE6     ------> SPI4_MOSI
      */
      HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6);

    /* USER CODE BEGIN SPI4_MspDeInit 1 */

    /* USER CODE END SPI4_MspDeInit 1 */
    }
    if(hspi->Instance == FLASH_BUS) {
        /* Peripheral clock disable */
        __HAL_RCC_SPI5_CLK_DISABLE();

        /**SPI5 GPIO Configuration
        PE11     ------> SPI5_NSS
        PE12     ------> SPI5_SCK
        PE13     ------> SPI5_MISO
        PE14     ------> SPI5_MOSI
        */
        HAL_GPIO_DeInit(GPIOE, FLASH_NSS_PIN | FLASH_SCK_PIN | FLASH_MISO_PIN | FLASH_MOSI_PIN);

        /*##-3- Disable the DMA ##############################################*/
        /* De-Initialize the DMA associated to transmission process */
        HAL_DMA_DeInit(hspi->hdmatx);
        /* De-Initialize the DMA associated to reception process */
        HAL_DMA_DeInit(hspi->hdmarx);

        /*##-4- Disable the NVIC for DMA #####################################*/
        HAL_NVIC_DisableIRQ(FLASH_DMA_TX_IRQn);
        HAL_NVIC_DisableIRQ(FLASH_DMA_RX_IRQn);

        /*##-5- Disable the NVIC for SPI #####################################*/
        HAL_NVIC_DisableIRQ(ExtFLASH_IRQn);
    }
}


void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
    flash_state_machine_callback();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    flash_state_machine_callback();
}

