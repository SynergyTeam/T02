/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <Drivers/driver_logs.h>
#include <Drivers/spi/SPITivaDMA.h>

#include "FreeRTOS.h"
#include "semphr.h"

/*----------------------------------------------------------------------------*/
// локальная копия структуры из файла stm32f4xx_hal_dma.c
typedef struct {
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


/* SPITiva functions */
void         SPITivaDMA_close(SPI_Handle handle);
int          SPITivaDMA_control(SPI_Handle handle, unsigned int cmd, void *arg);
void         SPITivaDMA_init(SPI_Handle handle);
SPI_Handle   SPITivaDMA_open(SPI_Handle handle, SPI_Params *params, uint32_t clock);
void         SPITivaDMA_serviceISR(SPI_Handle handle);
bool         SPITivaDMA_transfer(SPI_Handle handle, SPI_Transaction *transaction);
void         SPITivaDMA_transferCancel(SPI_Handle handle);
static void  SPITivaDMA_transferCallback(SPI_Handle handle, SPI_Transaction *transaction);
bool         SPITivaDMA_configDMA(SPI_Handle handle, SPI_Transaction *transaction);

/* SPI function table for SPITivaDMA implementation */
const SPI_FxnTable SPITivaDMA_fxnTable = {
    SPITivaDMA_close,
    SPITivaDMA_control,
    SPITivaDMA_init,
    SPITivaDMA_open,
    SPITivaDMA_transfer,
    SPITivaDMA_transferCancel,
    SPITivaDMA_serviceISR,
    SPITivaDMA_configDMA
};

/* Default SPI params */
extern const SPI_Params SPI_defaultParams;

/*
 * This lookup table is used to configure the DMA channels for the appropriate
 * (8bit / 16bit / 32bit) transfer sizes.
 * Table for an SPI DMA TX channel
 */
const uint32_t dmaTxConfig[] = {
    // 8 bit
    DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE | DMA_MINC_ENABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_BYTE     | DMA_MDATAALIGN_BYTE     | DMA_NORMAL | DMA_PRIORITY_LOW,
    // 16 bit
    DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE | DMA_MINC_ENABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD | DMA_NORMAL | DMA_PRIORITY_LOW,
    // 32 bit
    DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE | DMA_MINC_ENABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_WORD     | DMA_MDATAALIGN_WORD     | DMA_NORMAL | DMA_PRIORITY_LOW,
};

/*
 * This lookup table is used to configure the DMA channels for the appropriate
 * (8bit / 16bit / 32bit) transfer sizes.
 * Table for an SPI DMA RX channel
 */
const uint32_t dmaRxConfig[] = {
    // 8 bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_MINC_ENABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_BYTE     | DMA_MDATAALIGN_BYTE     | DMA_NORMAL | DMA_PRIORITY_LOW,
    // 16 bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_MINC_ENABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD | DMA_NORMAL | DMA_PRIORITY_LOW,
    // 32 bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_MINC_ENABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_WORD     | DMA_MDATAALIGN_WORD     | DMA_NORMAL | DMA_PRIORITY_LOW,
};

/*
 * This lookup table is used to configure the DMA channels for the appropriate
 * (8bit / 16bit / 32bit) transfer sizes when either txBuf or rxBuf are NULL
 */
const uint32_t dmaNullConfig[] = {
    // 8 bit
    DMA_PINC_DISABLE | DMA_MINC_DISABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_BYTE     | DMA_MDATAALIGN_BYTE     | DMA_NORMAL | DMA_PRIORITY_LOW,
    // 16 bit
    DMA_PINC_DISABLE | DMA_MINC_DISABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_HALFWORD | DMA_MDATAALIGN_HALFWORD | DMA_NORMAL | DMA_PRIORITY_LOW,
    // 32 bit
    DMA_PINC_DISABLE | DMA_MINC_DISABLE | DMA_MBURST_INC4 | DMA_PBURST_INC4 | DMA_FIFOMODE_DISABLE | DMA_FIFO_THRESHOLD_FULL | DMA_PDATAALIGN_WORD     | DMA_MDATAALIGN_WORD     | DMA_NORMAL | DMA_PRIORITY_LOW,
};

/**
  * @brief  DMA SPI communication error callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = (SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

/* Stop the disable DMA transfer on SPI side */
  CLEAR_BIT(hspi->Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

  SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
  hspi->State = HAL_SPI_STATE_READY;
//  HAL_SPI_ErrorCallback(hspi);                                                //FIXME
}

/*------------------------------------------------------------------------------
 * Настройка параметров uDMA транзакции
 */
static void uDMAChannelControlSet(DMA_HandleTypeDef *chnl, uint32_t control) {
    static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
    uint32_t tmp;

    /* Disable the peripheral */
    __HAL_DMA_DISABLE(chnl);

    /* XXX возможно требуется доработка
     * Check if the DMA Stream is effectively disabled
     */
    for(tmp = HAL_GetTick(); (chnl->Instance->CR & DMA_SxCR_EN) != RESET; ) {
        /* Check for the Timeout */
        if((HAL_GetTick() - tmp) > 100) {
            /* Update error code */
            chnl->ErrorCode = HAL_DMA_ERROR_TIMEOUT;
            /* Change the DMA state */
            chnl->State = HAL_DMA_STATE_TIMEOUT;
            return;
        }
    }

    /* Get the CR register value */
    tmp = chnl->Instance->CR;

    /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits */
    tmp &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                        DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
                        DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
                        DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

    /* Prepare the DMA Stream configuration */
    tmp |= control;

    /* Write to DMA Stream CR register */
    chnl->Instance->CR = tmp;

    /* Get the FCR register value */
    tmp = chnl->Instance->FCR;
    /* Clear Direct mode and FIFO threshold bits */
    tmp &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
    /* Prepare the DMA Stream FIFO configuration */
    tmp |= DMA_FIFOMODE_DISABLE;
    /* Write to DMA Stream FCR */
    chnl->Instance->FCR = tmp;

    /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
       DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
    tmp = (((uint32_t)chnl->Instance & 0xFFU) - 16U) / 24U;
    chnl->StreamIndex = flagBitshiftOffset[tmp];
    if (tmp > 3U) {
        /* return pointer to HISR and HIFCR */
        chnl->StreamBaseAddress = (((uint32_t)chnl->Instance & (uint32_t)(~0x3FFU)) + 4U);
    }
    else {
        /* return pointer to LISR and LIFCR */
        chnl->StreamBaseAddress = ((uint32_t)chnl->Instance & (uint32_t)(~0x3FFU));
    }

    /* Clear all interrupt flags */
    ((DMA_Base_Registers*)chnl->StreamBaseAddress)->IFCR = 0x3FU << chnl->StreamIndex;

    /* Initialize the error code */
    chnl->ErrorCode = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state */
    chnl->State = HAL_DMA_STATE_READY;
}
/*
 *  ======== SPITivaDMA_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void SPITivaDMA_close(SPI_Handle handle)
{
    SPITivaDMA_Object          *object = handle->object;
    SPIDMA_HWAttrs const   *hwAttrs = handle->hwAttrs;

    __HAL_SPI_DISABLE(hwAttrs);

    /* De-Initialize the DMA associated to transmission process */
    HAL_DMA_DeInit(object->hdmatx);
    vPortFree(object->hdmatx);

    /* De-Initialize the DMA associated to reception process */
    HAL_DMA_DeInit(object->hdmarx);
    vPortFree(object->hdmarx);

    /* Destroy the Hwi */
    /*##-4- Disable the NVIC for DMA #########################################*/
    HAL_NVIC_DisableIRQ(hwAttrs->uDMA_txChnl);
    HAL_NVIC_DisableIRQ(hwAttrs->uDMA_rxInt);

    /*##-5- Disable the NVIC for SPI #########################################*/
    HAL_NVIC_DisableIRQ(hwAttrs->intNum);

    /* Destroy the semaphore */
    vSemaphoreDelete(object->transferComplete);

    Log_print1(Diags_USER1, "SPI:(%p) closed", hwAttrs->Instance);

    object->isOpen = false;
}

/*
 *  ======== SPITivaDMA_control ========
 *  @pre    Function assumes that the handle is not NULL
 */
int SPITivaDMA_control(SPI_Handle handle, unsigned int cmd, void *arg)
{
	/* No implementation yet */
	return (SPITivaDMA_CMD_UNDEFINED);
}

/*
 *  ======== SPITivaDMA_configDMA ========
 *  This functions configures the transmit and receive DMA channels for a given
 *  SPI_Handle and SPI_Transaction
 *
 *  @pre    Function assumes that the handle and transaction is not NULL
 */
bool SPITivaDMA_configDMA(SPI_Handle handle, SPI_Transaction *transaction)
{
    void                      *buf;
    uint32_t                   channelControlOptions;
    SPITivaDMA_Object         *object = handle->object;
    SPIDMA_HWAttrs const  *hwAttrs = handle->hwAttrs;

    if (transaction->txBuf) {
        channelControlOptions = dmaTxConfig[object->frameSize];
        buf = transaction->txBuf;
    }
    else {
        channelControlOptions = dmaNullConfig[object->frameSize];
        channelControlOptions |= DMA_PERIPH_TO_MEMORY;
        *hwAttrs->scratchBufPtr = hwAttrs->defaultTxBufValue;
        buf = hwAttrs->scratchBufPtr;
    }

    /* Setup the TX transfer characteristics */
    uDMAChannelControlSet(object->hdmatx, channelControlOptions);
    object->hdmatx->XferHalfCpltCallback = NULL;
    object->hdmatx->XferCpltCallback     = NULL;
    object->hdmatx->XferErrorCallback    = NULL;
    object->hdmatx->XferAbortCallback    = NULL;

    /* Enable the Tx DMA Stream */
    HAL_DMA_Start_IT(object->hdmatx, (uint32_t)buf, (uint32_t)&hwAttrs->Instance->DR, transaction->count);

    if (transaction->rxBuf) {
        channelControlOptions = dmaRxConfig[object->frameSize];
        buf = transaction->rxBuf;
    }
    else {
        channelControlOptions = dmaNullConfig[object->frameSize];
        channelControlOptions |= DMA_MEMORY_TO_PERIPH;
        buf = hwAttrs->scratchBufPtr;
    }

    /* Setup the RX transfer characteristics */
    uDMAChannelControlSet(object->hdmarx, channelControlOptions);
    object->hdmarx->XferHalfCpltCallback = NULL;
    object->hdmarx->XferCpltCallback = SPITivaDMA_transferCallback;             //FIXME
    /* Set the DMA error callback */
    object->hdmarx->XferErrorCallback = SPI_DMAError;
    /* Set the DMA AbortCpltCallback */
    object->hdmarx->XferAbortCallback = NULL;

    /* Setup the RX transfer buffers */
    HAL_DMA_Start_IT(object->hdmarx, (uint32_t)&hwAttrs->Instance->DR, (uint32_t)buf, transaction->count);

    Log_print1(Diags_USER1,"SPI:(%p) DMA transfer enabled", hwAttrs->Instance);

    Log_print5(Diags_USER2,"SPI:(%p) DMA transaction: %p, "
                           "rxBuf: %p; txBuf: %p; Count: %d",
                            hwAttrs->Instance,
                            (UArg)transaction,
                            (UArg)transaction->rxBuf,
                            (UArg)transaction->txBuf,
                            (UArg)transaction->count);

    /* Enable Rx DMA Request */
    SET_BIT(hwAttrs->Instance->CR2, SPI_CR2_RXDMAEN);

    /* Check if the SPI is already enabled */
    if((hwAttrs->Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE) {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(hwAttrs);
    }
    /* Enable the SPI Error Interrupt Bit */
    SET_BIT(hwAttrs->Instance->CR2, SPI_CR2_ERRIE);

    /* Enable Tx DMA Request */
    SET_BIT(hwAttrs->Instance->CR2, SPI_CR2_TXDMAEN);
    return (true);
}

/*
 *  ======== SPITivaDMA_hwiFxn ========
 *  ISR for the SPI when we use the DMA
 */
void SPITivaDMA_hwiFxn(uint32_t arg)
{
    SPI_Transaction      *msg;
    SPITivaDMA_Object    *object = ((SPI_Handle)arg)->object;
    SPIDMA_HWAttrs const *hwAttrs = ((SPI_Handle)arg)->hwAttrs;

    Log_print1(Diags_USER2, "SPI:(%p) interrupt context start", hwAttrs->Instance);

//    if (SSIMODE(hwAttrs->Instance)) {
//        /* We know that at least the Tx channel triggered the interrupt */
//        MAP_SSIIntDisable(hwAttrs->Instance, SSI_DMATX);
//    }

    /* Determine if the TX and RX DMA channels have completed */
    if ((object->transaction) /*&&
        (MAP_uDMAChannelIsEnabled(hwAttrs->rxChannelIndex) == false)*/) {

//        if (SSIMODE(hwAttrs->Instance)) {
//            /* Now we know that the Rx channel may have triggered the interrupt */
//            MAP_SSIIntDisable(hwAttrs->Instance, SSI_DMARX);
//        }

        /*
         * Use a temporary transaction pointer in case the callback function
         * attempts to perform another SPI_transfer call
         */
        msg = object->transaction;

        /* Indicate we are done with this transfer */
        object->transaction = NULL;

        Log_print2(Diags_USER1,"SPI:(%p) DMA transaction: %p complete",
                                hwAttrs->Instance, (UArg)msg);

        /* Perform callback */
        object->transferCallbackFxn((SPI_Handle)arg, msg);
    }

    Log_print1(Diags_USER2, "SPI:(%p) interrupt context end",
                             hwAttrs->Instance);
}

/*
 *  ======== SPITivaDMA_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
void SPITivaDMA_init(SPI_Handle handle)
{
    /* Mark the object as available */
    ((SPITivaDMA_Object *)(handle->object))->isOpen = false;
}

/*
 *  ======== SPITivaDMA_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
SPI_Handle SPITivaDMA_open(SPI_Handle handle, SPI_Params *params, uint32_t clock)
{
    SPITivaDMA_Object *object = handle->object;
    SPIDMA_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Determine if the device index was already opened */
    if (object->isOpen == true) {
        return (NULL);
    }

    /* Mark the handle as being used */
    object->isOpen = true;

    /* Store the SPI parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (SPI_Params *) &SPI_defaultParams;
    }

    assert_param((params->dataSize >= 4) && (params->dataSize <= 16));          //FIXME

    /* Determine if we need to use an 8-bit or 16-bit framesize for the DMA */
    object->frameSize = (params->dataSize < 9) ? SPITivaDMA_8bit : SPITivaDMA_16bit;

    Log_print2(Diags_USER2,"SPI:(%p) DMA buffer incrementation size: %s",
                            hwAttrs->Instance,
                           (object->frameSize) ? (UArg)"16-bit" : (UArg)"8-bit");

    /* Store the current mode */
    object->transferMode = params->transferMode;
    object->transaction = NULL;

    /*
     * Create a semaphore to block task execution for the duration of the SPI transfer
     */
    object->transferComplete = xSemaphoreCreateBinary();

    if (object->transferMode == SPI_MODE_BLOCKING) {
        Log_print1(Diags_USER2, "SPI:(%p) in SPI_MODE_BLOCKING mode", hwAttrs->Instance);
        /* Store internal callback function */
        object->transferCallbackFxn = SPITivaDMA_transferCallback;
    }
    else {
        Log_print1(Diags_USER2, "SPI:(%p) in SPI_MODE_CALLBACK mode", hwAttrs->Instance);

        /* Check to see if a callback function was defined for async mode */
        assert_param(params->transferCallbackFxn != NULL);

        /* Save the callback function pointer */
        object->transferCallbackFxn = params->transferCallbackFxn;
    }

    /* Disable the selected SPI peripheral */
    __HAL_SPI_DISABLE(hwAttrs);

    /* SPIx CR1 & CR2 Configuration */
    /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
    Communication speed, First bit and CRC calculation state */
    WRITE_REG(hwAttrs->Instance->CR1, (params->mode | SPI_DIRECTION_2LINES | params->dataSize |
                                       params->frameFormat | (params->nss & SPI_CR1_SSM) |
                                       params->bitRate | params->firstBit  | SPI_CRCCALCULATION_DISABLE) );

    /* Configure : NSS management */
    WRITE_REG(hwAttrs->Instance->CR2, (((params->nss >> 16U) & SPI_CR2_SSOE) | SPI_TIMODE_DISABLE));

    /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
    CLEAR_BIT(hwAttrs->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);

    Log_print3(Diags_USER1, "SPI:(%p) CPU freq: %d; SPI freq to %d",
                             hwAttrs->Instance, freq.lo, params->bitRate);

    object->hdmatx = pvPortMalloc(sizeof(DMA_HandleTypeDef));
    /* Configure the DMA handler for Transmission process */
    object->hdmatx->Instance = (DMA_Stream_TypeDef*)hwAttrs->uDMA_txStream;
    object->hdmatx->Init.Channel = hwAttrs->uDMA_txChnl;
    object->hdmatx->Parent = handle;

    object->hdmarx = pvPortMalloc(sizeof(DMA_HandleTypeDef));
    /* Configure the DMA handler for Transmission process */
    object->hdmarx->Instance = (DMA_Stream_TypeDef*)hwAttrs->uDMA_rxStream;;
    object->hdmarx->Init.Channel = hwAttrs->uDMA_rxChnl;
    object->hdmarx->Parent = handle;

    /*##-3- Configure the NVIC for SPI #######################################*/
    /* NVIC for SPI */
    HAL_NVIC_SetPriority(hwAttrs->intNum, 1, 0);
    HAL_NVIC_EnableIRQ(hwAttrs->intNum);

    /*##-4- Configure the NVIC for DMA #######################################*/
    /* NVIC configuration for DMA transfer complete interrupt (SPI2_TX) */
    HAL_NVIC_SetPriority(hwAttrs->uDMA_txInt, 1, 1);
    HAL_NVIC_EnableIRQ(hwAttrs->uDMA_txInt);

    /* NVIC configuration for DMA transfer complete interrupt (SPI2_RX) */
    HAL_NVIC_SetPriority(hwAttrs->uDMA_rxInt, 1, 0);
    HAL_NVIC_EnableIRQ(hwAttrs->uDMA_rxInt);

    /*##-5- Configure the NVIC for SPI #######################################*/
    HAL_NVIC_SetPriority(hwAttrs->intNum, 1, 2);
    HAL_NVIC_EnableIRQ(hwAttrs->intNum);

    Log_print1(Diags_USER1, "SPI:(%p) opened", hwAttrs->Instance);

    return (handle);
}

/*
 *  ======== SPITivaDMA_serviceISR ========
 */
void SPITivaDMA_serviceISR(SPI_Handle handle)
{
    /* Function is not supported */
    assert_param(handle != NULL);
    SPITivaDMA_hwiFxn((uint32_t)handle);
}

/*
 *  ======== SPITivaDMA_transfer ========
 *  @pre    Function assumes that handle and transaction is not NULL
 */
bool SPITivaDMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    SPITivaDMA_Object         *object = handle->object;
    SPIDMA_HWAttrs const  *hwattrs = handle->hwAttrs;

    /* Check the transaction arguments */
    if ((transaction->count == 0) || (transaction->count > 1024) ||
       !(transaction->rxBuf || transaction->txBuf) ||
      (!(transaction->rxBuf && transaction->txBuf) && !hwattrs->scratchBufPtr)) {
        return (false);
    }

    /* Make sure that the buffers are aligned properly */
    if (object->frameSize == SPITivaDMA_16bit) {
        assert_param(!((uint32_t)transaction->txBuf & 0x1));
        assert_param(!((uint32_t)transaction->rxBuf & 0x1));
    }

    /* Check if a transfer is in progress */
//    MAP_IntDisable(hwattrs->intNum);
    if (object->transaction) {
//        MAP_IntEnable(hwattrs->intNum);

        Log_error1("SPI:(%p) transaction still in progress",
                   hwattrs->Instance);

        /* Transfer is in progress */
        return (false);
    }
    else {
        /* Save the pointer to the transaction */
        object->transaction = transaction;
    }
//    MAP_IntEnable(hwattrs->intNum);

    SPITivaDMA_configDMA(handle, transaction);

    if (object->transferMode == SPI_MODE_BLOCKING) {
        Log_print1(Diags_USER1,
                   "SPI:(%p) transfer pending on transferComplete semaphore",
                    hwattrs->Instance);
        xSemaphoreTake(object->transferComplete, portMAX_DELAY);
    }

    return (true);
}

/*
 *  ======== SPITivaDMA_transferCancel ========
 *  A function to cancel a transaction (if one is in progress) when the driver
 *  is in SPI_MODE_CALLBACK.
 *
 *  @pre    Function assumes that the handle is not NULL
 */
void SPITivaDMA_transferCancel(SPI_Handle handle)
{
	/* No implementation yet */
    assert_param(false);
}

/*
 *  ======== SPITivaDMA_transferCallback ========
 *  Callback function for when the SPI is in SPI_MODE_BLOCKING
 *
 *  @pre    Function assumes that the handle is not NULL
 */
static void SPITivaDMA_transferCallback(SPI_Handle handle,
                                        SPI_Transaction *transaction)
{
    SPITivaDMA_Object *object = handle->object;
    BaseType_t TaskWoken = pdFALSE;

    Log_print1(Diags_USER1, "SPI:(%p) posting transferComplete semaphore",
                ((SPIDMA_HWAttrs const *)(handle->hwAttrs))->Instance);

     xSemaphoreGiveFromISR(object->transferComplete, &TaskWoken);
}

