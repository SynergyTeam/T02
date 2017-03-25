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
#include <Drivers/spi/SPITivaDMA.h>

#include "FreeRTOS.h"
#include "semphr.h"

/* driverlib header files */
//#include <inc/hw_memmap.h>
//#include <inc/hw_types.h>
//#include <inc/hw_ssi.h>
//#include <driverlib/rom_map.h>
//#include <driverlib/gpio.h>
//#include <driverlib/ssi.h>
//#include <driverlib/udma.h>
//#include <driverlib/sysctl.h>
//#include <driverlib/interrupt.h>

/*
 * Macro to access "Mode of Operation" bits in the SSIPP register
 * If it returns 0; we're using a SSI controller
 * else, we're using a Bi-/Quad-SSI peripheral
 */
//#define SSIMODE(baseAddr) HWREG(baseAddr + SSI_O_PP) & (SSI_PP_MODE_M)

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
 * (8bit or 16bit) transfer sizes.
 * Table for an SPI DMA TX channel
 */
const uint32_t dmaTxConfig[] = {
//    UDMA_SIZE_8  | UDMA_SRC_INC_8    | UDMA_DST_INC_NONE | UDMA_ARB_4, // 8bit
    DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE | DMA_PDATAALIGN_BYTE | DMA_MINC_ENABLE | DMA_MDATAALIGN_BYTE,
//    UDMA_SIZE_16 | UDMA_SRC_INC_16   | UDMA_DST_INC_NONE | UDMA_ARB_4  // 16bit
    DMA_MEMORY_TO_PERIPH | DMA_PINC_DISABLE | DMA_PDATAALIGN_HALFWORD | DMA_MINC_ENABLE | DMA_MDATAALIGN_HALFWORD,
};

/*
 * This lookup table is used to configure the DMA channels for the appropriate
 * (8bit or 16bit) transfer sizes.
 * Table for an SPI DMA RX channel
 */
const uint32_t dmaRxConfig[] = {
//    UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_8    | UDMA_ARB_4, // 8bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_PDATAALIGN_BYTE | DMA_MINC_ENABLE | DMA_MDATAALIGN_BYTE,
//    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16   | UDMA_ARB_4  // 16bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_PDATAALIGN_HALFWORD | DMA_MINC_ENABLE | DMA_MDATAALIGN_HALFWORD,
};

/*
 * This lookup table is used to configure the DMA channels for the appropriate
 * (8bit or 16bit) transfer sizes when either txBuf or rxBuf are NULL
 */
const uint32_t dmaNullConfig[] = {
//    UDMA_SIZE_8  | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4, // 8bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_PDATAALIGN_BYTE | DMA_MINC_DISABLE | DMA_MDATAALIGN_BYTE,
//    UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4  // 16bit
    DMA_PERIPH_TO_MEMORY | DMA_PINC_DISABLE | DMA_PDATAALIGN_HALFWORD | DMA_MINC_DISABLE | DMA_MDATAALIGN_HALFWORD,
};

/*
 *  ======== SPITivaDMA_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
void SPITivaDMA_close(SPI_Handle handle)
{
    SPITivaDMA_Object          *object = handle->object;
    SPITivaDMA_HWAttrs const   *hwAttrs = handle->hwAttrs;

    MAP_SSIDisable(hwAttrs->baseAddr);

    /* Destroy the Hwi */
    MAP_IntDisable(hwAttrs->intNum);
//    IntUnregister(hwAttrs->intNum);                                           //TODO

    /* Destroy the semaphore */
    vSemaphoreDelete(object->transferComplete);

    Log_print1(Diags_USER1, "SPI:(%p) closed", hwAttrs->baseAddr);

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
    SPIDataType                dummy;
    void                      *buf;
    uint32_t                   channelControlOptions;
    SPITivaDMA_Object         *object = handle->object;
    SPITivaDMA_HWAttrs const  *hwAttrs = handle->hwAttrs;

    /* Clear out the FIFO */
    while (MAP_SSIDataGetNonBlocking(hwAttrs->baseAddr, &dummy)) {
    }

    if (transaction->txBuf) {
        channelControlOptions = dmaTxConfig[object->frameSize];
        buf = transaction->txBuf;
    }
    else {
        channelControlOptions = dmaNullConfig[object->frameSize];
        *hwAttrs->scratchBufPtr = hwAttrs->defaultTxBufValue;
        buf = hwAttrs->scratchBufPtr;
    }

    /* Setup the TX transfer characteristics */
    MAP_uDMAChannelControlSet(hwAttrs->txChannelIndex | UDMA_PRI_SELECT,
                              channelControlOptions);

    /* Setup the TX transfer buffers */
    MAP_uDMAChannelTransferSet(hwAttrs->txChannelIndex | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               buf,
                               (void *)(hwAttrs->baseAddr + SSI_O_DR),
                               transaction->count);

    if (transaction->rxBuf) {
        channelControlOptions = dmaRxConfig[object->frameSize];
        buf = transaction->rxBuf;
    }
    else {
        channelControlOptions = dmaNullConfig[object->frameSize];
        buf = hwAttrs->scratchBufPtr;
    }

    /* Setup the RX transfer characteristics */
    MAP_uDMAChannelControlSet(hwAttrs->rxChannelIndex | UDMA_PRI_SELECT,
                              channelControlOptions);

    /* Setup the RX transfer buffers */
    MAP_uDMAChannelTransferSet(hwAttrs->rxChannelIndex | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(hwAttrs->baseAddr + SSI_O_DR),
                               buf,
                               transaction->count);

    Log_print1(Diags_USER1,"SPI:(%p) DMA transfer enabled", hwAttrs->baseAddr);

    Log_print5(Diags_USER2,"SPI:(%p) DMA transaction: %p, "
                           "rxBuf: %p; txBuf: %p; Count: %d",
                            hwAttrs->baseAddr,
                            (UArg)transaction,
                            (UArg)transaction->rxBuf,
                            (UArg)transaction->txBuf,
                            (UArg)transaction->count);

    /* A lock is needed because we are accessing shared uDMA registers.*/
    MAP_IntDisable(hwAttrs->intNum);

    /* Configure channel mapping */
    hwAttrs->channelMappingFxn(hwAttrs->txChannelMappingFxnArg);
    hwAttrs->channelMappingFxn(hwAttrs->rxChannelMappingFxnArg);

    /* Enable DMA Channels */
    MAP_uDMAChannelEnable(hwAttrs->txChannelIndex);
    MAP_uDMAChannelEnable(hwAttrs->rxChannelIndex);

    if (SSIMODE(hwAttrs->baseAddr)) {
        MAP_SSIIntClear(hwAttrs->baseAddr, SSI_DMATX | SSI_DMARX);
        MAP_SSIIntEnable(hwAttrs->baseAddr, SSI_DMATX | SSI_DMARX);
    }

    MAP_IntEnable(hwAttrs->intNum);
    return (true);
}

/*
 *  ======== SPITivaDMA_hwiFxn ========
 *  ISR for the SPI when we use the DMA
 */
void SPITivaDMA_hwiFxn(uint32_t arg)
{
    SPI_Transaction           *msg;
    SPITivaDMA_Object         *object = ((SPI_Handle)arg)->object;
    SPITivaDMA_HWAttrs const  *hwAttrs = ((SPI_Handle)arg)->hwAttrs;

    Log_print1(Diags_USER2, "SPI:(%p) interrupt context start", hwAttrs->baseAddr);

#if !defined(MWARE)
    if (SSIMODE(hwAttrs->baseAddr)) {
        /* We know that at least the Tx channel triggered the interrupt */
        MAP_SSIIntDisable(hwAttrs->baseAddr, SSI_DMATX);
    }
#endif

    /* Determine if the TX and RX DMA channels have completed */
    if ((object->transaction) &&
        (MAP_uDMAChannelIsEnabled(hwAttrs->rxChannelIndex) == false)) {

#if !defined(MWARE)
        if (SSIMODE(hwAttrs->baseAddr)) {
            /* Now we know that the Rx channel may have triggered the interrupt */
            MAP_SSIIntDisable(hwAttrs->baseAddr, SSI_DMARX);
        }
#endif

        /*
         * Use a temporary transaction pointer in case the callback function
         * attempts to perform another SPI_transfer call
         */
        msg = object->transaction;

        /* Indicate we are done with this transfer */
        object->transaction = NULL;

        Log_print2(Diags_USER1,"SPI:(%p) DMA transaction: %p complete",
                                hwAttrs->baseAddr, (UArg)msg);

        /* Perform callback */
        object->transferCallbackFxn((SPI_Handle)arg, msg);
    }

    Log_print1(Diags_USER2, "SPI:(%p) interrupt context end",
                             hwAttrs->baseAddr);
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
    SPITivaDMA_Object         *object = handle->object;
    SPITivaDMA_HWAttrs const  *hwAttrs = handle->hwAttrs;

    /* Determine if the device index was already opened */
    MAP_IntDisable(hwAttrs->intNum);
    if (object->isOpen == true) {
        MAP_IntEnable(hwAttrs->intNum);
        return (NULL);
    }

    /* Mark the handle as being used */
    object->isOpen = true;

    /* Store the SPI parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (SPI_Params *) &SPI_defaultParams;
    }

    Assert_isTrue((params->dataSize >= 4) &&
                  (params->dataSize <= 16), NULL);

    /* Determine if we need to use an 8-bit or 16-bit framesize for the DMA */
    object->frameSize = (params->dataSize < 9) ? SPITivaDMA_8bit : SPITivaDMA_16bit;

    Log_print2(Diags_USER2,"SPI:(%p) DMA buffer incrementation size: %s",
                            hwAttrs->baseAddr,
                           (object->frameSize) ? (UArg)"16-bit" : (UArg)"8-bit");

    MAP_SSIIntDisable(hwAttrs->baseAddr, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
    if (SSIMODE(hwAttrs->baseAddr)) {
        MAP_SSIIntDisable(hwAttrs->baseAddr, SSI_DMATX | SSI_DMARX);
    }

    /* Create the Hwi for this SPI peripheral */
    MAP_IntEnable(hwAttrs->intNum);
//    IntRegister(hwAttrs->intNum, void (*pfnHandler)(void));                   //TODO

    /* Store the current mode */
    object->transferMode = params->transferMode;
    object->transaction = NULL;

    /*
     * Create a semaphore to block task execution for the duration of the SPI transfer
     */
    object->transferComplete = xSemaphoreCreateBinary();

    if (object->transferMode == SPI_MODE_BLOCKING) {
        Log_print1(Diags_USER2, "SPI:(%p) in SPI_MODE_BLOCKING mode", hwAttrs->baseAddr);
        /* Store internal callback function */
        object->transferCallbackFxn = SPITivaDMA_transferCallback;
    }
    else {
        Log_print1(Diags_USER2, "SPI:(%p) in SPI_MODE_CALLBACK mode", hwAttrs->baseAddr);

        /* Check to see if a callback function was defined for async mode */
        Assert_isTrue(params->transferCallbackFxn != NULL, NULL);

        /* Save the callback function pointer */
        object->transferCallbackFxn = params->transferCallbackFxn;
    }

    /* Set the SPI configuration */
    MAP_SSIConfigSetExpClk(hwAttrs->baseAddr,
                           clock,
                           params->frameFormat,
                           params->mode,
                           params->bitRate,
                           params->dataSize);

    Log_print3(Diags_USER1, "SPI:(%p) CPU freq: %d; SPI freq to %d",
                             hwAttrs->baseAddr, freq.lo, params->bitRate);

    MAP_SSIDMAEnable(hwAttrs->baseAddr, SSI_DMA_TX | SSI_DMA_RX);

    MAP_SSIEnable(hwAttrs->baseAddr);

    Log_print1(Diags_USER1, "SPI:(%p) opened", hwAttrs->baseAddr);

    return (handle);
}

/*
 *  ======== SPITivaDMA_serviceISR ========
 */
void SPITivaDMA_serviceISR(SPI_Handle handle)
{
    /* Function is not supported */
    Assert_isTrue(handle != NULL, NULL);
    SPITivaDMA_hwiFxn((uint32_t)handle);
}

/*
 *  ======== SPITivaDMA_transfer ========
 *  @pre    Function assumes that handle and transaction is not NULL
 */
bool SPITivaDMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    SPITivaDMA_Object         *object = handle->object;
    SPITivaDMA_HWAttrs const  *hwattrs = handle->hwAttrs;

    /* Check the transaction arguments */
    if ((transaction->count == 0) || (transaction->count > 1024) ||
       !(transaction->rxBuf || transaction->txBuf) ||
      (!(transaction->rxBuf && transaction->txBuf) && !hwattrs->scratchBufPtr)) {
        return (false);
    }

    /* Make sure that the buffers are aligned properly */
    if (object->frameSize == SPITivaDMA_16bit) {
        Assert_isTrue(!((uint32_t)transaction->txBuf & 0x1), NULL);
        Assert_isTrue(!((uint32_t)transaction->rxBuf & 0x1), NULL);
    }

    /* Check if a transfer is in progress */
    MAP_IntDisable(hwattrs->intNum);
    if (object->transaction) {
        MAP_IntEnable(hwattrs->intNum);

        Log_error1("SPI:(%p) transaction still in progress",
                   hwattrs->baseAddr);

        /* Transfer is in progress */
        return (false);
    }
    else {
        /* Save the pointer to the transaction */
        object->transaction = transaction;
    }
    MAP_IntEnable(hwattrs->intNum);

    SPITivaDMA_configDMA(handle, transaction);

    if (object->transferMode == SPI_MODE_BLOCKING) {
        Log_print1(Diags_USER1,
                   "SPI:(%p) transfer pending on transferComplete semaphore",
                    hwattrs->baseAddr);
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
    Assert_isTrue(false, NULL);
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
    SPITivaDMA_Object         *object = handle->object;

    Log_print1(Diags_USER1, "SPI:(%p) posting transferComplete semaphore",
                ((SPITivaDMA_HWAttrs const *)(handle->hwAttrs))->baseAddr);

    object->h_pr_TaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(object->transferComplete, &object->h_pr_TaskWoken);
}

