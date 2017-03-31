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
/*
 *  ======== SPI.c ========
 */

#include <stdint.h>
#include <stdlib.h>
#include <Drivers/spi/SPI.h>



/* Externs */
extern const SPI_Config SPI_config[];

/* Used to check status and initialization */
static int SPI_count = -1;
static uint32_t cpuClock;

/* Default SPI parameters structure */
const SPI_Params SPI_defaultParams = {
    SPI_MODE_BLOCKING,          /* transferMode */
    NULL,                       /* transferCallbackFxn */
    SPI_MASTER,                 /* mode */
    SPI_OUTPUT,
    SPI_BAUDRATEPRESCALER_32,   /* bitRate */
    8,                          /* dataSize */
    SPI_FIRSTBIT_MSB,
    SPI_POL0_PHA0               /* frameFormat */
};

/*
 *  ======== SPI_close ========
 */
void SPI_close(SPI_Handle handle)
{
    assert_param((handle != NULL) && (SPI_count != -1));

    handle->fxnTablePtr->closeFxn(handle);
}

/*
 *  ======== SPI_control ========
 */
int SPI_control(SPI_Handle handle, unsigned int cmd, void *arg)
{
    assert_param(handle != NULL);

    return (handle->fxnTablePtr->controlFxn(handle, cmd, arg));
}

/*
 *  ======== SPI_init ========
 */
void SPI_init(uint32_t clock)
{
    if (SPI_count == -1) {
        cpuClock = clock;
        /* Call each driver's init function */
        for (SPI_count = 0; SPI_config[SPI_count].fxnTablePtr != NULL; SPI_count++) {
            SPI_config[SPI_count].fxnTablePtr->initFxn((SPI_Handle)&(SPI_config[SPI_count]));
        }
    }
}

/*
 *  ======== SPI_open ========
 */
SPI_Handle SPI_open(unsigned int index, SPI_Params *params)
{
    SPI_Handle handle;

    assert_param(index < SPI_count);

    /* Get handle for this driver instance */
    handle = (SPI_Handle)&(SPI_config[index]);

    return (handle->fxnTablePtr->openFxn(handle, params, cpuClock));
}

/*
 *  ======== SPI_Params_init ========
 */
void SPI_Params_init(SPI_Params *params)
{
    assert_param(params != NULL);
    *params = SPI_defaultParams;
}

/*
 *  ======== SPI_serviceISR ========
 */
void SPI_serviceISR(SPI_Handle handle)
{
    assert_param(handle != NULL);

    handle->fxnTablePtr->serviceISRFxn(handle);
}

/*
 *  ======== SPI_transfer ========
 */
bool SPI_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    assert_param((handle != NULL) && (transaction != NULL));

    return (handle->fxnTablePtr->transferFxn(handle, transaction));
}

/*
 *  ======== SPI_transferCancel ========
 */
void SPI_transferCancel(SPI_Handle handle)
{
    assert_param(handle != NULL);

    handle->fxnTablePtr->transferCancelFxn(handle);
}

/*
 *  ======== SPI_DMA_transfer ========
 */
bool SPI_DMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    assert_param((handle != NULL) && (transaction != NULL));

    return (handle->fxnTablePtr->transferDMA(handle, transaction));
}
