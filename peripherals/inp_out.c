/*
 * inp_out.c
 *
 *  Created on: 20 марта 2017 г.
 *      Author: a_cherepanov
 */

#include <stdint.h>
#include "usb_device.h"
#include "peripherals/inp_out.h"
#include "console/console.h"
#include "sflash/ext_memory.h"

//переменные
osThreadId InpOutTaskHandle;

/*------------------------------------------------------------------------------
 * Основная (самая быстрая) задача
 * Осуществляется инициализация, обработка входов/выходов, подсчет архива и пр.
 */
void InputOutputTask(void const * argument) {
    uint32_t flash_size;

    MX_USB_DEVICE_Init();

    osDelay(3000);
    ConsoleMes("Device starting...\r\n");
    flash_init(&flash_size);

    for(;;)
    {
        osDelay(1000);
        ConsoleMes("%s inp_out\r\n", SysTime());
    }
}

