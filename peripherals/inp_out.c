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
#include "archive/archive.h"
#include "navigation/navigation.h"
#include "settings.h"
#include "Drivers/spi_bus.h"
#include "Drivers/spi_flash/flash_25xxx.h"

//переменные
osThreadId InpOutTaskHandle;

/*------------------------------------------------------------------------------
 * Основная (самая быстрая) задача
 * Осуществляется инициализация, обработка входов/выходов, подсчет архива и пр.
 */
void InputOutputTask(void const * argument) {
    int32_t rslt;

    MX_USB_DEVICE_Init();
    HW_initSPI();

    osDelay(3000);
    // Вывод приветствия в консоль
    ConsoleMes(msg_Welcome);
    ConsoleMes(msg_Version, SYSTEM_VER/100, SYSTEM_VER%100, msg_BuildDate);
    // Инициализация внешней памяти, чтение настроек
    ArchiveChipInit();
    rslt = ReadSettings(&nSolution);
    if(rslt != 0)
        ConsoleMes(msg_BadSettings, rslt);                                      //Ошибка в настройках


    for(;;)
    {
        osDelay(1000);
        ConsoleMes("%s inp_out\r\n", SysTime());
    }
}

