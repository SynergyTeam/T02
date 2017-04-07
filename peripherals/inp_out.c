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
#include "archive/queue_mng.h"
#include "archive/archive.h"
#include "navigation/navigation.h"
#include "settings.h"
#include "Drivers/spi_bus.h"
#include "Drivers/spi_flash/flash_25xxx.h"

//переменные
osThreadId InpOutTaskHandle;
sys_events Events;

/*------------------------------------------------------------------------------
 * ќсновна€ (сама€ быстра€) задача
 * ќсуществл€етс€ инициализаци€, обработка входов/выходов, подсчет архива и пр.
 */
void InputOutputTask(void const * argument) {
    uint32_t sTime;
    int32_t rslt;

    MX_USB_DEVICE_Init();
    HW_initSPI();

    osDelay(3000);
    // ¬ывод приветстви€ в консоль
    ConsoleMes(msg_Welcome);
    ConsoleMes(msg_Version, SYSTEM_VER/100, SYSTEM_VER%100, msg_BuildDate);
    // »нициализаци€ внешней пам€ти, чтение настроек
    ArchiveChipInit();
    sTime = ReadSettings(&nSolution);
    if(sTime != 0)
        ConsoleMes(msg_BadSettings, sTime);                                     //ќшибка в настройках
    Events.device_cfg = 1;

    Events.StoreNavData = ArchiveGetBorder(config->STR_percent, LGD.ptrArchive);
    {
        uint32_t time = 0;
        for(rslt = 0; rslt < MAX_ARCHIVES; ++rslt) {
            PACKET_NUM(rslt) = 0;
            if(config->STR_percent[rslt] > 0) {                                 //размер > 0
                sTime = ArchiveCalcSize(rslt, LGD.ptrArchive[rslt], &PACKET_NUM(rslt));
                ConsoleMes(msg_ArchivePkts, ArchiveGetInfo(rslt, AMNT_PACKETS), 1 + rslt);
                if(sTime > time) time = sTime;
            }
            ++PACKET_NUM(rslt);
        }
//FIXME        CorrectFirstNavTime(&nSolution, time);                                  //начальные услови€ навигационного решени€
    }
    QueueInit();


    for(;;)
    {
        vTaskDelay(50);
        ConsoleMes("%s inp_out\r\n", SysTime());
        /***** сохран€ем данные последнего состо€ни€ *****/
        if (Events.StoreNavData) {
            SaveLastState();
            Events.StoreNavData = 0;
        }
    }
}

