/*
 * inp_out.c
 *
 *  Created on: 20 ����� 2017 �.
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

//����������
osThreadId InpOutTaskHandle;

/*------------------------------------------------------------------------------
 * �������� (����� �������) ������
 * �������������� �������������, ��������� ������/�������, ������� ������ � ��.
 */
void InputOutputTask(void const * argument) {
    int32_t rslt;

    MX_USB_DEVICE_Init();
    HW_initSPI();

    osDelay(3000);
    // ����� ����������� � �������
    ConsoleMes(msg_Welcome);
    ConsoleMes(msg_Version, SYSTEM_VER/100, SYSTEM_VER%100, msg_BuildDate);
    // ������������� ������� ������, ������ ��������
    ArchiveChipInit();
    rslt = ReadSettings(&nSolution);
    if(rslt != 0)
        ConsoleMes(msg_BadSettings, rslt);                                      //������ � ����������


    for(;;)
    {
        osDelay(1000);
        ConsoleMes("%s inp_out\r\n", SysTime());
    }
}

