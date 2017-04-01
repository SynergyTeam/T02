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
#include "sflash/ext_memory.h"

//����������
osThreadId InpOutTaskHandle;

/*------------------------------------------------------------------------------
 * �������� (����� �������) ������
 * �������������� �������������, ��������� ������/�������, ������� ������ � ��.
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

