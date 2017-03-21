/*
 *    ======== uartconsole.c ========
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usbd_cdc_if.h"
#include "console/console.h"
#include "console/interpreter.h"

#define PROCESSING                  (0x01)

// Глобальные переменные
//sys_events          Events;
osThreadId ConsoleHandle;

// Локальные переменные
const char overflow[] = {" >>>\r\n"};
console_io_buf      cons;
char                *mstr = &cons.out[CON_OUT + 1];
char strBuf[24];

/*------------------------------------------------------------------------------
 * Запись произвольных данных в консоль
 * pcString - указатель на буфер, len - размер данных
 * Возращает размер записанных данных
 */
int ConsoleWrite (uint8_t *pcString, int len) {
    CDC_Transmit_Data(pcString, (uint16_t)len, 1);
    return len;
}

/*------------------------------------------------------------------------------
 * Функция вывода сообщения в консоль (USB virtual com port)
 */
int ConsoleMes (const char *pcString, ...) {
    va_list vaArgP;
    va_start(vaArgP, pcString);                                                 // Start the varargs processing

    int bufSize, iRet = 0;
	char *bufPtr;

    if(cons.mode == LOG) {
        xSemaphoreTake(cons.gate, 10);                                          //блокируем доступ к ресурсу
        // Буфер свободен
        bufPtr = cons.out;
        bufSize = sizeof(cons.out);

        // печать "символа" переноса командной строки (в буфер)
        if(cons.len > 0) {
            if (cons.timer != true) {
                iRet += usnprintf(bufPtr, bufSize, " >\r\n");
            }
            cons.timer = true;
            cons.ticks = xTaskGetTickCount();
        }
        // печать сообщения (в буфер)
        iRet += uvsnprintf(bufPtr + iRet, bufSize - iRet, pcString, vaArgP);

        //вывод новой строки
        CDC_Transmit_Data((uint8_t*)bufPtr, iRet, 1);
        xSemaphoreGive(cons.gate);
    }
    va_end(vaArgP);                                                             // End the varargs processing
    return iRet;
}

/*------------------------------------------------------------------------------
 * Функция вывода массива данных %02X в консоль (USB virtual com port)
 * msg - указатель на сообщение (NULL - не используется)
 * sepChar - байт разделитель (NULL - не используется)
 * buf - указатель на данные
 * len - их размер
 */
int ConsoleArray (char *msg, uint8_t sepChar, char *buf, uint16_t size) {
    int bufSize, idx, iRet = 0;
    char *bufPtr;

    xSemaphoreTake(cons.gate, 10);                                              //блокируем доступ к ресурсу
    // Буфер свободен
    bufPtr = cons.out;
    bufSize = sizeof(cons.out);

    // печать "символа" переноса командной строки (в буфер)
    if(cons.len > 0) {
        if (cons.timer != true) {
            iRet += usnprintf(bufPtr, bufSize, " >\r\n");
        }
        cons.timer = true;
        cons.ticks = xTaskGetTickCount();
    }

    // печать сообщения (в буфер)
    if (msg != NULL)
        iRet += usnprintf(bufPtr + iRet, bufSize - iRet, msg);
    for(idx = 0; idx < size; ) {
        iRet += usnprintf(bufPtr + iRet, bufSize - iRet, "%02X", buf[idx]);
        ++idx;
        if ((sepChar != 0) && (idx < size) && ((iRet - 1) < bufSize)) {
            bufPtr[iRet++] = sepChar;
        }
    }
    iRet += usnprintf(bufPtr + iRet, bufSize - iRet, "\r\n");

    //вывод новой строки
    CDC_Transmit_Data((uint8_t*)bufPtr, iRet, 1);
    xSemaphoreGive(cons.gate);
    return iRet;
}

/*------------------------------------------------------------------------------
 * Task for this function is created statically. See the project's .cfg file.
 */
void ConsoleTask(void const *pvParameters) {
    char  *conData;
    int   conLen;

    cons.gate = xSemaphoreCreateRecursiveMutex();
    cons.mode = LOG;
    /* Loop forever receiving commands */
    while(true) {
        uint16_t conBufSize = (cons.mode == LOG) ? CON_INP : (CON_INP + CON_OUT);
        //консоль. режим USB-UART моста с устройством
        while(cons.mode == BRIDGE) {
            vTaskDelay(1000);
        }
        //консоль. обработка принятых данных
        if (cons.timer == true && (xTaskGetTickCount() - cons.ticks) >= 950) {
            cons.echo = 1;
            cons.timer = false;
        }
        //FIXME если запрещен вывод сообщений увеличиваем размер входного буфера
        conLen = CDC_Receive_Data((uint8_t*)&cons.inp[cons.len],
                                  conBufSize - cons.len,
                                  (cons.mode == LOG) ? 50 : 1000);
        if (conLen == (conBufSize - cons.len)) {                                //буфер полностью заполнен
            if(cons.mode == LOG) {
                xSemaphoreTake(cons.gate, portMAX_DELAY);                       //блокируем доступ к ресурсу
                cons.mode = CONFIG;                                             //переходим в режим конфигурации
                cons.len += conLen;
            }
            else {
                xSemaphoreGive(cons.gate);
                cons.mode = LOG;                                                //переходим в режим консоли
                cons.len = 0;                                                   //очищаем буфер
            }
            continue;
        }
        conData = &cons.inp[cons.len];
        /***** режим консоли *****/
        if(cons.mode == LOG || cons.mode == CONFIG) {
            char *echoStrg, *str;
            int echoLen;

            xSemaphoreTake(cons.gate, portMAX_DELAY);                           //блокируем доступ к ресурсу
            str = (cons.mode == LOG) ? &cons.out[CON_OUT] : &cons.inp[cons.len + conLen];
            *str = '\0';

            if (conLen > 0) {
                char *ptr = memchr(conData, 0x08, conLen);
                if (ptr) {                                                      //введен код <- (забой)
                    cons.len += (ptr - conData);                                //учитываем данные до "забоя"
                    if(cons.len > 0) {
                        usnprintf(str, (&cons.out[sizeof(cons.out)] - str), "\r\n");
                        --cons.len;
                        cons.echo = 1;
                    }
                    conLen = 0;
                }
                else {
                    if (cons.timer == true)
                        cons.echo = 1;
                    cons.len += conLen;
                }
                cons.timer = false;
                cons.len += conLen;
            }

            echoLen = (cons.out + sizeof(cons.out)) - str;                      //максимальный размер строки
            if(cons.mode == LOG) {                                              //формируем эхо строку
                if(cons.echo) {
                    echoStrg = cons.inp;
                    echoStrg[cons.len] = '\0';
                    cons.echo = 0;
                }
                else {
                    echoStrg = conData;
                    echoStrg[conLen] = '\0';
                }
                if(echoLen > sizeof(overflow))
                    ustrncat(str, echoStrg, echoLen - sizeof(overflow));
                if(echoLen < cons.len)
                    ustrcat(str, overflow);
            }
            if (conLen > 0) {
                char *ptr;
                ptr = memchr(conData, '\r', conLen);                            //поиск "ввода"
                if (ptr) {
                    if (cons.len > 1) {
                        echoStrg = conData;
                        //формируем эхо строку для режима конфигурации
                        if(cons.mode == CONFIG) {
                            echoStrg = str;
                            if(echoLen > sizeof(overflow)) {
                                if(echoLen < cons.len)
                                    echoLen -= sizeof(overflow);
                                ustrncat(echoStrg, cons.inp, echoLen);
                                if(echoLen < cons.len)
                                    ustrcat(echoStrg, overflow);
                            }
                        }
                        echoLen = ustrlen(echoStrg) - 1;
                        str = cons.inp;
                        conLen = cons.len - 1;
                        cons.len = 0;
                        execute_cmd(&str[echoLen], cons.inp, conLen);           //выполнить команду
                        if(echoLen > 0)
                            memmove(str, echoStrg, echoLen);
                        if(cons.mode == CONFIG) {
                            xSemaphoreGive(cons.gate);
                            cons.mode = LOG;
                        }
                    }
                    cons.len = 0;
                }
                else {
                    if (cons.len == conBufSize) {								//обработка переполнения буфера
                        ustrcat(str, overflow);
                        --cons.len;
                    }
                }
            }
            // Вывод итоговой строки/ответа на команду
            conLen = ustrlen(str);
            if(conLen > 0) {
                ConsoleWrite((uint8_t*)str, conLen);
            }
            xSemaphoreGive(cons.gate);
            // выполнена команда перехода в системный режим
            if(cons.mode == SYSTEM) {
                conLen = 0;
                vTaskDelay(100);
            }
        }
    }
}

/*------------------------------------------------------------------------------
 * Cборка строки с текущем временем [HH:MM:SS.TTT]
 */
char *SysTime(void) {
    uint32_t ui32Temp, ticks;
    char *str;

    str = strBuf;
    ticks = xTaskGetTickCount();
    ui32Temp = ticks / 1000;
    *str++ = '[';
    *str++ = '0' + ((ui32Temp / 36000) % 10);
    *str++ = '0' + ((ui32Temp / 3600) % 10);
    *str++ = ':';
    *str++ = '0' + ((ui32Temp / 600) % 6);
    *str++ = '0' + ((ui32Temp / 60) % 10);
    *str++ = ':';
    *str++ = '0' + ((ui32Temp / 10) % 6);
    *str++ = '0' + (ui32Temp % 10);
    *str++ = '.';
    ui32Temp = ticks % 1000;
    *str++ = '0' + (ui32Temp / 100);
    *str++ = '0' + ((ui32Temp % 100) / 10);
    *str++ = '0' + (ui32Temp % 10);
    *str++ = ']';
    *str++ = '\0';
    return strBuf;
}

