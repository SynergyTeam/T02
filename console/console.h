/*
 * console.h
 *
 *  Created on: 05 мая 2016 г.
 *      Author: a_cherepanov
 */

#ifndef UTILS_CONSOLE_H_
#define UTILS_CONSOLE_H_

#include "cmsis_os.h"
#include "console/message.h"
#include "utils/ustdlib.h"

// определения
#define CON_INP                     (1024)
#define CON_OUT                     (1024)
#define TMP_BUF                     (512)

// структура консоли
typedef struct console_io_buf {
    SemaphoreHandle_t   gate;
    uint32_t            ticks;
    uint16_t            len;
    uint8_t             timer;
    uint8_t             echo;
    enum {              // режимы работы консоли
        LOG = 0,
        CONFIG,
        BRIDGE,
        SYSTEM
    } mode;
    char inp[CON_INP];
    char out[CON_OUT + TMP_BUF];
} console_io_buf;

extern char *mstr;
extern console_io_buf cons;
extern osThreadId ConsoleHandle;

extern void ConsoleTask(void const *pvParameters);
extern int ConsoleWrite (uint8_t *pcString, int len);
extern int ConsoleMes (const char *pcString, ...);
extern int ConsoleArray (char *msg, uint8_t sepChar, char *buf, uint16_t size);
extern char *SysTime(void);

#endif /* UTILS_CONSOLE_H_ */
