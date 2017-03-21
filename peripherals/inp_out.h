/*
 * inp_out.h
 *
 *  Created on: 21 марта 2017 г.
 *      Author: a_cherepanov
 */

#ifndef PERIPHERALS_INP_OUT_H_
#define PERIPHERALS_INP_OUT_H_

#include "cmsis_os.h"

//внешние переменные
extern osThreadId InpOutTaskHandle;

//внешние функции
extern void InputOutputTask(void const * argument);

#endif /* PERIPHERALS_INP_OUT_H_ */
