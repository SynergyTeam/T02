/*
 * inp_out.h
 *
 *  Created on: 21 ����� 2017 �.
 *      Author: a_cherepanov
 */

#ifndef PERIPHERALS_INP_OUT_H_
#define PERIPHERALS_INP_OUT_H_

#include "cmsis_os.h"

//������� ����������
extern osThreadId InpOutTaskHandle;

//������� �������
extern void InputOutputTask(void const * argument);

#endif /* PERIPHERALS_INP_OUT_H_ */
