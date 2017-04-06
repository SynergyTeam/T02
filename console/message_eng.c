/*
 * message_eng.c
 *
 *  Created on: 6 апр. 2017 г.
 *      Author: a_cherepanov
 */

#ifndef CONSOLE_MESSAGE_ENG_C_
#define CONSOLE_MESSAGE_ENG_C_

const char msg_Welcome[]={"\f\r\nDevice unit is starting...\r\n"};
const char msg_Version[]={"Firmware version %d.%02d from %s\r\n"};
const char msg_BuildDate[]={__DATE__};

const char msg_BadSettings[]={"Error settings! Code %02X. Default settings are used!\r\n"};

#endif /* CONSOLE_MESSAGE_ENG_C_ */
