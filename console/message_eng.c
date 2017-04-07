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

const char msg_PutPacket[]={"Put packet #%d in queue %d. Queue size #%d\r\n"};
const char msg_DelPacket[]={"Delete packet #%d from queue %d. Queue size #%d\r\n"};
const char msg_SavePacket[]={"Save packet #%d in archive %d\r\n"};
const char msg_DelArchPacket[]={"Delete packet #%d from archive %d\r\n"};
const char msg_LostPacket[]={"Lost packet #%d. Not enough space in archive %d\r\n"};
const char msg_ArchivePkts[]={"%d Packet(s) stored in Archive#%d\r\n"};
const char msg_ArchOverflow[]={"Archive %d overflow! Lost %d packets!\r\n"};
const char msg_ArchiveNotDefined[]={"\r\nWARNING! Archive size is not defined!\r\n"};

#endif /* CONSOLE_MESSAGE_ENG_C_ */
