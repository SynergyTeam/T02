#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "stm32f4xx_hal.h"
#include "struct.h"
#include "archive/queue_mng.h"
#include "archive/archive.h"
#include "settings.h"
//#include "protocol_descr.h"
#include "console/console.h"

// определения
#define ASSERT                      assert_param

// внешние переменные
extern sys_events Events;

//локальные переменные
qInfo_t Queue[MAX_ARCHIVES];

//------------------------------------------------------------------------------
//инициализация очереди
void QueueInit(void) {
    int16_t qu, pckt;

    for (qu = 0; qu < MAX_ARCHIVES; ++qu) {
        Queue[qu].gate = xSemaphoreCreateRecursiveMutex();                      //инициализация Mutex
        Queue[qu].size = Queue[qu].tx_indx = 0;                                 //сброс счетчика и указателя
        memset(Queue[qu].packet, 0, sizeof(Queue[qu].packet));                  //очистка массива
        for (pckt = 0; pckt < MAX_PACKET_QUEUE; ++pckt) {                       //установка указателя
            Queue[qu].packet[pckt].data = Queue[qu].buf;
        }
    }
}

//------------------------------------------------------------------------------
//удаление пакета из очереди. packet номер пакета в очереди [0..N]
void QueueDelPacket(uint8_t qNum, uint8_t pNum) {
    qInfo_t *queue = &Queue[qNum];
    pInfo_t *pckt = &queue->packet[pNum];
    uint8_t *s_data, *d_data;
    uint16_t offset, tLen;

    if (pNum < queue->size) {
        d_data = pckt->data;                                                    //куда
        offset = pckt->size;
        s_data = d_data + offset;                                               //откуда
        if (pNum < queue->tx_indx) --queue->tx_indx;                            //коррекция номера в очереди

        for (tLen = 0; ++pNum < queue->size; ++pckt) {                          //цикл подсчета данных
            pckt[0] = pckt[1];
            tLen += pckt->size;
            if (queue->size > 1)
                pckt->data -= offset;
        }
        pckt->data = queue->buf;                                                //новый заголовок
        pckt->size = 0x00;
        pckt->tx_mark = 0;
        pckt->ack_net = 0;
        pckt->ack_timer = 0x00;
        memmove(d_data, s_data, tLen);                                          //копируем данные

        if (queue->size > 0) --queue->size;                                     //коррекция размера очереди
        if (queue->tx_indx >= queue->size)                                      //проверка и коррекция номера в очереди
            queue->tx_indx = (queue->size > 0) ? (queue->size - 1) : 0;
    }
}

//------------------------------------------------------------------------------
//зарезервировать место в очереди под пакет размером len
void *QueueReservationData(uint8_t qNum, uint16_t len) {
    qInfo_t *queue = &Queue[qNum];                                              //используемые переменные
    pInfo_t *pckt;
    int num, type;
    uint8_t *ptr;
    ASSERT (len <= MAX_OUT_PACKET_SIZE);
    pckt = queue->packet;                                                       //начало очереди
/*
    do {
        if (!queue->size) {
            pckt = queue->packet;                                               //начало очереди
            pckt->data = queue->buf;
            pckt->size = 0;
        } else
            pckt = &queue->packet[queue->size - 1];                             //последний элемент
        if (queue->size <= (MAX_PACKET_QUEUE - 1) &&
                (pckt->data + pckt->size + len) <= &queue->buf[MAX_OUT_PACKET_SIZE])
            break;
        pckt = queue->packet;                                                   //указатель на 1-й пакет в очереди
        type = pckt_GetType(pckt->data);
        if (pckt_DataType(type)) {                                              //проверяем типы пакетов
            type = SavePacket(qNum, (char*)pckt->data, pckt->size);
            num = pckt_GetID(pckt->data);
            if (type == 0) {
                ConsoleMes(msg_LostPacket, num, 1 + qNum);
            }
            if (type < 0) {
                type = -type;
                ConsoleMes(msg_ArchOverflow, qNum + 1, type);                   //Архив переполнен. Cтарые данные стерты
            }
            if (type > 0) {
                ConsoleMes(msg_SavePacket, num, qNum + 1);                      //запись пакета в архив
            }
        }
        QueueDelPacket(qNum, 0);                                                //удаляем сохраненный пакет
    } while (1);
*/
    ptr = pckt->data + pckt->size;                                              //буфер данных
    return ptr;
}

//------------------------------------------------------------------------------
//зарегистрировать пакет в очереди
void QueueRegisterPacket(uint8_t qNum, void *p) {
/*
    pInfo_t *pckt;                                                              //указатель на элемент очереди
    uint16_t pId;
    pckt = &Queue[qNum].packet[Queue[qNum].size++];
    pckt->data = ((p_descr*)p)->Data;
    pckt->size = ((p_descr*)p)->len;
    pckt->app_conf = ((p_descr*)p)->type;
    pckt->tx_mark = 0;
    pckt->tx_attempts = config->M_hconnections;
    pckt->ack_net = 0;
    pId = pckt_GetType(pckt->data);                                             //тип пакета
    if (pckt_DataType(pId)) {
        pId = pckt_GetID(pckt->data);                                           //id пакета
        ConsoleMes(msg_PutPacket, pId, 1 + qNum, Queue[qNum].size);             //номер пакета, состояние очереди
    }
*/
}

//------------------------------------------------------------------------------
//поиск пакета в очереди по id номеру
//если пакет не найден возращает -1, иначе номер в очереди
int32_t QueueSearchPacket(uint8_t qNum, uint16_t idpacket) {
    pInfo_t *pckt = Queue[qNum].packet;
    uint16_t qs;

    for (qs = 0; qs < Queue[qNum].size; ++qs, ++pckt) {
        if (idpacket == pckt_GetID(pckt->data))
            break;
    }
    return ((qs < Queue[qNum].size) ? qs : -1);
}
