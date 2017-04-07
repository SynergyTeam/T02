//------------------------------------------------------------------------------
#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "config.h"
#include "cmsis_os.h"

//определение сквозного номера пакета
#define PACKET_NUM(x)               (Queue[x].PacketNum)
#define QUEUE_LOCK(x)               (xSemaphoreTakeRecursive(Queue[x].gate, portMAX_DELAY))
#define QUEUE_UNLOCK(x)             (xSemaphoreGiveRecursive(&Queue[x].gate))

//описание заголовка пакета в очереди
typedef struct {
    uint8_t     *data;                              // указатель на данные
    unsigned    size :12;                           // размер данных
    unsigned    tx_attempts :5;                     // число попыток передачи
    unsigned    tx_mark :1;                         // флаг переданного пакета
    unsigned    app_conf :1;                        // необходимо подтверждение на уровне приложения
    unsigned    ack_net :1;                         // флаг сетевого подтверждения
    unsigned    ack_timer :12;                      // таймер подтверждения
} pInfo_t;

//Структура описания очереди
typedef struct qInfo_t {
    SemaphoreHandle_t   gate;                       // мютекс доступа
    pInfo_t             packet[MAX_PACKET_QUEUE];   // массив описаний пакетов
    uint16_t            PacketNum;                  // номер пакета
    uint8_t             size;                       // размер пакета
    uint8_t             tx_indx;
    uint8_t             buf[MAX_OUT_PACKET_SIZE];   // массив пакетов
} qInfo_t;

extern qInfo_t Queue[MAX_ARCHIVES];
/*
*/
extern void QueueInit(void);
extern void QueueDelPacket(uint8_t qNum, uint8_t packet);
extern void *QueueReservationData(uint8_t qNum, uint16_t len);
extern void QueueRegisterPacket(uint8_t qNum, void *P);
extern int32_t QueueSearchPacket(uint8_t qNum, uint16_t idpacket);

#endif /* __APPLICATION_H__ */
