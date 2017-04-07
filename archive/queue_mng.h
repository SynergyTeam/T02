//------------------------------------------------------------------------------
#ifndef __APPLICATION_H__
#define __APPLICATION_H__

#include "config.h"
#include "cmsis_os.h"

//����������� ��������� ������ ������
#define PACKET_NUM(x)               (Queue[x].PacketNum)
#define QUEUE_LOCK(x)               (xSemaphoreTakeRecursive(Queue[x].gate, portMAX_DELAY))
#define QUEUE_UNLOCK(x)             (xSemaphoreGiveRecursive(&Queue[x].gate))

//�������� ��������� ������ � �������
typedef struct {
    uint8_t     *data;                              // ��������� �� ������
    unsigned    size :12;                           // ������ ������
    unsigned    tx_attempts :5;                     // ����� ������� ��������
    unsigned    tx_mark :1;                         // ���� ����������� ������
    unsigned    app_conf :1;                        // ���������� ������������� �� ������ ����������
    unsigned    ack_net :1;                         // ���� �������� �������������
    unsigned    ack_timer :12;                      // ������ �������������
} pInfo_t;

//��������� �������� �������
typedef struct qInfo_t {
    SemaphoreHandle_t   gate;                       // ������ �������
    pInfo_t             packet[MAX_PACKET_QUEUE];   // ������ �������� �������
    uint16_t            PacketNum;                  // ����� ������
    uint8_t             size;                       // ������ ������
    uint8_t             tx_indx;
    uint8_t             buf[MAX_OUT_PACKET_SIZE];   // ������ �������
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
