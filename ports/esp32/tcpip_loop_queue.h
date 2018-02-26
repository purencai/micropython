/* tcp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef __TCPIP_LOOP_QUEUE_H__
#define __TCPIP_LOOP_QUEUE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>

#define TCPIP_QUEUE_LENGTH    3
#define TCPIP_BUFF_LENGTH     1024
#define TCPIP_QUEUE_TIME_OUT  2

typedef struct loop_queue
{
    uint8_t front;
    uint8_t rear;
    uint16_t cnt;
    
    uint16_t timer_cnt;
    uint16_t buff_cnt;
    uint8_t *buff;
}LOOP_QUEUE;

static inline uint8_t *loop_queue_insert(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    uint8_t *head = NULL;
    if ((q->front + 1) % TCPIP_QUEUE_LENGTH == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % TCPIP_QUEUE_LENGTH;
    head = &queue[q->front][0];
    q->cnt++;
    return head;
}

static inline uint8_t *loop_queue_read(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    uint8_t *head = NULL;
    if (q->rear == q->front)
    {
        return NULL;
    }
    head = &queue[q->rear][0];
    q->rear = (q->rear + 1) % TCPIP_QUEUE_LENGTH;
    q->cnt--;
    return head;
}

static inline uint8_t *loop_queue_get_front(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    return &queue[q->front][0];
}

static inline uint8_t *is_loop_queue_empty(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    if (q->rear == q->front)
    {
        return NULL;
    }
    return &queue[q->rear][0];
}

static inline void loop_queue_init(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    q->front = 0;
    q->rear = 0;
    q->cnt = 0;
    q->timer_cnt = 0;
    q->buff_cnt = 0;
    q->buff = loop_queue_get_front(q, queue);
}

#ifdef __cplusplus
}
#endif

#endif //__TCPIP_LOOP_QUEUE_H__
