#include "tcpip_transfer.h"
#include "tcpip_loop_queue.h"
//#include "system.h"

uint8_t tcpip_tx_queue[TCPIP_QUEUE_LENGTH][TCPIP_BUFF_LENGTH]={0x00};
LOOP_QUEUE tcpip_tx_lq;

static const char *TAG = "tcpip_transfer ";

static inline uint8_t *tx_irq(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    q->buff = loop_queue_insert(q, queue);
    if (q->buff == NULL)
    {
        ESP_LOGE( TAG, "tcpip_tx_queue full!\r\n" );
    }
    xSemaphoreGive( tcpip_tx_semaphore );
    return q->buff;
}

uint8_t *tcpip_tx_queue_insert(void)
{
    return loop_queue_insert(&tcpip_tx_lq, tcpip_tx_queue);
}

uint8_t *tcpip_tx_queue_read(void)
{
    return loop_queue_read(&tcpip_tx_lq, tcpip_tx_queue);
}

uint8_t *tcpip_tx_queue_get_front(void)
{
    return loop_queue_get_front(&tcpip_tx_lq, tcpip_tx_queue);
}

void tcpip_tx_queue_init(void)
{
    loop_queue_init(&tcpip_tx_lq, tcpip_tx_queue);
}

uint8_t *tcpip_tx_irq(void)
{
    return tx_irq(&tcpip_tx_lq, tcpip_tx_queue);
}
