#include "tcpip_recevie.h"
#include "tcpip_loop_queue.h"
//#include "system.h"

uint8_t tcpip_rx_queue[TCPIP_QUEUE_LENGTH][TCPIP_BUFF_LENGTH]={0x00};
LOOP_QUEUE tcpip_rx_lq;

static const char *TAG = "tcpip_recevie ";

static inline uint8_t *rx_irq(LOOP_QUEUE *q, uint8_t queue[][TCPIP_BUFF_LENGTH])
{
    q->buff = loop_queue_insert(q, queue);
    if (q->buff == NULL)
    {
        ESP_LOGE( TAG, "tcpip_rx_queue full!\r\n" );
    }
    xSemaphoreGive( tcpip_rx_semaphore );
    return q->buff;
}

uint8_t *tcpip_rx_queue_insert(void)
{
    return loop_queue_insert(&tcpip_rx_lq, tcpip_rx_queue);
}

uint8_t *tcpip_rx_queue_read(void)
{
    return loop_queue_read(&tcpip_rx_lq, tcpip_rx_queue);
}

uint8_t *tcpip_rx_queue_get_front(void)
{
    return loop_queue_get_front(&tcpip_rx_lq, tcpip_rx_queue);
}

void tcpip_rx_queue_init(void)
{
    loop_queue_init(&tcpip_rx_lq, tcpip_rx_queue);
}

uint8_t *tcpip_rx_irq(void)
{
    return rx_irq(&tcpip_rx_lq, tcpip_rx_queue);
}
