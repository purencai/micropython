#include "message_manager.h"
#include "tcpip_loop_queue.h"

static const char *TAG = "message_manager ";

SemaphoreHandle_t tcpip_rx_semaphore = NULL;
static inline void create_tcpip_rx_semaphore(void)
{
    tcpip_rx_semaphore = xSemaphoreCreateCounting(TCPIP_QUEUE_LENGTH, 0);
    if( tcpip_rx_semaphore == NULL )
    {
        ESP_LOGE(TAG, "create tcpip_rx_semaphore fail\r\n");
    }
}

SemaphoreHandle_t tcpip_tx_semaphore = NULL;
static inline void create_tcpip_tx_semaphore(void)
{
    tcpip_tx_semaphore = xSemaphoreCreateCounting(TCPIP_QUEUE_LENGTH, 0);
    if( tcpip_tx_semaphore == NULL )
    {
        ESP_LOGE(TAG, "create tcpip_tx_semaphore fail\r\n");
    }
}

void create_queue_semaphore(void)
{
    create_tcpip_rx_semaphore();
    create_tcpip_tx_semaphore();
}
