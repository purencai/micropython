#include "tcpip_loop_queue.h"
#include "tcpip_recevie.h"
#include "tcpip_transfer.h"
#include "tcpip_server.h"
#include "wifi_cmd.h"
#include "file_recevie.h"

static const char *TAG = "tcpip_task ";

void tcpip_recevie_process( uint8_t *received_data )
{
	uint16_t header = 0;
	header = *(uint16_t *)(received_data);
    switch(header)
    {
    case FILE_TYPE_FRAME : 
        //file_type(received_data);  
    break;
    case FILE_START_FRAME: 
        //file_start(received_data); 
    break;
    case FILE_DATA_FRAME:  
        //file_data(received_data);  
    break;
    case FILE_END_FRAME:  
        //file_end(received_data);   
    break;          
    default: 
        wifi_cmd_recevie(received_data); 
    break;
    }
}

//receive data
void tcpip_recevie_task(void *pvParameters)
{
    uint8_t *tcpip_rx_cmd = NULL;
    for(;;)
    {
        while( xSemaphoreTake( tcpip_rx_semaphore, portMAX_DELAY ) != pdPASS )
        {
            ESP_LOGE(TAG, "take tcpip_rx_semaphore error\r\n");
        }
        tcpip_rx_cmd = tcpip_rx_queue_read();
        if ( tcpip_rx_cmd )
        {
            tcpip_recevie_process( tcpip_rx_cmd );
            memset( tcpip_rx_cmd, 0x00, TCPIP_BUFF_LENGTH );
        }
        taskYIELD();
    }
}

//transfer data
void tcpip_transfer_task(void *pvParameters)
{
    uint8_t *tcpip_tx_cmd = NULL;
    for(;;)
    {
        while( xSemaphoreTake( tcpip_tx_semaphore, portMAX_DELAY ) != pdPASS )
        {
            ESP_LOGE(TAG, "take tcpip_tx_semaphore error\r\n");
        }
        tcpip_tx_cmd = tcpip_tx_queue_read();
        if ( tcpip_tx_cmd )
        {
            tcpip_send_data( tcpip_tx_cmd, TCPIP_BUFF_LENGTH);
            memset( tcpip_tx_cmd, 0x00, TCPIP_BUFF_LENGTH );
        }
        taskYIELD();
    }
}

void create_tcpip_rx_tx_task(void)
{
    xTaskCreatePinnedToCore(tcpip_recevie_task, "tcpip_recevie_task", 4096, NULL, (ESP_TASK_PRIO_MIN + 2), NULL, 0 );
    xTaskCreatePinnedToCore(tcpip_transfer_task, "tcpip_transfer_task", 4096, NULL, (ESP_TASK_PRIO_MIN + 2), NULL, 0 );
}

