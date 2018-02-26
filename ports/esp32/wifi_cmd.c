#include "system.h"
#include "crc16.h"
#include "stm32_serial.h"
#include "tcpip_transfer.h"
#include "message_manager.h"

static const char *TAG = "wifi_cmd ";

#define QUERY_ONLINE_MODULES_NUM_CMD        0X00B0
#define QUERY_BATTERY_POWER_CMD             0x01B0
#define QUERY_DRIVER_POSITION_SPEED_CMD     0X02B0
#define QUERY_COLOR_VALUE_CMD               0x03B0
#define QUERY_INFRARED_VALUE_CMD            0X04B0
#define QUERY_TOUCH_VALUE_CMD               0x05B0
#define QUERY_HSERVO_ANGLE_CMD              0x02B4
#define QUERY_VSERVO_ANGLE_CMD              0x02B5

#define CONTROL_DRIVER_MOTRO_CMD            0x02B1
#define CONTROL_DRIVER_LED_CMD              0xEDB1
#define CONTROL_HSERVO_ANGLE_CMD            0x02B6
#define CONTROL_VSERVO_ANGLE_CMD            0x02B7

void query_online_module_nums(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 4);
        vTaskDelay(pdMS_TO_TICKS(50));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_ONLINE_MODULES_NUM_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[10] ) == crc16( usart_reply_cmd, 10 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<12; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void query_battery_power(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_BATTERY_POWER_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void query_driver_position_speed(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_DRIVER_POSITION_SPEED_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[8] ) == crc16( usart_reply_cmd, 8 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<10; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}
void query_color_value(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_COLOR_VALUE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[6] ) == crc16( usart_reply_cmd, 6 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<8; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void query_infrared_value(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_INFRARED_VALUE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void query_touch_value(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_TOUCH_VALUE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void query_hservo_angle(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_HSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void query_vservo_angle(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == QUERY_VSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void control_driver_motor(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == CONTROL_DRIVER_MOTRO_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void control_driver_led(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 7);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == CONTROL_DRIVER_LED_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void control_hservo_angle(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == CONTROL_HSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

void control_vservo_angle(uint8_t *data)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
	uint8_t *usart_reply_cmd = NULL;
    uint8_t *tcpip_reply_cmd = NULL;
	while( retry_times-- )
	{
		stm32_send_cmd((const char*)data, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        usart_reply_cmd = stm32_usart_queue_read();
        if ( usart_reply_cmd == NULL )continue;
        if ( ( *( uint16_t * ) usart_reply_cmd == CONTROL_VSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &usart_reply_cmd[4] ) == crc16( usart_reply_cmd, 4 ) ) )
        {
            tcpip_reply_cmd = tcpip_tx_queue_get_front();
            for(i=0; i<6; i++)
            {
                tcpip_reply_cmd[i] = usart_reply_cmd[i];
            }
            xSemaphoreGive( tcpip_tx_semaphore );
            tcpip_tx_queue_insert();
            break;
        }
	}
}

typedef struct _cmd_callback_t
{
    uint16_t cmd;
    void (*callback)(uint8_t *data);
    struct _cmd_callback_t *next;
}cmd_callback_t;

cmd_callback_t *cmd_header[17];

void register_wifi_cmd( const uint16_t cmd, void (*cb)(uint8_t*) )
{
    uint8_t index = 0;
    cmd_callback_t *cmd_callback;
    cmd_callback = (cmd_callback_t *)malloc( sizeof( cmd_callback_t ) );
    index =((uint32_t)(cmd * 40503)>>15)%17;
    if (cmd_header[index] == NULL)
    {
        cmd_header[index] = cmd_callback;
        cmd_header[index]->next = NULL;
    }
    else
    {
        cmd_callback->next = cmd_header[index];
        cmd_header[index] = cmd_callback;
    }
    cmd_header[index]->cmd = cmd;	
    cmd_header[index]->callback = cb;
}

cmd_callback_t * find_cmd(const uint16_t cmd)
{
    uint8_t index = 0;
    cmd_callback_t *cmd_callback;
    index =((uint32_t)(cmd * 40503)>>15)%17;
    if (cmd_header[index] == NULL)
    {
        return NULL;
    }
		
    cmd_callback = cmd_header[index];
    while(cmd_callback)
    {
        if (cmd_callback->cmd == cmd)
        {
            return cmd_callback;
        }
        cmd_callback = cmd_callback->next;
    }
    return NULL;
}
void response_cmd( cmd_callback_t *cmd, uint8_t *data )
{
    if (cmd->callback != NULL)
    {
        (cmd->callback)(data);
    }
}

void register_all_cmd(void)
{  
    cmd_callback_t *cmd;
    uint8_t i=0;

    register_wifi_cmd( QUERY_ONLINE_MODULES_NUM_CMD,    query_online_module_nums );
    register_wifi_cmd( QUERY_BATTERY_POWER_CMD,         query_battery_power );
    register_wifi_cmd( QUERY_DRIVER_POSITION_SPEED_CMD, query_driver_position_speed );
    register_wifi_cmd( QUERY_COLOR_VALUE_CMD,           query_color_value );
    register_wifi_cmd( QUERY_INFRARED_VALUE_CMD,        query_infrared_value );
    register_wifi_cmd( QUERY_TOUCH_VALUE_CMD,           query_touch_value );
    register_wifi_cmd( QUERY_HSERVO_ANGLE_CMD,          query_hservo_angle );
    register_wifi_cmd( QUERY_VSERVO_ANGLE_CMD,          query_vservo_angle );

    register_wifi_cmd( CONTROL_DRIVER_MOTRO_CMD,        control_driver_motor );
    register_wifi_cmd( CONTROL_DRIVER_LED_CMD,          control_driver_led );
    register_wifi_cmd( CONTROL_HSERVO_ANGLE_CMD,        control_hservo_angle );
    register_wifi_cmd( CONTROL_VSERVO_ANGLE_CMD,        control_vservo_angle );

    for (i=0;i<17;i++)
    {
        //ESP_LOGI(TAG, "usb_cmd[%d]->", i);
        printf("usb_cmd[%d]->",i);
        cmd = cmd_header[i];
        while(cmd)
        {
            printf("%04X->",cmd->cmd);
            cmd = cmd->next;
        }
        printf("end\r\n");
    }
}

void wifi_cmd_recevie(uint8_t *rx_buff)
{
	cmd_callback_t *wifi_cmd=NULL;
    // 数据解析
	uint16_t header = 0;
	header = *(uint16_t *)(rx_buff);
    //LOG("esp32 uart headr = 0x%04x\r\n" ,  *(uint16_t *)(rx_buff));
    //ESP_LOGE(TAG, "create tcpip_rx_semaphore fail\r\n");
	if(header != 0)
    {
		wifi_cmd = find_cmd(header);
		response_cmd( wifi_cmd, rx_buff);	
	}
}
