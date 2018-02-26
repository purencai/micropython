/* tcp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"

#include "tcpip_loop_queue.h"
#include "tcpip_server.h"
#include "tcpip_recevie.h"
#include "tcpip_transfer.h"
#include "message_manager.h"
#include "tcpip_task.h"
#include "wifi_cmd.h"

static int server_socket = 0;
static struct sockaddr_in server_addr;
static struct sockaddr_in client_addr;
static unsigned int socklen = sizeof(client_addr);
static int connect_socket = 0;

static const char *TAG = "tcpip_server ";

TaskHandle_t tcpip_receive_irp_task_handle;
TaskHandle_t tcpip_accept_task_handle;

void close_socket(void)
{
    close(connect_socket);
    close(server_socket);
}

int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) 
    {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}

int show_socket_error_reason(const char *str, int socket)
{
    int err = get_socket_error_code(socket);
    if (err != 0) 
    {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }
    return err;
}

void socket_error_handler(int err)
{
    switch(err)
    {
    case 104:
        close(connect_socket);
        connect_socket = 0;
        vTaskResume( tcpip_accept_task_handle );
        vTaskDelete( tcpip_receive_irp_task_handle );
    break;
    default:
    break;
    }
}

static esp_err_t wifi_event_handler( void *ctx, system_event_t *event )
{
    switch ( event->event_id ) 
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
    break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join,AID=%d\n",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
    break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave,AID=%d\n",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        //close_socket();
        vTaskResume( tcpip_accept_task_handle );
        vTaskDelete( tcpip_receive_irp_task_handle );
    break;
    default:
    break;
    }
    return ESP_OK;
}

//wifi_init_softap
void wifi_init_softap(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init( wifi_event_handler, NULL ) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = 0,
            .max_connection = SERVER_MAX_CONN,
            .password = AP_PWD,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(AP_PWD) == 0) 
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_AP ) );
    ESP_ERROR_CHECK( esp_wifi_set_config( ESP_IF_WIFI_AP, &wifi_config ) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID: %s password: %s \n", AP_SSID, AP_PWD);
}

void update_tcpip_config(void)
{
    tcpip_adapter_ip_info_t ip_info;
    // tcpip_adapter_dns_info_t dns_info;
    ESP_ERROR_CHECK( tcpip_adapter_get_ip_info( TCPIP_ADAPTER_IF_AP, &ip_info ) );
    // ESP_ERROR_CHECK( tcpip_adapter_get_dns_info( TCPIP_ADAPTER_IF_AP, TCPIP_ADAPTER_DNS_MAIN, &dns_info ) );
    IP4_ADDR( &ip_info.ip, 192, 168, 2, 2 );
    IP4_ADDR( &ip_info.netmask, 255, 255, 255, 0 );
    IP4_ADDR( &ip_info.gw, 192, 168, 2, 1 );
    //IP4_ADDR( &dns_info.ip, 8, 8, 8, 8 );
    ESP_ERROR_CHECK( tcpip_adapter_dhcps_stop( TCPIP_ADAPTER_IF_AP ) );
    //if (e != ESP_OK && e != ESP_ERR_TCPIP_ADAPTER_DHCP_ALREADY_STOPPED) _esp_exceptions(e);
    ESP_ERROR_CHECK( tcpip_adapter_set_ip_info( TCPIP_ADAPTER_IF_AP, &ip_info ) );
    // ESP_ERROR_CHECK( tcpip_adapter_set_dns_info( TCPIP_ADAPTER_IF_AP, TCPIP_ADAPTER_DNS_MAIN, &dns_info) );
    ESP_ERROR_CHECK( tcpip_adapter_dhcps_start( TCPIP_ADAPTER_IF_AP ) );
}

//send data
void tcpip_send_data(uint8_t *buff, uint16_t length)
{
    int len = 0;
    len = send(connect_socket, buff, length, 0);
    if (len <= 0) 
    {
        int err = get_socket_error_code(connect_socket);
        if (err != ENOMEM) 
        {
            show_socket_error_reason("send_data", connect_socket);
        }
    }
}

//receive data
void tcpip_receive_irp_task(void *pvParameters)
{
    int len = 0;
    uint8_t i = 0;
    uint8_t *buff = tcpip_rx_queue_get_front();
    for(;;)
    {
        len = recv(connect_socket, buff, TCPIP_BUFF_LENGTH, 0);
        if(len > 0)
        {
            buff = tcpip_rx_irq();
        }
        else
        {
            socket_error_handler( show_socket_error_reason( "recv_data", connect_socket ) );
        }
    }
}
//receive data
void tcpip_server_task(void *pvParameters)
{
    ESP_LOGI( TAG, "server socket....port = %d\n", SERVER_PORT );
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) 
    {
        show_socket_error_reason( "create_server", server_socket );
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons( SERVER_PORT );
    server_addr.sin_addr.s_addr = htonl( INADDR_ANY );
    if ( bind( server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr) ) < 0 ) 
    {
        show_socket_error_reason( "bind_server", server_socket );
        close( server_socket );
    }
    if (listen(server_socket, 5) < 0) 
    {
        show_socket_error_reason("listen_server", server_socket);
        close( server_socket );
    }

    ESP_LOGI(TAG, "wait tcp connection ...");

    for(;;)
    {
        connect_socket = accept( server_socket, (struct sockaddr *)&client_addr, &socklen );
        if (connect_socket < 0) 
        {
            show_socket_error_reason( "accept_server", connect_socket );
        }
        else
        {
            /*connection established，now can send/recv*/
            ESP_LOGI( TAG, "tcp connection established!" );
            xTaskCreatePinnedToCore(tcpip_receive_irp_task, "tcpip_receive_irp_task", 4096, NULL, (ESP_TASK_PRIO_MAX - 10), &tcpip_receive_irp_task_handle, 0 );
            vTaskSuspend( tcpip_accept_task_handle );
        }
    }
}

void tcpip_server_init(void)
{
    wifi_init_softap();
    update_tcpip_config();
    tcpip_rx_queue_init();
    tcpip_tx_queue_init();   
    register_all_cmd(); 
    //xTaskCreatePinnedToCore(tcpip_server_task, "tcpip_server_task", 4096, NULL, (ESP_TASK_PRIO_MAX - 10), &tcpip_accept_task_handle, 0 );
    //create_tcpip_rx_tx_task();
}
