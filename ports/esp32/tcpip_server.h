/* tcp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef __TCPIP_SERVER_H__
#define __TCPIP_SERVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/*AP info and tcp_server info*/
#define AP_SSID            "mabot-test"
#define AP_PWD             ""
#define SERVER_IP          "192.168.2.2"
#define SERVER_PORT        8899
#define SERVER_MAX_CONN    1

extern void tcpip_send_data(uint8_t *buff, uint16_t length);
extern void tcpip_server_init(void);

#ifdef __cplusplus
}
#endif

#endif //__TCPIP_SERVER_H__
