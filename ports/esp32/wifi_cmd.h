/* tcp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef __WIFI_CMD_H__
#define __WIFI_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

extern void register_all_cmd(void);
extern void wifi_cmd_recevie(uint8_t *rx_buff);

#ifdef __cplusplus
}
#endif

#endif //__WIFI_CMD_H__
