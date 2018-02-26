/* tcp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef __TCPIP_TRANSFER_H__
#define __TCPIP_TRANSFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"

extern uint8_t *tcpip_tx_queue_insert(void);
extern uint8_t *tcpip_tx_queue_read(void);
extern uint8_t *tcpip_tx_queue_get_front(void);
extern void tcpip_tx_queue_init(void);
extern uint8_t *tcpip_tx_irq(void);

#ifdef __cplusplus
}
#endif

#endif //__TCPIP_TRANSFER_H__
