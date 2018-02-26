#ifndef __TCPIP_RECEVIE_H__
#define __TCPIP_RECEVIE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"

extern uint8_t *tcpip_rx_queue_insert(void);
extern uint8_t *tcpip_rx_queue_read(void);
extern uint8_t *tcpip_rx_queue_get_front(void);
extern void tcpip_rx_queue_init(void);
extern uint8_t *tcpip_rx_irq(void);

#ifdef __cplusplus
}
#endif

#endif //__TCPIP_RECEVIE_H__
