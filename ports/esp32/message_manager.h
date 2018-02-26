#ifndef __MESSAGE_MANAGER_H__
#define __MESSAGE_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"

extern SemaphoreHandle_t tcpip_rx_semaphore;
extern SemaphoreHandle_t tcpip_tx_semaphore;

void create_queue_semaphore(void);

#ifdef __cplusplus
}
#endif

#endif //__MESSAGE_MANAGER_H__
