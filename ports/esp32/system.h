#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/list.h"
#include "freertos/event_groups.h"

#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdarg.h"

#include "esp_task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "message_manager.h"

static __inline void assert_failed(uint8_t* file, uint32_t line)
{
    printf("file:%s-line[%d] param error!\r\n",file,line);while(1);
}

#define ASSERT(expr) ((expr) ? (void )0 : assert_failed(__FILE__,__LINE__))

#endif // __SYSTEM_H__
