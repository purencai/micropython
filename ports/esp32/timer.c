/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWAREI.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "esp_task.h"

#include "crc16.h"
#include "timer.h"
#include "stm32_serial.h"

#define TIMER_DIVIDER           80        //  Hardware timer clock divider
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds 80M/80=1000KHz
#define TIMER_INTERVAL0_COUNT   (1000-1)  // sample test interval for the first timer
#define WITHOUT_RELOAD          0         // testing will be done without auto reload
#define WITH_RELOAD             1         // testing will be done with auto reload

void IRAM_ATTR timer_group0_timer0_isr(void *para)
{
    int timer_idx = (int) para;
    TIMERG0.hw_timer[timer_idx].update = 1;
    if ( TIMERG0.int_st_timers.val & BIT(timer_idx) )
    {
        switch (timer_idx)
        {
        case TIMER_0:
            //
            stm32_usart_timer_irq();
            TIMERG0.int_clr_timers.t0 = 1;
        break;
        default:
        break;
        }
    }
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

void group0_timer0_init(int divider,double period)
{
    timer_config_t timer_config;
    timer_config.divider = period;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.alarm_en = TIMER_ALARM_EN;
    timer_config.intr_type = TIMER_INTR_LEVEL;
    timer_config.auto_reload = WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);

    timer_set_counter_value ( TIMER_GROUP_0, TIMER_0, 0x00000000ULL );

    timer_set_alarm_value ( TIMER_GROUP_0, TIMER_0, period );
    timer_enable_intr ( TIMER_GROUP_0, TIMER_0 );

    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_timer0_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, TIMER_0);
}
