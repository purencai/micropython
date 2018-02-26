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
#include "driver/uart.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "esp_task.h"

#include "crc16.h"
#include "stm32_serial.h"

#define STM32_SERIAL_TXD  (GPIO_NUM_17)
#define STM32_SERIAL_RXD  (GPIO_NUM_16)
#define STM32_SERIAL_RTS  (UART_PIN_NO_CHANGE)
#define STM32_SERIAL_CTS  (UART_PIN_NO_CHANGE)

typedef struct loop_queue
{
    uint8_t front;
    uint8_t rear;
    uint16_t cnt;
    
    uint16_t timer_cnt;
    uint16_t buff_cnt;
    uint8_t *buff;
}LOOP_QUEUE;

QueueHandle_t stm32_serial_event_queue = NULL;

uint8_t stm32_usart_queue[QUEUE_LENGTH][BUFF_LENGTH]={0x00};

LOOP_QUEUE stm32_usart_lq;

SemaphoreHandle_t USART_Semaphore = NULL;

void usart_rx_process(uint8_t *rx_buff);

static const char *TAG = "stm32_serial : ";

static inline void create_usart_semaphore(void)
{
    USART_Semaphore = xSemaphoreCreateCounting(QUEUE_LENGTH, 0);
    if(USART_Semaphore == NULL)
    {
        ESP_LOGI( TAG, "create USART_Semaphore fail\r\n" );
    }
}

static inline uint8_t *loop_queue_insert(LOOP_QUEUE *q, uint8_t queue[][BUFF_LENGTH])
{
    uint8_t *head = NULL;
    if ((q->front + 1) % QUEUE_LENGTH == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % QUEUE_LENGTH;
    head = &queue[q->front][0];
    q->cnt++;
    return head;
}

static inline uint8_t *loop_queue_read(LOOP_QUEUE *q, uint8_t queue[][BUFF_LENGTH])
{
    uint8_t *head = NULL;
    if (q->rear == q->front)
    {
        return NULL;
    }
    head = &queue[q->rear][0];
    q->rear = (q->rear + 1) % QUEUE_LENGTH;
    q->cnt--;
    return head;
}

static inline uint8_t *loop_queue_get_front(LOOP_QUEUE *q, uint8_t queue[][BUFF_LENGTH])
{
    uint8_t *head = NULL;
    head = &queue[q->front][0];
    return head;
}

static inline uint8_t *is_loop_queue_empty(LOOP_QUEUE *q, uint8_t queue[][BUFF_LENGTH])
{
    if (q->rear == q->front)
    {
        return NULL;
    }
    return &queue[q->rear][0];
}

static inline void loop_queue_init(LOOP_QUEUE *q, uint8_t queue[][BUFF_LENGTH])
{
    q->front = 0;
    q->rear = 0;
    q->cnt = 0;
    q->timer_cnt = 0;
    q->buff_cnt = 0;
    q->buff = NULL;
    q->buff = loop_queue_get_front(q, queue);
}

static inline void usart_irq(LOOP_QUEUE *q, const uint8_t *data,uint16_t length)
{
    if (q->buff == NULL)
    {
        q->timer_cnt = QUEUE_TIME_OUT + 1;
        return;
    }
    memcpy(&q->buff[q->buff_cnt],(char *)data,length);
    q->buff_cnt = q->buff_cnt + length;
    if (q->buff_cnt == BUFF_LENGTH)
    {
        q->buff_cnt = BUFF_LENGTH - 1;
    }
    q->timer_cnt = 1;
}

static inline void timer_irq(LOOP_QUEUE *q, uint8_t queue[][BUFF_LENGTH])
{
    // BaseType_t xHigherPriorityTaskWoken;
    if (q->timer_cnt > 0)
    {
        q->timer_cnt++;
    }
    if (q->timer_cnt > QUEUE_TIME_OUT)
    {
        q->buff = loop_queue_insert(q, queue);

        if (q->buff == NULL)
        {
            ESP_LOGE( TAG, "stm32_usart_queue full\r\n" );
        }
        q->buff_cnt = 0;
        q->timer_cnt = 0;

        // give a counting semaphore
        // if ( xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED )return;
        // xSemaphoreGiveFromISR(USART_Semaphore, &xHigherPriorityTaskWoken);
        // portYIELD_FROM_ISR();
    }
}

static inline uint8_t *stm32_usart_queue_insert()
{
    return loop_queue_insert(&stm32_usart_lq, stm32_usart_queue);
}

uint8_t *stm32_usart_queue_read()
{
    return loop_queue_read(&stm32_usart_lq, stm32_usart_queue);
}

static inline void stm32_usart_queue_init()
{
    loop_queue_init(&stm32_usart_lq, stm32_usart_queue);
}

static inline void stm32_usart_rev_irq(const uint8_t *data,uint16_t length)
{
    usart_irq(&stm32_usart_lq, data, length);
}

void stm32_usart_timer_irq(void)
{
    timer_irq(&stm32_usart_lq, stm32_usart_queue);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t rev_buff[20] = {NULL};
    memset( rev_buff, 0x00, 20 );
    while (1) 
    {
        if ( xQueueReceive( stm32_serial_event_queue, (void * )&event, pdMS_TO_TICKS(portMAX_DELAY) ) )
        {   
            switch (event.type)
            {
            case UART_DATA:
                uart_read_bytes( STM32_SERIAL_NUM, rev_buff, event.size, pdMS_TO_TICKS(portMAX_DELAY) );
                stm32_usart_rev_irq( rev_buff, event.size );
                memset( rev_buff, 0x00, 20 );
            break;
            case UART_FIFO_OVF:
                uart_flush(STM32_SERIAL_NUM);
            break;
            case UART_BUFFER_FULL:
                uart_flush(STM32_SERIAL_NUM);
            break;
            case UART_BREAK:
            break;
            case UART_PARITY_ERR:
            break;
            case UART_FRAME_ERR:
            break;
            case UART_PATTERN_DET:
            break;
            default:
            break;
            }
        }
    }
}

void usart_rx_task(void *pvParameter)
{
    uint8_t *usart_cmd = NULL;
    while(1)
    {
        while( xSemaphoreTake( USART_Semaphore, pdMS_TO_TICKS(portMAX_DELAY) ) != pdPASS )
        {
            ESP_LOGE( TAG, "USART_Semaphore error\r\n" );
        }
        usart_cmd = stm32_usart_queue_read();
        if ( usart_cmd )
        {
            usart_rx_process(usart_cmd);
            memset(usart_cmd,0x00,BUFF_LENGTH);
        }
        taskYIELD();
    }
}

void stm32_serial_init(void)
{
    uart_config_t uart_config = 
    {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(STM32_SERIAL_NUM, &uart_config);
    uart_set_pin(STM32_SERIAL_NUM, STM32_SERIAL_TXD, STM32_SERIAL_RXD, STM32_SERIAL_RTS, STM32_SERIAL_CTS);
    uart_driver_install(STM32_SERIAL_NUM, 256, 0, 10, &stm32_serial_event_queue, 0);

    stm32_usart_queue_init();

    create_usart_semaphore();

    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 1024, NULL, (ESP_TASK_PRIO_MAX - 2), NULL, 0);
}
void uart_puts(uart_port_t uart_num, const char *buff, uint16_t length)
{
    uart_write_bytes(uart_num, buff, length);
}

void stm32_send_cmd(const char * cmd, uint16_t length)
{
    uart_write_bytes(STM32_SERIAL_NUM, cmd, length);
}
