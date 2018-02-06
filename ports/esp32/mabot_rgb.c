/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Paul Sokolovsky
 * Copyright (c) 2017 Eric Poulsen
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
 * THE SOFTWARE.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "soc/soc.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "modmabot.h"

#define R_LED_GPIO            26
#define G_LED_GPIO            25
#define B_LED_GPIO            27

#define RGB_LED_CH_NUM        3

enum rgb_status{RGB_OFF=0,RGB_ON,RGB_BREATHE,RGB_FLASH };

typedef struct _rgb_led_parameter_t
{
    uint8_t r_led_gpio;
    uint8_t g_led_gpio;
    uint8_t b_led_gpio;
    enum rgb_status status;
    uint32_t color;
    uint16_t flash_interval;
}rgb_led_parameter_t;

typedef struct _mabot_rbg_led_obj_t {
    mp_obj_base_t base;
    rgb_led_parameter_t rgb_led;
} mabot_rgb_led_obj_t;

static QueueHandle_t rgb_led_msg_queue = NULL;

STATIC void set_r_value(uint32_t light)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0, 32768-light);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_0);
}
STATIC void set_g_value(uint32_t light)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_1, 32768-light);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_1);
}
STATIC void set_b_value(uint32_t light)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_2, 32768-light);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE,LEDC_CHANNEL_2);
}
//light [0~128]
STATIC void set_rgb_color(uint32_t rgb24,uint8_t light)
{
    uint16_t r_value = 0;
    uint16_t g_value = 0;
    uint16_t b_value =0;
    if(light > 0x7f)light=0x7f;
    r_value = ((rgb24>>16)&0x000000ff)*light;
    g_value = ((rgb24>>8)&0x000000ff)*light;
    b_value = ((rgb24>>0)&0x000000ff)*light;
    set_b_value(b_value);
    set_g_value(g_value);
    set_r_value(r_value);
}

STATIC void set_rgb_off(void)
{
    set_rgb_color(0x00000000,0x00);
}

STATIC void set_rgb_breathe(uint32_t rgb24)
{
    static uint32_t rgb24_current = 0;
    static uint8_t light = 0;
    static uint8_t breathe = 0;
    if ( rgb24_current != rgb24)
    {
        rgb24_current = rgb24;
        breathe = 0;
        light = 0;
    }
    if ( breathe == 0 )
    {
        light++;
        if ( light == 128 )
        {
            breathe = 1;
        }  
    }
    else
    {
        light--;
        if ( light == 0 )
        {
            breathe = 0;
        }  
    }
    set_rgb_color(rgb24_current,light);   
}

STATIC void set_rgb_flash(uint32_t rgb24,uint16_t flash_interval,uint32_t sys_interval)
{
    STATIC uint8_t cnt = 0;
    STATIC uint16_t time = 0;
    if ( cnt%2 == 0 )
    {
        set_rgb_color(rgb24,0xff);
    }
    else
    {
        set_rgb_color(0,0);
    }
    time ++;
    if ( time == flash_interval/sys_interval )
    {
        cnt ++;
        time = 0;
    }
}

STATIC void rgb_task(void *pvParameter)
{
    rgb_led_parameter_t rgb_led_para=
    {
        .r_led_gpio = R_LED_GPIO,
        .g_led_gpio = G_LED_GPIO,
        .b_led_gpio = B_LED_GPIO,
        .status = RGB_OFF,
        .color = 0x00000000,
        .flash_interval = 10
    };
    for(;;)
    {
        xQueueReceive(rgb_led_msg_queue, (void * )&rgb_led_para, (portTickType)0);
        switch(rgb_led_para.status)
        {
        case RGB_OFF:set_rgb_off();break;
        case RGB_ON:set_rgb_color(rgb_led_para.color,0xff);break;
        case RGB_BREATHE:set_rgb_breathe(rgb_led_para.color);break;
        case RGB_FLASH:set_rgb_flash(rgb_led_para.color,rgb_led_para.flash_interval,10);break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

STATIC void rgb_main(void)
{
    xTaskCreatePinnedToCore(rgb_task, "rgb_task", 512, NULL, (ESP_TASK_PRIO_MIN + 1), NULL, 0);
}

STATIC mp_obj_t mabot_rgb_led_make_new(const mp_obj_type_t *type_in, size_t n_args, size_t n_kw, const mp_obj_t *args) 
{
    int ch;

    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    ledc_timer_config_t ledc_timer = 
    {
        .bit_num = LEDC_TIMER_15_BIT,
        .freq_hz = 2000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_timer.bit_num = LEDC_TIMER_15_BIT,           
    ledc_timer.freq_hz = 2000,                        
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;     
    ledc_timer.timer_num = LEDC_TIMER_1;              
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[RGB_LED_CH_NUM] = 
    {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 32768-1,
            .gpio_num   = R_LED_GPIO,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 32768-1,
            .gpio_num   = G_LED_GPIO,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 32768-1,
            .gpio_num   = B_LED_GPIO,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (ch = 0; ch < RGB_LED_CH_NUM; ch++) 
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    set_rgb_off();
    
    mabot_rgb_led_obj_t *self = m_new_obj(mabot_rgb_led_obj_t);
    self->base.type = &mabot_rgb_type;
    self->rgb_led.r_led_gpio = R_LED_GPIO;
    self->rgb_led.g_led_gpio = G_LED_GPIO;
    self->rgb_led.b_led_gpio = B_LED_GPIO;
    self->rgb_led.status = RGB_OFF;

    rgb_led_msg_queue = xQueueCreate(5, sizeof(rgb_led_parameter_t));

    rgb_main();

    return MP_OBJ_FROM_PTR(self);
}   

STATIC mp_obj_t mabot_rgb_led_value (mp_obj_t self_in, mp_obj_t value_in )
{
    mabot_rgb_led_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t color = mp_obj_get_int(value_in);
    self->rgb_led.status = RGB_ON;
    self->rgb_led.color = color;
    xQueueSend(rgb_led_msg_queue, &self->rgb_led, 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mabot_rgb_led_value_obj, mabot_rgb_led_value);

STATIC mp_obj_t mabot_rgb_led_off(mp_obj_t self_in)
{
    mabot_rgb_led_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->rgb_led.status=RGB_OFF;
    xQueueSend(rgb_led_msg_queue, &self->rgb_led, 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_rgb_led_off_obj, mabot_rgb_led_off);

STATIC mp_obj_t mabot_rgb_led_breathe (mp_obj_t self_in, mp_obj_t value_in )
{
    mabot_rgb_led_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t color = mp_obj_get_int(value_in);
    self->rgb_led.status=RGB_BREATHE;
    self->rgb_led.color=color;
    xQueueSend(rgb_led_msg_queue, &self->rgb_led, 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mabot_rgb_led_breathe_obj, mabot_rgb_led_breathe);

STATIC mp_obj_t mabot_rgb_led_flash (mp_obj_t self_in, mp_obj_t value_in ,mp_obj_t flash_interval_in)
{
    mabot_rgb_led_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t color = mp_obj_get_int(value_in);
    uint16_t flash_interval = mp_obj_get_int(flash_interval_in);
    self->rgb_led.status=RGB_FLASH;
    self->rgb_led.color=color;
    self->rgb_led.flash_interval=flash_interval;
    xQueueSend(rgb_led_msg_queue, &self->rgb_led, 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mabot_rgb_led_flash_obj, mabot_rgb_led_flash);

STATIC const mp_map_elem_t mabot_rgb_led_locals_dict_table[] = 
{
    { MP_OBJ_NEW_QSTR(MP_QSTR_value), (mp_obj_t)&mabot_rgb_led_value_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_off), (mp_obj_t)&mabot_rgb_led_off_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_breathe), (mp_obj_t)&mabot_rgb_led_breathe_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_flash), (mp_obj_t)&mabot_rgb_led_flash_obj },
};
STATIC MP_DEFINE_CONST_DICT(mabot_rgb_led_locals_dict, mabot_rgb_led_locals_dict_table);

const mp_obj_type_t mabot_rgb_type = 
{
    { &mp_type_type },
    .name = MP_QSTR_RGB,
    .make_new = mabot_rgb_led_make_new,
    .locals_dict = (mp_obj_t)&mabot_rgb_led_locals_dict,
};