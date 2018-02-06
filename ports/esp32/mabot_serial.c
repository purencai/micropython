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
#include "soc/soc.h"
#include "driver/uart.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "modmabot.h"

#include "stm32_serial.h"
#include "timer.h"
#include "crc16.h"

#define CRC16_VALUE_SIZEOF                  2

#define QUERY_ONLINE_MODULES_NUM_CMD        0X00B0
#define QUERY_BATTERY_POWER_CMD             0x01B0
#define QUERY_DRIVER_POSITION_SPEED_CMD     0X02B0
#define QUERY_COLOR_VALUE_CMD               0x03B0
#define QUERY_INFRARED_VALUE_CMD            0X04B0
#define QUERY_TOUCH_VALUE_CMD               0x05B0
#define QUERY_HSERVO_ANGLE_CMD              0x02B4
#define QUERY_VSERVO_ANGLE_CMD              0x02B5

#define CONTROL_DRIVER_MOTRO_CMD            0x02B1
#define CONTROL_DRIVER_LED_CMD              0xEDB1
#define CONTROL_HSERVO_ANGLE_CMD            0x02B6
#define CONTROL_VSERVO_ANGLE_CMD            0x02B7

typedef struct _mabot_serial_obj_t {
    mp_obj_base_t base;
    uart_port_t uart_num;
    void  (*putstr)(const char * cmd, uint16_t length);
} mabot_serial_obj_t;

STATIC mp_obj_t mabot_serial_make_new(const mp_obj_type_t *type_in, size_t n_args, size_t n_kw, const mp_obj_t *args) 
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    mabot_serial_obj_t *self = m_new_obj(mabot_serial_obj_t);

    self->base.type = &mabot_serial_type;
    self->uart_num = STM32_SERIAL_NUM;
    self->putstr = stm32_send_cmd;

    group0_timer0_init(80,1000-1);

    stm32_serial_init();

    return MP_OBJ_FROM_PTR(self);
}   

STATIC mp_obj_t query_online_modules_num (mp_obj_t self_in)
{
    uint8_t i = 0;
	uint8_t retry_times = 4;
    vstr_t vstr = {NULL};
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[4] = {0xB0,0X00,0X70,0X74};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 4);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_ONLINE_MODULES_NUM_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[10] ) == crc16( retry_cmd, 10 ) ) )
        {
            vstr_init_len(&vstr,8);
            for(i=0; i<8; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(query_online_modules_num_obj, query_online_modules_num);

STATIC mp_obj_t query_battery_power ( mp_obj_t self_in, mp_obj_t index )
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t buff[5] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    buff[0] = (uint8_t)(QUERY_BATTERY_POWER_CMD&0xff);
    buff[1] = (uint8_t)(QUERY_BATTERY_POWER_CMD>>8);
    buff[2] = (uint8_t)mp_obj_get_int(index);
    crc16_parity = crc16( (uint8_t *)buff, 3 );
    buff[3] = (uint8_t)(crc16_parity&0xff);
    buff[4] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)buff, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_BATTERY_POWER_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(query_battery_power_obj, query_battery_power);

STATIC mp_obj_t query_driver_position_speed ( mp_obj_t self_in, mp_obj_t index )
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
    uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;   
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[5] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(QUERY_DRIVER_POSITION_SPEED_CMD&0xff);
    send_buff[1] = (uint8_t)(QUERY_DRIVER_POSITION_SPEED_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    crc16_parity = crc16( (uint8_t *)send_buff, 3 );
    send_buff[3] = (uint8_t)(crc16_parity&0xff);
    send_buff[4] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_DRIVER_POSITION_SPEED_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[8] ) == crc16( retry_cmd, 8 ) ) )
        {
            vstr_init_len(&vstr,6);
            for(i=0; i<6; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(query_driver_position_speed_obj, query_driver_position_speed);

STATIC mp_obj_t query_color_value ( mp_obj_t self_in, mp_obj_t index, mp_obj_t mode)
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};      
	uint8_t retry_times = 4; 
    uint16_t crc16_parity = 0;   
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[6] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(QUERY_COLOR_VALUE_CMD&0xff);
    send_buff[1] = (uint8_t)(QUERY_COLOR_VALUE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    send_buff[3] = (uint8_t)mp_obj_get_int(mode);   
    crc16_parity = crc16( (uint8_t *)send_buff, 4 );
    send_buff[4] = (uint8_t)(crc16_parity&0xff);
    send_buff[5] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_COLOR_VALUE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[6] ) == crc16( retry_cmd, 6 ) ) )
        {
            vstr_init_len(&vstr,4);
            for(i=0; i<4; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(query_color_value_obj, query_color_value);

STATIC mp_obj_t query_infrared_value ( mp_obj_t self_in, mp_obj_t index )
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[5] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(QUERY_INFRARED_VALUE_CMD&0xff);
    send_buff[1] = (uint8_t)(QUERY_INFRARED_VALUE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    crc16_parity = crc16( (uint8_t *)send_buff, 3 );
    send_buff[3] = (uint8_t)(crc16_parity&0xff);
    send_buff[4] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_INFRARED_VALUE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(query_infrared_value_obj, query_infrared_value);

STATIC mp_obj_t query_touch_value ( mp_obj_t self_in, mp_obj_t index )
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[5] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(QUERY_TOUCH_VALUE_CMD&0xff);
    send_buff[1] = (uint8_t)(QUERY_TOUCH_VALUE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    crc16_parity = crc16( (uint8_t *)send_buff, 3 );
    send_buff[3] = (uint8_t)(crc16_parity&0xff);
    send_buff[4] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_TOUCH_VALUE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(query_touch_value_obj, query_touch_value);

STATIC mp_obj_t control_driver_motor ( mp_obj_t self_in, mp_obj_t index, mp_obj_t speed)
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[6] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(CONTROL_DRIVER_MOTRO_CMD&0xff);
    send_buff[1] = (uint8_t)(CONTROL_DRIVER_MOTRO_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    send_buff[3] = (uint8_t)mp_obj_get_int(speed); 
    crc16_parity = crc16( (uint8_t *)send_buff, 4 );
    send_buff[4] = (uint8_t)(crc16_parity&0xff);
    send_buff[5] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == CONTROL_DRIVER_MOTRO_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(control_driver_motor_obj, control_driver_motor);

STATIC mp_obj_t control_driver_led ( size_t n_args, const mp_obj_t *args)
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[7] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    send_buff[0] = (uint8_t)(CONTROL_DRIVER_LED_CMD&0xff);
    send_buff[1] = (uint8_t)(CONTROL_DRIVER_LED_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(args[1]);
    send_buff[3] = (uint8_t)mp_obj_get_int(args[2]);
    send_buff[4] = (uint8_t)mp_obj_get_int(args[3]);
    crc16_parity = crc16( (uint8_t *)send_buff, 5 );
    send_buff[5] = (uint8_t)(crc16_parity&0xff);
    send_buff[6] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 7);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == CONTROL_DRIVER_LED_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(control_driver_led_obj, 4, control_driver_led);

STATIC mp_obj_t query_hservo_angle ( mp_obj_t self_in, mp_obj_t index )
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[5] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR( self_in );
    send_buff[0] = (uint8_t)(QUERY_HSERVO_ANGLE_CMD&0xff);
    send_buff[1] = (uint8_t)(QUERY_HSERVO_ANGLE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    crc16_parity = crc16( (uint8_t *)send_buff, 3 );
    send_buff[3] = (uint8_t)(crc16_parity&0xff);
    send_buff[4] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_HSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(query_hservo_angle_obj, query_hservo_angle);

STATIC mp_obj_t query_vservo_angle ( mp_obj_t self_in, mp_obj_t index )
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[5] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(QUERY_VSERVO_ANGLE_CMD&0xff);
    send_buff[1] = (uint8_t)(QUERY_VSERVO_ANGLE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    crc16_parity = crc16( (uint8_t *)send_buff, 3 );
    send_buff[3] = (uint8_t)(crc16_parity&0xff);
    send_buff[4] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 5);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == QUERY_VSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(query_vservo_angle_obj, query_vservo_angle);

STATIC mp_obj_t control_hservo_angle ( mp_obj_t self_in, mp_obj_t index, mp_obj_t angle)
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[6] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(CONTROL_HSERVO_ANGLE_CMD&0xff);
    send_buff[1] = (uint8_t)(CONTROL_HSERVO_ANGLE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    send_buff[3] = (uint8_t)mp_obj_get_int(angle);    
    crc16_parity = crc16( (uint8_t *)send_buff, 4 );
    send_buff[4] = (uint8_t)(crc16_parity&0xff);
    send_buff[5] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == CONTROL_HSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(control_hservo_angle_obj, control_hservo_angle);

STATIC mp_obj_t control_vservo_angle ( mp_obj_t self_in, mp_obj_t index, mp_obj_t angle)
{
    uint8_t i = 0;
    vstr_t vstr = {NULL};
	uint8_t retry_times = 4;
    uint16_t crc16_parity = 0;
	uint8_t *retry_cmd = NULL;
    uint8_t send_buff[6] = {NULL};
    mabot_serial_obj_t *self = MP_OBJ_TO_PTR(self_in);
    send_buff[0] = (uint8_t)(CONTROL_VSERVO_ANGLE_CMD&0xff);
    send_buff[1] = (uint8_t)(CONTROL_VSERVO_ANGLE_CMD>>8);
    send_buff[2] = (uint8_t)mp_obj_get_int(index);
    send_buff[3] = (uint8_t)mp_obj_get_int(angle);    
    crc16_parity = crc16( (uint8_t *)send_buff, 4 );
    send_buff[4] = (uint8_t)(crc16_parity&0xff);
    send_buff[5] = (uint8_t)(crc16_parity>>8);
	while( retry_times-- )
	{
		self->putstr((const char*)send_buff, 6);
        vTaskDelay(pdMS_TO_TICKS(30));
        retry_cmd = stm32_usart_queue_read();
        if ( retry_cmd == NULL )continue;
        if ( ( *( uint16_t * ) retry_cmd == CONTROL_VSERVO_ANGLE_CMD ) && \
             ( *( uint16_t * ) ( &retry_cmd[4] ) == crc16( retry_cmd, 4 ) ) )
        {
            vstr_init_len(&vstr,2);
            for(i=0; i<2; i++)
            {
                vstr.buf[i] = retry_cmd[i+2];
            }
            return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
        }
	}
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(control_vservo_angle_obj, control_vservo_angle);

STATIC const mp_map_elem_t mabot_serial_locals_dict_table[] = 
{
    { MP_OBJ_NEW_QSTR(MP_QSTR_queryOnlineModules),      (mp_obj_t)&query_online_modules_num_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_queryBattery),            (mp_obj_t)&query_battery_power_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_queryDriver),             (mp_obj_t)&query_driver_position_speed_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_queryColorSensor),        (mp_obj_t)&query_color_value_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_queryIRSensor),           (mp_obj_t)&query_infrared_value_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_queryTouchSensor),        (mp_obj_t)&query_touch_value_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_driverTest),              (mp_obj_t)&control_driver_motor_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_setSimpleDriverLight),    (mp_obj_t)&control_driver_led_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_getHorzionJointDegree),   (mp_obj_t)&query_hservo_angle_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_setHorzionJointDegree),   (mp_obj_t)&control_hservo_angle_obj },

    { MP_OBJ_NEW_QSTR(MP_QSTR_getRockJointDegree),      (mp_obj_t)&query_vservo_angle_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_setRockJointDegree),      (mp_obj_t)&control_vservo_angle_obj },
};
STATIC MP_DEFINE_CONST_DICT(mabot_serial_locals_dict, mabot_serial_locals_dict_table);

const mp_obj_type_t mabot_serial_type = 
{
    { &mp_type_type },
    .name = MP_QSTR_SERIAL,
    .make_new = mabot_serial_make_new,
    .locals_dict = (mp_obj_t)&mabot_serial_locals_dict,
};