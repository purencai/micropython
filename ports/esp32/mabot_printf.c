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
#include <stdio.h>
#include <stdarg.h>	 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc.h"
#include "driver/uart.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "modmabot.h"

#define DEBUG_SERIAL_NUM  (UART_NUM_1)
#define DEBUG_SERIAL_TXD  (GPIO_NUM_32)
#define DEBUG_SERIAL_RXD  (GPIO_NUM_34)
#define DEBUG_SERIAL_RTS  (UART_PIN_NO_CHANGE)
#define DEBUG_SERIAL_CTS  (UART_PIN_NO_CHANGE)

QueueHandle_t debug_serial_queue = NULL;

typedef struct _mabot_printf_obj_t 
{
    mp_obj_base_t base;
    uart_port_t uart_num;
    void  (*putstr)(uart_port_t uart_num,const char *fmt,...);
} mabot_printf_obj_t;

static void debug_serial_init(void)
{
    uart_config_t uart_config = 
    {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(DEBUG_SERIAL_NUM, &uart_config);
    uart_set_pin(DEBUG_SERIAL_NUM, DEBUG_SERIAL_TXD, DEBUG_SERIAL_RXD, DEBUG_SERIAL_RTS, DEBUG_SERIAL_CTS);
    uart_driver_install(DEBUG_SERIAL_NUM, 256, 0, 0, NULL,0);
}

char serial_tx_buf[256];
void debug_serial_putstr(uart_port_t uart_num,const char *fmt,...)
{
	uint16_t length = 0;
	va_list ap;
	va_start(ap,fmt);
	vsprintf(serial_tx_buf,fmt,ap);
	va_end(ap);
	length = strlen((const char*)serial_tx_buf);
    uart_tx_chars(uart_num,serial_tx_buf,length);
}

STATIC mp_obj_t mabot_printf_make_new(const mp_obj_type_t *type_in, size_t n_args, size_t n_kw, const mp_obj_t *args) 
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mabot_printf_obj_t *self = m_new_obj(mabot_printf_obj_t);
    self->base.type = &mabot_printf_type;
    self->uart_num = DEBUG_SERIAL_NUM;
    self->putstr = debug_serial_putstr;
    debug_serial_init();
    return MP_OBJ_FROM_PTR(self);
}   

STATIC mp_obj_t mabot_printf_str (size_t n_args, const mp_obj_t *args)
{
    mabot_printf_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    switch(n_args - 2)
    {
    case 0:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]));
    break;
    case 1:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]));
    break;
    case 2:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3]));
    break;
    case 3:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3])\
        ,mp_obj_str_get_str(args[4]));
    break;
    case 4:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3])\
        ,mp_obj_str_get_str(args[4]),mp_obj_str_get_str(args[5]));
    break;
    case 5:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3])\
        ,mp_obj_str_get_str(args[4]),mp_obj_str_get_str(args[5]),mp_obj_str_get_str(args[6]));
    break;
    case 6:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3])\
        ,mp_obj_str_get_str(args[4]),mp_obj_str_get_str(args[5]),mp_obj_str_get_str(args[6]),mp_obj_str_get_str(args[7]));
    break;
    case 7:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3])\
        ,mp_obj_str_get_str(args[4]),mp_obj_str_get_str(args[5]),mp_obj_str_get_str(args[6]),mp_obj_str_get_str(args[7])\
        ,mp_obj_str_get_str(args[8]));
    break;
    case 8:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_str_get_str(args[2]),mp_obj_str_get_str(args[3])\
        ,mp_obj_str_get_str(args[4]),mp_obj_str_get_str(args[5]),mp_obj_str_get_str(args[6]),mp_obj_str_get_str(args[7])\
        ,mp_obj_str_get_str(args[8]),mp_obj_str_get_str(args[9]));
    break;
    default:break;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(mabot_printf_str_obj, 2, mabot_printf_str);

STATIC mp_obj_t mabot_printf_int (size_t n_args, const mp_obj_t *args)
{
    mabot_printf_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    switch(n_args - 2)
    {
    case 1:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]));
    break;
    case 2:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3]));
    break;
    case 3:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3])\
        ,mp_obj_get_int(args[4]));
    break;
    case 4:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3])\
        ,mp_obj_get_int(args[4]),mp_obj_get_int(args[5]));
    break;
    case 5:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3])\
        ,mp_obj_get_int(args[4]),mp_obj_get_int(args[5]),mp_obj_get_int(args[6]));
    break;
    case 6:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3])\
        ,mp_obj_get_int(args[4]),mp_obj_get_int(args[5]),mp_obj_get_int(args[6]),mp_obj_get_int(args[7]));
    break;
    case 7:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3])\
        ,mp_obj_get_int(args[4]),mp_obj_get_int(args[5]),mp_obj_get_int(args[6]),mp_obj_get_int(args[7])\
        ,mp_obj_get_int(args[8]));
    break;
    case 8:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_int(args[2]),mp_obj_get_int(args[3])\
        ,mp_obj_get_int(args[4]),mp_obj_get_int(args[5]),mp_obj_get_int(args[6]),mp_obj_get_int(args[7])\
        ,mp_obj_get_int(args[8]),mp_obj_get_int(args[9]));
    break;
    default:break;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(mabot_printf_int_obj, 2, mabot_printf_int);

STATIC mp_obj_t mabot_printf_float (size_t n_args, const mp_obj_t *args)
{
    mabot_printf_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    switch(n_args - 2)
    {
    case 1:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]));
    break;
    case 2:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3]));
    break;
    case 3:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3])\
        ,mp_obj_get_float(args[4]));
    break;
    case 4:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3])\
        ,mp_obj_get_float(args[4]),mp_obj_get_float(args[5]));
    break;
    case 5:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3])\
        ,mp_obj_get_float(args[4]),mp_obj_get_float(args[5]),mp_obj_get_float(args[6]));
    break;
    case 6:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3])\
        ,mp_obj_get_float(args[4]),mp_obj_get_float(args[5]),mp_obj_get_float(args[6]),mp_obj_get_float(args[7]));
    break;
    case 7:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3])\
        ,mp_obj_get_float(args[4]),mp_obj_get_float(args[5]),mp_obj_get_float(args[6]),mp_obj_get_float(args[7])\
        ,mp_obj_get_float(args[8]));
    break;
    case 8:
        self->putstr(self->uart_num,mp_obj_str_get_str(args[1]),mp_obj_get_float(args[2]),mp_obj_get_float(args[3])\
        ,mp_obj_get_float(args[4]),mp_obj_get_float(args[5]),mp_obj_get_float(args[6]),mp_obj_get_float(args[7])\
        ,mp_obj_get_float(args[8]),mp_obj_get_float(args[9]));
    break;
    default:break;
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR(mabot_printf_float_obj, 2, mabot_printf_float);

STATIC const mp_map_elem_t mabot_printf_locals_dict_table[] = 
{
    { MP_OBJ_NEW_QSTR(MP_QSTR_str), (mp_obj_t)&mabot_printf_str_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_int), (mp_obj_t)&mabot_printf_int_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_float), (mp_obj_t)&mabot_printf_float_obj },
};
STATIC MP_DEFINE_CONST_DICT(mabot_printf_locals_dict, mabot_printf_locals_dict_table);

const mp_obj_type_t mabot_printf_type = 
{
    { &mp_type_type },
    .name = MP_QSTR_PRINTF,
    .make_new = mabot_printf_make_new,
    .locals_dict = (mp_obj_t)&mabot_printf_locals_dict,
};
