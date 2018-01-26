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

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "modmabot.h"

typedef struct _mabot_delay_obj_t {
    mp_obj_base_t base;
} mabot_delay_obj_t;

STATIC mp_obj_t mabot_delay_make_new(const mp_obj_type_t *type_in, size_t n_args, size_t n_kw, const mp_obj_t *args) 
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    mabot_delay_obj_t *self = m_new_obj(mabot_delay_obj_t);
    self->base.type = &mabot_delay_type;

    return MP_OBJ_FROM_PTR(self);
}   

STATIC mp_obj_t mabot_delay_ms (mp_obj_t self_in, mp_obj_t value_in )
{
    mabot_delay_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self = self;
    uint32_t delay = mp_obj_get_int(value_in);
    vTaskDelay(pdMS_TO_TICKS(delay));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mabot_delay_ms_obj, mabot_delay_ms);

STATIC const mp_map_elem_t mabot_delay_locals_dict_table[] = 
{
    { MP_OBJ_NEW_QSTR(MP_QSTR_ms), (mp_obj_t)&mabot_delay_ms_obj },
};
STATIC MP_DEFINE_CONST_DICT(mabot_delay_locals_dict, mabot_delay_locals_dict_table);

const mp_obj_type_t mabot_delay_type = 
{
    { &mp_type_type },
    .name = MP_QSTR_DELAY,
    .make_new = mabot_delay_make_new,
    .locals_dict = (mp_obj_t)&mabot_delay_locals_dict,
};