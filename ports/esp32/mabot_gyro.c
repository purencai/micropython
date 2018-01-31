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
#include "esp_task.h"
#include "soc/soc.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "modmabot.h"
#include "inv_mpu.h"
#include "mpu6050.h"

#define ANGULAR_LSB 				65.5f

typedef struct _gyro_parameter_t {
	float angle_x;
	float angle_y;
	float angle_z;
	float angular_x;
	float angular_y;
	float angular_z;
} gyro_parameter_t;

typedef struct _mabot_gyro_obj_t {
    mp_obj_base_t base;
    gyro_parameter_t gyro;
} mabot_gyro_obj_t;

gyro_parameter_t gyro_parameter = 
{
    .angle_x = 0,
    .angle_y = 0,
    .angle_z = 0,
    .angular_x = 0,
    .angular_y = 0,
    .angular_z = 0
};

STATIC void gyro_task(void *pvParameter)
{
	float pitch,roll,yaw;
    short gyro[3],accel[3];
    float yaw_fix = 0,last_yaw = 0;
    for(;;)
    {
        if(mpu_dmp_get_data(&pitch,&roll,&yaw,gyro,accel) == 0)
        {
            gyro_parameter.angle_x = roll;
            gyro_parameter.angle_y = pitch;
            if(yaw - last_yaw < -350.0f)
                yaw_fix+=360.0f;
            else if(yaw - last_yaw > 350.0f)
                yaw_fix-=360.0f;
            gyro_parameter.angle_z = yaw+yaw_fix;
            gyro_parameter.angular_x = ((float)gyro[0])/ANGULAR_LSB;
            gyro_parameter.angular_y = ((float)gyro[1])/ANGULAR_LSB;
            gyro_parameter.angular_z = ((float)gyro[2])/ANGULAR_LSB;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

STATIC void gyro_main(void)
{
    xTaskCreatePinnedToCore(gyro_task, "gyro_task", 4096, NULL, (ESP_TASK_PRIO_MAX - 1), NULL, 0);
}
STATIC mp_obj_t mabot_gyro_make_new(const mp_obj_type_t *type_in, size_t n_args, size_t n_kw, const mp_obj_t *args) 
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    mabot_gyro_obj_t *self = m_new_obj(mabot_gyro_obj_t);
    self->base.type = &mabot_gyro_type;
    self->gyro.angle_x = 0;
    self->gyro.angle_y = 0;
    self->gyro.angle_z = 0;
    self->gyro.angular_x = 0;
    self->gyro.angular_y = 0;
    self->gyro.angular_z = 0;

	while(mpu_dmp_init()) 
	{
        printf("mpu6050_init err!!\r\n");
	}

    vTaskDelay(pdMS_TO_TICKS(500));

    return MP_OBJ_FROM_PTR(self);
} 

STATIC mp_obj_t mabot_enable_gyro (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->gyro.angle_x = 0;
    self->gyro.angle_y = 0;
    self->gyro.angle_z = 0;
    self->gyro.angular_x = 0;
    self->gyro.angular_y = 0;
    self->gyro.angular_z = 0;
    gyro_main();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_enable_gyro_obj, mabot_enable_gyro);

STATIC void get_angle_x (mabot_gyro_obj_t *self_in)
{
    self_in->gyro.angle_x = gyro_parameter.angle_x;
}
STATIC void get_angle_y (mabot_gyro_obj_t *self_in)
{
    self_in->gyro.angle_y = gyro_parameter.angle_y;
}
STATIC void get_angle_z (mabot_gyro_obj_t *self_in)
{
    self_in->gyro.angle_z = gyro_parameter.angle_z;
}
STATIC void get_angular_x (mabot_gyro_obj_t *self_in)
{
    self_in->gyro.angular_x = gyro_parameter.angular_x;
}
STATIC void get_angular_y (mabot_gyro_obj_t *self_in)
{
    self_in->gyro.angular_y = gyro_parameter.angular_y;
}
STATIC void get_angular_z (mabot_gyro_obj_t *self_in)
{
    self_in->gyro.angular_z = gyro_parameter.angular_z;
}
STATIC mp_obj_t mabot_read_angle_x (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    get_angle_x(self);
    return mp_obj_new_float(self->gyro.angle_x);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_read_angle_x_obj, mabot_read_angle_x);

STATIC mp_obj_t mabot_read_angle_y (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    get_angle_y(self);
    return mp_obj_new_float(self->gyro.angle_y);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_read_angle_y_obj, mabot_read_angle_y);

STATIC mp_obj_t mabot_read_angle_z (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    get_angle_z(self);
    return mp_obj_new_float(self->gyro.angle_z);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_read_angle_z_obj, mabot_read_angle_z);

STATIC mp_obj_t mabot_read_angular_x (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    get_angular_x(self);
    return mp_obj_new_float(self->gyro.angular_x);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_read_angular_x_obj, mabot_read_angular_x);

STATIC mp_obj_t mabot_read_angular_y (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    get_angular_y(self);
    return mp_obj_new_float(self->gyro.angular_y);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_read_angular_y_obj, mabot_read_angular_y);

STATIC mp_obj_t mabot_read_angular_z (mp_obj_t self_in)
{
    mabot_gyro_obj_t *self = MP_OBJ_TO_PTR(self_in);
    get_angular_z(self);
    return mp_obj_new_float(self->gyro.angular_z);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mabot_read_angular_z_obj, mabot_read_angular_z);

STATIC const mp_map_elem_t mabot_gyro_locals_dict_table[] = 
{
    { MP_OBJ_NEW_QSTR(MP_QSTR_enable), (mp_obj_t)&mabot_enable_gyro_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_angle_x), (mp_obj_t)&mabot_read_angle_x_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_angle_y), (mp_obj_t)&mabot_read_angle_y_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_angle_z), (mp_obj_t)&mabot_read_angle_z_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_angular_x), (mp_obj_t)&mabot_read_angular_x_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_angular_y), (mp_obj_t)&mabot_read_angular_y_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_angular_z), (mp_obj_t)&mabot_read_angular_z_obj },
};
STATIC MP_DEFINE_CONST_DICT(mabot_gyro_locals_dict, mabot_gyro_locals_dict_table);

const mp_obj_type_t mabot_gyro_type = 
{
    { &mp_type_type },
    .name = MP_QSTR_GYRO,
    .make_new = mabot_gyro_make_new,
    .locals_dict = (mp_obj_t)&mabot_gyro_locals_dict,
};