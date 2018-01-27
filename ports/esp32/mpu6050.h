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
 * THE SOFTWARE.
 */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "esp_system.h"

//#define MPU_ACCEL_OFFS_REG		0X06
//#define MPU_PROD_ID_REG			0X0C
#define MPU_SELF_TESTX_REG		0X0D
#define MPU_SELF_TESTY_REG		0X0E
#define MPU_SELF_TESTZ_REG		0X0F
#define MPU_SELF_TESTA_REG		0X10
#define MPU_SAMPLE_RATE_REG		0X19
#define MPU_CFG_REG				0X1A
#define MPU_GYRO_CFG_REG		0X1B
#define MPU_ACCEL_CFG_REG		0X1C
#define MPU_MOTION_DET_REG		0X1F
#define MPU_FIFO_EN_REG			0X23
#define MPU_I2CMST_CTRL_REG		0X24
#define MPU_I2CSLV0_ADDR_REG	0X25
#define MPU_I2CSLV0_REG			0X26
#define MPU_I2CSLV0_CTRL_REG	0X27
#define MPU_I2CSLV1_ADDR_REG	0X28
#define MPU_I2CSLV1_REG			0X29
#define MPU_I2CSLV1_CTRL_REG	0X2A
#define MPU_I2CSLV2_ADDR_REG	0X2B
#define MPU_I2CSLV2_REG			0X2C
#define MPU_I2CSLV2_CTRL_REG	0X2D
#define MPU_I2CSLV3_ADDR_REG	0X2E
#define MPU_I2CSLV3_REG			0X2F
#define MPU_I2CSLV3_CTRL_REG	0X30
#define MPU_I2CSLV4_ADDR_REG	0X31
#define MPU_I2CSLV4_REG			0X32
#define MPU_I2CSLV4_DO_REG		0X33
#define MPU_I2CSLV4_CTRL_REG	0X34
#define MPU_I2CSLV4_DI_REG		0X35

#define MPU_I2CMST_STA_REG		0X36
#define MPU_INTBP_CFG_REG		0X37
#define MPU_INT_EN_REG			0X38
#define MPU_INT_STA_REG			0X3A

#define MPU_ACCEL_XOUTH_REG		0X3B
#define MPU_ACCEL_XOUTL_REG		0X3C
#define MPU_ACCEL_YOUTH_REG		0X3D
#define MPU_ACCEL_YOUTL_REG		0X3E
#define MPU_ACCEL_ZOUTH_REG		0X3F
#define MPU_ACCEL_ZOUTL_REG		0X40

#define MPU_TEMP_OUTH_REG		0X41
#define MPU_TEMP_OUTL_REG		0X42

#define MPU_GYRO_XOUTH_REG		0X43
#define MPU_GYRO_XOUTL_REG		0X44
#define MPU_GYRO_YOUTH_REG		0X45
#define MPU_GYRO_YOUTL_REG		0X46
#define MPU_GYRO_ZOUTH_REG		0X47
#define MPU_GYRO_ZOUTL_REG		0X48

#define MPU_I2CSLV0_DO_REG		0X63
#define MPU_I2CSLV1_DO_REG		0X64
#define MPU_I2CSLV2_DO_REG		0X65
#define MPU_I2CSLV3_DO_REG		0X66

#define MPU_I2CMST_DELAY_REG	0X67
#define MPU_SIGPATH_RST_REG		0X68
#define MPU_MDETECT_CTRL_REG	0X69
#define MPU_USER_CTRL_REG		0X6A
#define MPU_PWR_MGMT1_REG		0X6B
#define MPU_PWR_MGMT2_REG		0X6C
#define MPU_FIFO_CNTH_REG		0X72
#define MPU_FIFO_CNTL_REG		0X73
#define MPU_FIFO_RW_REG			0X74
#define MPU_DEVICE_ID_REG		0X75
 
#define MPU_ADDR				0X68

extern void mpu6050_delay_ms(uint32_t ms);
extern void i2c_master_init(void);
extern esp_err_t mpu6050_init(void);
extern esp_err_t mpu6050_read_bytes(uint8_t iic_addr,uint8_t reg_addr,uint8_t len,uint8_t *data);
extern esp_err_t mpu6050_write_bytes(uint8_t iic_addr,uint8_t reg_addr,uint8_t len,uint8_t *data);

#endif // __MPU6050_H__

