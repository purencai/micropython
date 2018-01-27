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
#include <sys/time.h>

#include "driver/gpio.h"

#include "driver/i2c.h"

#include "py/obj.h"

#include "mpu6050.h"

#define ACK_VAL                    0
#define NACK_VAL                   1

#define WRITE_BIT                  0
#define READ_BIT                   1

#define I2C_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */ 

#define ACK_CHECK_EN               0x1
#define ACK_CHECK_DIS              0x0

void mpu6050_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void i2c_master_init(void)
{
    esp_err_t ret = ESP_OK; 
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if(ret != ESP_OK)
    {
        i2c_driver_delete(I2C_MASTER_NUM);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    }
}

static inline esp_err_t mpu6050_write_byte(uint8_t reg_addr,uint8_t data) 				 
{ 
	esp_err_t ret = ESP_OK; 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU_ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static inline uint8_t mpu6050_read_byte(uint8_t reg_addr)
{
	uint8_t data = 0xff;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU_ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU_ADDR << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return data;
}
esp_err_t mpu6050_write_bytes(uint8_t iic_addr,uint8_t reg_addr,uint8_t len,uint8_t *data) 				 
{ 
	esp_err_t ret = ESP_OK; 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU_ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, (size_t)len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t mpu6050_read_bytes(uint8_t iic_addr,uint8_t reg_addr,uint8_t len,uint8_t *data) 
{
	esp_err_t ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU_ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MPU_ADDR << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read(cmd, data, (size_t)(len - 1), ACK_VAL);
    i2c_master_read_byte(cmd, data+len-1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}
static inline esp_err_t mpu6050_set_gyro_fsr(uint8_t fsr)
{
	return mpu6050_write_byte(MPU_GYRO_CFG_REG,fsr<<3);
}

static inline esp_err_t mpu6050_set_accel_fsr(uint8_t fsr)
{
	return mpu6050_write_byte(MPU_ACCEL_CFG_REG,fsr<<3);
}

static inline esp_err_t mpu6050_set_lpf(uint16_t lpf)
{
	uint8_t data = 0;
	if(lpf>=188)data = 1;
	else if(lpf>=98)data = 2;
	else if(lpf>=42)data = 3;
	else if(lpf>=20)data = 4;
	else if(lpf>=10)data = 5;
	else data = 6; 
	return mpu6050_write_byte(MPU_ACCEL_CFG_REG,data);
}

static inline esp_err_t mpu6050_set_rate(uint16_t rate)
{
	uint8_t data;
	esp_err_t ret = ESP_OK;
	if(rate>1000)
	{
		rate = 1000;
	}
	if(rate<4)
	{
		rate = 4;
	}
	data = 1000/rate-1;
	ret = mpu6050_write_byte(MPU_SAMPLE_RATE_REG,data);
	if( ret != ESP_OK)
	{
		return ret;	
	}
 	return mpu6050_set_lpf(rate/2);	
}

esp_err_t mpu6050_init(void)
{ 
    uint8_t addres = 0Xff;
	i2c_master_init();
	mpu6050_write_byte(MPU_PWR_MGMT1_REG,0X80);	
    vTaskDelay(pdMS_TO_TICKS(100));
	mpu6050_write_byte(MPU_PWR_MGMT1_REG,0X00);	
	mpu6050_set_gyro_fsr(3);					
	mpu6050_set_accel_fsr(0);					
	mpu6050_set_rate(50);						
	mpu6050_write_byte(MPU_INT_EN_REG,0X00);	
	mpu6050_write_byte(MPU_USER_CTRL_REG,0X00);	
	mpu6050_write_byte(MPU_FIFO_EN_REG,0X00);	
	mpu6050_write_byte(MPU_INTBP_CFG_REG,0X80);	
    addres = mpu6050_read_byte(MPU_DEVICE_ID_REG);
	if(addres == MPU_ADDR)
	{
		mpu6050_write_byte(MPU_PWR_MGMT1_REG,0X01);
		mpu6050_write_byte(MPU_PWR_MGMT2_REG,0X00);
		mpu6050_set_rate(50);
 	}
    else
    {
        return ESP_FAIL;
    }
    // while(1)
    // {
    //     printf("addres = 0x%02x\r\n",mpu6050_read_byte(MPU_DEVICE_ID_REG));
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    return ESP_OK;
}


