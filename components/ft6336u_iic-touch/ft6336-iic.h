/*
*********************************************************************************************************
*
*	模块名称 : i2c模块
*	文件名称 : bsp_i2c.c
*	版    本 : V1.0
*	说    明 : 
*********************************************************************************************************
*/
#ifndef __BSP_I2C_H
#define	__BSP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           3                      //scl
#define I2C_MASTER_SDA_IO           8                      //sda
#define I2C_MASTER_NUM              I2C_NUM_0               //i2c1
#define I2C_MASTER_FREQ_HZ          100000                  //频率100K

#define WRITE_BIT   I2C_MASTER_WRITE                        //写:0
#define READ_BIT    I2C_MASTER_READ                         //读:1
#define ACK_CHECK_EN        0x1                             //主机检查从机的ACK
#define ACK_CHECK_DIS       0x0                             //主机不检查从机的ACK
#define ACK_VAL             0x0                             //应答
#define NACK_VAL            0x1                             //不应答
esp_err_t i2c_master_read_slave(uint8_t reg_addr, uint8_t addr, uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_slave(uint8_t reg_addr, uint8_t addr, uint8_t *data_wr, size_t size);
esp_err_t i2c_master_init(void);

#ifdef __cplusplus
}
#endif


#endif