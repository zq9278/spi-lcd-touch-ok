/*
*********************************************************************************************************
*
*	模块名称 : i2c模块
*	文件名称 : bsp_i2c.c
*	版    本 : V1.0
*	说    明 : 
*
*	修改记录 :
*	版本号          日期        		作者     		说明
*	V1.0           2022-09-15 		  DOUBLE  		 正式发布
*********************************************************************************************************
*/
#include "ft6336-iic.h"


static const char *TAG = "bspI2c";//log标记

/*
*********************************************************************************************************
*	函 数 名: i2c_master_read_slave
*	功能说明: 读从机数据
*	形    参：无
*	返 回 值: 无
* _______________________________________________________________________________________
* | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
* --------|--------------------------|----------------------|--------------------|------|
*********************************************************************************************************
*/
esp_err_t i2c_master_read_slave(uint8_t reg_addr, uint8_t addr, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (addr<< 1) | WRITE_BIT, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<< 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/*
*********************************************************************************************************
*	函 数 名: i2c_master_write_slave
*	功能说明: 写从机
*	形    参：无
*	返 回 值: 无
* ___________________________________________________________________
* | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
* --------|---------------------------|----------------------|------|
*********************************************************************************************************
*/
esp_err_t i2c_master_write_slave(uint8_t reg_addr, uint8_t addr, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr<< 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/*
*********************************************************************************************************
*	函 数 名: i2c_master_init
*	功能说明: 主机初始化
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
esp_err_t i2c_master_init(void)
{ 
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "i2c%d_master_init",I2C_MASTER_NUM);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}
