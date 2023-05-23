#include "driver/i2c.h"
#include "rom/ets_sys.h"
#include <stdio.h>
#include <string.h>
#include "bmi270.h"
#include "esp_log.h"

const uint8_t DEV_ADDR = 0x68;

int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr) {
    if (len == 0) {
        return ESP_OK;
    }

    // ESP_LOGE("BMI270", "reading... %X, %X, %X", reg_addr, data[0], len);
    i2c_port_t i2c_num = (i2c_port_t)intf_ptr;
    esp_err_t err = i2c_master_write_read_device(i2c_num, DEV_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));
    // ESP_LOGE("BMI270", "res: %x", data[0]);
    return err == ESP_OK ? 0 : -1;
}


int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr) {
    // ESP_LOGE("BMI270", "writing...");
    i2c_port_t i2c_num = (i2c_port_t)intf_ptr;
    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(buffer + 1, data, len);
    esp_err_t err = i2c_master_write_to_device(i2c_num, DEV_ADDR, buffer, len + 1, pdMS_TO_TICKS(1000));
    return err == ESP_OK ? 0 : -1;
}

void delay_us(uint32_t us, void *intf_ptr) {
    ets_delay_us(us);
}

esp_err_t bmi270_init_mine(i2c_port_t i2c_num, struct bmi2_dev *bmi270_sensor) {
    if (!bmi270_sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    bmi270_sensor->read = i2c_read;
    bmi270_sensor->write = i2c_write;
    bmi270_sensor->delay_us = delay_us;
    bmi270_sensor->intf_ptr = (void *)i2c_num;
    bmi270_sensor->read_write_len = 16;
    bmi270_sensor->aps_status = BMI2_DISABLE;

    int8_t result = bmi270_init(bmi270_sensor);
    if (result != BMI2_OK) {
        ESP_LOGE("BMI270", "bmi270_init failed: %d", result);
        return ESP_FAIL;
    }

    return ESP_OK;
}
