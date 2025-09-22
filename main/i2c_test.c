//
// Created by Kaeshev Alapati on 9/20/25.
//
/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_check.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "driver/i2c_master.h"

#define TAG "SCD41"

#define I2C_SDA_GPIO   21     // needs to be these pins
#define I2C_SCL_GPIO   22
//#define I2C_CLK_HZ     100000 // pick lower speed because data rate is choked by device not by com
#define I2C_CLK_HZ     50000 // pick lower speed because data rate is choked by device not by com


#define CMD_STOP_PERIODIC    0x3F86
#define CMD_REINIT           0x3646

#define SCD41_ADDR           0x62
#define CMD_START_PERIODIC   0x21B1
#define CMD_READ_MEAS        0xEC05
#define CMD_DATA_READY       0xE4B8
#define CMD_START_LP         0x21AC
#define CMD_WAKEUP           0x36F6
#define CMD_SLEEP            0x36E0
#define CMD_GET_SERIAL       0x3682

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

static i2c_master_bus_handle_t master_bus;
static i2c_master_dev_handle_t scd_device_handle;

// given by sensiron
uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
/* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

static esp_err_t i2c_init(void) {
    i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = I2C_SDA_GPIO,
            .scl_io_num = I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = false,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &master_bus), TAG, "bus");

    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SCD41_ADDR,
            .scl_speed_hz = I2C_CLK_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(master_bus, &dev_cfg, &scd_device_handle), TAG, "dev");
    return ESP_OK;
}

static esp_err_t send_stop_periodic(void) {
    uint8_t writebuffer[2] = {0x3f, 0x86};
    esp_err_t err = i2c_master_transmit(scd_device_handle, writebuffer, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(500));
    return err;
}

static esp_err_t send_reinit(void) {
    uint8_t writebuffer[2] = {0x36, 0x46};

    send_stop_periodic();

    esp_err_t output = i2c_master_transmit(scd_device_handle, writebuffer, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(20));

    return output;
}

static esp_err_t send_start_periodic(void) {
    uint8_t writebuffer[2] = {0x21, 0xb1 };

    esp_err_t resp = i2c_master_transmit(scd_device_handle, writebuffer, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(20));
    return resp;
}

static esp_err_t check_if_data_ready(bool * is_data_ready) {
    uint8_t writebuffer[2] = {0xe4, 0xb8};
    uint8_t readbuffer[3];

    esp_err_t resp = i2c_master_transmit_receive(scd_device_handle, writebuffer, 2,
                                                 readbuffer, 3, 50);
    vTaskDelay(pdMS_TO_TICKS(20));

    if (((uint16_t)((uint16_t)readbuffer[0]<<8 | (uint16_t)readbuffer[1]) & 0b11111111111) == 0) {
        *is_data_ready = false;
    } else {
        *is_data_ready = true;
    }
    return resp;
}

static esp_err_t read_measurement(uint16_t * co2, uint16_t * temp, uint16_t * humidity) {
    uint8_t writebuffer[2] = {0xec, 0x05};
    uint8_t readbuffer[9];

    esp_err_t resp = i2c_master_transmit_receive(scd_device_handle, writebuffer, 2,
                                                 readbuffer, 9, 50);
    if (resp != ESP_OK) return resp;

    vTaskDelay(pdMS_TO_TICKS(20));
    if (sensirion_common_generate_crc(&readbuffer[0], 2) == readbuffer[2]) {
        *co2 = (uint16_t)((uint16_t)readbuffer[0] << 8 | (uint16_t)readbuffer[1]);
    } else {
        *co2 = 0xFFFF;
    }

    if (sensirion_common_generate_crc(&readbuffer[3], 2) == readbuffer[5]) {
        *temp = (uint16_t)((uint16_t)readbuffer[3] << 8 | (uint16_t)readbuffer[4]);
    } else {
        *temp = 0xFFFF;
    }

    if (sensirion_common_generate_crc(&readbuffer[6], 2) == readbuffer[8]) {
        *humidity = (uint16_t)((uint16_t)readbuffer[6] << 8 | (uint16_t)readbuffer[7]);
    } else {
        *humidity = 0xFFFF;
    }
    return resp;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());
    printf("init i2c\n");

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_probe(master_bus, 0x62, 150));
    printf("probed\n");

    esp_err_t output;

    output = send_stop_periodic();
    ESP_LOGI(TAG, "stop_periodic returned %s", esp_err_to_name(output));

    output = send_reinit();
    ESP_LOGI(TAG, "stop_periodic returned %s", esp_err_to_name(output));

    output = send_start_periodic();
    ESP_LOGI(TAG, "start_periodic returned %s", esp_err_to_name(output));

    bool data_ready = 0;
    uint16_t co2 = 0;
    uint16_t temp = 0;
    uint16_t humidity = 0;
    uint16_t failcount = 0;
    for (;;) {
        if (failcount > 700) {
            printf("failed, resetting bus\n");
            output = send_stop_periodic();
            ESP_LOGI(TAG, "stop_periodic returned %s", esp_err_to_name(output));

            output = send_reinit();
            ESP_LOGI(TAG, "stop_periodic returned %s", esp_err_to_name(output));

            output = send_start_periodic();
            ESP_LOGI(TAG, "start_periodic returned %s", esp_err_to_name(output));
            failcount = 0;
        }

        output = check_if_data_ready(&data_ready);
//        ESP_LOGI(TAG, "data_ready returned %s", esp_err_to_name(output));

        if (data_ready) {
            output = read_measurement(&co2, &temp, &humidity);
            ESP_LOGI(TAG, "read measurement returned %s", esp_err_to_name(output));

            printf("%d,\t %d,\t %d\n", co2, temp, humidity);
        } else {
            failcount++;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}