//
// Created by Kaeshev Alapati on 9/20/25.
// some attributed to esp32 examples, rest to me
//

/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "driver/i2c_master.h"
#include "lwip/sys.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include <string.h>
#include "esp_system.h"
#include "esp_check.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "freertos/task.h"

#define EXAMPLE_ESP_WIFI_SSID      "*******"
#define EXAMPLE_ESP_WIFI_PASS      "*******"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

// Choose the minimum auth level your network needs:
#ifndef ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK   // or WIFI_AUTH_OPEN if it's open
#endif

#ifndef ESP_WIFI_SAE_MODE
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH   // safe default; works for WPA2/WPA3-Personal
#endif

#ifndef EXAMPLE_H2E_IDENTIFIER
#define EXAMPLE_H2E_IDENTIFIER ""                 // leave empty unless your WPA3 setup needs it
#endif


#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

//#define TAG "SCD41"

#define I2C_SDA_GPIO   21     // needs to be these pins
#define I2C_SCL_GPIO   22
#define I2C_CLK_HZ     50000 // pick lower speed because data rate is choked by device not by com

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

#define TEST_INPUT_ADC_PIN GPIO_NUM_34
#define TEST_INPUT_ADC_CHANNEL ADC1_CHANNEL_6
#define SCD41_ADDR           0x62



static EventGroupHandle_t s_wifi_event_group;

static i2c_master_bus_handle_t master_bus;
static i2c_master_dev_handle_t scd_device_handle;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static bool new_data_published = 0;
static uint16_t co2 = 0;
static float temp = 0;
static float humidity = 0;

static void udp_send_task(void *arg) {
    const char *dst_kaeshev_ip = "******";
    uint16_t port_kaeshev = 0xdeadbeef;

    const char *dst_vincent_ip = "****";
    uint16_t port_vincent = 0xbeefcafe;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(port_vincent);
    inet_aton(dst_vincent_ip, &dest.sin_addr);

    char msg[128];
    for (;;) {
        if (new_data_published) {
            new_data_published = 0;

            int reading = adc1_get_raw(TEST_INPUT_ADC_CHANNEL);
            printf("raw adc reading %d", reading);
            int voltage = reading * 150 / 2450;
            int n = snprintf(msg, sizeof(msg), "adc reading: %d, co2: (ppm) %d, temp: %f, humidity: %f\n",
                             voltage, co2, temp, humidity);

            if (n < 0 || n >= (int) sizeof(msg)) {
                printf("something went wrong with the message");
            } else {
                int r = sendto(sock, msg, n, 0, (struct sockaddr *) &dest, sizeof(dest));
                if (r < 0) {
                    ESP_LOGE(TAG, "sendto failed, errno=%d (%s)", errno, strerror(errno));
                } else {
                    ESP_LOGI(TAG, "sendto ok, bytes=%d", r);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = EXAMPLE_ESP_WIFI_SSID,
                    .password = EXAMPLE_ESP_WIFI_PASS,
                    /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
                     * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
                     * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
                     * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
                     */
                    .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
                    .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
                    .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


void configure_ADC() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEST_INPUT_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

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
    configure_ADC();

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

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    xTaskCreate(udp_send_task, "udp_send", 4096, NULL, 5, NULL);

    bool data_ready = 0;

    uint16_t co2_raw = 0;
    uint16_t temp_raw = 0;
    uint16_t humidity_raw = 0;
    uint16_t failcount = 0;

    for (;;) {
        if (failcount > 300) {
            printf("failed, resetting bus\n");
//            output = send_stop_periodic();
//            ESP_LOGI(TAG, "stop_periodic returned %s", esp_err_to_name(output));
//
//            output = send_reinit();
//            ESP_LOGI(TAG, "stop_periodic returned %s", esp_err_to_name(output));

            output = send_start_periodic();
            ESP_LOGI(TAG, "start_periodic returned %s", esp_err_to_name(output));
            failcount = 0;
        }

        output = check_if_data_ready(&data_ready);
//        ESP_LOGI(TAG, "data_ready returned %s", esp_err_to_name(output));

        if (data_ready) {
            output = read_measurement(&co2_raw, &temp_raw, &humidity_raw);
            ESP_LOGI(TAG, "read measurement returned %s", esp_err_to_name(output));
            co2 = co2_raw;
            temp = -45 + (float)(175 * (float)temp_raw / (0b1<<16));
            humidity = 100 * (float)humidity_raw/(0b1<<16);
            printf("co2: %d,\t temp: %.2f,\t humidity: %.2f\n", co2, temp, humidity);
            new_data_published = true;
        } else {
            failcount++;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}