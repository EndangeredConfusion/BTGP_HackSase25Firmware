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
#include "driver/adc.h"
#include "driver/i2c_master.h"


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "TP-Link_1F14"
#define EXAMPLE_ESP_WIFI_PASS      "65291221"
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

#define TEST_INPUT_ADC_PIN GPIO_NUM_34
#define TEST_INPUT_ADC_CHANNEL ADC1_CHANNEL_6

void configure_ADC() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEST_INPUT_ADC_CHANNEL, ADC_ATTEN_DB_11);
}


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

#define TAG "SCD41"

// ==== I2C pins/speed (adjust for your board) ====
#define I2C_SDA_GPIO   21
#define I2C_SCL_GPIO   22
#define I2C_CLK_HZ     100000   // SCD4x supports Std(100k) & Fast(400k); datasheet tables are at 100k

// ==== SCD41 basics ====
#define SCD41_ADDR           0x62
#define CMD_START_PERIODIC   0x21B1
#define CMD_READ_MEAS        0xEC05
#define CMD_STOP_PERIODIC    0x3F86
#define CMD_DATA_READY       0xE4B8
#define CMD_REINIT           0x3646
#define CMD_START_LP         0x21AC
#define CMD_SINGLE_SHOT      0x219D          // SCD41 only
#define CMD_SINGLE_SHOT_RHT  0x2196          // SCD41 only
#define CMD_WAKEUP          0x36F6
#define CMD_SLEEP           0x36E0

static i2c_master_bus_handle_t scd_bus;
static i2c_master_dev_handle_t scd_dev;

// more-forgiving tx during bring-up
static esp_err_t scd_tx16(uint16_t cmd) {
    uint8_t b[2] = { cmd >> 8, cmd & 0xFF };
    return i2c_master_transmit(scd_dev, b, 2, pdMS_TO_TICKS(300));
}

#define CMD_EXIT_SLEEP        0x36F6
#define CMD_ENTER_SLEEP       0x36E0
#define CMD_STOP_PERIODIC     0x3F86   // exec ~500 ms
#define CMD_REINIT            0x3646   // exec ~20 ms
#define CMD_START_PERIODIC    0x21B1
#define CMD_GET_SERIAL        0x3682   // then read 3 words (9 bytes total)

static esp_err_t tx16(uint16_t cmd) {
    uint8_t b[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_transmit(scd_dev, b, 2, pdMS_TO_TICKS(300));
}
static uint8_t crc8(const uint8_t *d, size_t n){
    uint8_t c=0xFF; for(size_t i=0;i<n;i++){ c^=d[i]; for(int b=0;b<8;b++) c=(c&0x80)?(c<<1)^0x31:(c<<1); } return c;
}


static void scd41_bringup(void) {
    // Wake (harmless if already awake; may NACK once—ignore)
    for (int i=0;i<3;i++){ (void)tx16(CMD_EXIT_SLEEP); vTaskDelay(pdMS_TO_TICKS(2)); }

    // Stop periodic if it was running from a previous boot
    esp_err_t e = tx16(CMD_STOP_PERIODIC);
    if (e == ESP_OK) vTaskDelay(pdMS_TO_TICKS(500));  // datasheet exec time ~500 ms

    // Now reinit (only accepted when idle)
    e = tx16(CMD_REINIT);
    if (e != ESP_OK) { ESP_LOGE("SCD41","REINIT failed: %s", esp_err_to_name(e)); return; }
    vTaskDelay(pdMS_TO_TICKS(20));

    // Optional: read serial to prove RX & CRC
    ESP_ERROR_CHECK(tx16(CMD_GET_SERIAL));
    vTaskDelay(pdMS_TO_TICKS(1));
    uint8_t raw[9]={0};
    ESP_ERROR_CHECK(i2c_master_receive(scd_dev, raw, 9, pdMS_TO_TICKS(300)));
    bool crc_ok = (crc8(raw+0,2)==raw[2]) && (crc8(raw+3,2)==raw[5]) && (crc8(raw+6,2)==raw[8]);
    if (!crc_ok) { ESP_LOGE("SCD41","serial CRC fail"); return; }
    uint32_t sn_hi = ((uint16_t)raw[0]<<8)|raw[1];
    uint32_t sn_md = ((uint16_t)raw[3]<<8)|raw[4];
    uint32_t sn_lo = ((uint16_t)raw[6]<<8)|raw[7];
    ESP_LOGI("SCD41","Serial: %04lx-%04lx-%04lx", sn_hi, sn_md, sn_lo);

    // Start periodic measurements (5 s interval)
    ESP_ERROR_CHECK(tx16(CMD_START_PERIODIC));
    ESP_LOGI("SCD41","started periodic");
}


// Sensirion CRC-8: poly 0x31, init 0xFF, MSB first
static uint8_t sensirion_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
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
            .flags.enable_internal_pullup = true, // ok for short wires; external 4.7k–10k recommended
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &scd_bus), TAG, "bus");

    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SCD41_ADDR,
            .scl_speed_hz = I2C_CLK_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(scd_bus, &dev_cfg, &scd_dev), TAG, "dev");
    return ESP_OK;
}

static esp_err_t scd41_send_cmd(uint16_t cmd) {
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_transmit(scd_dev, buf, 2, pdMS_TO_TICKS(20));
}

// Read 16-bit word + CRC from sensor
static esp_err_t scd41_read_word(uint16_t *out_word) {
    uint8_t raw[3] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_receive(scd_dev, raw, 3, pdMS_TO_TICKS(20)), TAG, "rx");
    if (sensirion_crc8(raw, 2) != raw[2]) return ESP_ERR_INVALID_CRC;
    *out_word = ((uint16_t)raw[0] << 8) | raw[1];
    return ESP_OK;
}

// Check data-ready status (bit0==1 means new sample ready)
static esp_err_t scd41_get_data_ready(bool *ready) {
    ESP_RETURN_ON_ERROR(scd41_send_cmd(CMD_DATA_READY), TAG, "cmd data_ready");
    vTaskDelay(pdMS_TO_TICKS(1)); // exec time ~1ms
    uint16_t status = 0;
    ESP_RETURN_ON_ERROR(scd41_read_word(&status), TAG, "status");
    *ready = (status & 0x07FF) != 0; // datasheet: any nonzero indicates ready
    return ESP_OK;
}

// Read measurement triple (CO2 ppm, degC, %RH)
static esp_err_t scd41_read_measurement(float *co2_ppm, float *temp_c, float *rh_percent) {
    // Issue read command, wait ~1ms, then read 3 words (each word + CRC)
    ESP_RETURN_ON_ERROR(scd41_send_cmd(CMD_READ_MEAS), TAG, "cmd read");
    vTaskDelay(pdMS_TO_TICKS(1));

    uint16_t w_co2=0, w_t=0, w_rh=0;
    ESP_RETURN_ON_ERROR(scd41_read_word(&w_co2), TAG, "co2");
    ESP_RETURN_ON_ERROR(scd41_read_word(&w_t), TAG, "t");
    ESP_RETURN_ON_ERROR(scd41_read_word(&w_rh), TAG, "rh");

    // Convert per datasheet
    *co2_ppm   = (float)w_co2;
    *temp_c    = -45.0f + 175.0f * ((float)w_t / 65535.0f);
    *rh_percent= 100.0f * ((float)w_rh / 65535.0f);
    return ESP_OK;
}


static void udp_send_task(void *arg) {
    const char *dst_ip = "192.168.0.12";
    uint16_t     port  = 5005;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in dest = {0};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(port);
    inet_aton(dst_ip, &dest.sin_addr);

    char msg[64];
    for (;;) {
        int reading = adc1_get_raw(TEST_INPUT_ADC_CHANNEL);
        int voltage = reading * 150 / 2450;
        int n = snprintf(msg, sizeof(msg), "adc reading: %d\n", voltage);

        if (n < 0 || n >= (int)sizeof(msg)) {
            printf("something went wrong with the message");
        } else {
            int r = sendto(sock, msg, n, 0, (struct sockaddr*)&dest, sizeof(dest));
            if (r < 0) {
                ESP_LOGE(TAG, "sendto failed, errno=%d (%s)", errno, strerror(errno));
            } else {
                ESP_LOGI(TAG, "sendto ok, bytes=%d", r);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void i2c_poll_task(void * arg) {
    while (1) {
        bool ready = false;
        ESP_ERROR_CHECK(scd41_get_data_ready(&ready));
        if (ready) {
            float co2, tc, rh;
            esp_err_t err = scd41_read_measurement(&co2, &tc, &rh);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "CO2=%.0f ppm, T=%.2f C, RH=%.1f %%", co2, tc, rh);
            } else {
                ESP_LOGE(TAG, "read_measurement err=0x%x", err);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // poll ~1 Hz; new data arrives every 5 s
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

void app_main(void)
{
    configure_ADC();

    ESP_ERROR_CHECK(i2c_init());
    printf("init i2c\n");

    // Datasheet: after power-up or soft reset, wait up to 1s before first command
    vTaskDelay(pdMS_TO_TICKS(2000));

//    esp_log_level_set("i2c.master", ESP_LOG_ERROR);  // quiet the driver while scanning

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_probe(scd_bus, 0x62, pdMS_TO_TICKS(150)));

    scd41_bringup();

    // Re-init is quick if you want a clean state:
    ESP_ERROR_CHECK(scd41_send_cmd(CMD_REINIT));
    vTaskDelay(pdMS_TO_TICKS(20));

//    // Start periodic (5 s update interval). For low-power mode use CMD_START_LP.
//    ESP_ERROR_CHECK(scd41_send_cmd(CMD_START_PERIODIC));
//    ESP_LOGI(TAG, "SCD41 started periodic measurements (5s interval).");

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    xTaskCreate(udp_send_task, "udp_send", 4096, NULL, 5, NULL);
    xTaskCreate(i2c_poll_task, "i2c_send", 4096, NULL, 5, NULL);
}

