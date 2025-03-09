#include <cstdio>
#include <cstring>
#include <nvs_flash.h>
#include <nimble/nimble_port_freertos.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "devices/device_component.h"
#include "devices/codec.cpp"
#include "devices/satellites.cpp"
#include "devices/sub.cpp"
#include "NimBLEDevice.h"
#include "ble/ble.cpp"

i2c_master_bus_handle_t i2c_bus;
Codec *codec;
Satellites *satellites;
Sub *sub;

uint3 volume;

void codec_task_source_autoselect(void *pvParameter) {
    codec->lock_active_source();
    //ampsErrorCheck();
    vTaskDelete(NULL); // Delete the task when done
}

void timer_callback(TimerHandle_t xTimer) {
    xTaskCreate(&codec_task_source_autoselect, "CodecSourceAutoselectTask", 4096, NULL, 5, NULL);
}

static void codec_event_cb(codec_cb_event_t event, codec_event_param_t *param) {
    switch (event) {
        case CODEC_SOURCE_EVT:
            switch (param->source_state.state) {
                case CODEC_SOURCE_UNLOCK:
                case CODEC_SOURCE_INACTIVE:
                    satellites->set_volume(uint3(0));
                    return;
                case CODEC_SOURCE_ACTIVE:
                    satellites->set_volume(volume);
                    return;
                default:
                    ESP_LOGI("Codec_CB", "Codec SOURCE_EVT CB: source_state %d", param->source_state.state);
                    return;
            }
        case CODEC_RECEIVER_ERR_EVT:
        default:
            ESP_LOGW("codec_event_cb", "Invalid codec event: %d", event);
            break;
    }
}

static void ble_volume_chg_cb(uint3 vol) {
    ESP_LOGI("Main", "Volume changed to %d", vol.value);
    volume = vol;
}

extern "C" void app_main(void) {
    // Set PDN high to enable amplifiers
    gpio_num_t PDN = GPIO_NUM_3;
    gpio_set_direction(PDN, GPIO_MODE_OUTPUT);
    gpio_set_level(PDN, 1);

    // Set up I2C bus
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_10,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));

    // Find the address of the sub dynamically since it tends to move around
    uint16_t sub_addr = 0x2C;
    for (int i = 0x2C; i <= 0x2F; i++) {
        esp_err_t status = i2c_master_probe(i2c_bus, i, 10);
        if (status == ESP_OK) {
            sub_addr = i;
            ESP_LOGI("main", "Found sub at 0x%02X", i);
            break;
        }
    }

    // Init flash memory
    esp_err_t flash = nvs_flash_init();
    if (flash == ESP_ERR_NVS_NO_FREE_PAGES || flash == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        flash = nvs_flash_init();
    }
    ESP_ERROR_CHECK(flash);

    // Init Bluetooth LE
    esp_log_level_set("NimBLEService", ESP_LOG_VERBOSE);
    NimBLEDevice::init("Flåsen");
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND);
    NimBLEDevice::setSecurityPasskey(120020);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);

    NimBLEServer *server = NimBLEDevice::createServer();

    NimBLEService *service = server->createService("A0FE");
    NimBLECharacteristic *volume = service->createCharacteristic("1200", READ | WRITE, 1);
    auto *chrCallbacks = new VolumeCharacteristic(ble_volume_chg_cb);
    volume->setCallbacks(chrCallbacks);
    volume->setValue(5);

    service->start();
    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID("A0FE");
    advertising->setName("Flåsen");
    advertising->start();

    codec = new Codec(i2c_bus);
    satellites = new Satellites(i2c_bus);
    sub = new Sub(i2c_bus, sub_addr);
    int init_codec = codec->device_init();
    int init_satellites = satellites->device_init();
    int init_sub = sub->device_init();
    if (init_codec | init_satellites | init_sub) {
        ESP_LOGE("ERROR", "Device initialization failed: codec %d, satellites %d, sub %d", init_codec, init_satellites,
                 init_sub);
    }

    // Initialize codec and satellites configuration
    codec->init();
    satellites->init();

    // Probe I2C bus status for all components
    esp_err_t probe_codec = codec->i2c_probe();
    esp_err_t probe_satellites = satellites->i2c_probe();
    esp_err_t probe_sub = sub->i2c_probe();

    if (probe_codec | probe_satellites | probe_sub) {
        ESP_LOGE("MAIN", "Device probe failed; codec: %x, TAS5827: %x, TAS5805M: %x", probe_codec, probe_satellites,
                 probe_sub);
    }

    codec->register_event_callback(codec_event_cb);

    // Initialize Sub only after we have acquired a PLL lock
    while (!codec->check_pll_locked()) {
        // Wait for PLL to lock before continuing
        codec->change_source();
        vTaskDelay(source_refresh_rate / portTICK_PERIOD_MS);
    }

    sub->init();

    ESP_LOGI("MAIN", "Amps initialised");

    // Log power state
    satellites->get_power_state(true);
    sub->get_power_state(true);

    // If they have no errors, attempt to set satellites and sub into play state
    int playing = 0;
    while (!playing) {
        vTaskDelay(TICK_DELAY);
        esp_err_t check_satellites = satellites->error_check();
        esp_err_t check_sub = sub->error_check();

        if (check_satellites != ESP_OK || check_sub != ESP_OK) {
            continue;
        }

        satellites->set_play();
        sub->set_play();

        uint8_t amplrpwrstate = satellites->get_power_state(true);
        uint8_t subpwrstate = sub->get_power_state(true);

        if ((amplrpwrstate == 0x03) & (subpwrstate == 0x03)) {
            playing = 1;
            ESP_LOGI("APP_MAIN", "Amps set to play state");
        } else {
            ESP_LOGE("APP_MAIN", "Failed to set amps to play state");
        }
    }

    TimerHandle_t timer = xTimerCreate("MyTimer", pdMS_TO_TICKS(source_refresh_rate), pdTRUE, 0, timer_callback);
    // Check if the timer was created successfully
    if (timer != nullptr) {
        // Start the timer
        xTimerStart(timer, 0);
    } else {
        ESP_LOGE("APP_MAIN", "Failed to create timer");
    }
}
