#include <esp_err.h>
#include <esp_log.h>
#include <portmacro.h>
#include <driver/i2c_master.h>
#include <driver/i2c_types.h>
#include "device_component.h"

constexpr uint32_t I2C_DEFAULT_100KHZ = 100000;

void DeviceComponent::init() {}

esp_err_t DeviceComponent::error_check() {
    return ESP_FAIL;
}

int DeviceComponent::device_init() {
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = this->device_address,
        .scl_speed_hz = I2C_DEFAULT_100KHZ,
    };

    const esp_err_t err = i2c_master_bus_add_device(this->i2c_bus, &cfg, &this->i2c_handle);
    if (err != ESP_OK) {
        ESP_LOGE(this->device_name, "I2C bus add failed: %s", esp_err_to_name(err));
        return -1;
    }

    return 0;
}

esp_err_t DeviceComponent::i2c_probe() const {
    esp_err_t status = i2c_master_probe(this->i2c_bus, this->device_address, 10);
    if (status != ESP_OK) {
        ESP_LOGE(this->device_name, "Failed probing codec: %d", status);
    }

    return status;
}
