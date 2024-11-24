#include "device_component.h"
#include "amplifier.h"
#include "regs.h"
#include "esp_log.h"
#include "../i2c.h"

esp_err_t Amplifier::error_check() {
    const uint8_t chan_fault = receive_single(this->i2c_handle, CHAN_FAULT);
    const uint8_t global_fault_1 = receive_single(this->i2c_handle, GLOBAL_FAULT_1);
    const uint8_t global_fault_2 = receive_single(this->i2c_handle, GLOBAL_FAULT_2);

    if (chan_fault | global_fault_1 | global_fault_2) {
        ESP_LOGE(this->device_name,
            "chan_fault: 0x%02X, global_fault1: 0x%02X, global_fault2: 0x%02X",
            chan_fault,
            global_fault_1,
            global_fault_2);

        constexpr uint8_t clear_analog_fault = 0x80;
        write_pair(this->i2c_handle, FAULT_CLEAR, clear_analog_fault);

        return ESP_FAIL;
    }

    return ESP_OK;
}

uint8_t Amplifier::get_power_state(const bool log = false) const {
    const uint8_t power_state = receive_single(this->i2c_handle, POWER_STATE);
    if (log) {
        ESP_LOGI(this->device_name, "Device is in state 0x%02X", power_state);
    }
    return power_state;
}

bool Amplifier::set_play() {
    write_pair(this->i2c_handle, DEVICE_CTRL_2, 0x03); // Set DEVICE_CTRL_2 to PLAY
    return get_power_state() == 0x03;
}
