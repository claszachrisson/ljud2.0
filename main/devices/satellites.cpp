#include "amplifier.h"
#include "regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../i2c.h"
#include "../uint3.h"
#include "esp_log.h"

static constexpr uint16_t SATELLITES_ADDR = 0x60;
static auto SATELLITES_TAG = "TAS5827";

class Satellites final : public Amplifier {
public:
    Satellites(i2c_master_bus_handle_t i2c_bus): Amplifier(i2c_bus, SATELLITES_ADDR, SATELLITES_TAG) {}
    ~Satellites() override = default;

    void init() override;
    void set_volume(uint3 volume);
};

void Satellites::init() {
    write_pair(this->i2c_handle, DEVICE_CTRL_2, 0x02); // Set power state to Hi-Z
    vTaskDelay(TICK_DELAY);
    write_pair(this->i2c_handle, 0x30, 0x01); // Set SDOUT to DSP input
    write_pair(this->i2c_handle, 0x60, 0x04); // Set GPIO2 to output
    write_pair(this->i2c_handle, 0x63, 0x09); // Set GPIO2 to output SDOUT
    write_pair(this->i2c_handle, 0x33, 0x32); // Set data format to left-justified and 24-bit
    set_volume(uint3(0));
}

uint8_t get_volume(uint3 volume);

void Satellites::set_volume(uint3 volume) {
    uint8_t vol = get_volume(volume);
    ESP_LOGI(SATELLITES_TAG, "Volume: %u", vol);
    write_pair(this->i2c_handle, 0x4C, vol); // Set left channel volume
    write_pair(this->i2c_handle, 0x4D, vol); // Set right channel volume
}

uint8_t get_volume(uint3 volume) {
    switch (volume.value) {
        case 0:
            return 255; // Mute
        case 1:
            return 120; // -30 dB
        case 2:
            return 112; // -28 dB
        case 3:
            return 104; // -26 dB
        case 4:
            return 96; // -24 dB
        case 5:
            return 88; // -22 dB
        case 6:
            return 80; // -20 dB
        case 7:
            return 72; // -18 dB
        default:
            return 96; // -24 dB
    }
}
