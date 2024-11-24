#include "amplifier.h"
#include "regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../i2c.h"

static constexpr uint16_t SATELLITES_ADDR = 0x60;
static auto SATELLITES_TAG = "TAS5827";

class Satellites final : public Amplifier {
public:
    Satellites(i2c_master_bus_handle_t i2c_bus): Amplifier(i2c_bus, SATELLITES_ADDR, SATELLITES_TAG) {}
    ~Satellites() override = default;

    void init() override;
};

void Satellites::init() {
    write_pair(this->i2c_handle, DEVICE_CTRL_2, 0x02); // Set power state to Hi-Z
    vTaskDelay(TICK_DELAY);
    write_pair(this->i2c_handle, 0x30, 0x01); // Set SDOUT to DSP input
    write_pair(this->i2c_handle, 0x60, 0x04); // Set GPIO2 to output
    write_pair(this->i2c_handle, 0x63, 0x09); // Set GPIO2 to output SDOUT
    write_pair(this->i2c_handle, 0x4C, 0x60); // Set left channel to -24 dB
    write_pair(this->i2c_handle, 0x4D, 0x60); // Set right channel to -24 dB
}
