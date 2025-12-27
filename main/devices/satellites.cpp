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
    write_pair(this->i2c_handle, 0x33, 0x32); // Set data format to left-justified and 24-bit
    write_pair(this->i2c_handle, 0x4C, 0x60); // Set left channel to -24 dB
    write_pair(this->i2c_handle, 0x4D, 0x60); // Set right channel to -24 dB

    // 250Hz butterworth HPF
    uint8_t bq_coeffs[21] = { 0x30,
        0x07, 0xe8, 0x70, 0xf0,
        0xf0, 0x2f, 0x1e, 0x1f,
        0x07, 0xe8, 0x70, 0xf0,
        0x0f, 0xd0, 0x9c, 0x7f,
        0xf8, 0x2e, 0xd8, 0xbe
    };

    write_pair(this->i2c_handle, GOTO_PG, 0x00); // Go to page 0
    write_pair(this->i2c_handle, GOTO_BK, 0xAA); // Go to book AA
    write_pair(this->i2c_handle, GOTO_PG, 0x01); // Go to page 01
    i2c_master_transmit(this->i2c_handle, bq_coeffs, 21, -1); // Write BQ coeffs

    // Reset book state
    write_pair(this->i2c_handle, GOTO_PG, 0x00); // Go to page 0
    write_pair(this->i2c_handle, GOTO_BK, 0x00); // Go to book 0

}
