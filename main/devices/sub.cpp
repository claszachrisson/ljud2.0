#include "amplifier.h"
#include "freertos/task.h"
#include "regs.h"
#include "../i2c.h"

static constexpr uint16_t SUB_ADDR = 0x2C;
static auto SUB_TAG = "TAS5805M";

class Sub final : public Amplifier {
public:
    Sub(i2c_master_bus_handle_t i2c_bus): Amplifier(i2c_bus, SUB_ADDR, SUB_TAG) {}
    ~Sub() override = default;

    void init() override;
};

void Sub::init() {
    write_pair(this->i2c_handle, DEVICE_CTRL_2, 0x02); // Set power state to Hi-Z
    vTaskDelay(TICK_DELAY);
    write_pair(this->i2c_handle, DEVICE_CTRL_1, 0x04); // Set DAMP to PBTL mode
    write_pair(this->i2c_handle, 0x4C, 0x48); // Set digital volume to -12 dB

    // 100Hz butterworth LPF
    uint8_t bq_coeffs[21] = { 0x18,
        0x00, 0x00, 0x05, 0x97,
        0x00, 0x00, 0x0b, 0x2d,
        0x00, 0x00, 0x05, 0x97,
        0x0f, 0xed, 0x0b, 0x39,
        0xf8, 0x12, 0xde, 0x06
    };

    write_pair(this->i2c_handle, GOTO_PG, 0x00); // Go to page 0
    write_pair(this->i2c_handle, GOTO_BK, 0xAA); // Go to book AA
    write_pair(this->i2c_handle, GOTO_PG, 0x24); // Go to page 24
    i2c_master_transmit(this->i2c_handle, bq_coeffs, 21, -1); // Write BQ coeffs

    // Reset book state
    write_pair(this->i2c_handle, GOTO_PG, 0x00); // Go to page 0
    write_pair(this->i2c_handle, GOTO_BK, 0x00); // Go to book 0
}