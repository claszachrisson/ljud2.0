#include "device_component.h"
#include "../i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static constexpr uint16_t CODEC_ADDR = 0x10;
static auto CODEC_TAG = "CODEC";

const int sourceRefreshRate = 250; // 250 ms
const int SOURCE_LOCK_MAX = 5000 / sourceRefreshRate;

class Codec : public DeviceComponent {
public:
    Codec(i2c_master_bus_handle_t i2c_bus) : DeviceComponent(i2c_bus, CODEC_ADDR, CODEC_TAG) {
    }

    int checkPLLLocked();

    int sourceIsActive();

    void changeSource();

    int lockActiveSource();

    void init() override;

private:
    int currentSource = 0;
    int sourceLockCounter = 0;
};

void Codec::init() {
    write_pair(this->i2c_handle, 0x04, 0x81); // Set state to RUN
    vTaskDelay(TICK_DELAY);
    write_pair(this->i2c_handle, 0x05, 0x80); // Set serial audio mode to master
    write_pair(this->i2c_handle, 0x06, 0x7F); // Set receiver error mask to ones (unmasked)
}

int Codec::checkPLLLocked() {
    uint8_t data[1];
    i2c_master_transmit_receive(this->i2c_handle, (uint8_t[]){0x0C}, 1, data, 1, -1);
    // Read register once to reset RERR
    i2c_master_transmit_receive(this->i2c_handle, (uint8_t[]){0x0C}, 1, data, 1, -1);
    ESP_LOGW(CODEC_TAG, "Receiver error: %02X", data[0]);
    if (data[0] & 16) {
        // UNLOCK bit is set
        return 0;
    }
    ESP_LOGI(CODEC_TAG, "PLL locked");
    return 1;
}

int Codec::sourceIsActive() {
    if (!checkPLLLocked()) return 0;
    uint8_t data[1];
    i2c_master_transmit_receive(this->i2c_handle, (uint8_t[]){0x0B}, 1, data, 1, -1);
    if (data[0] & 2) {
        // Digital silence is set;
        ESP_LOGW(CODEC_TAG, "Digital silence detected");
        return 0;
    }
    ESP_LOGI(CODEC_TAG, "Format Detect Status: 0x%02X, valid: %d", data[0], ((data[0] & 0xF0)));
    return (data[0] & 0xF0) > 1;
}

void Codec::changeSource() {
    if (currentSource) {
        // Current source is RXP1, set to RXP0
        i2c_master_transmit(this->i2c_handle, (uint8_t[]){0x04, 0x81}, 2, -1);
        currentSource = 0;
    } else {
        i2c_master_transmit(this->i2c_handle, (uint8_t[]){0x04, 0x89}, 2, -1);
        currentSource = 1;
    }
}

int Codec::lockActiveSource() {
    // Needs rewriting as PLL may be locked without an audio data stream
    // Either check codec for digital silence or !AUDIO on GPIO (needs configuring)
    if (sourceIsActive()) {
        if (sourceLockCounter < SOURCE_LOCK_MAX) sourceLockCounter++;
        return 1;
    }
    if (sourceLockCounter > 0) sourceLockCounter--;
    if (sourceLockCounter == 0) {
        changeSource();
        ESP_LOGI(CODEC_TAG, "Source changed");
    }
    return sourceIsActive();
}
