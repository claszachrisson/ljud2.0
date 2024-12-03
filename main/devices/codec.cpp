#include "device_component.h"
#include "../i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "codec_regs.h"

static constexpr uint16_t CODEC_ADDR = 0x10;
static auto CODEC_TAG = "CODEC";

constexpr int source_refresh_rate = 250; // 250 ms
constexpr int SOURCE_LOCK_MAX = 5000 / source_refresh_rate;

typedef enum
{
    CODEC_SOURCE_UNLOCK = -1,
    CODEC_SOURCE_LOCK = 0,
    CODEC_SOURCE_INACTIVE = 1,
    CODEC_SOURCE_ACTIVE = 2,
} codec_source_state_t;

typedef enum {
    CODEC_SOURCE_EVT = 0,
    CODEC_RECEIVER_ERR_EVT,
} codec_cb_event_t;

typedef union {
    struct codec_source_state_param {
        int source;
        codec_source_state_t state;
    } source_state;
    uint8_t error_state;
} codec_event_param_t;


class Codec : public DeviceComponent {
public:
    Codec(i2c_master_bus_handle_t i2c_bus) : DeviceComponent(i2c_bus, CODEC_ADDR, CODEC_TAG) {
    }

    int check_pll_locked(bool log);

    int source_is_active();

    void change_source();

    int lock_active_source();

    void init() override;

    void register_event_callback(void (*event_callback)(codec_cb_event_t event, codec_event_param_t *param));

private:
    int current_source = 0;
    int source_lock_counter = 0;
    codec_source_state_t source_state = CODEC_SOURCE_UNLOCK;
    int previous_event_source = current_source;
    void (*event_callback);
    void state_update(codec_source_state_t state);
};

void Codec::init() {
    write_pair(this->i2c_handle, CONTROL_4, 0x81); // Set state to RUN
    vTaskDelay(TICK_DELAY);
    write_pair(this->i2c_handle, SERIAL_AUDIO_FORMAT, 0x80); // Set serial audio mode to master
    write_pair(this->i2c_handle, 0x01, 0x04); // Set HOLD to "replace the current audio sample with all zeros"
    write_pair(this->i2c_handle, RECEIVER_ERROR_MASK, 0x7F); // Set receiver error mask to ones (unmasked)
}

int Codec::check_pll_locked(const bool log = false) {
    // Codec errors are sticky and have to be read twice in order to ensure they still are valid
    receive_single(this->i2c_handle, RECEIVER_ERROR);
    const uint8_t err = receive_single(this->i2c_handle, RECEIVER_ERROR);
    if (err != 0x0) {
        ESP_LOGW(CODEC_TAG, "Receiver error: %02X", err);
        codec_event_param_t param = {.error_state = err};
        event_callback(CODEC_RECEIVER_ERR_EVT, &param);
    }

    if (err & 0x10) {
        // UNLOCK bit is set
        state_update(CODEC_SOURCE_UNLOCK);
        return 0;
    }
    if (log)
        ESP_LOGI(CODEC_TAG, "PLL locked");

    state_update(CODEC_SOURCE_LOCK);
    return 1;
}

int Codec::source_is_active() {
    if (!check_pll_locked()) {
        return 0;
    }

    const uint8_t format = receive_single(this->i2c_handle, AUDIO_FORMAT_DETECT);
    if (format & 2) {
        // Digital silence is set;
        ESP_LOGI(CODEC_TAG, "Digital silence detected");
        state_update(CODEC_SOURCE_INACTIVE);
        return 0;
    }
    ESP_LOGD(CODEC_TAG, "Format Detect Status: 0x%02X", format);
    uint8_t bits = format & 0x78;
    if (bits && !(bits & (bits-1))) { // Exactly one bit is set
        state_update(CODEC_SOURCE_ACTIVE);
        return 1;
    }
    return -1;
}

void Codec::change_source() {
    if (current_source) {
        // Current source is RXP1, set to RXP0
        write_pair(this->i2c_handle, CONTROL_4, SOURCE_RXP0);
        current_source = 0;
    } else {
        write_pair(this->i2c_handle, CONTROL_4, SOURCE_RXP1);
        current_source = 1;
    }
    ESP_LOGI(CODEC_TAG, "Source changed");
}

int Codec::lock_active_source() {
    // Needs rewriting as PLL may be locked without an audio data stream
    // Either check codec for digital silence or !AUDIO on GPIO (needs configuring)
    if (source_is_active()) {
        if (source_lock_counter < SOURCE_LOCK_MAX) source_lock_counter++;
        return 1;
    }
    if (source_lock_counter > 0) source_lock_counter--;
    if (source_lock_counter == 0) {
        change_source();
    }
    return source_is_active();
}

void Codec::register_event_callback(void (*event_callback)(codec_cb_event_t event, codec_event_param_t *param)){
    this->event_callback = event_callback;
}

void Codec::state_update(codec_source_state_t state)
{
    // Only send callback when state is changed, or source has updated since previous event
    if (source_state != state || previous_event_source != current_source) {
        source_state = state;
        previous_event_source = current_source;
        codec_event_param_t param = {.source_state = {
            .source = current_source,
            .state = source_state
        }};
        event_callback(CODEC_SOURCE_EVT, &param);
    }
}
