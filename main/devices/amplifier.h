#ifndef AMPLIFIER_H
#define AMPLIFIER_H

#include "device_component.h"
#include <cstdint>

class Amplifier : public DeviceComponent {
public:
    esp_err_t error_check() override;
    uint8_t get_power_state(bool log) const;
    bool set_play();

    Amplifier(i2c_master_bus_handle_t i2c_bus, const uint16_t device_address, const char *device_name) : DeviceComponent(i2c_bus, device_address, device_name) {}
};

#endif //AMPLIFIER_H
