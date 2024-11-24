#ifndef DEVICE_COMPONENT_H
#define DEVICE_COMPONENT_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"

const TickType_t TICK_DELAY = 20 / portTICK_PERIOD_MS;

class DeviceComponent {

protected:
    uint16_t device_address;
    const char *device_name;
    i2c_master_dev_handle_t i2c_handle;
    i2c_master_bus_handle_t i2c_bus;

public:
    virtual ~DeviceComponent() = default;

    virtual int device_init();
    virtual void init();
    virtual esp_err_t error_check();
    esp_err_t i2c_probe() const;

    DeviceComponent(i2c_master_bus_handle_t i2c_bus, const uint16_t device_address, const char *device_name) {
        this->i2c_bus = i2c_bus;
        this->device_address = device_address;
        this->device_name = device_name;
    }
};


#endif //DEVICE_COMPONENT_H
