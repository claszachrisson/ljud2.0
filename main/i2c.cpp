#include <driver/i2c_master.h>
#include <driver/i2c_types.h>
#include "i2c.h"

uint8_t receive_single(i2c_master_dev_handle_t device, const uint8_t write) {
    uint8_t data[1];
    i2c_master_transmit_receive(device, (uint8_t[]){write}, 1, data, 1, DEFAULT_TIMEOUT);
    return data[0];
}

void write_pair(i2c_master_dev_handle_t device, const uint8_t key, const uint8_t value) {
    i2c_master_transmit(device, (uint8_t[]){key, value}, 2, DEFAULT_TIMEOUT);
}
