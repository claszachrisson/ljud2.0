#ifndef I2C_H
#define I2C_H

constexpr int DEFAULT_TIMEOUT = -1;

uint8_t receive_single(i2c_master_dev_handle_t device, const uint8_t write);
void write_pair(i2c_master_dev_handle_t device, const uint8_t key, const uint8_t value);

#endif //I2C_H
