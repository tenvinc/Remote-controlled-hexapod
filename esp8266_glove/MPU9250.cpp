#include "MPU9250.h"
#include "espressif/esp_common.h"

// Uncomment to get debugging messages
#define MPU9250_DEBUG

#ifdef MPU9250_DEBUG
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#ifdef __cplusplus
}
#endif
#define debug(fmt, ...) printf("%s: " fmt "\n", "MPU9250", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

MPU9250Core::MPU9250Core(i2c_dev_t dev) {
  this->dev = dev;
}

mpu9250_err_t MPU9250Core::writeRegister(const uint8_t regAddr, const uint8_t *data) {
  printf("Dev bus:%d, Addr: %x, Register: %x, Data: %d\n", dev.bus, dev.addr, regAddr, *data);
  int err = i2c_slave_write(dev.bus, dev.addr, &regAddr, data, 1);
  printf("Error code is %d\n", err);
  if (err != 0) {
    printf("Cannot even write properly lol.\n");
    return MPU9250_ERR;
  }
  sdk_os_delay_us(10000); // to let changes update

  uint8_t buf;
  uint8_t attempts = 0;
  while (attempts < 5) {
    if (readRegisters(&regAddr, 1, &buf) == 0) break;
    attempts++;
  }
  if (attempts >= 5 && buf != *data) {
    return MPU9250_ERR;
  }
  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250Core::readRegisters(const uint8_t *startAddr, uint8_t len, uint8_t *buf) {
  int err = i2c_slave_read(dev.bus, dev.addr, startAddr, buf, len);
  if (err != 0) {
    return MPU9250_ERR;
  }
  return MPU9250_ERR_OK;
}

MPU9250::MPU9250(i2c_dev_t &i2c, MPU9250Config_t *pConfigToPassIn)
: MPU9250Core(i2c)
{
  if (pConfigToPassIn == NULL) {
    config = MPU9250Config_t();
    config.accelDisabled = 0;
    config.gyroDisabled = 0;
    config.accelRange = ACCEL_RANGE_2G;
    config.gyroRange = GYRO_RANGE_250DPS;
    config.lpAccODR = LP_ACCEL_ODR_OFF;
    config.dlpfBandwidth = DLPF_BANDWIDTH_OFF;
    config.accelFIFOEnabled = 0;
  } else config = *pConfigToPassIn;
}

int MPU9250::begin() {
}