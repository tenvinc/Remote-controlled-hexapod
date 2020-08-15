#include "MPU9250.h"

#include <stdio.h>

#include "espressif/esp_common.h"

#define MPU9250_DEBUG 0

#if MPU9250_DEBUG
#define DBG1_PRINTF(fmt, ...) \
  printf("%s: " fmt "\n", "MPU9250", ##__VA_ARGS__)  // 1st level debug prints
#define DBG2_PRINTF(...) \
  //printf(__VA_ARGS__)  // 2nd level debug prints(newline not included)
#else
#define DBG1_PRINTF(...)
#define DBG2_PRINTF(fmt, ...)
#endif

#define GYRO_FCHOICEB_MSK 0x03
#define GYRO_DLPF_CFG_MSK 0x07
#define ACCEL_FCHOICEB_MSK 0x08
#define ACCEL_DLPF_CFG_MSK 0x07

const float _d2r = 3.14159265359f / 180.0f;

/* transform the accel and gyro axes to match the magnetometer axes */
const int16_t tX[3] = {0, 1, 0};
const int16_t tY[3] = {1, 0, 0};
const int16_t tZ[3] = {0, 0, -1};

MPU9250Core::MPU9250Core(i2c_dev_t dev) { this->dev = dev; }

mpu9250_err_t MPU9250Core::writeRegister(const uint8_t regAddr,
                                         const uint8_t data) {
  return writeRegister(regAddr, &data);
}

mpu9250_err_t MPU9250Core::writeRegister(const uint8_t regAddr,
                                         const uint8_t *data) {
  DBG2_PRINTF("WRITING Dev bus:%d, Addr: %x, Register: %x, Data: %x\n", dev.bus,
              dev.addr, regAddr, *data);
  int err = i2c_slave_write(dev.bus, dev.addr, &regAddr, data, 1);
  if (err != 0) {
    DBG2_PRINTF("Error in writeRegister op.\n");
    return MPU9250_ERR;
  }
  sdk_os_delay_us(1000);  // to let changes update

  uint8_t buf;
  uint8_t attempts = 0;
  while (attempts < 5) {
    if (readRegister(regAddr, &buf) == 0) break;
    attempts++;
  }
  if (attempts >= 5 || buf != *data) {
    DBG2_PRINTF("Not able to find a match %d, %x.\n", attempts, buf);
    return MPU9250_ERR;
  }
  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250Core::readRegisters(const uint8_t *startAddr, uint8_t len,
                                         uint8_t *buf) {
  int err = i2c_slave_read(dev.bus, dev.addr, startAddr, buf, len);
  DBG2_PRINTF("%d bytes read from reg %x: %x.\n", len, *startAddr, *buf);
  return err != 0 ? MPU9250_ERR : MPU9250_ERR_OK;
}

#if !USE_MPU9250_MASTER_I2C
mpu9250_err_t MPU9250Core::_writeAK8963Register(uint8_t regAddr, uint8_t data) {
  DBG2_PRINTF("WRITING Dev bus:%d, Addr: %x, Register: %x, Data: %x\n", dev.bus,
              MPU9250_AK8963_I2C_ADDR, regAddr, data);
  int err =
      i2c_slave_write(dev.bus, MPU9250_AK8963_I2C_ADDR, &regAddr, &data, 1);
  if (err != 0) {
    DBG2_PRINTF("Error in writeRegister op.\n");
    return MPU9250_ERR;
  }
  sdk_os_delay_us(1000);  // to let changes update

  uint8_t buf;
  uint8_t attempts = 0;
  while (attempts < 5) {
    if (_readAK8963Registers(&regAddr, 1, &buf) == 0) break;
    attempts++;
  }
  if (attempts >= 5 || buf != data) {
    return MPU9250_ERR;
  }
  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250Core::_readAK8963Registers(const uint8_t *startAddr,
                                                uint8_t len, uint8_t *buf) {
  int err =
      i2c_slave_read(dev.bus, MPU9250_AK8963_I2C_ADDR, startAddr, buf, len);
  DBG2_PRINTF("%d bytes read from reg %x.\n", len, *startAddr);
  return err != 0 ? MPU9250_ERR : MPU9250_ERR_OK;
}
#endif

mpu9250_err_t MPU9250Core::readRegister(const uint8_t regAddr, uint8_t *buf) {
  return readRegisters(&regAddr, 1, buf);
}

#if USE_MPU9250_MASTER_I2C
mpu9250_err_t MPU9250::writeAK8963Register(uint8_t subAddr, uint8_t data) {
  uint8_t original;
  DBG2_PRINTF("Write AK8963: %x, %x\n", subAddr, data);
  // to stop transmitting i2c messages when changing values
  if (readRegister(MPU9250_USER_CTRL, &original) != 0) {
    return MPU9250_ERR;
  }
  original &= ~I2C_MST_EN;
  writeRegister(MPU9250_USER_CTRL, original);

  // set slave 0 to the AK8963 and set for write
  if (writeRegister(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR) != 0) {
    return MPU9250_ERR;
  }
  // set the register to the desired AK8963 sub address
  if (writeRegister(MPU9250_I2C_SLV0_REG, subAddr) != 0) {
    return MPU9250_ERR;
  }
  // store the data for write
  if (writeRegister(MPU9250_I2C_SLV0_DO, data) != 0) {
    return MPU9250_ERR;
  }
  // enable I2C and send 1 byte
  if (writeRegister(MPU9250_I2C_SLV0_CTRL, I2C_SLV0_EN | (uint8_t)1) != 0) {
    return MPU9250_ERR;
  }

  // Renable transmission
  if (readRegister(MPU9250_USER_CTRL, &original) != 0) {
    return MPU9250_ERR;
  }
  original |= I2C_MST_EN;
  writeRegister(MPU9250_USER_CTRL, original);

  sdk_os_delay_us(1000);

  // read the register and confirm
  if (readAK8963Registers(subAddr, 1, _buffer) != 0) {
    return MPU9250_ERR;
  }
  DBG2_PRINTF("Double check: %x.\n", _buffer[0]);
  return _buffer[0] == data ? MPU9250_ERR_OK : MPU9250_ERR;
}

mpu9250_err_t MPU9250::readAK8963Registers(uint8_t subAddr, uint8_t count,
                                           uint8_t *buf) {
  uint8_t original;
  DBG2_PRINTF("Read AK8963: %x, %d\n", subAddr, count);

  // to stop transmitting i2c messages when changing values
  if (readRegister(MPU9250_USER_CTRL, &original) != 0) {
    return MPU9250_ERR;
  }
  original &= ~I2C_MST_EN;
  writeRegister(MPU9250_USER_CTRL, original);

  if (writeRegister(MPU9250_I2C_SLV0_ADDR,
                    MPU9250_AK8963_I2C_ADDR | I2C_READ_FLAG) != 0) {
    return MPU9250_ERR;
  }
  // set the register to the desired AK8963 sub address
  if (writeRegister(MPU9250_I2C_SLV0_REG, subAddr) != 0) {
    return MPU9250_ERR;
  }
  // enable I2C and request the bytes
  if (writeRegister(MPU9250_I2C_SLV0_CTRL, I2C_SLV0_EN | count) != 0) {
    return MPU9250_ERR;
  }

  // Renable transmission
  if (readRegister(MPU9250_USER_CTRL, &original) != 0) {
    return MPU9250_ERR;
  }
  original |= I2C_MST_EN;
  writeRegister(MPU9250_USER_CTRL, original);

  sdk_os_delay_us(1000);  // takes some time for these registers to fill

  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  uint8_t startAddr = MPU9250_EXT_SENS_DATA_00;
  return readRegisters(&startAddr, count, buf);
}
#else  // Use only in bypass mode
mpu9250_err_t MPU9250::writeAK8963Register(uint8_t subAddr, uint8_t data) {
  return _writeAK8963Register(subAddr, data);
}

mpu9250_err_t MPU9250::readAK8963Registers(uint8_t subAddr, uint8_t count,
                                           uint8_t *buf) {
  return _readAK8963Registers(&subAddr, count, buf);
}
#endif

mpu9250_err_t MPU9250::readAK8963Register(uint8_t subAddr, uint8_t *buf) {
  return readAK8963Registers(subAddr, 1, buf);
}

MPU9250::MPU9250(i2c_dev_t &i2c, MPU9250Config_t *pConfigToPassIn)
    : MPU9250Core(i2c) {
  if (pConfigToPassIn == NULL) {
    config = MPU9250Config_t();
    config.accelDisabled = 0;
    config.gyroDisabled = 0;
    config.accelRange = ACCEL_RANGE_2G;
    config.gyroRange = GYRO_RANGE_250DPS;
    config.lpAccODR = LP_ACCEL_ODR_OFF;
    config.gyroDlpfBandwidth = GYRO_DLPF_BW_184HZ;
    config.accelDlpfBandwidth = ACCEL_DLPF_BW_184HZ;
    config.accelFIFOEnabled = 0;
  } else
    config = *pConfigToPassIn;
}

int MPU9250::begin() {
  // SW reset of both MPU9250 and AK8963
  DBG1_PRINTF("Resetting both devices.");
#if USE_MPU9250_MASTER_I2C
  writeRegister(MPU9250_USER_CTRL, (uint8_t)I2C_MST_EN);
  writeAK8963Register(MPU9250_MAG_CNTL1, AK8963_PWR_DOWN);
  writeRegister(MPU9250_PWR_MGMT_1, (uint8_t)PWR_RESET);
  sdk_os_delay_us(100 * 1000);  // wait for reset to finish
  writeAK8963Register(MPU9250_MAG_CNTL2, AK8963_RESET);
#else
  if (writeRegister(MPU9250_USER_CTRL, (uint8_t)I2C_MST_DIS) != 0) {
    return MPU9250_ERR;
  }
  if (writeRegister(MPU9250_INT_PIN_CFG, (uint8_t)I2C_INT_BYPASS) != 0) {
    return MPU9250_ERR;
  }
  writeAK8963Register(MPU9250_MAG_CNTL1, AK8963_PWR_DOWN);
  sdk_os_delay_us(100 * 1000);  // wait for reset to finish
  writeAK8963Register(MPU9250_MAG_CNTL2, AK8963_RESET);
  writeRegister(MPU9250_PWR_MGMT_1, (uint8_t)PWR_RESET);
#endif

// Enable I2C communication with AK8963
#if USE_MPU9250_MASTER_I2C
  // Enable I2C master mode (needed for AK8963)
  if (writeRegister(MPU9250_USER_CTRL, (uint8_t)I2C_MST_EN) != 0) {
    DBG1_PRINTF("Step COMMS 1a failed.");
    return MPU9250_ERR;
  }
  // Set clock speed of I2C to 400kHz
  if (writeRegister(MPU9250_I2C_MST_CTRL, (uint8_t)I2C_MST_CLK) != 0) {
    DBG1_PRINTF("Step COMMS 1b failed.");
    return MPU9250_ERR;
  }
#else
  if (writeRegister(MPU9250_USER_CTRL, (uint8_t)I2C_MST_DIS) != 0) {
    DBG1_PRINTF("Step COMMS 1a failed.");
    return MPU9250_ERR;
  }
  if (writeRegister(MPU9250_INT_PIN_CFG, (uint8_t)I2C_INT_BYPASS) != 0) {
    DBG1_PRINTF("Step COMMS 1b failed.");
    return MPU9250_ERR;
  }
#endif
  // Do a who am I check
  if (whoAmI() != 0x71 || whoAmIAK8963() != 0x48) {
    DBG1_PRINTF("Step WAI failed.");
    return MPU9250_ERR;
  }

  // /** Initalize settings for main unit **/
  // // Select clock automatically and enable all sensors
  if (writeRegister(MPU9250_PWR_MGMT_1, (uint8_t)CLOCK_SEL_PLL) != 0) {
    DBG1_PRINTF("Step CLK_SEL failed.");
    return MPU9250_ERR;
  }

  if (writeRegister(MPU9250_PWR_MGMT_2, (uint8_t)ALL_SEN_EN) != 0) {
    DBG1_PRINTF("Step SEN_EN failed.");
    return MPU9250_ERR;
  }

  if (setAccelRange(config.accelRange) != 0) {
    DBG1_PRINTF("Step ACCEL_RANGE failed.");
    return MPU9250_ERR;
  };

  if (setGyroRange(config.gyroRange) != 0) {
    DBG1_PRINTF("Step GYRO_RANGE failed.");
    return MPU9250_ERR;
  };

  /** Setup magnetometer **/
  writeAK8963Register(MPU9250_MAG_CNTL1, AK8963_PWR_DOWN);
  sdk_os_delay_us(100 * 1000);
  if (writeAK8963Register(MPU9250_MAG_CNTL1, AK8963_FUSE_ROM) != 0) {
    DBG1_PRINTF("Step FUSE_ROM failed.\n");
    return MPU9250_ERR;
  }

  sdk_os_delay_us(100 * 1000);  // long wait between AK8963 mode changes
  if (readAK8963Registers(MPU9250_MAG_ASAX, 3, _buffer) != 0) {
    DBG1_PRINTF("Step MAG_ASA failed.\n");
    return MPU9250_ERR;
  }

  // read the AK8963 ASA registers and compute magnetometer scale factors
  _magScale.x = ((((float)_buffer[0]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f /
                32760.0f;  // micro Tesla
  _magScale.y = ((((float)_buffer[1]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f /
                32760.0f;  // micro Tesla
  _magScale.z = ((((float)_buffer[2]) - 128.0f) / (256.0f) + 1.0f) * 4912.0f /
                32760.0f;  // micro Tesla

  // Power down then activate continuous 16 bit mode
  writeAK8963Register(MPU9250_MAG_CNTL1, AK8963_PWR_DOWN);
  sdk_os_delay_us(100 * 1000);  // long wait between AK8963 mode changes
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if (writeAK8963Register(MPU9250_MAG_CNTL1, AK8963_CNT_MEAS2) != 0) {
    DBG1_PRINTF("Step MAG_MEAS failed.\n");
    return MPU9250_ERR;
  }
  sdk_os_delay_us(100 * 1000);  // long wait between AK8963 mode change

  // Change sampling frequency
  if (writeRegister(MPU9250_SMPLRT_DIV, (uint8_t)0x00) != 0) {
    DBG1_PRINTF("Step SMPLRT failed.");
    return MPU9250_ERR;
  }
  if (setAccelDlpfBW(config.accelDlpfBandwidth) != 0) {
    DBG1_PRINTF("Step ACCEL_DLPF failed.");
    return MPU9250_ERR;
  }
  if (setGyroDlpfBW(config.gyroDlpfBandwidth) != 0) {
    DBG1_PRINTF("Step GYRO_DLPF failed.");
    return MPU9250_ERR;
  }

  // Instruct MPU9250 to keep reading mag readings at sample rate
  readAK8963Registers(MPU9250_MAG_HXL, 7, _buffer);

  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250::setAccelRange(uint8_t range) {
  float multiplier;
  switch (range) {
    case ACCEL_RANGE_2G:
      multiplier = 2.0f;
      break;
    case ACCEL_RANGE_4G:
      multiplier = 4.0f;
      break;
    case ACCEL_RANGE_8G:
      multiplier = 8.0f;
      break;
    case ACCEL_RANGE_16G:
      multiplier = 16.0f;
      break;
    default:
      DBG2_PRINTF("Invalid range for accel given.\n");
      return MPU9250_ERR;
  }

  if (writeRegister(MPU9250_ACCEL_CONFIG,
                    (uint8_t)(config.accelRange << 3) != 0)) {
    return MPU9250_ERR;
  }
  _accelScale = G * multiplier / 32767.5f;
  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250::setGyroRange(uint8_t range) {
  float multiplier;
  switch (range) {
    case GYRO_RANGE_250DPS:
      multiplier = 250.0f;
      break;
    case GYRO_RANGE_500DPS:
      multiplier = 500.0f;
      break;
    case GYRO_RANGE_1000DPS:
      multiplier = 1000.0f;
      break;
    case GYRO_RANGE_2000DPS:
      multiplier = 2000.0f;
      break;
    default:
      DBG2_PRINTF("Invalid range for gyro given.\n");
      return MPU9250_ERR;
  }

  uint8_t value;
  readRegister(MPU9250_GYRO_CONFIG, &value);
  if (writeRegister(MPU9250_GYRO_CONFIG, (value | (range << 3))) != 0) {
    return MPU9250_ERR;
  }
  _gyroScale =
      multiplier / 32767.5f * _d2r;  // normalize gyro scale in degrees per step
  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250::setGyroDlpfBW(uint8_t gyroBW) {
  uint8_t readvalue;
  switch (gyroBW) {
    case GYRO_DLPF_BW_8800HZ:
      if (readRegister(MPU9250_GYRO_CONFIG, &readvalue) != 0)
        return MPU9250_ERR;
      readvalue |= 0x01;  // set f_choice_b[0]
      if (writeRegister(MPU9250_GYRO_CONFIG, &readvalue) != 0)
        return MPU9250_ERR;
      break;
    case GYRO_DLPF_BW_3600HZ_FS32:
      if (readRegister(MPU9250_GYRO_CONFIG, &readvalue) != 0)
        return MPU9250_ERR;
      readvalue = (readvalue & ~GYRO_FCHOICEB_MSK) | 0x01;
      if (writeRegister(MPU9250_GYRO_CONFIG, &readvalue) != 0)
        return MPU9250_ERR;
      break;
    case GYRO_DLPF_BW_250HZ:
    case GYRO_DLPF_BW_184HZ:
    case GYRO_DLPF_BW_92HZ:
    case GYRO_DLPF_BW_41HZ:
    case GYRO_DLPF_BW_20HZ:
    case GYRO_DLPF_BW_10HZ:
    case GYRO_DLPF_BW_5HZ:
    case GYRO_DLPF_BW_3600HZ_FS8:
      // change f_choice_b
      if (readRegister(MPU9250_GYRO_CONFIG, &readvalue) != 0)
        return MPU9250_ERR;
      readvalue &= ~GYRO_FCHOICEB_MSK;
      if (writeRegister(MPU9250_GYRO_CONFIG, &readvalue) != 0)
        return MPU9250_ERR;
      // change dlpf_cfg
      if (readRegister(MPU9250_CONFIG, &readvalue) != 0) return MPU9250_ERR;
      readvalue = (readvalue & ~GYRO_DLPF_CFG_MSK) | gyroBW;
      if (writeRegister(MPU9250_CONFIG, &readvalue) != 0) return MPU9250_ERR;
      break;
    default:
      DBG2_PRINTF("Invalid Gyro DLPF value.\n");
      return MPU9250_ERR;
  }
  return MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250::setAccelDlpfBW(uint8_t accelBW) {
  uint8_t readvalue;
  switch (accelBW) {
    case ACCEL_DLPF_BW_1130HZ:
      if (readRegister(MPU9250_ACCEL_CONFIG_2, &readvalue) != 0)
        return MPU9250_ERR;
      readvalue |= ACCEL_FCHOICEB_MSK;
      if (writeRegister(MPU9250_ACCEL_CONFIG_2, &readvalue) != 0)
        return MPU9250_ERR;
      break;
    case ACCEL_DLPF_BW_460HZ:
    case ACCEL_DLPF_BW_184HZ:
    case ACCEL_DLPF_BW_92HZ:
    case ACCEL_DLPF_BW_41HZ:
    case ACCEL_DLPF_BW_20HZ:
    case ACCEL_DLPF_BW_10HZ:
    case ACCEL_DLPF_BW_5HZ:
      if (readRegister(MPU9250_ACCEL_CONFIG_2, &readvalue) != 0)
        return MPU9250_ERR;
      readvalue &= ACCEL_FCHOICEB_MSK;
      readvalue = (readvalue & ~ACCEL_DLPF_CFG_MSK) | accelBW;
      if (writeRegister(MPU9250_ACCEL_CONFIG_2, &readvalue) != 0)
        return MPU9250_ERR;
      break;
    default:
      DBG2_PRINTF("Invalid Accel DLPF value.\n");
      return MPU9250_ERR;
  }
  return MPU9250_ERR_OK;
}

uint8_t MPU9250::whoAmI() {
  uint8_t res;
  if (readRegister(MPU9250_WHO_AM_I, &res) != 0) {
    return 0xFF;
  }
  return res;
}

uint8_t MPU9250::whoAmIAK8963() {
  uint8_t res;
  // read the WHO AM I register
  if (readAK8963Registers(MPU9250_MAG_WIA, 1, &res) != 0) {
    return 0xFF;
  }
  // return the register value
  return res;
}

mpu9250_err_t MPU9250::enableDRInterrupt() {
  uint8_t err;
  err = writeRegister(MPU9250_INT_PIN_CFG, (uint8_t)INT_PULSE_50US);
  err = writeRegister(MPU9250_INT_ENABLE, (uint8_t)INT_RAW_RDY_EN);
  return (err != 0) ? MPU9250_ERR : MPU9250_ERR_OK;
}

mpu9250_err_t MPU9250::disableDRInterrupt() {
  return writeRegister(MPU9250_INT_ENABLE, (uint8_t)INT_DISABLE);
}

mpu9250_err_t MPU9250::readSensors() {
  // Start from ACCEL_XOUT_H and read till EXT_SENS_DATA_07
  uint8_t startReg = MPU9250_ACCEL_XOUT_H;
  bool magValid = true;
  bool accgygroValid = true;  // Accelerometer and gyroscope

#if USE_MPU9250_MASTER_I2C
  if (readRegisters(&startReg, 21, _buffer)) {
    return MPU9250_ERR;
  }
#else
  if (readRegisters(&startReg, 14, _buffer) != 0) {
    return MPU9250_ERR;
  }
  uint8_t rdy_flag;
  // Only read data when it is ready
  if (readAK8963Register(MPU9250_MAG_ST1, &rdy_flag) != 0) {
    return MPU9250_ERR;
  } else if (rdy_flag & AK8963_DATA_READY_MSK) {
    uint8_t ovf_flag;
    if (readAK8963Registers(MPU9250_MAG_HXL, 6, _buffer + 14) != 0) {
      return MPU9250_ERR;
    }
    if (readAK8963Register(MPU9250_MAG_ST2, &ovf_flag) != 0) {
      return MPU9250_ERR;
    } else if (ovf_flag & AK8963_MAG_OVF_MSK != 0) {
      magValid = false;
    }
  } else {
    magValid = false;
  }
#endif

  if (accgygroValid) {
    _aRaw.x = (((int16_t)_buffer[0]) << 8) | _buffer[1];
    _aRaw.y = (((int16_t)_buffer[2]) << 8) | _buffer[3];
    _aRaw.z = (((int16_t)_buffer[4]) << 8) | _buffer[5];
    _tRaw = (((int16_t)_buffer[6]) << 8) | _buffer[7];
    _gRaw.x = (((int16_t)_buffer[8]) << 8) | _buffer[9];
    _gRaw.y = (((int16_t)_buffer[10]) << 8) | _buffer[11];
    _gRaw.z = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  }
  if (magValid) {
    _mRaw.x = (((int16_t)_buffer[15]) << 8) | _buffer[14];
    _mRaw.y = (((int16_t)_buffer[17]) << 8) | _buffer[16];
    _mRaw.z = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  }

  // transform and convert to float values
  _aFloat.x = (float)(tX[0] * _aRaw.x + tX[1] * _aRaw.y + tX[2] * _aRaw.z) *
              _accelScale;
  _aFloat.y = (float)(tY[0] * _aRaw.x + tY[1] * _aRaw.y + tY[2] * _aRaw.z) *
              _accelScale;
  _aFloat.z = (float)(tZ[0] * _aRaw.x + tZ[1] * _aRaw.y + tZ[2] * _aRaw.z) *
              _accelScale;
  _gFloat.x =
      (float)(tX[0] * _gRaw.x + tX[1] * _gRaw.y + tX[2] * _gRaw.z) * _gyroScale;
  _gFloat.y =
      (float)(tY[0] * _gRaw.x + tY[1] * _gRaw.y + tY[2] * _gRaw.z) * _gyroScale;
  _gFloat.z =
      (float)(tZ[0] * _gRaw.x + tZ[1] * _gRaw.y + tZ[2] * _gRaw.z) * _gyroScale;
  _mFloat.x = (float)(_mRaw.x) * _magScale.x;
  _mFloat.y = (float)(_mRaw.y) * _magScale.y;
  _mFloat.z = (float)(_mRaw.z) * _magScale.z;

  return MPU9250_ERR_OK;
}

float MPU9250::getXAccelMs2() { return _aFloat.x; }
float MPU9250::getYAccelMs2() { return _aFloat.y; }
float MPU9250::getZAccelMs2() { return _aFloat.z; }
float MPU9250::getXGyroDps() { return _gFloat.x; }
float MPU9250::getYGyroDps() { return _gFloat.y; }
float MPU9250::getZGyroDps() { return _gFloat.z; }
float MPU9250::getMagXuT() { return _mFloat.x; }
float MPU9250::getMagYuT() { return _mFloat.y; }
float MPU9250::getMagZuT() { return _mFloat.z; }

int16_t MPU9250::getXAccelRaw() { return _aRaw.x; }
int16_t MPU9250::getYAccelRaw() { return _aRaw.y; }
int16_t MPU9250::getZAccelRaw() { return _aRaw.z; }
int16_t MPU9250::getXGyroRaw() { return _gRaw.x; }
int16_t MPU9250::getYGyroRaw() { return _gRaw.y; }
int16_t MPU9250::getZGyroRaw() { return _gRaw.z; }
int16_t MPU9250::getXMagRaw() { return _mRaw.x; }
int16_t MPU9250::getYMagRaw() { return _mRaw.y; }
int16_t MPU9250::getZMagRaw() { return _mRaw.z; }