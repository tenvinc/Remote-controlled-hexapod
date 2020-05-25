#ifndef __MPU9250_H__
#define __MPU9250_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <i2c.h>
#include <stdint.h>
#include "dev.h"

#ifdef __cplusplus
}
#endif

// Determines whether to use MPU9250 as master for comms with external chips
#define USE_MPU9250_MASTER_I2C 1  // MASTER_I2C needed for FIFO implementation

#define G 9.807f

#define CLOCK_SEL_PLL 0x01
#define I2C_MST_EN 0X20
#define I2C_MST_DIS 0x00
#define I2C_INT_BYPASS 0x02
#define I2C_MST_CLK 0x0D
#define PWR_RESET 0X80
#define ALL_SEN_EN 0x00
#define INT_PULSE_50US 0x00
#define I2C_READ_FLAG 0x80
#define I2C_SLV0_EN 0x80
#define AK8963_PWR_DOWN 0x00
#define AK8963_RESET 0x01
#define AK8963_FUSE_ROM 0x0F
#define AK8963_CNT_MEAS1 0x12
#define AK8963_CNT_MEAS2 0x16
#define AK8963_DATA_READY_MSK 0x02
#define AK8963_MAG_OVF_MSK 0x08
#define INT_RAW_RDY_EN 0x01
#define INT_DISABLE 0x00

#define MPU9250_FIFO_TEMP_EN_MSK 0x80
#define MPU9250_FIFO_GYRO_MSK 0x70
#define MPU9250_FIFO_ACCEL 0x08
#define MPU9250_FIFO_SLV2 0x04
#define MPU9250_FIFO_SLV1 0x02
#define MPU9250_FIFO_SLV0 0x01  // default to be used for AK8963

typedef enum {
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
} AccelRange;

typedef enum {
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
} GyroRange;

typedef enum {
  GYRO_DLPF_BW_250HZ = 0,        // Delay: 0.97ms, Fs: 8kHz.<BW: 4000Hz, delay: 0.04ms> for temperature  
  GYRO_DLPF_BW_184HZ = 1,        // Delay: 2.9ms, Fs: 1kHz.<BW: 188Hz, delay: 1.9ms> for temperature  
  GYRO_DLPF_BW_92HZ = 2,         // Delay: 3.9ms, Fs: 1kHz.<BW: 98Hz, delay: 2.8ms> for temperature  
  GYRO_DLPF_BW_41HZ = 3,         // Delay: 5.9ms, Fs: 1kHz.<BW: 42Hz, delay: 4.8ms> for temperature  
  GYRO_DLPF_BW_20HZ = 4,         // Delay: 9.9ms, Fs: 1kHz.<BW: 20Hz, delay: 8.3ms> for temperature  
  GYRO_DLPF_BW_10HZ = 5,         // Delay: 17.85ms, Fs: 1kHz.<BW: 10Hz, delay: 13.4ms> for temperature  
  GYRO_DLPF_BW_5HZ = 6,          // Delay: 33.84ms, Fs: 1kHz.<BW: 5Hz, delay: 18.6ms> for temperature  
  GYRO_DLPF_BW_3600HZ_FS8 = 7,   // Delay: 0.17ms, Fs: 8kHz.<BW: 4000Hz, delay: 0.04ms> for temperature  
  GYRO_DLPF_BW_8800HZ,           // Delay: 0.064ms, Fs: 32kHz. <BW: 4000Hz, delay: 0.04ms> for temperature  
  GYRO_DLPF_BW_3600HZ_FS32       // Delay: 0.11ms, Fs: 32kHz.<BW: 4000Hz, delay: 0.04ms> for temperature  
} DlpfBandwidthGyro;

typedef enum {
  ACCEL_DLPF_BW_460HZ = 0,      // Delay: 1.94ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_184HZ,          // Delay: 5.80ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_92HZ,           // Delay: 7.80ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_41HZ,           // Delay: 11.80ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_20HZ,           // Delay: 19.80ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_10HZ,           // Delay: 35.70ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_5HZ,            // Delay: 66.96ms, Noise: 250ug/rtHz, Rate: 1kHz
  ACCEL_DLPF_BW_1130HZ          // Delay: 0.75ms, Noise: 250ug/rtHz, Rate: 4kHz
} DlpfBandwidthAccel;

typedef enum {
  LP_ACCEL_ODR_0_24HZ = 0,  // 0_24 represents 0.24
  LP_ACCEL_ODR_0_49HZ = 1,
  LP_ACCEL_ODR_0_98HZ = 2,
  LP_ACCEL_ODR_1_95HZ = 3,
  LP_ACCEL_ODR_3_91HZ = 4,
  LP_ACCEL_ODR_7_81HZ = 5,
  LP_ACCEL_ODR_15_63HZ = 6,
  LP_ACCEL_ODR_31_25HZ = 7,
  LP_ACCEL_ODR_62_50HZ = 8,
  LP_ACCEL_ODR_125HZ = 9,
  LP_ACCEL_ODR_250HZ = 10,
  LP_ACCEL_ODR_500HZ = 11,
  LP_ACCEL_ODR_OFF =
      12,  // this enum does not map to anything in the register map
} LpAccODR;

typedef enum {
  FIFO_BUF_OVERFLOW_OVERWRITE,  // will overwrite oldest data
  FIFO_BUF_OVERFLOW_NO_WRITE
} FifoBufOverflow;

typedef enum {
  ACCX_DIS = 0x20,
  ACCY_DIS = 0x10,
  ACCZ_DIS = 0x08,
  GYROX_DIS = 0x04,
  GYROY_DIS = 0x02,
  GYROZ_DIS = 0x01
} AccelGyroDisable;

typedef enum {
  TEMP_EN = 0x80,
  GYROX_EN = 0x40,
  GYROY_EN = 0x20,
  GYROZ_EN = 0x10,
  GYRO_FULL_EN = 0x70,
  ACCEL_EN = 0x08
} FifoEnable;

typedef enum { MPU9250_ERR_OK, MPU9250_ERR } mpu9250_err_t;

typedef struct MPU9250Config {
  uint8_t accelDisabled;
  uint8_t gyroDisabled;
  uint8_t accelRange;
  uint8_t gyroRange;
  uint8_t lpAccODR;
  uint8_t gyroDlpfBandwidth;
  uint8_t accelDlpfBandwidth;
  uint8_t accelFIFOEnabled;
  uint8_t gyroFIFOEnabled;
  uint8_t fifoBufOverflow;
} MPU9250Config_t;

typedef struct MagScale {
  float x;
  float y;
  float z;
} MagScale_t;

typedef struct AccelRaw {
  int16_t x;
  int16_t y;
  int16_t z;
} AccelRaw_t;

typedef struct AccelFloat {
  float x;
  float y;
  float z;
} AccelFloat_t;

typedef struct GyroRaw {
  int16_t x;
  int16_t y;
  int16_t z;
} GyroRaw_t;

typedef struct GyroFloat {
  float x;
  float y;
  float z;
} GyroFloat_t;

typedef struct MagRaw {
  int16_t x;
  int16_t y;
  int16_t z;
} MagRaw_t;

typedef struct MagFloat {
  float x;
  float y;
  float z;
} MagFloat_t;

class MPU9250Core {
 protected:
  MPU9250Core(i2c_dev_t dev);
  int beginCore();
  mpu9250_err_t writeRegister(const uint8_t regAddr, const uint8_t data);
  mpu9250_err_t writeRegister(const uint8_t regAddr, const uint8_t *data);
  mpu9250_err_t readRegisters(const uint8_t *startAddr, uint8_t len,
                              uint8_t *buf);
  mpu9250_err_t readRegister(const uint8_t regAddr, uint8_t *buf);
#if !USE_MPU9250_MASTER_I2C
  mpu9250_err_t _writeAK8963Register(uint8_t regAddr, uint8_t data);
  mpu9250_err_t _readAK8963Registers(const uint8_t *startAddr, uint8_t len,
                                     uint8_t *buf);
#endif

 private:
  MPU9250Config_t config;
  i2c_dev_t dev;
};

class MPU9250 : public MPU9250Core {
 public:
  MPU9250(i2c_dev_t &i2c, MPU9250Config_t *pConfigToPassIn);
  mpu9250_err_t writeAK8963Register(uint8_t subAddr, uint8_t data);
  mpu9250_err_t readAK8963Registers(uint8_t subAddr, uint8_t count,
                                    uint8_t *buf);
  mpu9250_err_t readAK8963Register(uint8_t subAddr, uint8_t *buf);
  int begin();
  uint8_t whoAmI(void);
  uint8_t whoAmIAK8963(void);
  mpu9250_err_t setAccelRange(uint8_t range);
  mpu9250_err_t setGyroRange(uint8_t range);
  mpu9250_err_t setGyroDlpfBW(uint8_t gyroBW);
  mpu9250_err_t setAccelDlpfBW(uint8_t accelBW);
  mpu9250_err_t enableDRInterrupt();   // enable Data Ready interrupt
  mpu9250_err_t disableDRInterrupt();  // disable Data Ready Interrupt
  mpu9250_err_t readSensors();

  // Getters for sensor readings after a readSensor call
  float getXAccelMs2();
  float getYAccelMs2();
  float getZAccelMs2();
  float getXGyroDps();
  float getYGyroDps();
  float getZGyroDps();
  float getMagXuT();
  float getMagYuT();
  float getMagZuT();

  int16_t getXAccelRaw();
  int16_t getYAccelRaw();
  int16_t getZAccelRaw();
  int16_t getXGyroRaw();
  int16_t getYGyroRaw();
  int16_t getZGyroRaw();
  int16_t getXMagRaw();
  int16_t getYMagRaw();
  int16_t getZMagRaw();

  MPU9250Config_t config;
 private:
  float _accelScale;
  float _gyroScale;
  MagScale_t _magScale;
  uint8_t _buffer[64];
  AccelRaw_t _aRaw;
  AccelFloat_t _aFloat;
  GyroRaw_t _gRaw;
  GyroFloat_t _gFloat;
  MagRaw_t _mRaw;
  MagFloat_t _mFloat;
  int16_t _tRaw;
};

#endif