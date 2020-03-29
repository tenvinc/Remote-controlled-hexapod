#ifndef __MPU9250_H__
#define __MPU9250_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <i2c.h>
#include <stdint.h>

#ifdef __cplusplus
}
#endif

/***************** MPU9250 Device Registers ************************/
/*** Gyroscope and Accelerometer **/
#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_SELF_TEST_Y_GYRO 0x01
#define MPU9250_SELF_TEST_Z_GYRO 0x02
#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F
#define MPU9250_XG_OFFSET_H 0x13
#define MPU9250_XG_OFFSET_L 0x14
#define MPU9250_YG_OFFSET_H 0x15
#define MPU9250_YG_OFFSET_L 0x16
#define MPU9250_ZG_OFFSET_H 0x17
#define MPU9250_ZG_OFFSET_L 0x18
#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D
#define MPU9250_LP_ACCEL_ODR 0x1E
#define MPU9250_WOM_THR 0x1F
#define MPU9250_FIFO_EN 0x23
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_SLV0_ADDR 0x25
#define MPU9250_I2C_SLV0_REG 0x26
#define MPU9250_I2C_SLV0_CTRL 0x27
#define MPU9250_I2C_SLV1_ADDR 0x28
#define MPU9250_I2C_SLV1_REG 0x29
#define MPU9250_I2C_SLV1_CTRL 0x2A
#define MPU9250_I2C_SLV2_ADDR 0x2B
#define MPU9250_I2C_SLV2_REG 0x2C
#define MPU9250_I2C_SLV2_CTRL 0x2D
#define MPU9250_I2C_SLV3_ADDR 0x2E
#define MPU9250_I2C_SLV3_REG 0x2F
#define MPU9250_I2C_SLV3_CTRL 0x30
#define MPU9250_I2C_SLV4_ADDR 0x31
#define MPU9250_I2C_SLV4_REG 0x32
#define MPU9250_I2C_SLV4_DO 0x33
#define MPU9250_I2C_SLV4_CTRL 0x34
#define MPU9250_I2C_SLV4_DI 0x35
#define MPU9250_I2C_MST_STATUS 0x36
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_I2C_SLV0_DO 0x63
#define MPU9250_I2C_SLV1_DO 0x64
#define MPU9250_I2C_SLV2_DO 0x65
#define MPU9250_I2C_SLV3_DO 0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET 0x68
#define MPU9250_MOT_DETECT_CTRL 0x69
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_FIFO_COUNTH 0x72
#define MPU9250_FIFO_COUNTL 0x73
#define MPU9250_FIFO_R_W 0x74
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_XA_OFFSET_H 0x77
#define MPU9250_XA_OFFSET_L 0x78
#define MPU9250_YA_OFFSET_H 0x7A
#define MPU9250_YA_OFFSET_L 0x7B
#define MPU9250_ZA_OFFSET_H 0x7D
#define MPU9250_ZA_OFFSET_L 0x7E
/** Magnetometer Registers **/
#define MPU9250_AK8963_I2C_ADDR 0x0C  // Magnetometer is on a separate die
/** Read only registers **/
#define MPU9250_MAG_WIA 0x00
#define MPU9250_MAG_INFO 0x01
#define MPU9250_MAG_ST1 0x02
#define MPU9250_MAG_HXL 0x03
#define MPU9250_MAG_HXH 0x04
#define MPU9250_MAG_HYL 0x05
#define MPU9250_MAG_HYH 0x06
#define MPU9250_MAG_HZL 0x07
#define MPU9250_MAG_HZH 0x08
#define MPU9250_MAG_ST2 0x09

#define MPU9250_MAG_ASAX 0x10
#define MPU9250_MAG_ASAY 0x11
#define MPU9250_MAG_ASAZ 0x12
/** Write/read **/
#define MPU9250_MAG_CNTL1 0x0A
#define MPU9250_MAG_CNTL2 0x0B
#define MPU9250_MAG_ASTC 0x0C
#define MPU9250_MAG_I2CDIS 0x0F
/***************** (END) MOU9250 Device Registers ************************/

#define G 9.807f

#define CLOCK_SEL_PLL 0x01
#define I2C_MST_EN 0X20
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
#define INT_RAW_RDY_EN 0x01
#define INT_DISABLE 0x00

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
  DLPF_BANDWIDTH_184HZ,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ,
  DLPF_BANDWIDTH_OFF  // this enum does not map to anything in the register map
} DlpfBandwidth;

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
  uint8_t dlpfBandwidth;
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
  float x;
  float y;
  float z;
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
  mpu9250_err_t setDlpfBW(uint8_t bandwidth);
  mpu9250_err_t enableDRInterrupt();  // enable Data Ready interrupt
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

 private:
  MPU9250Config_t config;
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