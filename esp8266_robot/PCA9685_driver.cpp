#include "PCA9685_driver.h"

extern "C" {
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "espressif/esp_common.h"
#include "i2c.h"
}
#define DEBUG 0

#define LOG_PRINTF(...) printf(__VA_ARGS__)
#if DEBUG
#define DEBUG_PRINTF(fmt, ...) printf("%s: " fmt "\n", "PCA9685", ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(fmt, ...)
#endif

#define INVALID_VAL 255  // encodes values which are not valid for servo list

#define SERVO_FREQ 50
#define SERVO_PERIOD 20000  // In us, calculated from SERVO_FREQ

#define PCA9685_I2C_ADDR 0x40
#define DEFAULT_SCL_PIN 5
#define DEFAULT_SDA_PIN 4
#define DEFAULT_BUS 0

/** REGISTER CONFIG MACROS **/
/** MODE1 **/
#define MODE1_RESTART_EN 0x80
#define MODE1_EXTCLK_EN 0X40
#define MODE1_AI_EN 0x20
#define MODE1_LOW_POW 0x10
#define MODE1_SUB1_RESP 0x08
#define MODE1_SUB2_RESP 0x04
#define MODE1_SUB3_RESP 0x02
#define MODE1_ALLCALL_RESP 0x01

/** MODE2 **/
#define MODE2_TOTEMPOLE 0x04

uint8_t data[16];

/******************* Internal functions **************************/
pca9685_err_t readRegisters(const uint8_t *startAddr, uint8_t len,
                            uint8_t *buf) {
  int err = i2c_slave_read(DEFAULT_BUS, PCA9685_I2C_ADDR, startAddr, buf, len);
  // DEBUG_PRINTF("%d bytes read from reg %x.", len, *startAddr);
  return err != 0 ? PCA9685_CMD_ERR : PCA9685_CMD_OK;
}

pca9685_err_t readRegister(const uint8_t regAddr, uint8_t *buf) {
  return readRegisters(&regAddr, 1, buf);
}
pca9685_err_t writeRegister(const uint8_t regAddr, const uint8_t *data) {
  // DEBUG_PRINTF("WRITING Dev bus:%d, Addr: %x, Register: %x, Data: %x\n",
  //  DEFAULT_BUS, PCA9685_I2C_ADDR, regAddr, *data);
  uint8_t err =
      i2c_slave_write(DEFAULT_BUS, PCA9685_I2C_ADDR, &regAddr, data, 1);

  if (err != 0) {
    DEBUG_PRINTF("Error in writeRegister op.");
    return PCA9685_CMD_ERR;
  }
  sdk_os_delay_us(1000);  // to let changes update

  uint8_t buf;
  uint8_t attempts = 0;
  while (attempts < 5) {
    if (readRegister(regAddr, &buf) == 0) break;
    attempts++;
  }
  if (attempts >= 5 && buf != *data) {
    return PCA9685_CMD_ERR;
  }
  return PCA9685_CMD_OK;
}

/**************************** Exposed API ************************************/
PCA9685_Servo_Driver::PCA9685_Servo_Driver() {
  memset(servos, 0, sizeof(servos));
  memset(free, 1, sizeof(free));
  head = INVALID_VAL;
  tail = INVALID_VAL;
  _oscilFreq = 25 * 1000 * 1000;  // 25 Mhz internal oscillator
  _sdaPin = DEFAULT_SDA_PIN;
  _sclPin = DEFAULT_SCL_PIN;
  _numServos = 0;
}

PCA9685_Servo_Driver::PCA9685_Servo_Driver(uint8_t sdaPin, uint8_t sclPin) {
  memset(servos, 0, sizeof(servos));
  memset(free, 1, sizeof(free));
  head = INVALID_VAL;
  tail = INVALID_VAL;
  _oscilFreq = 25500000;  // 25 Mhz internal oscillator
  _sdaPin = sdaPin;
  _sclPin = sclPin;
  _numServos = 0;
}

pca9685_err_t PCA9685_Servo_Driver::attach(uint8_t pin) {
  return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 180);
}

pca9685_err_t PCA9685_Servo_Driver::attach(uint8_t pin, uint16_t min_pw,
                                           uint16_t max_pw,
                                           uint8_t angle_range) {
  if (pin >= MAX_SERVOS) {
    LOG_PRINTF("Pin %d invalid!\n", pin);
    return PCA9685_SERVO_INVALID_PIN;
  }

  if (!free[pin]) {
    LOG_PRINTF("Servo at pin %d already attached!\n", pin);
    return PCA9685_SERVO_ALR_ATTACHED;
  }

  if (head == INVALID_VAL && tail == INVALID_VAL) {  // first servo
    head = pin;
    tail = pin;
    next[tail] = INVALID_VAL;  // For tail, next is NA
    prev[head] = INVALID_VAL;  // For head, prev is NA
  } else {
    next[tail] = pin;
    prev[pin] = tail;
    tail = pin;
    next[tail] = INVALID_VAL;  // For tail, next is NA
  }
  servos[pin] = {pin, 0, min_pw, 0, min_pw, max_pw, angle_range};
  free[pin] = 0;
  _numServos++;
  return PCA9685_CMD_OK;
}

pca9685_err_t PCA9685_Servo_Driver::detach(uint8_t pin) {
  uint8_t prevIdx, nextIdx;
  if (pin >= MAX_SERVOS) {
    DEBUG_PRINTF("Pin %d invalid!", pin);
    return PCA9685_SERVO_INVALID_PIN;
  }

  if (free[pin]) {
    DEBUG_PRINTF("Servo at pin %d not attached!", pin);
    return PCA9685_SERVO_NOT_ATTACHED;
  }

  // Update servo list
  nextIdx = next[pin];
  prevIdx = prev[pin];
  if (prevIdx != INVALID_VAL) next[prevIdx] = nextIdx;
  if (nextIdx != INVALID_VAL) prev[nextIdx] = prevIdx;

  free[pin] = 1;
  _numServos--;
  return PCA9685_CMD_OK;
}

pca9685_err_t PCA9685_Servo_Driver::begin() {
  uint8_t towrite, reg, readvalue;
  if (i2c_init(DEFAULT_BUS, _sclPin, _sdaPin, I2C_FREQ_400K) != 0) {
    LOG_PRINTF("Error has occurred while initializing PCA9685 I2C.\n");
    return PCA9685_CMD_ERR;
  }

  // do a software reset to init readRegisters (Special SWRST call)
  reg = 0x06;
  if (i2c_slave_write(DEFAULT_BUS, 0x00, &reg, data, 0) != 0) {
    LOG_PRINTF("SWRST failed.\n");
    return PCA9685_CMD_ERR;
  }

  // Sleep needed to change PWM freq
  towrite = MODE1_LOW_POW;
  if (writeRegister(PCA9685_MODE1, &towrite) != 0) {
    LOG_PRINTF("Write to MODE1 failed.\n");
    return PCA9685_CMD_ERR;
  }
  // default set to 50 hz for servo
  towrite = roundf((float)(_oscilFreq) / (4096 * SERVO_FREQ)) - 1;
  if (writeRegister(PCA9685_PRE_SCALE, &towrite) != 0) {
    LOG_PRINTF("Write to PRE_SCALE failed.\n");
    return PCA9685_CMD_ERR;
  }
  // Restart without losing PWM CONFIG
  if (readRegister(PCA9685_MODE1, &readvalue) != 0) {
    LOG_PRINTF("Read from MODE1 failed.\n");
    return PCA9685_CMD_ERR;
  }
  
  towrite = readvalue & ~(1 << 4);  // clear the SLEEP bit
  if (writeRegister(PCA9685_MODE1, &towrite) != 0) {
    LOG_PRINTF("Write to MODE1 for restart failed.\n");
    return PCA9685_CMD_ERR;
  }

  sdk_os_delay_us(600);  // wait for oscillator to stabilise

  towrite = MODE2_TOTEMPOLE;
  if (writeRegister(PCA9685_MODE2, &towrite) != 0) {
    LOG_PRINTF("Write to MODE2 failed.\n");
    return PCA9685_CMD_ERR;
  }

  return PCA9685_CMD_OK;
}

pca9685_err_t PCA9685_Servo_Driver::writeAngle(uint8_t pin, uint8_t angle) {
  uint16_t pwidth, oncount, delaytime;
  uint8_t towrite;
  // check if attached
  if (free[pin]) return PCA9685_SERVO_NOT_ATTACHED;

  // check if angle valid
  if (angle > 180) return PCA9685_SERVO_INVALID_ANGLE;

  pwidth = (uint16_t)((angle / (float)servos[pin].max_angle_range) *
                      (servos[pin].max_pw - servos[pin].min_pw)) +
           servos[pin].min_pw;

  // update internal rep of servo
  servos[pin].angle = angle;
  servos[pin].pulse_width = pwidth;
  servos[pin].isActive = 1;

  return writeMicroseconds(pin, pwidth);
}

pca9685_err_t PCA9685_Servo_Driver::writeMicroseconds(uint8_t pin,
                                                      uint16_t pwidth) {
  uint16_t oncount, delaytime;
  uint8_t towrite;
  delaytime = 0;                                           // no delay
  oncount = round(((float)pwidth / SERVO_PERIOD) * 4096);  // 12-bit counter

  // TODO: shorten this with auto increment registers
  towrite = (delaytime & 0xFF);
  if (writeRegister(PCA9685_LED_ON_L(pin), &towrite) != 0) {
    LOG_PRINTF("Write LED_ON_L %d failed.\n", pin);
    return PCA9685_CMD_ERR;
  }

  towrite = (delaytime >> 8) & 0xFF;
  if (writeRegister(PCA9685_LED_ON_H(pin), &towrite) != 0) {
    LOG_PRINTF("Write LED_ON_H %d failed.\n", pin);
    return PCA9685_CMD_ERR;
  }

  towrite = (oncount + delaytime) & 0xFF;
  if (writeRegister(PCA9685_LED_OFF_L(pin), &towrite) != 0) {
    LOG_PRINTF("Write LED_OFF_L %d failed.\n", pin);
    return PCA9685_CMD_ERR;
  }

  towrite = ((oncount + delaytime) >> 8) & 0xFF;
  if (writeRegister(PCA9685_LED_OFF_H(pin), &towrite) != 0) {
    LOG_PRINTF("Write LED_OFF_H %d failed.\n", pin);
    return PCA9685_CMD_ERR;
  }

  return PCA9685_CMD_OK;
}

void PCA9685_Servo_Driver::setOscilFreq(uint32_t freq) { _oscilFreq = freq; }

uint8_t PCA9685_Servo_Driver::getServoCount() { return _numServos; }