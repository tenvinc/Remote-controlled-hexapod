/**
 * This class contains the driver needed to interact and command
 * PCA9685 Servo driver board. This class exposes high level API to control
 * PWM output at different pins, while hiding the low level I2C communications
 *and register manipulation. This class is to be used for servo control only.
 * Note that according to PCA9685 Datasheet, all 16 pins must be of the same
 *frequency.
 **/

#ifndef __PCA9685_DRIVER_H
#define __PCA9685_DRIVER_H

#include <stdint.h>

/**** Registers ***/
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01
#define PCA9685_SUBADR1 0x02
#define PCA9685_SUBADR2 0x03
#define PCA9685_SUBADR3 0x04
#define PCA9685_ALLCALLADR 0x05
#define PCA9685_LED_BASE_ADDR 0x06

#define PCA9685_LED_ON_L(pin) (PCA9685_LED_BASE_ADDR + (pin * 0x04))
#define PCA9685_LED_ON_H(pin) (PCA9685_LED_BASE_ADDR + (pin * 0x04) + 0x01)
#define PCA9685_LED_OFF_L(pin) (PCA9685_LED_BASE_ADDR + (pin * 0x04) + 0x02)
#define PCA9685_LED_OFF_H(pin) (PCA9685_LED_BASE_ADDR + (pin * 0x04) + 0x03)

#define PCA9685_ALL_LED_ON_L 0xFA
#define PCA9685_ALL_LED_ON_H 0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRE_SCALE 0xFE
#define PCA9685_TESTMODE 0xFF

/*** Convenience macros ***/
#define BYTE_HIGH(x) ((x >> 8) & 0xFF)
#define BYTE_LOW(x) (x & 0xFF)

#define MIN_PULSE_WIDTH 544       // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH 2400      // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH 1500  // default pulse width when servo is attached
#define MAX_SERVOS 16

typedef enum {
  PCA9685_CMD_OK = 0,
  PCA9685_CMD_ERR,
  PCA9685_SERVO_NOT_ATTACHED,
  PCA9685_SERVO_ALR_ATTACHED,
  PCA9685_SERVO_INVALID_PIN,
  PCA9685_SERVO_INVALID_ANGLE,
  PCA9685_SERVO_INVALID_PW,  // invalid pulse width
} pca9685_err_t;

// Contains all the information related to a servo
typedef struct servo_struct {
  uint8_t pin;
  uint8_t angle;         // angle commanded
  uint16_t pulse_width;  // corresponding pulse width commmanded
  uint8_t isActive;
} servo_struct_t;

class PCA9685_Servo_Driver {
 public:
  PCA9685_Servo_Driver();
  PCA9685_Servo_Driver(uint8_t sdaPin, uint8_t sclPin, uint16_t minPulseWidth,
                       uint16_t maxPulseWidth);
  pca9685_err_t setPWMFreq(uint16_t freq);
  uint8_t getServoCount();
  pca9685_err_t begin();  // Start communicating with board
  pca9685_err_t attach(uint8_t pin);
  pca9685_err_t detach(uint8_t pin);
  pca9685_err_t writeAngle(uint8_t pin,
                           uint8_t angle);  // Command servo at pin to go a
                                            // certain angle from (0 to 180)
  pca9685_err_t writeMicroseconds(
      uint8_t pin, uint16_t microSeconds);  // Command servo at pin with pulse
                                            // width in microSeconds
  uint8_t readAngle(uint8_t pin);           // reads previously set angle
  uint16_t readMicroseconds(uint8_t pin);   // reads previously set pulse width
  void setOscilFreq(uint32_t freq);

 private:
  servo_struct_t servos[MAX_SERVOS];
  uint8_t next[MAX_SERVOS];  // indicates next valid servo
  uint8_t prev[MAX_SERVOS];  // indicates prev valid servo
  uint8_t head;              // start of servo list
  uint8_t tail;              // end of servo list
  uint8_t free[MAX_SERVOS];  // indicates if pin is free
  uint8_t _numServos;
  uint16_t _minPulseWidth;
  uint16_t _maxPulseWidth;
  uint32_t _oscilFreq;  // internal freq used to calibrate pwm
  uint8_t _sdaPin;
  uint8_t _sclPin;
};

#endif