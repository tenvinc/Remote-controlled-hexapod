/**
 * Interrupt driven Servo library for ESP8266 using 23-bit timer FRC1.
 * Adapted from Arduino's Servo library.
 */

/*
  ** Original description **
  A servo is activated by creating an instance of the Servo class passing 
  the desired pin to the attach() method.
  The servos are pulsed in the background using the value most recently 
  written using the write() method.
  Note that analogWrite of PWM on pins associated with the timer are 
  disabled when the first servo is attached.
  Timers are seized as needed in groups of 12 servos - 24 servos use two 
  timers, 48 servos will use four.
  The sequence used to sieze timers is defined in timers.h
  The methods are:
    Servo - Class for manipulating servo motors connected to physical/virtual pins.
    attach(pin )  - Attaches a servo motor to an i/o pin.
    attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds
    default min is 544, max is 2400  
 
    write()     - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
    writeMicroseconds() - Sets the servo pulse width in microseconds 
    read()      - Gets the last written servo pulse width as an angle between 0 and 180. 
    readMicroseconds()   - Gets the last written servo pulse width in microseconds. (was read_us() in first release)
    attached()  - Returns true if there is a servo attached. 
    detach()    - Stops an attached servos from pulsing its i/o pin. 
 */
#ifndef __SERVO_H_
#define __SERVO_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#define MIN_PULSE_WIDTH 544      // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH 2400     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH 1500 // default pulse width when servo is attached
#define REFRESH_INTERVAL 20000   // minumim time to refresh servos in microseconds

#define SERVOS_PER_TIMER 12 // the maximum number of servos controlled by one timer
#define MAX_SERVOS 12
#define MAX_SERVOS_PER_SR 8

#define INVALID_SERVO 255 // flag indicating an invalid servo index

typedef enum
{
  SERVO_ERR_OK,
  SERVO_ERR_INVALID_ANGLE,
  SERVO_ERR_INVLAID_PIN,
  SERVO_ERR_INVALID_MINMAX
} servo_err_t;

typedef struct
/* Represents a virtual/physical pin that a servo is tied to */
{
  uint8_t nbr : 8;      // An actual/virtual pin number that indicates the pin that the servo is connected to
  uint8_t isActive : 1; // true if this channel is activated. Pulsed only when it is activated
} ServoPin_t;

typedef struct
/* Represents a servo */
{
  ServoPin_t pin;
  volatile unsigned int ticks; // indicates the number of ticks to wait for;
} Servo_t;

typedef struct
/* Represents a request packet that is send to a shift register callback */
{
  uint8_t bitPattern;
  uint8_t cnt;
} srReq;

typedef void (*srCallback_t) (srReq);  // Callback for shifting out bits to shift register called from ISR

class Servo
{
public:
  Servo();
  Servo(srCallback_t cb); // alternative constructor for applications using shift registers
  ~Servo();
  uint8_t attach(int pin);                   // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max); // same as above but also sets min and max values for writeMicroseconds
  void detach();
  uint8_t write(int value);             // if value is < 200, its treated as angle, otherwise as pulse width in microseconds
  uint8_t writeMicroseconds(int value); // writes pulse width in microseconds;
  uint8_t read();                       // reads current pulse width and returns an angle between 0 and 180 degrees
  uint16_t readMicroseconds();          // reads current pulse width in microseconds
  bool isAttached();                    // indicates if current servo is attached
  bool isUsingShiftReg();

  Servo_t servo;

private:
  uint8_t _servoIndex;
  uint16_t _min;
  uint16_t _max;
  bool _usesShiftReg;
};

#endif