#ifndef __HEXAPOD_H
#define __HEXAPOD_H

extern "C" {
#include <stdint.h>
}

#include "PCA9685_driver.h"

#define BACK_L_LOWERLEG 0
#define BACK_L_UPPERLEG 0
#define BACK_R_LOWERLEG 1
#define BACK_R_UPPERLEG 1

#define MID_L_LOWERLEG 5
#define MID_L_UPPERLEG 5
#define MID_R_LOWERLEG 2
#define MID_R_UPPERLEG 2

#define FRONT_L_LOWERLEG 4
#define FRONT_L_UPPERLEG 4
#define FRONT_R_LOWERLEG 3
#define FRONT_R_UPPERLEG 3

// Servo characteristics
typedef struct servo_intrinsics {
  uint16_t min_pw;      // min pulse width
  uint16_t max_pw;      // max pulse width
  uint8_t angle_range;  // corresponding angle range with 0 degrees when pulse
                        // width = min_pw
} servo_intrinsics_t;

class Hexapod {

 public:
  typedef enum State_t {
    STANDBY,
    STAND,
    WALKING
  };
  
  State_t state;

  void init();
  void keep();
  void stand();
  void turn_left();
  void turn_right();
  void walkForward();

 private:
  bool lower_limb_move(uint8_t idx, uint8_t angle);
  bool upper_limb_move(uint8_t idx, uint8_t angle);

  PCA9685_Servo_Driver driver;
};

#endif