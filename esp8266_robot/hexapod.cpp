#include "hexapod.h"

#include <algorithm>

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

servo_intrinsics_t SERVO_INTRINSICS_LUT[12] = {
    {600, 2620, 180}, {600, 2620, 170}, {600, 2650, 180}, {600, 2650, 175},
    {600, 2650, 175}, {600, 2650, 175}, {700, 2650, 180}, {700, 2650, 170},
    {700, 2650, 175}, {700, 2750, 180}, {600, 2550, 180}, {500, 2550, 180},
};

bool SERVO_INVRT_LUT[12] = {  // indicates which servo is inverted
    false, false, false, false, true,  false,
    false, false, false, false, false, true};

void Hexapod::init() {
  driver.begin();
  for (int i = 0; i < 12; i++) {
    driver.attach(i, SERVO_INTRINSICS_LUT[i].min_pw,
                  SERVO_INTRINSICS_LUT[i].max_pw,
                  SERVO_INTRINSICS_LUT[i].angle_range);
  }
  // Initialise to curl position
  for (int upper_idx = 0; upper_idx < 6; upper_idx++) {
    upper_limb_move(upper_idx, 75);  // neutral angle
  }
  for (int lower_idx = 0; lower_idx < 6; lower_idx++) {
    #if 0
      lower_limb_move(lower_idx, 0);
    #else
      lower_limb_move(lower_idx, 180);
    #endif
  }
}

void Hexapod::stand() {
  for (int ud_angle = 0; ud_angle <= 180; ud_angle += 5) {
    for (int lower_idx = 0; lower_idx < 6; lower_idx++) {
      lower_limb_move(lower_idx, ud_angle);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void Hexapod::turn_left() {
    // lift up legs
    lower_limb_move(MID_L_LOWERLEG, 160);
    lower_limb_move(BACK_R_LOWERLEG, 160);
    lower_limb_move(FRONT_R_LOWERLEG, 160);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    upper_limb_move(MID_L_UPPERLEG, 0);
    upper_limb_move(BACK_R_UPPERLEG, 0);
    upper_limb_move(FRONT_R_UPPERLEG, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // put down legs
    lower_limb_move(MID_L_LOWERLEG, 180);
    lower_limb_move(BACK_R_LOWERLEG, 180);
    lower_limb_move(FRONT_R_LOWERLEG, 180);

    vTaskDelay(200 / portTICK_PERIOD_MS);

    lower_limb_move(MID_R_LOWERLEG, 160);
    lower_limb_move(BACK_L_LOWERLEG, 160);
    lower_limb_move(FRONT_L_LOWERLEG, 160);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    upper_limb_move(MID_R_UPPERLEG, 0);
    upper_limb_move(BACK_L_UPPERLEG, 0);
    upper_limb_move(FRONT_L_UPPERLEG, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    lower_limb_move(MID_R_LOWERLEG, 180);
    lower_limb_move(BACK_L_LOWERLEG, 180);
    lower_limb_move(FRONT_L_LOWERLEG, 180);

    vTaskDelay(200 / portTICK_PERIOD_MS);  

    for (int i=0; i<6; i++) {
        upper_limb_move(i, 75);
    }
}

void Hexapod::turn_right() {
    // lift up legs
    lower_limb_move(MID_L_LOWERLEG, 160);
    lower_limb_move(BACK_R_LOWERLEG, 160);
    lower_limb_move(FRONT_R_LOWERLEG, 160);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    upper_limb_move(MID_L_UPPERLEG, 150);
    upper_limb_move(BACK_R_UPPERLEG, 150);
    upper_limb_move(FRONT_R_UPPERLEG, 150);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // put down legs
    lower_limb_move(MID_L_LOWERLEG, 180);
    lower_limb_move(BACK_R_LOWERLEG, 180);
    lower_limb_move(FRONT_R_LOWERLEG, 180);

    vTaskDelay(200 / portTICK_PERIOD_MS);

    lower_limb_move(MID_R_LOWERLEG, 160);
    lower_limb_move(BACK_L_LOWERLEG, 160);
    lower_limb_move(FRONT_L_LOWERLEG, 160);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    upper_limb_move(MID_R_UPPERLEG, 150);
    upper_limb_move(BACK_L_UPPERLEG, 150);
    upper_limb_move(FRONT_L_UPPERLEG, 150);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    lower_limb_move(MID_R_LOWERLEG, 180);
    lower_limb_move(BACK_L_LOWERLEG, 180);
    lower_limb_move(FRONT_L_LOWERLEG, 180);

    vTaskDelay(200 / portTICK_PERIOD_MS);  

    for (int i=0; i<6; i++) {
        upper_limb_move(i, 75);
    }
}

void Hexapod::walkForward() {
  int lift_angle = 100;
  int lift_delay = 50;
  int shift_delay = 200;

  // move front left and back right leg
  lower_limb_move(FRONT_L_LOWERLEG, lift_angle);
  lower_limb_move(BACK_R_LOWERLEG, lift_angle);
  vTaskDelay(lift_delay / portTICK_PERIOD_MS);
  upper_limb_move(FRONT_L_UPPERLEG, 105);
  upper_limb_move(BACK_R_UPPERLEG, 45);
  vTaskDelay(lift_delay / portTICK_PERIOD_MS);
  lower_limb_move(FRONT_L_LOWERLEG, 180);
  lower_limb_move(BACK_R_LOWERLEG, 180);
  vTaskDelay(shift_delay / portTICK_PERIOD_MS);

  // move mid left and mid right leg
  lower_limb_move(MID_L_LOWERLEG, lift_angle);
  lower_limb_move(MID_R_LOWERLEG, lift_angle);
  vTaskDelay(lift_delay / portTICK_PERIOD_MS);
  upper_limb_move(MID_L_UPPERLEG, 115);
  upper_limb_move(MID_R_UPPERLEG, 35);
  vTaskDelay(lift_delay / portTICK_PERIOD_MS);
  lower_limb_move(MID_L_LOWERLEG, 180);
  lower_limb_move(MID_R_LOWERLEG, 180);
  vTaskDelay(shift_delay / portTICK_PERIOD_MS);
  
  // move front right and back left leg
  lower_limb_move(FRONT_R_LOWERLEG, lift_angle);
  lower_limb_move(BACK_L_LOWERLEG, lift_angle);
  vTaskDelay(lift_delay / portTICK_PERIOD_MS);
  upper_limb_move(FRONT_R_UPPERLEG, 45);
  upper_limb_move(BACK_L_UPPERLEG, 105);
  vTaskDelay(lift_delay / portTICK_PERIOD_MS);
  lower_limb_move(FRONT_R_LOWERLEG, 180);
  lower_limb_move(BACK_L_LOWERLEG, 180);
  vTaskDelay(shift_delay / portTICK_PERIOD_MS);

  // push body forward
  upper_limb_move(BACK_L_UPPERLEG, 45);
  upper_limb_move(BACK_R_UPPERLEG, 105);
  upper_limb_move(MID_L_UPPERLEG, 35);
  upper_limb_move(MID_R_UPPERLEG, 115);
  upper_limb_move(FRONT_L_LOWERLEG, 45);
  upper_limb_move(FRONT_R_UPPERLEG, 105);
}

bool Hexapod::lower_limb_move(uint8_t idx, uint8_t angle) {
  if (angle > 180) return false;
  driver.writeAngle(
      idx * 2,
      SERVO_INVRT_LUT[idx * 2]
          ? std::max((int16_t)SERVO_INTRINSICS_LUT[idx * 2].angle_range -
                         (int16_t)angle,
                     0)
          : std::min(angle, SERVO_INTRINSICS_LUT[idx * 2].angle_range));
  return true;
}

bool Hexapod::upper_limb_move(uint8_t idx, uint8_t angle) {
  if (angle > 180) return false;
  driver.writeAngle(
      idx * 2 + 1,
      SERVO_INVRT_LUT[idx * 2 + 1]
          ? std::max((int16_t)SERVO_INTRINSICS_LUT[idx * 2 + 1].angle_range -
                         (int16_t)angle,
                     0)
          : std::min(angle, SERVO_INTRINSICS_LUT[idx * 2 + 1].angle_range));
  return true;
}