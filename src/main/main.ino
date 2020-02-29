#include <SparkFunLSM6DS3.h>

#include "MatrixT.h"
#include "Kalman.h"

#define US_IN_1_SEC 1e6

#define GYRO_RANGE 2000  //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
#define GYRO_SR 833  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
#define GYRO_BW 200  //Hz.  Can be: 50, 100, 200, 400;

#define ACCEL_RANGE 16  //Max G force readable.  Can be: 2, 4, 8, 16
#define ACCEL_SR 833   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
#define ACCEL_BW 200  //Hz.  Can be: 50, 100, 200, 400;

#define CALI_WINDOW 50

LSM6DS3 myIMU;

double gyroRoll = 0, gyroPitch = 0, gyroYaw = 0;
float gyroXBias, gyroYBias, gyroZBias;
unsigned long prevTime = 0;
KalmanFilter<4, 2, 2, float> kalmanFilter;

void setup() {
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = GYRO_RANGE;
  myIMU.settings.gyroSampleRate = GYRO_SR;
  myIMU.settings.gyroBandWidth = GYRO_BW;
  myIMU.settings.gyroFifoEnabled = 0;

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = ACCEL_RANGE;
  myIMU.settings.accelBandWidth = ACCEL_BW;
  myIMU.settings.accelSampleRate = ACCEL_SR;
  myIMU.settings.accelFifoEnabled = 0;
  
    //Non-basic mode settings
  myIMU.settings.commMode = 1;

  Serial.begin(112500);
  delay(1000); // wait for a while...
  Serial.println("Processor came out of reset.\n");
  
  if (myIMU.begin() != 0) {
    Serial.println("Problem contacting the sensor with I2C. Please check.");
  } else {
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_ORIENT_CFG_G, 
      LSM6DS3_ACC_GYRO_SIGN_X_G_NEGATIVE | LSM6DS3_ACC_GYRO_SIGN_Y_G_NEGATIVE | LSM6DS3_ACC_GYRO_SIGN_Z_G_NEGATIVE); // Pitch up: +ve, Roll right: +ve, Yaw right: +ve
    Serial.println("Connection with sensor established.");
  }

  
  Serial.println("Starting calibration. Do not touch the device.");
  float gyroXAccumulator = 0, gyroYAccumulator = 0, gyroZAccumulator = 0;
  float accelXAccumulator = 0, accelYAccumulator = 0, accelZAccumulator = 0;
  for (int n=0; n<CALI_WINDOW; n++) {
    Serial.print(".");
    gyroXAccumulator += myIMU.readFloatGyroX();
    gyroYAccumulator += myIMU.readFloatGyroY();
    gyroZAccumulator += myIMU.readFloatGyroZ();
  }
  Serial.println();
  gyroXBias = gyroXAccumulator / CALI_WINDOW;
  gyroYBias = gyroYAccumulator / CALI_WINDOW;
  gyroZBias = gyroZAccumulator / CALI_WINDOW;

  float accelX, accelY, accelZ;
  double roll, pitch;
  accelX = myIMU.readFloatAccelX();
  accelY = myIMU.readFloatAccelY();
  accelZ = myIMU.readFloatAccelZ();
  getAngleFromAcc((double)accelX, (double)accelY, accelZ, roll, pitch);
  kalmanFilter.setInitial((float)roll, (float)pitch);
  
  Serial.println("Calibration complete. You may begin.");
  Serial.println("Results of calibration: ");
  Serial.print("Gyro bias: ");
  Serial.print(gyroXBias); Serial.print(", ");
  Serial.print(gyroYBias); Serial.print(", ");
  Serial.println(gyroZBias);
}

void loop() {
  float accelX, accelY, accelZ;
  accelX = myIMU.readFloatAccelX();
  accelY = myIMU.readFloatAccelY();
  accelZ = myIMU.readFloatAccelZ();

  // Serial.print("AccelX: ");
  // Serial.print(accelX, 5);
  // Serial.print(" ");
  // Serial.print(accelY, 5);
  // Serial.print(" ");
  // Serial.println(accelZ, 5);

  double roll, pitch;
  getAngleFromAcc((double)accelX, (double)accelY, accelZ, roll, pitch);
  // Serial.print(", ");

  float gyroX, gyroY, gyroZ;
  gyroX = myIMU.readFloatGyroX() - gyroXBias;
  gyroY = myIMU.readFloatGyroY() - gyroYBias;
  gyroZ = myIMU.readFloatGyroZ() - gyroZBias;

  // Serial.print("Gyro: ");
  // Serial.print(gyroX, 5);
  // Serial.print(", ");
  // Serial.print(gyroY, 5);
  // Serial.print(", ");
  // Serial.println(gyroZ, 5);

  float timeDelta = (float)(micros() - prevTime) / US_IN_1_SEC;
  prevTime = micros();
  MatrixT<2, 1, float> measurement = {roll, pitch};
  MatrixT<2, 1, float> update = {gyroY, gyroX};  // flipped x and y axis
  MatrixT<4, 1, float> final = kalmanFilter.filter(update, measurement, timeDelta);

  Serial.print("Data: ");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(final(0, 0), 5);
  Serial.print(",");
  Serial.println(final(2, 0), 5);
}

void getAngleFromAcc(double accelX, double accelY, double accelZ, double &roll, double &pitch) {
  // Follow aviation rotation sequence roll, pitch then yaw. 
  // Pitch abt x, roll abt y, yaw abt z
  roll = atan2(accelX, accelZ) * (180.0/M_PI);
  pitch = atan2(-accelY, sqrt(accelX * accelX + accelZ * accelZ)) * (180.0/ M_PI); 
}
