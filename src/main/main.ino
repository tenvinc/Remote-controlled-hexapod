#include "MatrixT.h"
#include "Kalman.h"

void setup() {
  Serial.begin(9600);
  KalmanFilter<2, 2, 2, float> kalmanCtrl;
  MatrixT<2> uK(3, 4);
  MatrixT<2> zK(1, 2);
  kalmanCtrl.filter(uK, zK).print();
}

void loop() {
}
