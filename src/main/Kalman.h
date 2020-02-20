#ifndef KALMAN_H
#define KALMAN_H

#include "MatrixT.h"
#include "Arduino.h"

// xDim: dimension of state vector, uDim: dimension of update vector, zDim: dimension of measurement vector
template <int xDim, int uDim, int zDim, class ElemT = float>
class KalmanFilter
{
public:
  mutable MatrixT<xDim, xDim> pKMinus; // Covariance Matrix to calculate Kalman Gain K
  mutable MatrixT<xDim> xKMinus;       // Store running value of state vector

  KalmanFilter<xDim, uDim, zDim, ElemT> (): xKMinus() {
    pKMinus = {10, 10, 10, 10};
    A = {0.1 , 0.2, 0.3, 0.4};
    B = {0.5, 0.6, 0.8, 0.9};
    R = {1.0, 1.1, 1.2, 1.3};
    Q = {0.1, 0.2, 0.3, 0.4};
    H = {2.0, 2.1, 2.2, 2.3};
  }

  MatrixT<xDim, 1, ElemT> filter(const MatrixT<xDim, 1, ElemT> uK, const MatrixT<zDim, 1, ElemT> zK)
  {
    int res = 0;
    MatrixT<xDim, 1, ElemT> xKp = A * xKMinus + B * uK;   // predict next State
    MatrixT<xDim, xDim, ElemT> pKp = A * pKMinus * A.transpose() + Q;  // calculate intermediate covariance matrix
    MatrixT<zDim, zDim, ElemT> S = H * pKp * H.transpose() + R;  // Step 1 of calculating Kalman Gain, K
    MatrixT<xDim, zDim, ElemT> K;
    // // Step 2 of calculating Kalman Gain, K
    if (S.Rows == 1 && S.Cols == 1) {
      K = pKp * H.transpose() * (1 / S(0, 0));  
    } else {
      K = pKp * H.transpose() * S.invert(&res);
    }
    xKMinus = xKp + K * (zK - H * xKp);  // Combine measurement and prediction to estimate current state
    pKMinus = (eye<xDim, ElemT>() - K * H) * pKp;  // update covariance matrix
    return xKMinus;
  }

private:
  MatrixT<xDim, xDim> A; // State transition matrix
  MatrixT<xDim, uDim> B; // Control transition matrix
  MatrixT<zDim, zDim> R; // Measurement noise
  MatrixT<xDim, xDim> Q; // Process noise (Uncertainty from state model)
  MatrixT<zDim, xDim> H; // Observation matrix
};

#endif
