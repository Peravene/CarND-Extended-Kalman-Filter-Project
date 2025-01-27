#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
    VectorXd y = z - H_ * x_;
    UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(const VectorXd &y) {
  /**
   * update the state by using Kalman Filter equations
   */
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    // new state
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */

    float radius = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
    
    // check division by zero
    if (radius == 0)
    {
        std::cout << "UpdateEKF () - Error - Division by 0:" << std::endl;
        return;
    }

    // convert x from kartesian to polar
    VectorXd x_polar = VectorXd(3);
    x_polar << radius, 
              atan2(x_[1],x_[0]),
              (x_[0]*x_[2]+x_[1]*x_[3])/radius;

    VectorXd y = z - x_polar;

    // In C++, atan2() returns values between -pi and pi. 
    // When calculating phi in y = z - h(x) for radar measurements, 
    // the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. 
    // The Kalman filter is expecting small angle values between the range -pi and pi. 
    // HINT: when working in radians, you can add 2π or subtract 2π 
    // until the angle is within the desired range.

    while (y(1)>M_PI){
      y(1) -= 2*M_PI;
    }

    while (y(1)<-M_PI){
      y(1) += 2*M_PI;
    }

    UpdateCommon(y);
}
