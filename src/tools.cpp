#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
 VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if ((estimations.size() != ground_truth.size()) || ground_truth.size() == 0)
  {
       std::cout << "CalculateRMSE () - Error - ivalidSize" << std::endl;
       return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float squareSum = px*px + py*py;
  float root = sqrt(squareSum);
  float root_3 = sqrt(squareSum * squareSum);
  float subs = vx*py - vy*px;
  
  // check division by zero
  if (squareSum == 0)
  {
      std::cout << "CalculateJacobian () - Error - Division by 0:" << std::endl;
      return Hj;
  }
  
  // compute the Jacobian matrix
    Hj << px/root, py/root, 0,0,
            -py/squareSum, px/squareSum, 0, 0,
            py*subs/root_3, px*-subs/root_3, py/root, py/root;
            
            
  return Hj;
}
