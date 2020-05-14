#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
      cout << "Vector size error";
      return rmse;
  }

  // TODO: accumulate squared residuals
  
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd res = estimations[i] - ground_truth[i];
    res = res.array()*res.array();
    rmse += res;
  }

  // TODO: calculate the mean
  rmse = rmse / estimations.size();

  // TODO: calculate the squared root
  rmse = rmse.array().sqrt(); 

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3,4);
  	// recover state parameters
  	float px = x_state(0);
  	float py = x_state(1);
  	float vx = x_state(2);
  	float vy = x_state(3);

  	// TODO: YOUR CODE HERE 

  	// if dividing by zero then set px and py to small values to be able to compute jacobain
  	if (fabs(px*px + py*py) < 0.0001){
    	px = 0.01;
        py = 0.01;
  		}
  
	
    	  float dr_dpx = px/pow((pow(px, 2) + pow(py, 2)), 0.5);
      	  float dr_dpy = py/pow((pow(px, 2) + pow(py, 2)), 0.5);
      	  float dr_dvx = 0;
      	  float dr_dvy = 0;
      	  float dphi_dpx = (-1)*py/(pow(px, 2) + pow(py, 2));
      	  float dphi_dpy = px/(pow(px, 2) + pow(py, 2));
      	  float dphi_dvx = 0;
      	  float dphi_dvy = 0;
      	  float drdot_dpx = (py*(vx*py - vy*px))/pow((pow(px, 2) + pow(py, 2)), 1.5);
      	  float drdot_dpy = (px*(vy*px - vx*py))/pow((pow(px, 2) + pow(py, 2)), 1.5);
      	  float drdot_dvx = px/pow((pow(px, 2) + pow(py, 2)), 0.5);
      	  float drdot_dvy = py/pow((pow(px, 2) + pow(py, 2)), 0.5);
      
      	  Hj << dr_dpx, dr_dpy, dr_dvx, dr_dvy,
          	    dphi_dpx, dphi_dpy, dphi_dvx, dphi_dvy,
          	    drdot_dpx, drdot_dpy, drdot_dvx, drdot_dvy;
  	  
  
 	   // compute the Jacobian matrix

  	  return Hj;
}
