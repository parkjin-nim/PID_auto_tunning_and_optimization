#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    Kp = Kp_;
    Kd = Kd_;
    Ki = Ki_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
    p_error = cte;
    i_error += cte;
    d_error = cte - prev_error;
    prev_error = p_error;
}

double PID::TotalError() {
    
    return p_error + d_error + i_error;
}
double PID::GetSteer() {
    
    return -Kp*p_error - Kd*d_error - Ki*i_error;
}
//double PID::GetThrottle(double speed, double terr) {
//
//    return lKp*(0.5 - speed) + lKd*err;
//}
