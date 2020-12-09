#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
    Kp = Kp_;
    Kd = Kd_;
    Ki = Ki_;
}

void PID::UpdateError(double cte) {
//    p_error = cte;
//    i_error += cte;
//    d_error = cte - prev_error;
//    prev_error = p_error;
    
    d_error = cte-p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    tot_error = p_error + d_error + i_error;
    return -Kp*p_error - Kd*d_error - Ki*i_error;
}
