#include "Speed_PID.h"

Speed_PID::Speed_PID() {}

Speed_PID::~Speed_PID() {}

void Speed_PID::Init(double Kp_, double Ki_, double Kd_, double Integral_) {
  
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  this->i_error = Integral_;
  this->prev_speed_diff = 0.0;
  this->safety_offset = 0.0;
}

void Speed_PID::UpdateError(double speed_diff) {
  p_error=speed_diff;
  i_error+=speed_diff;
  d_error=speed_diff - prev_speed_diff;
  prev_speed_diff = speed_diff;
}

double Speed_PID::TotalError(double cte) {
  /*
  if (fabs(cte) > 1.0 && safety_offset < 10.0)
  {
    safety_offset+=0.4;
  }
  else
  {
    if (safety_offset >= 0.1)
    {
      safety_offset-=0.1;
    }
  }
  std::cout<<"Safety offset = "<<safety_offset<<"\n";
  */
  return -Kp * p_error - Ki*i_error - Kd * d_error; //- safety_offset;
  
}