#include "PID.h"
#include <math.h>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  
  //this->Kp = Kp_;
  //this->Ki = Ki_;
  //this->Kd = Kd_;
  this->prev_cte = 0.0;
  
  this->Kpid.resize(3);
  this->Kpid[0] = Kp_;
  this->Kpid[1] = Ki_;
  this->Kpid[2] = Kd_;
  
  this->Best_Kpid.resize(3);
  this->Best_Kpid[0] = Kp_;
  this->Best_Kpid[1] = Ki_;
  this->Best_Kpid[2] = Kd_;
  
  this->twiddle_p_index = 0;
  
  this->dp.resize(3);
  this->dp[0] = 0.002;
  this->dp[1] = 0.0002;
  this->dp[2] = 0.1;
  
  this->twiddle_stage = 1;
  this->step_count = 0;
  this->twiddle_iterations = 0;
  
  this->twiddle_error = 0.0;
  this->twiddle_best_error = 10000.0;

  
}

void PID::UpdateError(double cte) {
  p_error=cte;
  i_error+=cte;
  d_error=cte - prev_cte;
  prev_cte = cte;
}

double PID::TotalError() {
  return -Kpid[0] * p_error - Kpid[1]*i_error - Kpid[2] * d_error;
}

void PID::Twiddle(double speed_diff, double cte){

  int min_steps = 500;
  double tol_speed = 2.0;
  //double tol_error = 0.000001;
  //int max_iterations = 10;


  if (this->twiddle_stage == 1) 
  {
    this->Kpid[twiddle_p_index] += this->dp[twiddle_p_index];
    this->twiddle_stage = 2;
    this->twiddle_iterations+=1;
  }
  
  if (this->twiddle_stage ==2)  // measure the effect of the 1st update of the parameter
  {
    if (fabs(speed_diff) < tol_speed)
    {
      step_count+=1;
      this->twiddle_error += fabs(cte);
      if (this->step_count > min_steps)
      {
        this->twiddle_stage = 3;
        this->step_count = 0;
      }
    }
  }
  
  if (this->twiddle_stage ==3)
  {
    if (this->twiddle_error < this->twiddle_best_error)
    {
      this->twiddle_best_error = this->twiddle_error;
      this->Best_Kpid[twiddle_p_index] = Kpid[twiddle_p_index];
      this->dp[twiddle_p_index]*=1.1;
      this->twiddle_stage = 1;
    }
    else
    {
      this->Kpid[twiddle_p_index]-=2 * this->dp[twiddle_p_index];
      this->twiddle_stage = 4;
    }
  this->twiddle_error = 0;
  }

  
  if (twiddle_stage == 4 )
  {
    if (fabs(speed_diff) < tol_speed)
    {
      this->step_count+=1;
      this->twiddle_error += fabs(cte);
      if (step_count > min_steps)
      {
        twiddle_stage = 5;
        this->step_count = 0;

      }
    }
  }
  
  if (this->twiddle_stage ==5)
  {
    if (this->twiddle_error < this->twiddle_best_error)
    {
      this->twiddle_best_error = this->twiddle_error;
      this->dp[twiddle_p_index]*=1.1;
      this->Best_Kpid[twiddle_p_index] = Kpid[twiddle_p_index];
    }
    else
    {
      this->Kpid[twiddle_p_index]+=this->dp[twiddle_p_index];
      this->dp[twiddle_p_index]*=0.9;
    }
  
  this->twiddle_error = 0;
  this->twiddle_stage = 1;
  }
  
  
  if (this->twiddle_iterations>5)
  { 
    if (this->twiddle_p_index<2)
    {
        this->twiddle_p_index+=1;
      	this->twiddle_iterations=0;
      	this->twiddle_best_error = 100000.0;
    }
    else
    {
      std::cout<<"TWIDDLE OVER \n";
      std::cout<<"Optimized parameters P,I,D = "<< Kpid[0]<<", "<<Kpid[1]<<", "<<Kpid[2]<<"\n";
      this->twiddle_stage = 6;
    }   
  }

  std::cout << "\x1B[2J\x1B[H";
  std::cout<<"Twiddle Stage: "<<this->twiddle_stage<<"\n";
  std::cout<<"Best error: "<<this->twiddle_best_error<<"\n";
  std::cout<<"Current error: "<<this->twiddle_error<<"\n";

  std::cout<<"Iterations = "<<this->twiddle_iterations<<"\n";

  std::cout<<"Step count = "<<this->step_count<<"\n";
  std::cout<<"PID Index = "<<this->twiddle_p_index<<"\n";
  
  std::cout<<"dp[] = "<<this->dp[this->twiddle_p_index]<<"\n";
  std::cout<<"Kp = "<<this->Kpid[this->twiddle_p_index]<<"\n";
  std::cout<<"Best Parameters Found: "<<this->Best_Kpid[0]<<", "<<this->Best_Kpid[1]<<", "<<this->Best_Kpid[2]<<"\n";
  std::cout<<"Current parameters P,I,D = "<< Kpid[0]<<", "<<Kpid[1]<<", "<<Kpid[2]<<"\n";
  
}

/*
Udacity Twiddle Algorithm in Python
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p

*/