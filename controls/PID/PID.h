#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();
  
  double Kp;
  double Ki;
  double Kd;
  
  std::vector <double> dp; // parameters for Twiddle
  std::vector <double> Kpid; 
  std::vector <double> Best_Kpid; // parameters for Twiddle

  int twiddle_stage;
  double twiddle_error;
  double twiddle_best_error;
  int step_count;
  int twiddle_p_index;
  int twiddle_iterations;
  
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  void Twiddle(double speed_diff, double cte);
  
 //private:
  /**
   * PID Errors
   */


  /**
   * PID Coefficients
   */ 
};

#endif  // PID_H