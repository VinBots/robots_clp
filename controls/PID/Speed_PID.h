#ifndef Speed_PID_H
#define Speed_PID_H

class Speed_PID {
 public:
  /**
   * Constructor
   */
  Speed_PID();

  /**
   * Destructor.
   */
  virtual ~Speed_PID();
  
  double Kp;
  double Ki;
  double Kd;
  
  double p_error;
  double i_error;
  double d_error;
  double prev_speed_diff;
  
  double safety_offset;

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, double Integral_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError(double cte);
  
  void Twiddle();
  
 private:
  
};

#endif  // Speed_PID_H