#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double _Kp;
  double _Ki;
  double _Kd;

  /*
  * Position Parameters
  */
  double _x_trajectory = 0;
  double _y_trajectory = 0;
  double _orientation = 0;
  double _steering_noise = 0;
  double _distance_noise = 0;
  double _steering_drift = 0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
