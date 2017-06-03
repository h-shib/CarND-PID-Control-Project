#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	_x_trajectory = 0;
	_y_trajectory = 0;
	_orientation = 0;
	_steering_noise = 0;
	_distance_noise = 0;
	_steering_drift = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	i_error += cte;
	p_error = cte;
}

double PID::TotalError() {
}
