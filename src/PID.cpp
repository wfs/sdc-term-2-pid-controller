#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;  // Proportional term : how hard you want to steer back to the centre of the road (x-axis).
    this->Ki = Ki;  // Integral term : wheels are out of alignment?
    this->Kd = Kd;  // Derivative term : smooth the turning back to the centre of the road (x-axis)

    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;  // aka "d_error = cte - previous_cte" ... how much we changed since our last Cross Track Error.
    p_error = cte;  // aka the CTE; how far we are off the centre of the road, which is where we want to be aka track to
    i_error += cte;  // the cumulative sum of all of our Cross Track Errors.

}

double PID::TotalError() {
    //return Kp*p_error + Ki*i_error + Kd*d_error;
}

