#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;
}

void PID::UpdateError(double cte) {

  // Update d_error which is current cte minus previous cte
  this->d_error =  cte - this->p_error;

  // Add cte vaue onto history vector
  this->previous_cte[this->track_cte_vec % this->n] = cte;

  // Update p_error which is current cte
  this->p_error = cte;

  // Update i_error which is sum of all previous ctes
  this->i_error = 0;
  for (auto& i : this->previous_cte){
    this->i_error += i;
   }
   this->track_cte_vec += 1;
}

double PID::TotalError() {
  double P_error = -(this->Kp * this->p_error);
  double I_error = -(this->Ki * this->i_error);
  double D_error = -(this->Kd * this->d_error);
  return P_error + I_error + D_error;
}
