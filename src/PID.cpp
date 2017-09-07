#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  update_iter = 0;

  twiddle_en = twiddle;
  p = {Kp, Ki, Kd};
  dp = {0.02*Kp, 0.02*Ki, 0.02*Kd};
  param_under_twiddle = 0;
  step_cnt = 0;
  settle_steps = 200;
  eval_steps = 2000;
  total_error = 0;
  best_total_error = 1000000*eval_steps;
  search_flag = 0; // 0->add; 1->subtract

  return;
}

void PID::UpdateError(double cte) {
  if (step_cnt) {
    p_error = cte;
    i_error += cte;
    d_error = cte - p_error;
  }
  else {
    p_error = cte;
    i_error += cte;
  }
  step_cnt++;

  if (twiddle_en) { // enable twiddle
    if (step_cnt == 1){
      if (!search_flag) {
        p[param_under_twiddle] += dp[param_under_twiddle];
      }
      Kp = p[0];
      Ki = p[1];
      Kd = p[2];
      cout << "New Trial: " << endl;
      cout << "P: " << Kp << "; I: " << Ki << "; D: " << Kd << "; param_under_twiddle: " << param_under_twiddle << "; search_flag: " << search_flag << endl;
    }
    if (step_cnt > settle_steps){
      total_error += cte*cte;
    }
    // compare total_error and start a new set of parameters
    if (step_cnt > (settle_steps + eval_steps)){
      if (total_error < best_total_error){
        best_total_error = total_error;
        cout << "Better Error Found: " << best_total_error << endl;
        cout << "new parameters: " << endl;
        cout << "P: " << Kp << "; I: " << Ki << "; D: " << Kd << endl;
        dp[param_under_twiddle] *= 1.1;
        param_under_twiddle = (param_under_twiddle + 1)%3;
      }
      else{
        if (!search_flag) {
          p[param_under_twiddle] -= 2*dp[param_under_twiddle];
          search_flag = 1;
        }
        else {
          p[param_under_twiddle] += dp[param_under_twiddle];
          dp[param_under_twiddle] *= 0.9;
          param_under_twiddle = (param_under_twiddle + 1)%3;
          search_flag = 0;
        }
      }
      p_error = 0.0;
      i_error = 0.0;
      d_error = 0.0;
      total_error = 0;
      step_cnt = 0; // reset everything
    }
  }
}

double PID::TotalError() {
}
