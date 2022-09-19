/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   //Initialize PID coefficients
   kpi = Kpi;
   kii = Kii;
   kdi = Kdi;

   //initialize PID outputs
   output_lim_min = output_lim_mini;
   output_lim_max = output_lim_maxi;

   //initialize errors
   prev_cte = 0.0;
   diff_cte = 0.0;
   int_cte = 0.0;

}


void PID::UpdateError(double cte) {
   // Update PID errors based on cte

   //check if delta_time > 0 to avoid division by zero
   if(delta_time > 0){
      diff_cte = (cte-prev_cte)/delta_time;
   }else{
      diff_cte = 0.0;
   }
   prev_cte = cte;
   int_cte += cte * delta_time;
   
}

double PID::TotalError() {
   // Calculate and return the total error
   double control = -(kpi * prev_cte + kii * int_cte + kdi * diff_cte);

   //constrain total errors to interval [output_lim_min, output_lim_max]
   if(control > output_lim_max){
      control = output_lim_max;
   }else if(control < output_lim_min){
      control = output_lim_min;
   }
   
   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   //Update the delta time with new value
   delta_time = new_delta_time;
   
}