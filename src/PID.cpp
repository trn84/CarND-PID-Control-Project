#include "PID.h"
#include <iostream>
#include <math.h>       /* fabs */

//using namespace std;

const double MAX_CTE = 4.5;


PID::PID() {}

PID::~PID() {}

void PID::Init(bool study, double Kp, double Ki, double Kd) {

    this->Kp=Kp;
    this->Ki=Ki;
    this->Kd=Kd;

    this->p_error=0;
    this->pre_p_error=0;

    this->i_error=0;
    this->d_error=0;

    this->avg_cte=0;

    this->timestep=1;
    this->study = study;
    this->dead = false;

}

void PID::UpdateError(double cte) {

    this->p_error = cte;
    this->i_error += cte;
    this->d_error = cte - this->pre_p_error;
    this->pre_p_error = this->p_error;

    this->timestep++;
    this->avg_cte += cte*cte;

    //std::cout << "P: " << this->p_error << " I: " << this->i_error << " D: " << this->d_error << std::endl;
    //std::cout << "Timestep: " << this->timestep << " avg_cte: " << this->avg_cte << std::endl;

    // Car in deadlock?
    if(this->study) {
        if( fabs(cte) > MAX_CTE && this->timestep > 20 ) {
            this->dead = true;
            std::cout << "Timestep: " << this->timestep << " Car dead: " << std::endl;
        }
    }

}

double PID::TotalError() {
    return -Kp*p_error-Kd*d_error-Ki*i_error;
}

