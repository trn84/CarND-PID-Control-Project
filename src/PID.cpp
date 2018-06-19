#include "PID.h"
#include <iostream>

//using namespace std;

/*
* TODO: Complete the PID class.
*/

// If car has absolute CTE larger than this, in meters, assume it has crashed.
const double MAX_CTE = 5.0;


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
    this->avg_cte += cte*cte / this->timestep;

    //std::cout << "P: " << this->p_error << " I: " << this->i_error << " D: " << this->d_error << std::endl;
    std::cout << "Timestep: " << this->timestep << " avg_cte: " << this->avg_cte << std::endl;

    // Car in deadlock?
    if(this->study) {
        if( abs(cte) > MAX_CTE ) {
            this->dead = true;
        }
    }

}

double PID::TotalError() {
    return -Kp*p_error-Kd*d_error-Ki*i_error;
}

