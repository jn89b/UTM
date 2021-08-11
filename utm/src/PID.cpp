#include <iostream>
#include <PID.h>

PID::PID(float kp, float ki, float kd, float dt, float target, float current)
{
    float pre_error = 0.0;
    float pre_ierror = 0.0;

    calcPID(target, current, dt, pre_error, pre_ierror);
}

void PID::calcPID(float target, float current, double dt, double pre_error, double pre_ierror)
{
    //ROS_INFO("calculating");
    //target x and target y is the position relative to quad 
    float error = target;
    //std::cout << "error x " << error_x << "error y" << error_y << std::endl;
    std::cout <<"P" << _kp << std::endl;

    float Pgain = _kp * error;

    float int_error = pre_ierror + ((error + pre_error)/ 2) * dt;
    float Igain = _ki * int_error;
    
    float der_error = (error - pre_error) / dt;
    float Dgain = _kd * der_error;

    float PID = Pgain + Igain + Dgain;
    
    //set gain constraint to prevent the guy from going to crazy
    float gain_constraint = 2.0;
    if (PID >= gain_constraint)
        {
            PID = gain_constraint;
        }

    if (PID <= -gain_constraint)
        {
            PID = -gain_constraint;
        }
        
    //save error
    pre_error = error;
    pre_ierror = int_error;
    std::cout << "error" << int_error << std::endl;
    std::cout << "function" << PID << std::endl;
    _PID = PID;
    //return _PID;
}