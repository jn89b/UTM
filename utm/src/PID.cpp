




class PID
{        
    float pre_error_x;
    float pre_error_y;

    float pre_ierror_x;
    float pre_ierror_y;

    float kp;
    float ki;
    float kd;
}


Eigen::Vector2d calc_PID(float target_x, float target_y, float curr_x, float curr_y, double dt)
{
    //ROS_INFO("calculating");
    //target x and target y is the position relative to quad 
    float error_x = target_x;
    float error_y = target_y;
    //std::cout << "error x " << error_x << "error y" << error_y << std::endl;

    //proportional val
    float Pgain_x = kp * error_x;
    float Pgain_y = kp * error_y;

    //integral error 
    float int_error_x = pre_ierror_x + ((error_x + pre_error_x)/ 2) * dt;
    float int_error_y = pre_ierror_y + ((error_y + pre_error_y)/ 2) * dt;
    //integral gain
    float Igain_x = ki * int_error_x;
    float Igain_y = ki * int_error_y;
    
    //derivative error
    double der_error_x = (error_x - pre_error_x) / dt;
    double der_error_y = (error_y - pre_error_y) / dt;
    float Dgain_x = kd * der_error_x;
    float Dgain_y = kd * der_error_y;
    
    float PID_x = Pgain_x + Igain_x + Dgain_x;
    float PID_y = Pgain_y + Igain_y + Dgain_y;
    
    //set gain constraint to prevent the guy from going to crazy
    float gain_constraint = 2.0;
    if (PID_x >= gain_constraint)
        {
            PID_x = gain_constraint;
        }
    else if (PID_y >= gain_constraint)
        {
            PID_y = gain_constraint;
        }

    if (PID_x <= -gain_constraint)
        {
            PID_x = -gain_constraint;
        }
    else if (PID_y <= -gain_constraint)
        {
            PID_y = -gain_constraint;
        }
    
    //save error
    pre_error_x = error_x; 
    pre_error_y = error_y;

    pre_ierror_x = int_error_x;
    pre_ierror_y = int_error_y;

    Eigen::Vector2d PID(PID_x, PID_y);
    return PID;
}