#include <iostream>
#include <ros/ros.h>
#include <uas_px4.h>
#include <stdlib.h>    
#include <PID.h>
#include <vector>
#include <tf/tf.h>
#include <string>
#include <math.h>

using namespace Eigen;

double constrainAngle(double x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

std::vector<float> get_offset_pos(ros::NodeHandle* nh)
{
    float offset_x;
    float offset_y; 

    std::vector<float> offset_pos;    
    nh->getParam("uav0/offboard_test/offset_x", offset_x);
    nh->getParam("uav0/offboard_test/offset_y", offset_y);
    offset_pos.push_back(0);
    offset_pos.push_back(-10.0);

    return offset_pos;
}

double calc_heading(std::vector<float> some_vec, std::string str1)
{   
    tf::Quaternion q(
    some_vec[3],
    some_vec[4],
    some_vec[5],
    some_vec[6]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;

    
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_px4_header");
    ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
    ros::Rate rate(20.0);

    std::vector<float> offset_pos = get_offset_pos(&_nh);
    
    PX4Drone px4drone(&_nh, offset_pos);

    const float kp = 0.5;
    const float ki = 0.0;
    const float kd = 0.0;
    const float dt = 0.1;
    
    double drone_yaw = 0.0;
    double at_yaw = 0.0;

    std::string str1 = "apriltag";
    std::string str2 = "quad";

    //this should be from parameters
    std::vector<float> init_pos = {3,5,18};
    px4drone.send_init_cmds(init_pos, rate);
    px4drone.set_mode.request.custom_mode = "OFFBOARD";
    px4drone.arm_cmd.request.value = true;


    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        px4drone.setmode_arm(last_request, "OFFBOARD", px4drone.arm_cmd);
        px4drone.send_global_waypoints(init_pos);
        
        PID pid_x(kp, ki, kd, dt, px4drone.kf_tag[0], px4drone.odom[0]);
        PID pid_y(kp, ki, kd, dt, px4drone.kf_tag[1], px4drone.odom[1]);
        PID pid_yaw(kp, ki, kd, dt, at_yaw, drone_yaw);
        float p_x = pid_x.getPID();
        float p_y = pid_y.getPID();
        float p_yaw = pid_yaw.getPID();

        Eigen::Vector2d gain(p_x, p_y);


        //switch case
        switch(px4drone.user_cmd)
        {
            //std::cout<<"tracking"<<std::endl
            case 0:
            {   
                std::cout<<"going to waypoints"<<std::endl;
                px4drone.send_global_waypoints(init_pos);
                break;
            }
            case 1:
            {   
                at_yaw = calc_heading(px4drone.rtag, str1);
                drone_yaw = calc_heading(px4drone.odom, str2);
                double heading_diff = -drone_yaw + at_yaw + (M_PI/2);
                //remainder(heading_diff, 2 *M_PI );
                //std::cout<< "heading diff" << heading_diff * 180 / 3.14159265 << std::endl;
                std::cout<< "at yaw " << at_yaw * 180/(M_PI) << std::endl;
                std::cout<< "drone yaw" << drone_yaw * 180/(M_PI) << std::endl;
                std::cout<< "heading diff" << heading_diff * 180/(M_PI) << std::endl;
                //px4drone.go_follow(gain, 10.0);
                px4drone.send_yaw_cmd(gain ,12, -heading_diff);
                break;
            }
            default:
            {
                px4drone.send_global_waypoints(init_pos);
                std::cout<<"default"<<std::endl;
            }
        }
        //std::cout<<"outside"<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();

    return 0; 
}