#include <iostream>
#include <ros/ros.h>
#include <uas_px4.h>
#include <stdlib.h>    
#include <PID.h>
#include <vector>
#include <tf/tf.h>
#include <string>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>


using namespace Eigen;

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

int main(int argc, char **argv)
{

    const float kp = 0.3; //0.35
    const float ki = 0.0;
    const float kd = 0.000001; //0.00001
    const float dt = 0.05;
    const float gain_tol = 0.075;
    const float heading_bound = M_PI/6; //45 degrees
    const float rate_val = 20.0;    

    bool spider_protocol = true;

    ros::init(argc, argv, "test_px4_header");
    ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
    ros::Rate rate(rate_val);


    std::vector<float> offset_pos = get_offset_pos(&_nh);
    
    PX4Drone px4drone(&_nh, offset_pos);
    double drone_yaw = 0.0;
    double at_yaw = 0.0;

    //this should be from parameters
    std::vector<float> init_pos = {5.0,5.0,15.0};
    px4drone.send_init_cmds(init_pos, rate);
    px4drone.set_mode.request.custom_mode = "AUTO.LAND";
    px4drone.arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    const float att_angle = 0.785398; //45 degrees

    while(ros::ok()){

        switch(px4drone.user_cmd)
        {
            case 0:
            {   
                if (px4drone.current_state.mode == "OFFBOARD"){
                    px4drone.send_global_waypoints(init_pos);
                }
                else{
                    px4drone.set_offboard(init_pos, rate);
                }
                
                break;
            }
            case 1: //IM TESTING THE ATTITUDE POSITION FOR DATA
            {
                px4drone.send_att_cmd(att_angle, 0.0, 0.0);
                break;
            }
            case 2: // Precision Land with LQR
            {
                px4drone.send_att_cmd(0.0, att_angle, 0.0);
                break;
            }
            default:
            {
                if (px4drone.current_state.mode == "OFFBOARD"){
                    px4drone.send_global_waypoints(init_pos);
                }
                else{
                    px4drone.set_offboard(init_pos, rate);
                }
            }
        }
        //std::cout<<"outside"<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    //ros::spin();

    return 0; 
}