#include <iostream>
#include <ros/ros.h>
#include <uas_px4.h>
#include <vector>

std::vector<float> get_offset_pos(ros::NodeHandle* nh)
{
    float offset_x;
    float offset_y; 

    std::vector<float> offset_pos;    
    nh->getParam("uav0/offboard_test/offset_x", offset_x);
    nh->getParam("uav0/offboard_test/offset_y", offset_y);
    offset_pos.push_back(0);
    offset_pos.push_back(-10.0);

    //offset_pos = {offset_x, offset_y};
    //std::cout<< "offset position" << offset_x <<" " << offset_y << std::endl;

    return offset_pos;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_px4_header");
    ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
    ros::Rate rate(20.0);

    std::vector<float> offset_pos = get_offset_pos(&_nh);
    
    PX4Drone px4drone(&_nh, offset_pos);
    
    //this should be from parameters
    std::vector<float> init_pos = {50,50,5};
    px4drone.send_init_cmds(init_pos, rate);

    px4drone.set_mode.request.custom_mode = "OFFBOARD";
    px4drone.arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        px4drone.setmode_arm(last_request, "OFFBOARD", px4drone.arm_cmd);
        px4drone.send_global_waypoints(init_pos);
        //std::cout<<"drone position: "<< px4drone.get_odom_pos() << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0; 
}