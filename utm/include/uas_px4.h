#ifndef NULL_UAS_PX4_H
#define UAS_PX4_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <PID.h>

class PX4Drone
{
    private:
        ros::NodeHandle nh;

        ros::Publisher local_pos_pub, vel_pub;

        ros::Subscriber state_sub, target_found_sub, 
            rtag_ekf_sub,rtag_quad_sub, quad_odom_sub;

        ros::Subscriber rtag_ekf_vel_sub, land_permit_sub,
             true_quad_odom_sub;
        
        //mavros service clients for arming and setting modes
        ros::ServiceClient arming_client, set_mode_client;

        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped pose;       
        
        //Odometry of quad with offset because Airsim does not like to play nice
        std::vector<float> odom;

        float true_odom_z; // this is the true odometry 
    
        float pre_error_x;
        float pre_error_y;

        float pre_ierror_x;
        float pre_ierror_y;

        float kp;
        float ki;
        float kd;
        float dt;

        //initial pose commands
        std::vector<float> init_pos;

        //offsets
        std::vector<float> _offset_pos;

    public:
        mavros_msgs::SetMode set_mode;
        mavros_msgs::CommandBool arm_cmd;

        PX4Drone(ros::NodeHandle* nh, std::vector<float> offset_pos);
        void init_vals(std::vector<float> offset_pos);
        
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void quad_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void true_odom_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
        
        //print logs 
        void get_odom_pos(); 

        void setmode_arm(ros::Time last_request,
            const std::string& mode_input, mavros_msgs::CommandBool arm_cmd);
        void send_init_cmds(std::vector<float> position, ros::Rate rate);
        void send_global_waypoints(std::vector<float> wp_vector);
        

};

#endif