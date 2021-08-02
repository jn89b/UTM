#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace Eigen;
using std::string;

/*
Controller class does all the handling of the arming and position navigation, inherit flight mode class
*/

class Controller
{
    private:
        ros::NodeHandle nh;
        ros::Publisher local_pos_pub, vel_pub;
        ros::Subscriber state_sub, target_found_sub, rtag_ekf_sub,rtag_quad_sub, quad_odom_sub;
        ros::Subscriber rtag_ekf_vel_sub, land_permit_sub;
        
        //mavros service clients for arming and setting modes
        ros::ServiceClient arming_client, set_mode_client;

        mavros_msgs::State current_state;
        mavros_msgs::SetMode set_mode;
        mavros_msgs::CommandBool arm_cmd;

        //
        geometry_msgs::PoseStamped pose;       
        
        //apriltag found
        bool target_found;
        bool land_permit;
        //Position vectors 
        float r_x; // x tag wrt quad 
        float r_y; // y tag wrt quad
        float r_z; // z tag wrt quad -> Don't really need this can use actual quad to true ground
        // Quaternion orientation vectors
        float r_qx;
        float r_qy;                 
        float r_qz;
        float r_qw;

        //kalman filter estimate
        float kf_x;
        float kf_y;
        float kf_vel_x;
        float kf_vel_y;
        //quad odom
        //Odometry of quad
        float odom_x;
        float odom_y;
        float odom_z;
    
        float pre_error_x;
        float pre_error_y;

        float pre_ierror_x;
        float pre_ierror_y;

        float kp;
        float ki;
        float kd;

        int decision_case;

        //Vector3f r_pos;

    public:
        Controller()
        {
            //subscribers
            state_sub = nh.subscribe<mavros_msgs::State>
                    ("mavros/state", 10, &Controller::state_cb, this);
            target_found_sub = nh.subscribe<std_msgs::Bool>
                    ("target_found", 10, &Controller::target_found_cb,this);
            rtag_quad_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("tag/pose", 10, &Controller::rtagquad_cb,this);
            rtag_ekf_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("kf_tag/pose", 10, &Controller::kftag_cb,this);
            quad_odom_sub = nh.subscribe<geometry_msgs::PoseStamped>
                ("mavros/local_position/pose",50, &Controller::quad_odom_callback, this);
            rtag_ekf_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("kf_tag/vel", 10, &Controller::kftagvel_cb,this);
            land_permit_sub = nh.subscribe<std_msgs::Bool>
                    ("precland", 10, &Controller::land_permit_cb,this);

            local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("mavros/setpoint_position/local", 10);
            vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                    ("/mavros/setpoint_velocity/cmd_vel", 10);
            

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                    ("mavros/set_mode");

            //the setpoint publishing rate MUST be faster than 2Hz
            ros::Rate rate(20.0);

            // wait for FCU connection
            while(ros::ok() && !current_state.connected){
                ros::spinOnce();
                rate.sleep();
            }

            //send initial points
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 3.0;
            pose.pose.position.z = 4.5;

            //send a few setpoints before starting
            for(int i = 100; ros::ok() && i > 0; --i){
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }

            init_vals();

            set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;

            ros::Time last_request = ros::Time::now();

            while(ros::ok()){
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(set_mode) &&
                        set_mode.response.mode_sent){
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    if( !current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                        }
                        last_request = ros::Time::now();
                    }
                }
                
                //set value to case then check case
                check_case();
                //std::cout << "case number:" << decision_case << std::endl;
                switch(decision_case)
                
                {
                    case 1: // we found the tag
                        go_follow();
                        break;
                    case 2: //we can land
                        go_land();
                        break;
                    case 3: //we don't have the tag
                        go_home();
                        break;
                }

                ros::spinOnce();
                rate.sleep();
            }       
        }

    void init_vals()
    {    
        target_found = false;
        land_permit = false;

        r_x = 0.0;
        r_y = 0.0;
        r_z = 0.0;
        r_qx = 0.0;
        r_qy = 0.0;
        r_qz = 0.0;
        r_qw = 0.0;

        kf_x = 0.0;
        kf_y = 0.0;
        kf_vel_x = 0.0;
        kf_vel_y = 0.0;

        odom_x = 0.0;
        odom_y = 0.0;
        odom_z = 0.0; 

        pre_error_x = 0.0;
        pre_error_y = 0.0;

        pre_ierror_x = 0.0;
        pre_ierror_y = 0.0;

        //set this into a text/config file 
        kp = 0.7;
        ki = 1E-8;
        kd = 0.0;

        decision_case = 3;

    }

    //recieve state of quad
    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    }

    void target_found_cb(const std_msgs::Bool::ConstPtr& msg)
    {
        target_found = msg->data;
        //std::cout << "target_found:" << target_found << std::endl;
    }

    void land_permit_cb(const std_msgs::Bool::ConstPtr& msg)
    {
        land_permit = msg->data;
        //std::cout << "target_found:" << target_found << std::endl;
    }

    void rtagquad_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        r_z = msg->pose.position.x;
        r_y = msg->pose.position.y;
        r_z = msg->pose.position.z;

        r_qx = msg->pose.orientation.x;
        r_qy = msg->pose.orientation.y;
        r_qz = msg->pose.orientation.z;
        r_qw = msg->pose.orientation.w;

        //Eigen::Vector3d r_pos(r_x, r_y, r_z);
    }

    void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        kf_x = msg->pose.position.x;
        kf_y = msg->pose.position.y;
    }

    void quad_odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        odom_x = msg->pose.position.x;
        odom_y = msg->pose.position.y;
        odom_z = msg->pose.position.z;
        //ROS_INFO("odom_x: %f, odom_y:", odom_x, odom_y); 
    }

    void kftagvel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {   
        kf_vel_x = msg->twist.linear.x;
        kf_vel_y = msg->twist.linear.y;
    }

    Eigen::Vector2d calc_PID(float desired_x, float desired_y, float curr_x, float curr_y, double dt)
    {
        ROS_INFO("calculating");
        //calculate error 
        float error_x = curr_x - desired_x;
        float error_y = curr_y - desired_y;
        //proportional val
        float Pgain_x = kp * error_x;
        float Pgain_y = kp * error_y;

        //integral error 
        float int_error_x = pre_ierror_x + ((error_x + pre_error_x)/ 2);
        float int_error_y = pre_ierror_y + ((error_y + pre_error_y)/ 2);
        //integral gain
        float Igain_x = ki * int_error_x;
        float Igain_y = ki * int_error_y;
        
        //derivative error
        double der_error_x = (error_x - pre_error_x) / dt;
        double der_error_y = (error_y - pre_error_y) / dt;
        float Dgain_x = kd * der_error_x;
        float Dgain_y = kd * der_error_y;
        //std::cout << "Dererror:" << der_error_x << std::endl;
        

        float PID_x = Pgain_x + Igain_x + Dgain_x;
        float PID_y = Pgain_y + Igain_y + Dgain_y;

        Eigen::Vector2d PID(PID_x, PID_y);
        std::cout << "Dgainx:" << Dgain_x << std::endl;
        std::cout << "Dgainy:" << Dgain_y << std::endl;
        //save error
        pre_error_x = error_x; 
        pre_error_y = error_y;

        pre_ierror_x = int_error_x;
        pre_ierror_y = int_error_y;

        return PID;
    }
    
    void check_case()
    {
        if(target_found==true and land_permit == false){
            decision_case = 1; // we found the tag
        }
        else if (target_found==true && land_permit == true){
            decision_case = 2; //we can land
        }
        else{
            decision_case = 3;
        }


    }
    //Pointer functions -> probably put in a seperate library like how ardupilot does it
    void go_follow()
    {   
        //kalman filter and kalman y are already the desired positions
        Eigen::Vector2d gain = calc_PID(kf_x, kf_y, odom_x, odom_y, 0.001);
        //publish position of tag with kalman fitler
        pose.pose.position.x = gain[0];
        pose.pose.position.y = gain[1];
        std::cout << "gain:" << gain << std::endl;
        pose.pose.position.z = 1.5; // just testing the loiter
        local_pos_pub.publish(pose);
        ros::spinOnce();
        check_case();
    //printf("woof woof I follow");
    }

    void go_land()
    {
        Eigen::Vector2d gain = calc_PID(kf_x, kf_y, odom_x, odom_y, 0.001);
        //publish position of tag with kalman fitler
        pose.pose.position.x = gain[0];
        pose.pose.position.y = gain[1];
        std::cout << "gain:" << gain << std::endl;
        pose.pose.position.z = 1.25; // just testing the loiter
        local_pos_pub.publish(pose);
        set_mode.request.custom_mode = "AUTO.LAND";        
        ros::spinOnce();
        check_case();
        //printf("I want to land");
    }

    void go_home()
    {
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 3.5;
        pose.pose.position.z = 4.5;
        //std::cout << "no target:" << target_found << std::endl;
        local_pos_pub.publish(pose);
        //printf("I want to go home");
    }
    
    void go_find()
    {   
        //want to keep last time we heard target found if we have it 
        printf("I'm lost");
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_tracking");
    Controller control;

    return 0; 
}


