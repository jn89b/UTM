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

#include <PID.h>

using namespace Eigen;
using std::string;


class Controller
{
    private:
        ros::NodeHandle nh;
        ros::Publisher local_pos_pub, vel_pub;
        ros::Subscriber state_sub, target_found_sub, rtag_ekf_sub,rtag_quad_sub, quad_odom_sub;
        ros::Subscriber rtag_ekf_vel_sub, land_permit_sub, true_quad_odom_sub;
        
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

        //Odometry of quad with offset because Airsim does not like to play nice
        float odom_x;
        float odom_y;
        float odom_z;

        float true_odom_z; // this is the true odometry 
    
        float pre_error_x;
        float pre_error_y;

        float pre_ierror_x;
        float pre_ierror_y;

        float kp;
        float ki;
        float kd;
        float dt;

        int decision_case;
        int landing_decision_case;

        //initial pose commands
        float init_x = 0.0;
        float init_y = 3.0;
        float init_z = 4.0;

        //class pid
        //PID(float kp, float ki, float kd, float dt, float target, float current);

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
                ("mavros/offset_local_position/pose",15, &Controller::quad_odom_callback, this);
            true_quad_odom_sub = nh.subscribe<geometry_msgs::PoseStamped>
                            ("mavros/local_position/pose",15, &Controller::true_quad_odom_callback, this);
            rtag_ekf_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("kf_tag/vel", 10, &Controller::kftagvel_cb,this);
            land_permit_sub = nh.subscribe<std_msgs::Bool>
                    ("precland", 10, &Controller::land_permit_cb,this);

            local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("mavros/setpoint_position/local", 10);
            vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                    ("/mavros/setpoint_velocity/cmd_vel", 10);
            
            //services
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
            pose.pose.position.x = init_x;
            pose.pose.position.y = init_y;
            pose.pose.position.z = init_z;

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
                
                setmode_arm(last_request, "OFFBOARD", arm_cmd);

                //PID gains
                PID pid_x(kp, ki, kd, dt, kf_x, odom_x);
                PID pid_y(kp, ki, kd, dt, kf_y, odom_y);

                float p_x = pid_x.getPID();
                float p_y = pid_y.getPID();
                Eigen::Vector2d gain(p_x, p_y);
                
                check_case();
                switch(decision_case)
                {
                    case 1: // we found the tag
                        go_follow(gain);
                        //ROS_INFO("Follow");
                        break;
                    case 2: //we can land
                        begin_land_protocol(gain);
                        //ROS_INFO("landing");
                        break;
                    case 3: //we don't have the tag
                        go_home();
                        //ROS_INFO("go home");
                        break;
                }
                ros::spinOnce();
                rate.sleep();
            }       
        }

    void init_vals()
    {   
        //set target found and landing permit to false initially 
        target_found = false;
        land_permit = false;
        
        //rotation matrix  
        r_x = 0.0;
        r_y = 0.0;
        r_z = 0.0;
        r_qx = 0.0;
        r_qy = 0.0;
        r_qz = 0.0;
        r_qw = 0.0;

        //kalman filter
        kf_x = 0.0;
        kf_y = 0.0;
        kf_vel_x = 0.0;
        kf_vel_y = 0.0;

        //odometry of quad
        odom_x = 0.0;
        odom_y = 0.0;
        odom_z = 0.0; 

        true_odom_z = 0.0;

        //intial error estimates
        pre_error_x = 0.0;
        pre_error_y = 0.0;

        pre_ierror_x = 0.0;
        pre_ierror_y = 0.0;

        //set this into a text/config file 
        kp = 0.45;
        ki = 1E-3;
        kd = 0.0;
        dt = 0.1;

        //set decision case to go stay where they are at initially
        decision_case = 3;
        landing_decision_case = 3;
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
    }

    void true_quad_odom_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        true_odom_z = msg->pose.position.z;
    }

    void kftagvel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {   
        kf_vel_x = msg->twist.linear.x;
        kf_vel_y = msg->twist.linear.y;
    }
    
    void setmode_arm(ros::Time last_request,const std::string& mode_input, 
    mavros_msgs::CommandBool arm_cmd)
    {
        if( current_state.mode != mode_input &&
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
    }

    void check_case()
    {
        if(target_found==true and land_permit == false){
            decision_case = 1; // we found the tag
        }
        else if (land_permit == true){
            decision_case = 2; //we can land
        }
        else{
            decision_case = 3;
        }
    }
    
    //Pointer functions -> probably put in a seperate library like how ardupilot does it
    void go_follow(Eigen::Vector2d gain)
    {   
        ROS_INFO("following");
        pose.pose.position.x = odom_x - gain[0];
        pose.pose.position.y = odom_y - gain[1];
        pose.pose.position.z = 4.0; // just testing the loiter

        local_pos_pub.publish(pose);
        ros::spinOnce();
    }

    // this function is too long 
    void begin_land_protocol(Eigen::Vector2d gain)
    {   
        check_landing_cases(); 
        switch(landing_decision_case)
        {
            case 1: //drop slowly to landing target
                pre_land_protocol(gain);
                break;
            case 2:
                land_disarm_protcol();
                break;
            case 3:
                already_landed();
                break;
            case 4:
                pre_land_protocol(gain);
                break;
        }
        local_pos_pub.publish(pose);   
    }

    void already_landed()
    {
        ROS_INFO("I have landed");
    }

    void go_home()
    {
        pose.pose.position.x = init_x;
        pose.pose.position.y = init_y;
        pose.pose.position.z = init_z;
        local_pos_pub.publish(pose);
        //printf("I want to go home");
    }
    
    void go_find()
    {   
        //want to keep last time we heard target found if we have it 
        printf("I'm lost");
    }

    void check_landing_cases()
    {
        // should be a hashtable or struct
        float pre_land = 0.95; //this is an offset from the actual height
        float z_land = 0.8; //need to set this as offset
        float tol = 0.15;
       
        if ((odom_z < pre_land) && (odom_z > z_land) && (abs(kf_x) < tol) && (abs(kf_y) < tol))
        {
            landing_decision_case = 1; // we're above the prelanding zone so keep dropping down
        }
        else if ((odom_z < z_land) && (abs(kf_x) > tol) && (abs(kf_y) > tol))
        {
            landing_decision_case = 2; // we're about to land
        }
        else if (current_state.armed == false && odom_z <= 0.6)
        {
            landing_decision_case = 3; // we lost sight?
        }
        else
        {
            landing_decision_case = 4; // keep dropping
        }
    }

    void pre_land_protocol(Eigen::Vector2d gain)
    {
        ROS_INFO("Beginning Prelanding");

        pose.pose.position.x = odom_x - gain[0];
        pose.pose.position.y = odom_y - gain[1]; // pretty much keep at where we are 
        pose.pose.position.z = odom_z - 0.001; //keep it at this general area 
    }

    void land_disarm_protcol()
    {   // need to clean this up or check if you're out of sight...
        while(ros::ok())
        {
            ROS_INFO("Beginning Land");
            ros::Rate rate(20.0);
            
            //set to prelanding hover/ hover to around 
            set_mode.request.custom_mode = "AUTO.LAND";
            arm_cmd.request.value = false;

            ros::Time last_request = ros::Time::now();

            //had to set 
            while(ros::ok())
            {
                setmode_arm(last_request, "AUTO.LAND", arm_cmd);
                ros::spinOnce();
                rate.sleep();
            }
        }
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_tracking");
    Controller control;

    return 0; 
}


