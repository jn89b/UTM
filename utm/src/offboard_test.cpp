#include <iostream>
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


#include <moving_avg.h>
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
        ros::Subscriber moving_avg_sub, service_input_sub,utm_position_cmd_sub;
        
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
        float * curr_z_ptr = nullptr;

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
        float init_x; 
        float init_y;
        float init_z;

        //offsets
        float offset_x;
        float offset_y;
        bool avg_stabilize;
        bool begin_land;

        int service_input = 4;
        //utm stuff
        float utm_x;
        float utm_y;
        float utm_z;
        //class pid
        //PID(float kp, float ki, float kd, float dt, float target, float current);

    public:
        Controller()
        {
            //subscribers
            state_sub = nh.subscribe<mavros_msgs::State>
                    ("uav0/mavros/state", 10, &Controller::state_cb, this);
            target_found_sub = nh.subscribe<std_msgs::Bool>
                    ("uav0/target_found", 10, &Controller::target_found_cb,this);
            rtag_quad_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("uav0/tag/pose", 10, &Controller::rtagquad_cb,this);
            rtag_ekf_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("uav0/kf_tag/pose", 10, &Controller::kftag_cb,this);
            quad_odom_sub = nh.subscribe<nav_msgs::Odometry>
                ("uav0/mavros/odometry/in",15, &Controller::quad_odom_callback, this);
            true_quad_odom_sub = nh.subscribe<geometry_msgs::PoseStamped>
                            ("uav0/mavros/local_position/pose",15, &Controller::true_quad_odom_callback, this);
            rtag_ekf_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
                ("uav0/kf_tag/vel", 10, &Controller::kftagvel_cb,this);
            land_permit_sub = nh.subscribe<std_msgs::Bool>
                    ("uav0/precland", 10, &Controller::land_permit_cb,this);
            moving_avg_sub = nh.subscribe<std_msgs::Bool>
                    ("uav0/stabilize_tag", 10, &Controller::moving_avg_cb,this);

            service_input_sub = nh.subscribe<std_msgs::Int8>
                            ("uav0/utm_control", 10, &Controller::landing_service_cb,this);

            utm_position_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("uav0/utm/mavros/setpoint_position/local", 10, &Controller::utm_cb,this);

            local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("uav0/mavros/setpoint_position/local", 10);
            vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                    ("uav0/mavros/setpoint_velocity/cmd_vel", 10);

            
            //services
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                ("uav0/mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                    ("uav0/mavros/set_mode");

            //params for offset from mavros
            nh.getParam("uav0/offboard_landing/offset_x", offset_x);
            nh.getParam("uav0/offboard_landing/offset_y", offset_y);
            
            nh.getParam("uav0/offboard_landing/init_x", init_x);
            nh.getParam("uav0/offboard_landing/init_y", init_y);
            nh.getParam("uav0/offboard_landing/init_z", init_z);
            //services

            //the setpoint publishing rate MUST be faster than 2Hz
            ros::Rate rate(20.0);

            // wait for FCU connection
            while(ros::ok() && !current_state.connected){
                ros::spinOnce();
                rate.sleep();
            }

            //send initial points
            pose.pose.position.x = init_x - offset_x;
            pose.pose.position.y = init_y - offset_y;
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
            while(ros::ok())
            {
                setmode_arm(last_request, "OFFBOARD", arm_cmd);
                float curr_z; 
                
                //PID gains
                PID pid_x(kp, ki, kd, dt, kf_x, odom_x);
                PID pid_y(kp, ki, kd, dt, kf_y, odom_y);

                float p_x = pid_x.getPID();
                float p_y = pid_y.getPID();
                Eigen::Vector2d gain(p_x, p_y);
            
                switch(service_input)
                {   
                    case 0: 
                        waypoint_assignment();
                        break;
                    case 1: // Track 
                        curr_z = go_follow(gain, 4.0);
                        break;
                    case 2: //we can land
                        ROS_INFO("landing");
                        begin_land_protocol(gain);
                        break;
                    case 3: //Land WHere you are at
                        ROS_INFO("sending home");
                        rearm_send_home();
                        //
                        //break;
                    case 4:
                        go_home();
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
        avg_stabilize = false;    
        begin_land = false;
    
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
        kp = 0.15;
        ki = 0.0;
        kd = 0.0;
        dt = 0.1;


        //set decision case to go stay where they are at initially
        service_input = 4;
        landing_decision_case = 1;
    }

    //recieve state of quad
    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    }


    void landing_service_cb(const std_msgs::Int8::ConstPtr& msg)
    {   
        service_input = msg->data;
    }

    void utm_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        utm_x = msg->pose.position.x;
        utm_y = msg->pose.position.y;
        utm_z = msg->pose.position.z;

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
    }

    void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        kf_x = msg->pose.position.x;
        kf_y = msg->pose.position.y;
    }

    void quad_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_x = msg->pose.pose.position.x;
        odom_y = msg->pose.pose.position.y;
        odom_z = msg->pose.pose.position.z;

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
    
    void moving_avg_cb(const std_msgs::Bool::ConstPtr& msg)
    {
        avg_stabilize = msg->data;
    }

    void setmode_arm(ros::Time last_request,const std::string& mode_input, 
    mavros_msgs::CommandBool arm_cmd)
    {
        if( current_state.mode != mode_input &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(set_mode) &&
                set_mode.response.mode_sent){
                //ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    //ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    }
    
    void waypoint_assignment()
    {   
        //listen to utm waypoint commands 
        pose.pose.position.x = utm_x - offset_x;;
        pose.pose.position.y = utm_y - offset_y;
        pose.pose.position.z = utm_z;
        local_pos_pub.publish(pose);
    }

    //Pointer functions -> probably put in a seperate library like how ardupilot does it
    float go_follow(Eigen::Vector2d gain, float z_cmd)
    {   
        //ROS_INFO("following");
        pose.pose.position.x = odom_x + gain[0];
        pose.pose.position.y = odom_y + gain[1];
        pose.pose.position.z = z_cmd; // just testing the loiter
        local_pos_pub.publish(pose);
        //ros::spinOnce();

        return odom_z;
    }

    // this function is too long 
    void begin_land_protocol(Eigen::Vector2d gain )
    {   
        {
            PID pid_x(kp, ki, kd, dt, kf_x, odom_x);
            PID pid_y(kp, ki, kd, dt, kf_y, odom_y);
            float p_x = pid_x.getPID();
            float p_y = pid_y.getPID();
            Eigen::Vector2d gain(p_x, p_y);
            check_landing_cases(); 
            switch(landing_decision_case)
            {
                case 1: //drop slowly to landing target
                    ROS_INFO("dropping down slowly");
                    pre_land_protocol(gain, 0.35);
                    break;
                case 2:
                    ROS_INFO("disarming");
                    land_disarm_protcol();
                    break;
                case 3:
                    ROS_INFO("already landed");
                    already_landed();
                    break;
                case 4:
                    ROS_INFO("stablizing");
                    stabilize(gain, odom_z);
                    break;
            }
            local_pos_pub.publish(pose);
        }
    }

    void already_landed()
    {
        ROS_INFO("I have landed");
    }

    void go_home()
    {
        pose.pose.position.x = init_x - offset_x;
        pose.pose.position.y = init_y - offset_y;
        pose.pose.position.z = init_z;
        local_pos_pub.publish(pose);
    }
    

    void check_landing_cases()
    {
        // should be a hashtable or struct
        const float z_land = 0.80; //this is an offset from the actual height
        //const float z_land = 0.9;    //need to set this as offset
        float tol = 0.1;
        
        if (odom_z >= z_land)
        {
            landing_decision_case = 1; // drop down slowly
        }
        else if (odom_z <= z_land)
        {
            landing_decision_case = 2; // begin landing
        }
        else if (current_state.armed == false)
        {
            landing_decision_case = 3; // we landed already
        }
        else
        {
            landing_decision_case = 4; // just stabilize
        }
    }

    void pre_land_protocol(Eigen::Vector2d gain, float drop_down_val)
    {
        pose.pose.position.x = odom_x + gain[0];
        pose.pose.position.y = odom_y + gain[1]; // pretty much keep at where we are 
        pose.pose.position.z = odom_z - drop_down_val; //keep it at this general area 
    }

    void stabilize(Eigen::Vector2d gain, float current_z)
    {
        pose.pose.position.x = odom_x + gain[0];
        pose.pose.position.y = odom_y + gain[1]; // pretty much keep at where we are 
        pose.pose.position.z = current_z; //keep it at this general area 
        local_pos_pub.publish(pose);
        ros::spinOnce();
    }

    void land_disarm_protcol()
    {   // need to clean this up or check if you're out of sight...
            //ROS_INFO("Beginning Land");
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
                ROS_INFO("I have disarmed");
                if (service_input == 3) 
                {
                    ROS_INFO("I hear a post flight");
                    break;
                }
            }
    }

    void rearm_send_home()
    {
        ros::Rate rate(20.0);
        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }
        
        //send initial points
        pose.pose.position.x = offset_x;
        pose.pose.position.y = offset_y;
        pose.pose.position.z = init_z;
                    
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        ros::Time last_request = ros::Time::now();
        set_mode.request.custom_mode = "OFFBOARD";
        arm_cmd.request.value = true;

        while(ros::ok()){
            setmode_arm(last_request, "OFFBOARD", arm_cmd);
            waypoint_assignment();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_tracking");
    Controller control;

    return 0; 
}

