#include <uas_px4.h>

PX4Drone::PX4Drone(ros::NodeHandle* nh, std::vector<float> offset_pos)
{
    //subscribers
    state_sub = nh->subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, &PX4Drone::state_cb, this);
            
    quad_odom_sub = nh->subscribe<nav_msgs::Odometry>
        ("uav0/mavros/odometry/in",15, &PX4Drone::quad_odom_cb, this);

    true_quad_odom_sub = nh->subscribe<geometry_msgs::PoseStamped>
                    ("uav0/mavros/local_position/pose",20, &PX4Drone::true_odom_cb, this);
    
    local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 20);
    
    vel_pub = nh->advertise<geometry_msgs::TwistStamped>
            ("uav0/mavros/setpoint_velocity/cmd_vel", 20);

    //raw publisher
    cmd_raw = nh->advertise<mavros_msgs::AttitudeTarget>
            ("uav0/mavros/setpoint_raw/attitude", 10);

    //apriltag crap
    rtag_quad_sub = nh->subscribe<geometry_msgs::PoseStamped>
            ("uav0/tag/pose", 10, &PX4Drone::rtagquad_cb,this); 
    rtag_ekf_sub = nh->subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/vision_pose/pose", 10, &PX4Drone::kftag_cb,this);

    rtag_ekf_vel_sub = nh->subscribe<geometry_msgs::TwistStamped>
            ("/uav0/kf_tag/vel", 10, &PX4Drone::kftag_vel_cb,this);

    //services
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>
        ("uav0/mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");

    //service input
    service_input_sub = nh->subscribe<std_msgs::Int8>
                    ("utm_control", 10, &PX4Drone::user_cmd_cb,this);

    lqr_gain_sub = nh->subscribe<utm::LQRGain>
                    ("K_gain", 20, &PX4Drone::lqr_cb,this);



    //velocity

    // offset_pos = {0,0};
    init_pos = {0,0,0};

    // //params for offset from mavros
    // nh->getParam("offboard_landing/offset_x", offset_pos[0]);
    // nh->getParam("offboard_landing/offset_y", offset_pos[1]);
    
    nh->getParam("offboard_landing/init_x", init_pos[0]);
    nh->getParam("offboard_landing/init_y", init_pos[1]);
    nh->getParam("offboard_landing/init_z", init_pos[2]);
    
    init_vals(offset_pos);
}

void PX4Drone::init_vals(std::vector<float> offset_pos)
{   
    //set target found and landing permit to false initially 
    _offset_pos = offset_pos;
    odom = {0,0,0,0,0,0,0};
    rtag = {0,0,0,0,0,0,0};
    kf_tag = {0,0,0,0,0,0,0};
    
    vel = {0,0};
    kf_vel = {0,0};

    true_odom_z = 0.0;

    //PID stuff
    pre_error_x = 0.0;
    pre_error_y = 0.0;
    pre_ierror_x = 0.0;
    pre_ierror_y = 0.0;
}

//recieve state of quad
void PX4Drone::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void PX4Drone::quad_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom[0] = msg->pose.pose.position.x;
    odom[1] = msg->pose.pose.position.y;
    odom[2] = msg->pose.pose.position.z;

    odom[3] = msg->pose.pose.orientation.x;
    odom[4] = msg->pose.pose.orientation.y;
    odom[5] = msg->pose.pose.orientation.z;
    odom[6] = msg->pose.pose.orientation.w;
}

//accessor function to get odometry of PX4 Drone
void PX4Drone::get_odom_pos()
{   
    float x = odom[0];
    float y = odom[1];
    std::cout<<"odometry of Drone is"<< x << "," <<  y << std::endl;
}

void PX4Drone::true_odom_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    true_odom_z = msg->pose.position.z;
}

void PX4Drone::setmode_arm(ros::Time last_request,
const std::string& mode_input, mavros_msgs::CommandBool arm_cmd)
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

void PX4Drone::send_init_cmds(std::vector<float> position, ros::Rate rate)
{
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    } 

    send_global_waypoints(position);
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

//send global waypoint commands
void PX4Drone::send_global_waypoints(std::vector<float> wp_vector)
{
    //send initial points
    pose.pose.position.x = wp_vector[0] - _offset_pos[0];
    pose.pose.position.y = wp_vector[1] - _offset_pos[1];
    pose.pose.position.z = wp_vector[2];
    local_pos_pub.publish(pose);
}

//track target with PID gains
void PX4Drone::go_follow(Eigen::Vector2d gain, float z_cmd)
{   
    //ROS_INFO("following");
    pose.pose.position.x = odom[0] + gain[0];
    pose.pose.position.y = odom[1] + gain[1];
    pose.pose.position.z = z_cmd; // just testing the loiter
    local_pos_pub.publish(pose);
    //ros::spinOnce();
}

void PX4Drone::lqr_track()
{
    // mavros_msgs::AttitudeTarget bodyrate_msg;
    // bodyrate_msg.body_rate.x = lqr_gain[3];
    // bodyrate_msg.body_rate.y = 0.0;
    // bodyrate_msg.body_rate.z = 0.0;
    // bodyrate_msg.thrust = 0.4;//lqr.getOutput()(3)/17;
    // bodyrate_msg.type_mask = 128;
    // cmd_raw.publish(bodyrate_msg);
    // //this tag is basically how far it is 

    cmd_vel.twist.linear.x = lqr_gain[0];
    cmd_vel.twist.linear.y = lqr_gain[1];
    //cmd_vel.twist.linear.y = vel[1] + gain[1];
    vel_pub.publish(cmd_vel);
}


void PX4Drone::lqr_cb(const utm::LQRGain::ConstPtr& msg)
{
    lqr_gain[0] = msg->data[0];
    lqr_gain[1] = msg->data[1];
    lqr_gain[2] = msg->data[2];
    lqr_gain[3] = msg->data[3];
    //std::cout<<"gains" << lqr_gain[3] << std::endl;
}

void PX4Drone::rtagquad_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    rtag[0] = msg->pose.position.x;
    rtag[1] = msg->pose.position.y;
    rtag[2] = msg->pose.position.z;
    
    rtag[3] = msg->pose.orientation.x;
    rtag[4] = msg->pose.orientation.y;
    rtag[5] = msg->pose.orientation.z;
    rtag[6] = msg->pose.orientation.w;
}

void PX4Drone::kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    kf_tag[0] = msg->pose.position.x;
    kf_tag[1] = msg->pose.position.y;
    kf_tag[2] = msg->pose.position.z;
    kf_tag[3] = msg->pose.orientation.x;
    kf_tag[4] = msg->pose.orientation.y;
    kf_tag[5] = msg->pose.orientation.z;
    kf_tag[6] = msg->pose.orientation.w;
}

void PX4Drone::kftag_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    kf_vel[0] = msg->twist.linear.x;
    kf_vel[1] = msg->twist.linear.y;
}

void PX4Drone::user_cmd_cb(const std_msgs::Int8::ConstPtr& msg)
{   
    user_cmd = msg->data;
}

void PX4Drone::send_yaw_cmd(Eigen::Vector2d gain, float z_cmd, float yaw)
{
    tf::Quaternion quaternion_;
    quaternion_.setRPY(0.01, 0.01, yaw);
    quaternion_.normalize();
    pose.pose.position.x = odom[0] + gain[0];
    pose.pose.position.y = odom[1] + gain[1];
    pose.pose.position.z = 10; // just testing the loiter
    pose.pose.orientation.x = quaternion_.x();
    pose.pose.orientation.y = quaternion_.y();
    pose.pose.orientation.z = quaternion_.z();
    pose.pose.orientation.w = quaternion_.w();
    local_pos_pub.publish(pose);
}

void PX4Drone::send_velocity_cmd(Eigen::Vector2d gain)
{
    cmd_vel.twist.linear.x = vel[0] + gain[0];
    cmd_vel.twist.linear.y = vel[1] + gain[1];
    vel_pub.publish(cmd_vel);
}

void PX4Drone::begin_land_protocol(Eigen::Vector2d gain, ros::Rate rate)
{   
    const float land_height = 0.8;
    const float dropping = 0.25;

    if (rtag[2]>= land_height){
        //std::cout<<"starting to land"<<std::endl;
        go_follow(gain, odom[2]-dropping);
    }
    else{
        ros::Time last_request = ros::Time::now();  
        set_mode.request.custom_mode = "AUTO.LAND";
        arm_cmd.request.value = false;
        while(ros::ok() && (current_state.mode != "AUTO.LAND")){
            go_follow(gain, odom[2]);
            setmode_arm(last_request, set_mode.request.custom_mode , arm_cmd);
            ros::spinOnce();
            rate.sleep();
            if (user_cmd != 2) 
            {
                ROS_INFO("I hear a different command");
                return;
            }
        }
    }
}

void PX4Drone::set_offboard(std::vector<float> pos_cmd,  ros::Rate rate)
{
    while ((ros::ok()) && (current_state.mode != "OFFBOARD"))
    {   
        // std::cout<<"sending command"<<std::endl;
        ros::Time last_request = ros::Time::now();
        
        if (current_state.mode == "OFFBOARD")
        {
            std::cout<<"i am in offboard"<<std::endl;
            return;
        }        

        send_init_cmds(pos_cmd, rate);
        set_mode.request.custom_mode = "OFFBOARD";
        arm_cmd.request.value = true;
        setmode_arm(last_request, set_mode.request.custom_mode , arm_cmd);
        send_global_waypoints(init_pos);
        ros::spinOnce();
        rate.sleep();
    }
}