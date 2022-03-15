#include <uas_px4.h>

PX4Drone::PX4Drone(ros::NodeHandle* nh, std::vector<float> offset_pos)
{
    //subscribers
    state_sub = nh->subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, &PX4Drone::state_cb, this);
            
    quad_odom_sub = nh->subscribe<nav_msgs::Odometry>
        ("uav0/mavros/odometry/in",15, &PX4Drone::quad_odom_cb, this);

    true_quad_odom_sub = nh->subscribe<geometry_msgs::PoseStamped>
                    ("uav0/mavros/local_position/pose",15, &PX4Drone::true_odom_cb, this);
    
    local_pos_pub = nh->advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    
    vel_pub = nh->advertise<geometry_msgs::TwistStamped>
            ("uav0/mavros/setpoint_velocity/cmd_vel", 10);

    //services
    arming_client = nh->serviceClient<mavros_msgs::CommandBool>
        ("uav0/mavros/cmd/arming");
    set_mode_client = nh->serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");

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
    odom = {0,0,0};
    true_odom_z = 0.0;

    //PID stuff
    pre_error_x = 0.0;
    pre_error_y = 0.0;
    pre_ierror_x = 0.0;
    pre_ierror_y = 0.0;
    kp = 0.8;
    ki = 0.0;
    kd = 0.0;

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
