#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <moving_avg.h>

double kf_x;
double kf_y;
double mag_avg;

void init_vals()
{
    kf_x = 0.0;
    kf_y = 0.0;
    mag_avg = 0.0;  
}

void kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   kf_x = msg->pose.position.x;
   kf_y = msg->pose.position.y;
}


int main(int argc, char **argv)
{   
    ros::init(argc,argv,"moving_avg_main");
    ros::NodeHandle nh;
     
    init_vals();    

    ros::Subscriber rtag_ekf_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("tag/pose", 10, &kftag_cb);
                    
    MovingAverage moving_avg(kf_x, kf_y);
    
    ros::Rate rate(20.0);
    
    while (ros::ok()){
        double kf_avg_x = moving_avg.compute_avg(kf_x);
        double kf_avg_y = moving_avg.compute_avg(kf_y);
        double _avg_mag = sqrt(pow(kf_avg_x,2) + pow(kf_avg_y,2));
        std::cout<< "avg: " << _avg_mag << std::endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;   
}



