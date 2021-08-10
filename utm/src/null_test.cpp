#include <iostream>
#include <null_test.h>

TestNull::TestNull(ros::NodeHandle* nodehandle, float kf_x, float kf_y)
{

    _rtag_ekf_sub = _nh.subscribe<geometry_msgs::PoseStamped>
            ("kf_tag/pose", 10, &TestNull::kftag_cb,this);

    _test_pub = _nh.advertise<geometry_msgs::PoseStamped>
                ("test_pub", 10);

    //initVals(float kf_x, float kf_y);
    //initSubscribers();
    printDetection();
    //mainMethod();

    //initiate any vaariables here
    _val_to_remember= 0.0;
}


void TestNull::printDetection()
{
    if (_kf_x == NULL && _kf_y == NULL)
    {
        ROS_INFO("no tag detected");
    } 
}

void TestNull::kftag_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    _kf_x = msg->pose.position.x;
    _kf_y = msg->pose.position.y;
    _val_to_remember += _kf_x;
    
    geometry_msgs::PoseStamped pose; 
    pose.pose.position.x = _val_to_remember;
    _test_pub.publish(pose);

    std::cout << "val:" << _val_to_remember << std::endl;
}
