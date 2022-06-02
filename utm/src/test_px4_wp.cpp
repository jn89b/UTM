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
#include <tf2/LinearMath/Quaternion.h>
#include <list>


using namespace Eigen;


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


bool is_at_position(std::vector<float> curr_pos, std::vector<float> des_wp, float radius)
{ /*check if reached to waypoint by checking radius bounds return true if so, false if not
  does this by calculating the norm between the current position and desired position 
  and checks within radius bubble of waypoint 
  */ 
  // VectorXcd des_vec = VectorXcd::Map(des_wp.data(), des_wp.size());
  Eigen::Vector3d curr_vec(curr_pos[0], curr_pos[1], curr_pos[2]);
  Eigen::Vector3d desired_wp(des_wp[0], des_wp[1], des_wp[2]);  
  // std::cout<< "desired position" << desired_wp << std::endl;
  // std::cout<< "current vector" << curr_vec << std::endl;
  return (abs((curr_vec-desired_wp).norm()) <= radius);
}

// bool reached_position(PX4Drone some_drone, std::vector<float> des_position, float radius, ros::Rate rate)
// {
//   //break off from this function if reached the waypoint, might need to add a timeoout catch
//   bool reached_wp = false;
//   while (ros::ok() && (reached_wp == false))
//   {
//     some_drone.get_true_pos();
//     if (is_at_position(some_drone.true_pos, des_position, radius))
//     {
//       return true;
//     }
//     else{
//       ros::spinOnce();
//       rate.sleep();
//     }
//     ros::spinOnce();
//     rate.sleep();
//   } 
// }

/*how to loop through vector of vectors*/
// for (const auto& inner : wp_vectors) {
//     for (const auto& item : inner) {
//       std::cout << "location is: " << item << ",";
//     }
  
// }
// std::cout << std::endl;


int main(int argc, char **argv)
{
  const float radius = 0.75;
  const float rate_val = 15.0;

  ros::init(argc, argv, "test_px4_wp");
  ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
  ros::Rate rate(rate_val);

  std::vector<float> offset_pos = get_offset_pos(&_nh);
  PX4Drone px4drone(&_nh, offset_pos);  
  std::vector<float> des_position = {5,5,30};

  //list of lists
  std::vector<std::vector<float>> wp_vectors{
  
  std::vector<float>{5,5,30},
  std::vector<float>{25,15,20},
  std::vector<float>{30,22,40}};

  std::vector<float> init_pos = {5.0, 5.0, 30.0};
  px4drone.send_init_cmds(init_pos, rate);
  px4drone.set_mode.request.custom_mode = "AUTO.LAND";
  px4drone.arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  
  while(ros::ok())
  {
    switch(px4drone.user_cmd)
    {
      // std::cout << "0. size: " << wp_lists.size() << '\n';
      
      /*feed in waypoints with for loop, 
      to get to the next point
      to reached_position function,
      if true then feed the next one
      wrap this in a case switch*/
      case 0:
      {
        std::cout<<"going" <<std::endl;
        for (const auto& wp : wp_vectors)
        {
          bool reached_wp = false;
          while (reached_wp == false)
          {
            px4drone.send_global_waypoints(wp);
            px4drone.get_true_pos();
            if (is_at_position(px4drone.true_pos, wp, radius))
            {
              reached_wp = true;
            }
            else
            {
              ros::spinOnce();
              rate.sleep();
            }
          }

        //check if at last waypoint 

        }
        break;
      }

      default:
      {
        if (px4drone.current_state.mode == "OFFBOARD")
        {
          px4drone.send_global_waypoints(init_pos);
        }
        else
        {
          px4drone.set_offboard(init_pos, rate);
        }
      }
    }
    ros::spinOnce();
    rate.sleep();
  }  


    return 0;
  }
