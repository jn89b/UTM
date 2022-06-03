#include <iostream>
#include <ros/ros.h>
#include <uas_px4.h>
#include <stdlib.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

#include <PID.h>
#include <vector>
#include <tf/tf.h>
#include <string>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <list>

#include <utm/WP.h>
#include <utm/Coords.h>

using namespace Eigen;

int val = -1;
// std::vector<int> wp_vectors;
std::vector<utm::Coords> coords;

std::vector<std::vector<float>> wp_vectors;

//functions
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

void wp_cb(const utm::WP::ConstPtr& msg)
{ //cb function for waypoints

  coords = msg->data;
  if (val != msg->unique_wp_num)
  {
    for (const auto& coord: coords)
    {
      wp_vectors.push_back(std::vector<float>{coord.data[0], coord.data[1], coord.data[2]});
    }
  }
  val = msg->unique_wp_num;
  //wp_vectors.push_back(msg->data.data);
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
  const float rate_val = 5;
  bool global_wp_flag = false;
  bool print_dumbass_once = false;

  ros::init(argc, argv, "test_px4_wp");
  ros::NodeHandle _nh; // create a node handle; need to pass this to the class constructor
  ros::Rate rate(rate_val);

  std::vector<float> offset_pos = get_offset_pos(&_nh);
  PX4Drone px4drone(&_nh, offset_pos);  
  std::vector<float> des_position = {5,5,30};

  //list of lists
  // std::vector<std::vector<float>> wp_vectors{
  // std::vector<float>{5,5,30},
  // std::vector<float>{25,15,20},
  // std::vector<float>{30,22,40}};

  std::vector<float> init_pos = {5.0, 5.0, 30.0};
  px4drone.send_init_cmds(init_pos, rate);
  px4drone.set_mode.request.custom_mode = "AUTO.LAND";
  px4drone.arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  
  ros::Subscriber wp_sub = _nh.subscribe<utm::WP>
          ("uav0/wp_list", 10, wp_cb);

  while(ros::ok())  
  {
    switch(px4drone.user_cmd)
    {
      //waypoint navigation, now I need to subscribe to the waypoints
      case 0:
      {
        //dont want to do duplicate waypoints 
        // std::cout<<"length of trip" << wp_vectors.size() << std::endl;
        if (global_wp_flag == true) 
        {
          if (print_dumbass_once == false){
            std::cout<< "im done dumbass" << std::endl;
            print_dumbass_once = true;
          } 
          break;
        }
        std::cout<<"going" <<std::endl;        
        size_t index = 0;
        for (const auto& wp : wp_vectors)
        {
          std::cout << "Index: " << index  << std::endl;
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
          ++index;
          if (index >= wp_vectors.size())
          {
            global_wp_flag = true; 
          }
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
