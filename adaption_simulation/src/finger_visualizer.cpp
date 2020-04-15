//
// Created by qianlin on 4/10/20.
//
#include <vector>
#include <string>

#include "adaption_simulation/finger_visualizer.h"

namespace adaption_simulation {
  VisualizeHelper::VisualizeHelper(ros::NodeHandle *nh) {
    this->nh_ = nh;
    this->finger_info_sub_ = nh->subscribe("adaption/finger_info", 1000, &VisualizeHelper::fingerInfoUpd, this);
    this->finger_joint_pub_ = nh->advertise<sensor_msgs::JointState>("joint_states", 100);

    ROS_INFO("Visualizer initiated.");
  }
  
  void VisualizeHelper::fingerInfoUpd(const adaption_msgs::FingerInfo &msg) {
    sensor_msgs::JointState state_msg;
    std::vector<std::string> names;
    std::vector<double> positions;
    
    state_msg.header.stamp = msg.stamp;
    // state_msg.header.frame_id = "base_link";
    
    names.resize(3);
    names[0] = "fake_joint";
    names[1] = "tail_joint";
    names[2] = "tip_joint";
    
    positions.resize(3);
    positions[0] = msg.position.x;
    positions[1] = msg.angle.theta1;
    positions[2] = msg.angle.theta2;
    
    state_msg.position = positions;
    state_msg.name = names;
    
    this->finger_joint_pub_.publish(state_msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "finger_visualize_node");
  ROS_INFO("finger visualizer init.");
  ros::NodeHandle nh;
  
  adaption_simulation::VisualizeHelper visualizeHelper(&nh);
  
  ros::spin();
  
  return 0;
}