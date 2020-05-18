/*
* @Author: Qian Lin
* @Date:   2020-04-29 12:51:43
* @Last Modified by:   Qian Lin
* @Last Modified time: 2020-05-15 16:06:54
*/


#include <vector>
#include <string>

#include "adaption_simulation/finger_visualizer.h"

namespace adaption_simulation {
  VisualizeHelper::VisualizeHelper(ros::NodeHandle *nh) {
    this->nh_ = nh;
    this->finger_info_sub_ = nh->subscribe("adaption/finger_info", 1000, &VisualizeHelper::fingerInfoUpd, this);
    this->finger_joint_pub_ = nh->advertise<sensor_msgs::JointState>("joint_states", 100);

    if (!ros::param::get("adaption/Surface", this->surface_type_)){
      ROS_ERROR("Fail to read surface parameters.");
      return;
    }

    ROS_INFO("Visualizer initiated.");

    if (surface_type_ == 2) {
      vis_pub = nh_->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "base_frame";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      std::vector<geometry_msgs::Point> points;
      std::vector<std_msgs::ColorRGBA> colors;
      // std::vector<geometry_msgs::Pose> poses;
      // std::vector<geometry_msgs::Vector3> scales;
      points.resize(3);
      colors.resize(3);
      // poses.resize(3);
      // scales.resize(3);
      points[0].x = 0.2;
      points[0].y = 0.0;
      points[0].z = 0.02;
      points[1].x = 0.2;
      points[1].y = 0.0;
      points[1].z = -0.02;
      points[2].x = -0.2;
      points[2].y = 0.0;
      points[2].z = -0.02;
      colors[0].a = 1.0;
      colors[0].r = 0.0;
      colors[0].g = 1.0;
      colors[0].b = 0.0;
      colors[1].a = 1.0;
      colors[1].r = 0.0;
      colors[1].g = 1.0;
      colors[1].b = 0.0;
      colors[2].a = 1.0;
      colors[2].r = 0.0;
      colors[2].g = 1.0;
      colors[2].b = 0.0;
      marker.points = points;
      marker.colors = colors;
      // poses[0].orientation.w = 1.0;
      // poses[1].orientation.w = 1.0;
      // poses[2].orientation.w = 1.0;
      // scales[0].x = 1;
      // scales[0].y = 1;
      // scales[0].z = 1;
      // scales[1].x = 1;
      // scales[1].y = 1;
      // scales[1].z = 1;
      // scales[2].x = 1;
      // scales[2].y = 1;
      // scales[2].z = 1;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
      
      vis_pub.publish( marker );
    }
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
    if (surface_type_ == 2) {
      this->vis_pub.publish(marker);
    }
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