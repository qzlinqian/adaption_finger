//
// Created by qianlin on 4/10/20.
//

#ifndef SRC_FINGER_VISUALIZER_H
#define SRC_FINGER_VISUALIZER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include "adaption_msgs/FingerInfo.h"

namespace adaption_simulation {
  class VisualizeHelper{
  private:
    ros::NodeHandle *nh_;
    ros::Subscriber finger_info_sub_;
    ros::Publisher finger_joint_pub_;
    int surface_type_;
    ros::Publisher vis_pub;
    visualization_msgs::Marker marker;
    
    void fingerInfoUpd(const adaption_msgs::FingerInfo &msg);

  public:
    VisualizeHelper(ros::NodeHandle *nh);
  };
}

#endif //SRC_FINGER_VISUALIZER_H
