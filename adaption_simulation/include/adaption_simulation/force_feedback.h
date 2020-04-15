//
// Created by qianlin on 3/31/20.
//

#ifndef ADAPTION_SIMULATION_FORCE_FEEDBACK_H
#define ADAPTION_SIMULATION_FORCE_FEEDBACK_H

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>
#include "adaption_msgs/ContactForce.h"
#include "adaption_msgs/FingerInfo.h"
//#include "tinyxml2.h"

namespace adaption_simulation {
  class ForceCal{
  private:
    ros::NodeHandle* _nh;
    adaption_msgs::FingerAngle angle;
    std::vector<int> finger_length_;
    //std::vector<int> finger_weight_;
    //std::map<std::string, float> init_pose_;
    int surface_type_;
    double surface_length_;
    double finger_k_;
    //double tau_;
    //double e_;
    //double beta_ve_;
    double mu_;
    //double g;
    ros::Publisher force_pub_;
    ros::Subscriber finger_info_sub_;

  public:
    adaption_msgs::ContactForce force;

    ForceCal(ros::NodeHandle* nh);

    //void publishForce(adaption_msgs::ContactForce &force_msg);

    void infoUpd(const adaption_msgs::FingerInfo &info);
  };
}

#endif //ADAPTION_SIMULATION_FORCE_FEEDBACK_H
