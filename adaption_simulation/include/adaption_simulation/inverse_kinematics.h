#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "adaption_msgs/FingerInfo.h"
#include "adaption_msgs/FingerAngle.h"
#include "adaption_msgs/ContactForce.h"
#include "adaption_msgs/JointForce.h"

namespace adaption_simulation {
  class InvKin {
  private:
    ros::NodeHandle* _nh;
    adaption_msgs::FingerAngle angle;
    std::vector<double> finger_length_;
    std::vector<double> finger_weight_;
    std::map<std::string, double> init_pose_;
    int surface_type_;
    double finger_k_;
    //double tau_;
    //double e_;
    //double beta_ve_;
    double mu_;
    double surface_length_;
    std::vector<double> surface_param;
    double g;
    double tail_vel;

    double delta_t;
    
    ros::Publisher finger_info_pub_;
    ros::Publisher force_pub_;
    ros::Subscriber tip_pose_sub_;
    
    // double matrix_k;
    double now_ang[2];//, ang_vel[2], ang_acc[2];
    double tail_pos[2], tip_pos[2];
    double a1, a2;
    adaption_msgs::ContactForce contactForce;

  public:
    InvKin(ros::NodeHandle* nh, float time_interval);
    
    void forceCal();
    
    void poseUpd();

    void positionCallback(const geometry_msgs::Point &msg);
    
    // void torqueUpd(const adaption_msgs::JointForce &msg);
    // void torqueUpd(const std_msgs::Float64 &msg);
    
    void posePub();
    
    void forcePub();
  };
}

#endif //INVERSE_KINEMATICS_H
