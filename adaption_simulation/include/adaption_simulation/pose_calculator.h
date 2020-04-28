//
// Created by qianlin on 4/6/20.
//

#ifndef SRC_POSE_CALCULATOR_H
#define SRC_POSE_CALCULATOR_H

#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>

#include "adaption_msgs/FingerInfo.h"
#include "adaption_msgs/FingerAngle.h"
#include "adaption_msgs/ContactForce.h"
#include "adaption_msgs/JointForce.h"

namespace adaption_simulation {
  class PoseCal {
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
    
    double torque[2], a1, a2;
    ros::Subscriber torque_sub_;
    ros::Publisher finger_info_pub_;
    ros::Publisher force_pub_;
    
    double matrix_k;
    double now_ang[2], ang_vel[2], ang_acc[2];
    double now_pos[2];
    adaption_msgs::ContactForce contactForce;

  public:
    PoseCal(ros::NodeHandle* nh);
    
    void forceCal();
    
    void poseUpd(double delta_t);
    
    void torqueUpd(const adaption_msgs::JointForce &msg);
    
    void posePub();
    
    void forcePub();
  };
  
  void integrate(double* init_pos, double* init_vel, const double* acc,
      //double* res_pos, double* res_vel,
      double delta_t);
}

#endif //SRC_POSE_CALCULATOR_H
