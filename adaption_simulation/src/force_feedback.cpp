//
// Created by qianlin on 3/31/20.
//
#include <cmath>

#include <ros/package.h>
#include "adaption_simulation/force_feedback.h"

namespace adaption_simulation{
  ForceCal::ForceCal(ros::NodeHandle* nh) {
    this->_nh = nh;
    if (!ros::param::get("adaption/Finger", this->finger_length_)){
      ROS_ERROR("Fail to read finger parameters.");
      return;
    }
    //if (!ros::param::get("adaption/weight", this->finger_weight_)){
    //  ROS_ERROR("Fail to read weight parameters.");
    //  return;
    //}
    //if (!ros::param::get("adaption/InitialPose", this->init_pose_)){
    //  ROS_ERROR("Fail to read init_pose parameters.");
    //  return;
    //}
    if (!ros::param::get("adaption/Surface", this->surface_type_)){
      ROS_ERROR("Fail to read surface parameters.");
      return;
    }
    if (!ros::param::get("adaption/SurfaceLength", this->surface_length_)){
      ROS_ERROR("Fail to read surface length parameters.");
      return;
    }
    if (!ros::param::get("adaption/k", this->finger_k_)){
      ROS_ERROR("Fail to read finger k parameters.");
      return;
    }
    //if (!ros::param::get("adaption/tau", this->tau_)){
    //  ROS_ERROR("Fail to read tau parameters.");
    //  return;
    //}
    //if (!ros::param::get("adaption/E", this->e_)){
    //  ROS_ERROR("Fail to read E parameters.");
    //  return;
    //}
    //if (!ros::param::get("adaption/beta_ve", this->beta_ve_)){
    //  ROS_ERROR("Fail to read beta parameters.");
    //  return;
    //}
    if (!ros::param::get("adaption/mu", this->mu_)){
      ROS_ERROR("Fail to read mu parameters.");
      return;
    }
    //if (!ros::param::get("adaption/g", this->g)){
    //  this->g = 9.8;
    //}

    this->force_pub_ = this->_nh->advertise<adaption_msgs::ContactForce>("adaption/contact_force", 100);

    this->finger_info_sub_ = this->_nh->subscribe("adaption/finger_info", 1000, &ForceCal::infoUpd, this);
  }

  //void ForceCal::publishForce(adaption_msgs::ContactForce &force_msg) {
  //  this->force_pub_.publish(force_msg);
  //}

  void ForceCal::infoUpd(const adaption_msgs::FingerInfo &info) {
    adaption_msgs::ContactForce contactForce;
    contactForce.stamp = info.stamp;
    //float n = (this->finger_weight_[0] + this->finger_weight_[1]) * g * cos(info.angle.alpha)
    //        + info.force.force * sin(info.angle.alpha);
    //float fm = (this->finger_weight_[0] + this->finger_weight_[1]) * g * sin(info.angle.alpha)
    //         - info.force.force * cos(info.angle.alpha);
    double theta2 = info.angle.theta2;
    double theta1 = theta2 + info.angle.theta1;

    double dx = this->finger_length_[0] * sin(theta1) + this->finger_length_[1] * sin(theta2);
    double dy = this->finger_length_[0] * cos(theta1) + this->finger_length_[1] * cos(theta2);
    double head_x = dx + info.position.x;
    double head_y = info.position.y - dy;
    if (head_x > this->surface_length_) {
      contactForce.Fx = 0;
      contactForce.Fy = 0;
      //this->publishForce(contactForce);
      this->force_pub_.publish(contactForce);
      return;
    }
    switch (this->surface_type_) {
      case 1:
        if (head_y > 0) {
          contactForce.Fx = 0;
          contactForce.Fy = 0;
        } else {
          double fn = - head_y / cos(theta1) * this->finger_k_;
          double ff = fn * this->mu_;
          contactForce.Fx = -ff;
          contactForce.Fy = fn;
        }
        break;
      default:
        break;
    }

    this->force_pub_.publish(contactForce);
  }
}

int main() {
  return 0;
}