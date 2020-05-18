/*
* @Author: Qian Lin
* @Date:   2020-05-11 15:09:02
* @Last Modified by:   Qian Lin
* @Last Modified time: 2020-05-15 16:05:39
*/

#include <cmath>

#include "adaption_controller/finger_controller.h"

namespace adaption_controller {
  PIDController::PIDController(ros::NodeHandle nh): FingerController(nh) {
    integrate = 0;
    diff = 0;
    prev_f = 0;
    if (!ros::param::get("adaption/Kp1", this->kp1)){
      ROS_ERROR("Fail to read kp parameters.");
      return;
    }
    if (!ros::param::get("adaption/Ki1", this->ki1)){
      ROS_ERROR("Fail to read ki parameters.");
      return;
    }
    if (!ros::param::get("adaption/Kd1", this->kd1)){
      ROS_ERROR("Fail to read kd parameters.");
      return;
    }
    if (!ros::param::get("adaption/Kp2", this->kp2)){
      ROS_ERROR("Fail to read kp parameters.");
      return;
    }
    if (!ros::param::get("adaption/Ki2", this->ki2)){
      ROS_ERROR("Fail to read ki parameters.");
      return;
    }
    if (!ros::param::get("adaption/Kd2", this->kd2)){
      ROS_ERROR("Fail to read kd parameters.");
      return;
    }
    if (!ros::param::get("adaption/F_threshold", this->f_thre)){
      ROS_ERROR("Fail to read threshold parameters.");
      return;
    }
  }
  
  void PIDController::controller(double fx, double fy, double &torque1, double &torque2) {
    double pres_f = sqrt(fx*fx + fy*fy);
    if (pres_f < f_thre) {   // keep force in (f_thre/10, f_thre)
      if (pres_f > f_thre / 10) {
        pres_f = 0;
      } else {
        pres_f -= f_thre / 10;
        pres_f *= 100;
      }
    } else {
      pres_f -= f_thre;
    }
    integrate += pres_f;
    diff = pres_f - prev_f;
    prev_f = pres_f;
    
    torque1 = kp1 * pres_f + ki1 * integrate + kd1 * diff;
    torque2 = kp2 * pres_f + ki2 * integrate + kd2 * diff;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "finger_controller");
  ros::NodeHandle nh;
  
  //ros::Subscriber force_sub = nh.subscribe("adaption/contact_force", 100, &callback);
  //ros::Publisher torque_pub = nh.advertise<adaption_msgs::JointForce>("adaption/controller_torque", 1000);
  //adaption_msgs::JointForce msg;
  //msg.torque1 = 0;
  //msg.torque2 = 0;
  //
  //while (ros::ok()) {
  //  torque_pub.publish(msg);
  //
  //  ros::spinOnce();
  //}
  adaption_controller::PIDController controller(nh);
  ros::spin();
}