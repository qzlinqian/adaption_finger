//
// Created by qianlin on 4/27/20.
//

#include "adaption_controller/finger_controller.h"

namespace adaption_controller {
  FingerController::FingerController(ros::NodeHandle nh) {
    this->nh_ = &nh;
    this->force_sub = nh_->subscribe("adaption/contact_force", 100, &FingerController::forceCallback, this);
    this->torque_pub = nh_->advertise<adaption_msgs::JointForce>("adaption/controller_torque", 1000);
  
    if (!ros::param::get("adaption/delta_t", this->delta_t)){
      ROS_ERROR("Fail to read delta_t parameters. Use default.");
      this->delta_t = 0.002;
    }
  }

  void FingerController::forceCallback(const adaption_msgs::ContactForce &msg) {
    adaption_msgs::JointForce torque_msg;
    controller(msg.Fx, msg.Fy, torque_msg.torque1, torque_msg.torque2);
    ros::Duration(delta_t).sleep();  // delay for delta_t
    torque_pub.publish(torque_msg);
  }
  
  void FakeController::controller(double fx, double fy, double &torque1, double &torque2) {
    torque1 = 0;
    torque2 = 0;
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
  adaption_controller::FakeController controller(nh);
  ros::spin();
}