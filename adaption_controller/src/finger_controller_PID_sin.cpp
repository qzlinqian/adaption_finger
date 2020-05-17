/*
* @Author: Qian Lin
* @Date:   2020-05-15 16:03:56
* @Last Modified by:   Qian Lin
* @Last Modified time: 2020-05-16 12:50:55
*/

#include <cmath>

#include "adaption_controller/finger_controller.h"

namespace adaption_controller {
	class PIDControllerSin: public FingerController{
	private:
    double integrate, diff, prev_f;
    double kp1, ki1, kd1;
    double kp2, ki2, kd2;
    double f_freq, f_amp, f_offset;
    ros::Time begin;
	public:
		PIDControllerSin(ros::NodeHandle nh);

		void controller(double fx, double fy, double &torque1, double &torque2) override;
	};


  PIDControllerSin::PIDControllerSin(ros::NodeHandle nh): FingerController(nh) {
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
    if (!ros::param::get("adaption/ForceCtrlFrequency", this->f_freq)){
      ROS_ERROR("Fail to read force parameters.");
      return;
    }
		if (!ros::param::get("adaption/ForceCtrlAmplitude", this->f_amp)){
      ROS_ERROR("Fail to read force parameters.");
      return;
    }
		if (!ros::param::get("adaption/ForceCtrlOffset", this->f_offset)){
      ROS_ERROR("Fail to read force parameters.");
      return;
    }

    begin = ros::Time::now();
  }
  
  void PIDControllerSin::controller(double fx, double fy, double &torque1, double &torque2) {
  	double time = (ros::Time::now() - begin).toSec();
  	double force_des = f_amp * sin(time * f_freq) + f_offset;

    double f_error = fy - force_des;
    integrate += f_error;
    diff = f_error - prev_f;
    prev_f = f_error;
    
    torque1 = kp1 * f_error + ki1 * integrate + kd1 * diff;
    torque2 = kp2 * f_error + ki2 * integrate + kd2 * diff;
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
  adaption_controller::PIDControllerSin controller(nh);
  ros::spin();
}