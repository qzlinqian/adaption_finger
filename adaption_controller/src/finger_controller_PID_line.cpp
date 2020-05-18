/*
* @Author: Qian Lin
* @Date:   2020-05-16 12:51:24
* @Last Modified by:   Qian Lin
* @Last Modified time: 2020-05-16 13:24:12
*/

#include <cmath>

#include "adaption_controller/finger_controller.h"

namespace adaption_controller {
	class PIDControllerLine: public FingerController{
	private:
    double integrate, diff, prev_f;
    double kp1, ki1, kd1;
    double kp2, ki2, kd2;
    double f_k;
    ros::Time begin;
	public:
		PIDControllerLine(ros::NodeHandle nh);

		void controller(double fx, double fy, double &torque1, double &torque2) override;
	};


  PIDControllerLine::PIDControllerLine(ros::NodeHandle nh): FingerController(nh) {
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
    if (!ros::param::get("adaption/ForceCtrlK", this->f_k)){
      ROS_ERROR("Fail to read force parameters.");
      return;
    }

    begin = ros::Time::now();
  }
  
  void PIDControllerLine::controller(double fx, double fy, double &torque1, double &torque2) {
  	double time = (ros::Time::now() - begin).toSec();
  	double force_des = time * f_k;
    if (force_des > 5) {
      force_des = 5;
    }

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
  
  adaption_controller::PIDControllerLine controller(nh);
  ros::spin();
}