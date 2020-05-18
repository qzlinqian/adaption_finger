/*
* @Author: Qian Lin
* @Date:   2020-05-18 01:02:11
* @Last Modified by:   Qian Lin
* @Last Modified time: 2020-05-18 14:40:19
*/

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "adaption_controller/impedance_controller.h"

namespace adaption_controller {
	ImpedanceController::ImpedanceController(ros::NodeHandle* nh, double delta) {
		this->nh_ = nh;
		this->delta_t = delta;
		this->force_sub_ = nh_->subscribe("adaption/contact_force", 1000, &ImpedanceController::forceCallback, this);
		this->tip_pos_pub_ = nh->advertise<geometry_msgs::Point>("adaption/tip_pos", 100);

		if (!ros::param::get("adaption/TailVel", this->tail_vel)){
      ROS_ERROR("Fail to read tail velocity parameters.");
      return;
    }
		if (!ros::param::get("adaption/F_threshold", this->f_thre)){
      ROS_ERROR("Fail to read tail force threshold parameters.");
      return;
    }
		if (!ros::param::get("adaption/ControllerM", this->m)){
      ROS_ERROR("Fail to read tail force m parameters.");
      return;
    }
		if (!ros::param::get("adaption/ControllerB", this->b)){
      ROS_ERROR("Fail to read tail force b parameters.");
      return;
    }
		if (!ros::param::get("adaption/ControllerK", this->k)){
      ROS_ERROR("Fail to read tail force k parameters.");
      return;
    }
    // geometry_msgs::Point point;
		point.y = 0;
		point.z = 0;
		point.x = 0.;
		// point.x = 0.02 * 1.73;
		a = 0;
		v = 0;
		x = 0;
		e = 0;
		ve = 0;
		ae = 0;

		fy = 0;
	}

	void ImpedanceController::forceCallback(const adaption_msgs::ContactForce &msg) {
		fy = msg.Fy;
	}

	void ImpedanceController::update() {
		a = (f_thre - fy - b * v - k * e) / m;
		// a += ae;
		e = v * delta_t + a * delta_t * delta_t/2;
		x += e;
		ve = a * delta_t;
		v += ve;
		ROS_INFO_STREAM("x is:" << x);

		point.x += tail_vel * delta_t;
		point.z = -x;
		tip_pos_pub_.publish(point);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	// ros::Publisher pub = nh.advertise<geometry_msgs::Point>("adaption/tip_pos", 100);
	int sensor_f;
	if (!ros::param::get("adaption/ForceFdbFrequency", sensor_f)){
    ROS_ERROR("Fail to read ForceFdbFrequency parameter.");
    return -1;
  }
  adaption_controller::ImpedanceController controller(&nh, 1.0/sensor_f);
	
	ros::Rate r(sensor_f);
	while (ros::ok()) {
		// point.x += 0.00005;
		// pub.publish(point);
		controller.update();
		ros::spinOnce();
		r.sleep();
	}
}