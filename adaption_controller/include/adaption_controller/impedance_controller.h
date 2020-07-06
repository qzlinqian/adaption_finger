#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include "adaption_msgs/ContactForce.h"

namespace adaption_controller {
	class ImpedanceController {
	private:
		ros::NodeHandle* nh_;
		ros::Subscriber force_sub_;
		ros::Publisher tip_pos_pub_;

		double tail_vel, delta_t;
		double f_thre, m, b, k;
		double x, v, a, e, ve, ae;
		geometry_msgs::Point point;
		double fy;
	public:
		ImpedanceController(ros::NodeHandle* nh, double delta);
		
		void forceCallback(const adaption_msgs::ContactForce &msg);

		void update();
	};
}

#endif //IMPEDANCE_CONTROLLER_H