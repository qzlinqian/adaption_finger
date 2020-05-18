#include <ros/ros.h>

#include "adaption_msgs/ContactForce.h"
#include "adaption_msgs/JointForce.h"

namespace adaption_controller {
	class FingerController {
	public:
		FingerController(ros::NodeHandle nh);
		// ~FingerController();
    void forceCallback(const adaption_msgs::ContactForce &msg);

  private:
    ros::NodeHandle* nh_;
    ros::Subscriber force_sub;
    ros::Publisher torque_pub;
    double delta_t;

    virtual void controller(double fx, double fy, double &torque1, double &torque2) = 0;
	};
  
  class FakeController: public FingerController {
  public:
    explicit FakeController(ros::NodeHandle nh): FingerController(nh) {}
    
    void controller(double fx, double fy, double &torque1, double &torque2) override;
  };
  
  class PIDController: public FingerController {
  private:
    double integrate, diff, prev_f;
    double kp1, ki1, kd1;
    double kp2, ki2, kd2;
    double f_thre;
  public:
    PIDController(ros::NodeHandle nh);
    
    void controller(double fx, double fy, double &torque1, double &torque2) override;
  };
}
