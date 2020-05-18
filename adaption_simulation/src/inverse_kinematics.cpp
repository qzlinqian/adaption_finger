/*
* @Author: Qian Lin
* @Date:   2020-05-17 23:07:56
* @Last Modified by:   Qian Lin
* @Last Modified time: 2020-05-18 15:18:06
*/

#include <cmath>

#include "adaption_simulation/inverse_kinematics.h"

namespace adaption_simulation {
  InvKin::InvKin(ros::NodeHandle *nh, float time_interval) {
    this->_nh = nh;
    this->delta_t = time_interval;
    if (!ros::param::get("adaption/Finger", this->finger_length_)){
      ROS_ERROR("Fail to read finger parameters.");
      return;
    }
    if (!ros::param::get("adaption/weight", this->finger_weight_)){
      ROS_ERROR("Fail to read weight parameters.");
      return;
    }
    if (!ros::param::get("adaption/InitialPose", this->init_pose_)){
      ROS_ERROR("Fail to read init_pose parameters.");
      return;
    }
    ROS_INFO("Init pose is: x: %f, y: %f, z: %f, theta1: %f, theta2: %f", init_pose_["x"], init_pose_["y"], init_pose_["z"], init_pose_["theta1"], init_pose_["theta2"]);

    if (!ros::param::get("adaption/Surface", this->surface_type_)){
      ROS_ERROR("Fail to read surface parameters.");
      return;
    }
    switch (surface_type_) {
      case 1:
        ROS_ERROR("case 1: plain");
        break;
      case 2:
      case 3:
      case 4: {
        ROS_ERROR("case 2: incline");
        std::map<std::string, double> line;
        if (!ros::param::get("adaption/Line", line)) {
          ROS_ERROR("Fail to read line parameters.");
          return;
        }
        this->surface_param.push_back(line["A"]);
        this->surface_param.push_back(line["B"]);
        this->surface_param.push_back(line["C"]);
        this->surface_param.push_back(std::atan(-line["B"] / line["A"]));
        break;
      }
      case 5:
        break;
      default:
        break;
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
    if (!ros::param::get("adaption/TailVel", this->tail_vel)){
      ROS_ERROR("Fail to read tail velocity parameters.");
      return;
    }
    if (!ros::param::get("adaption/g", this->g)){
      this->g = 9.8;
    }
    //this->g *= 1000;  // mm/s^2
  
    this->force_pub_ = this->_nh->advertise<adaption_msgs::ContactForce>("adaption/contact_force", 100);
    // this->force_pub_2 = this->_nh->advertise<std_msgs::Float64>("adaption/normal_force", 100);
    this->finger_info_pub_ = this->_nh->advertise<adaption_msgs::FingerInfo>("adaption/finger_info", 100);
    this->tip_pose_sub_ = this->_nh->subscribe("adaption/tip_pos", 1000, &InvKin::positionCallback, this);
    // this->torque_sub_2 = this->_nh->subscribe("adaption/controller_torque2", 1000, &PoseCal::torqueUpd, this);
    
    // this->matrix_k = this->finger_weight_[1] * this->finger_length_[1] * finger_length_[1];
    
    this->now_ang[0] = this->init_pose_["theta1"];
    this->now_ang[1] = this->init_pose_["theta2"];
    this->tail_pos[0] = this->init_pose_["x"];
    this->tail_pos[1] = this->init_pose_["z"];
    this->tip_pos[0] = 0;
    this->tip_pos[1] = 0;
    // this->ang_vel[0] = 0;
    // this->ang_vel[1] = 0;
    // this->torque[0] = 0;
    // this->torque[1] = 0;
    this->a1 = this->finger_length_[0];
    this->a2 = this->finger_length_[1];

    ROS_INFO("Pose calculator initiated.");
  }
  
  void InvKin::forceCal() {
    //adaption_msgs::ContactForce contactForce;
    contactForce.stamp = ros::Time::now();
    //float n = (this->finger_weight_[0] + this->finger_weight_[1]) * g * cos(info.angle.alpha)
    //        + info.force.force * sin(info.angle.alpha);
    //float fm = (this->finger_weight_[0] + this->finger_weight_[1]) * g * sin(info.angle.alpha)
    //         - info.force.force * cos(info.angle.alpha);
    double theta1 = this->now_ang[0];
    double theta2 = theta1 + this->now_ang[1];
  
    // double dx = this->finger_length_[0] * sin(theta1) + this->finger_length_[1] * sin(theta2);
    // double dy = this->finger_length_[0] * cos(theta1) + this->finger_length_[1] * cos(theta2);
    double head_x = this->tip_pos[0];
    double head_y = this->tip_pos[1];
    ROS_INFO("head_x is: %f, head_y is: %f", head_x, head_y);
    if (head_x > this->surface_length_) {
      contactForce.Fx = 0;
      contactForce.Fy = 0;
      //this->force_pub_.publish(contactForce);
      return;
    }
    switch (this->surface_type_) {
      case 1: {
        if (head_y > 0) {
          contactForce.Fx = 0;
          contactForce.Fy = 0;
        } else {
          double d_length = -head_y / cos(theta1);
          this->a2 = finger_length_[1] - d_length;
          double fn = d_length * this->finger_k_;
          double ff = fn * this->mu_;
          contactForce.Fx = -ff;
          contactForce.Fy = fn;
        }
        break;
      }
      case 2: {
        // incline
        double dis = this->surface_param[0] * head_x + surface_param[1] * head_y + surface_param[2];
        ROS_INFO_STREAM("dis is" << dis);
        if (dis > 0) {
          contactForce.Fx = 0;
          contactForce.Fy = 0;
        } else {
          dis /= std::sqrt(surface_param[0] * surface_param[0] + surface_param[1] * surface_param[1]);
          dis /= std::abs(cos(surface_param[3] - now_ang[0] - now_ang[1]));
          double fn = -dis * this->finger_k_;
          double ff = fn * this->mu_;
          // contactForce.Fy = fn * cos(surface_param[3]) + ff * sin(surface_param[3]);
          // contactForce.Fx = -fn * sin(surface_param[3]) + ff * cos(surface_param[3]);
          contactForce.Fx = -ff;
          contactForce.Fy = fn;
        }
        break;
      }
      case 3: {
        // incline
        double dis = this->surface_param[0] * head_x + surface_param[1] * head_y + surface_param[2];
        ROS_INFO_STREAM("dis is" << dis);
        if (dis > 0) {
          contactForce.Fx = 0;
          contactForce.Fy = 0;
        } else {
          dis /= std::sqrt(surface_param[0] * surface_param[0] + surface_param[1] * surface_param[1]);
          dis /= std::abs(cos(surface_param[3] - now_ang[0] - now_ang[1]));
          double fn = -dis * this->finger_k_;
          double mu = this->mu_;
          if (head_x > 0.02) {
            mu = 0.6;
            if (head_x > 0.06) {
              mu = 0.8;
            }
          }
          double ff = fn * mu;
          // contactForce.Fy = fn * cos(surface_param[3]) + ff * sin(surface_param[3]);
          // contactForce.Fx = -fn * sin(surface_param[3]) + ff * cos(surface_param[3]);
          contactForce.Fx = -ff;
          contactForce.Fy = fn;
        }
        break;
      }
      case 4: {
        // incline+plain
        if (head_x > 0.04) {
          if (head_y > 0.002) {
            contactForce.Fx = 0;
            contactForce.Fy = 0;
          } else {
            double d_length = -(head_y-0.002) / cos(theta1);
            this->a2 = finger_length_[1] - d_length;
            double fn = d_length * this->finger_k_;
            double ff = fn * this->mu_;
            contactForce.Fx = -ff;
            contactForce.Fy = fn;
          }
        } else {
          double dis = this->surface_param[0] * head_x + surface_param[1] * head_y + surface_param[2];
          ROS_INFO_STREAM("dis is" << dis);
          if (dis > 0) {
            contactForce.Fx = 0;
            contactForce.Fy = 0;
          } else {
            dis /= std::sqrt(surface_param[0] * surface_param[0] + surface_param[1] * surface_param[1]);
            dis /= std::abs(cos(surface_param[3] - now_ang[0] - now_ang[1]));
            double fn = -dis * this->finger_k_;
            double ff = fn * this->mu_;
            // contactForce.Fy = fn * cos(surface_param[3]) + ff * sin(surface_param[3]);
            // contactForce.Fx = -fn * sin(surface_param[3]) + ff * cos(surface_param[3]);
            contactForce.Fx = -ff;
            contactForce.Fy = fn;
          }
        }
        break;
      }
      default:
        contactForce.Fx = 0;
        contactForce.Fy = 0;
        break;
    }
    //this->force_pub_.publish(contactForce);
    ROS_INFO("Contact force is: %f, %f", contactForce.Fx, contactForce.Fy);
  }
  
  void InvKin::positionCallback(const geometry_msgs::Point &msg) {
    this->tip_pos[0] = msg.x;
    this->tip_pos[1] = msg.z;
  }

  void InvKin::poseUpd() {
    double alpha = std::atan((tip_pos[0] - tail_pos[0]) / (tail_pos[1] - tip_pos[1]));
    double ap2 = (tip_pos[0] - tail_pos[0]) * (tip_pos[0] - tail_pos[0]) + (tail_pos[1] - tip_pos[1]) * (tail_pos[1] - tip_pos[1]);
    double ap = std::sqrt(ap2);
    double beta = std::acos((finger_length_[0] * finger_length_[0] + ap2 - finger_length_[1] * finger_length_[1]) / (2 * finger_length_[0] * ap));
    double theta_2_now = std::acos((ap2 - finger_length_[0] * finger_length_[0] - finger_length_[1] * finger_length_[1]) / (2 * finger_length_[0] * finger_length_[1]));
    now_ang[0] = alpha + beta;
    now_ang[1] = -theta_2_now;

    tail_pos[0] += delta_t * tail_vel;

    ROS_INFO_STREAM("Tail is: " << tail_pos[0] << " " << tail_pos[1]);
    ROS_INFO_STREAM("Pose is: " << now_ang[0] << " " << now_ang[1]);
  }

  void InvKin::posePub() {
    adaption_msgs::FingerInfo info;
    info.stamp = ros::Time::now();
    info.angle.theta1 = this->now_ang[0];
    info.angle.theta2 = this->now_ang[1];
    info.position.x = this->tail_pos[0];
    info.position.y = this->tail_pos[1];
    
    this->finger_info_pub_.publish(info);
  }
  
  void InvKin::forcePub() {
    this->force_pub_.publish(this->contactForce);
    // this->force_pub_2.publish(this->contactForce.Fy);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_calculate_node");
  ros::NodeHandle nh;

  ROS_INFO("pose calculator init.");
  
  int pose_f, sensor_f;
  
  if (!ros::param::get("adaption/PoseUpdFrequency", pose_f)){
    ROS_ERROR("Fail to read PoseUpdFrequency parameter.");
    return -1;
  }
  if (!ros::param::get("adaption/ForceFdbFrequency", sensor_f)){
    ROS_ERROR("Fail to read ForceFdbFrequency parameter.");
    return -1;
  }
  
  ros::Rate r(pose_f);
  int report = pose_f / sensor_f;
  double time_interval = 1.0 / pose_f;
  adaption_simulation::InvKin calculator(&nh, time_interval);
  int i = 0;
  while (ros::ok()) {
    ros::spinOnce();
    calculator.forceCal();
    calculator.poseUpd();
    calculator.posePub();
    // i++;
    // if (i > report) {
      calculator.forcePub();
      // i = 0;
    // }
    r.sleep();
  }
  
  return 0;
}