//
// Created by qianlin on 4/6/20.
//
#include <cmath>

#include <eigen3/Eigen/Dense>

#include "adaption_simulation/pose_calculator.h"

namespace adaption_simulation {
  PoseCal::PoseCal(ros::NodeHandle *nh) {
    this->_nh = nh;
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
      case 2: {
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
      case 3:
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
    this->force_pub_2 = this->_nh->advertise<std_msgs::Float64>("adaption/normal_force", 100);
    this->finger_info_pub_ = this->_nh->advertise<adaption_msgs::FingerInfo>("adaption/finger_info", 100);
    // this->torque_sub_ = this->_nh->subscribe("adaption/controller_torque", 1000, &PoseCal::torqueUpd, this);
    this->torque_sub_2 = this->_nh->subscribe("adaption/controller_torque2", 1000, &PoseCal::torqueUpd, this);
    
    this->matrix_k = this->finger_weight_[1] * this->finger_length_[1] * finger_length_[1];
    
    this->now_ang[0] = this->init_pose_["theta1"];
    this->now_ang[1] = this->init_pose_["theta2"];
    this->now_pos[0] = this->init_pose_["x"];
    this->now_pos[1] = this->init_pose_["z"];
    this->ang_vel[0] = 0;
    this->ang_vel[1] = 0;
    this->torque[0] = 0;
    this->torque[1] = 0;
    this->a1 = this->finger_length_[0];
    this->a2 = this->finger_length_[1];

    ROS_INFO("Pose calculator initiated.");
  }
  
  void PoseCal::forceCal() {
    //adaption_msgs::ContactForce contactForce;
    contactForce.stamp = ros::Time::now();
    //float n = (this->finger_weight_[0] + this->finger_weight_[1]) * g * cos(info.angle.alpha)
    //        + info.force.force * sin(info.angle.alpha);
    //float fm = (this->finger_weight_[0] + this->finger_weight_[1]) * g * sin(info.angle.alpha)
    //         - info.force.force * cos(info.angle.alpha);
    double theta1 = this->now_ang[0];
    double theta2 = theta1 + this->now_ang[1];
  
    double dx = this->finger_length_[0] * sin(theta1) + this->finger_length_[1] * sin(theta2);
    double dy = this->finger_length_[0] * cos(theta1) + this->finger_length_[1] * cos(theta2);
    double head_x = dx + this->now_pos[0];
    double head_y = this->now_pos[1] - dy;
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
          double fn = dis * this->finger_k_;
          double ff = fn * this->mu_;
          contactForce.Fy = fn * cos(surface_param[3]) + ff * sin(surface_param[3]);
          contactForce.Fx = -fn * sin(surface_param[3]) + ff * cos(surface_param[3]);
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
  
  void PoseCal::poseUpd(double delta_t) {
    Eigen::Matrix2d m_theta;
    m_theta(1,1) = this->matrix_k/2;  // k
    m_theta(1,0) = this->matrix_k/12 + this->finger_weight_[1] * a1 * a2 * cos(this->now_ang[1])/2;
    m_theta(0,1) = m_theta(1,0);
    m_theta(0,0) = (finger_weight_[0]/3 + finger_weight_[1]) * a1 * a1 + this->matrix_k/12;
    // m_theta(1,0) = (finger_length_[1] * finger_length_[1] / 2 + finger_length_[0] * finger_length_[0] + finger_length_[0] * finger_length_[1] /2 * cos(now_ang[1])) * finger_weight_[1];
    // m_theta(1,1) = (3 * finger_length_[1] * finger_length_[1] + finger_length_[0] * finger_length_[1] /2 * cos(now_ang[1])) * finger_weight_[1];
    // m_theta(0,0)
    
    Eigen::Vector2d b_dt_ddt, g_theta, u_jf, dd_theta;
    g_theta(1) = finger_weight_[1]/2 * g * a2 * sin(now_ang[0] + now_ang[1]);
    g_theta(0) = g_theta(1) + (finger_weight_[0]/2 + finger_weight_[1]) * g * a1 * sin(now_ang[0]);
    // double temp = finger_weight_[1] * a1 * a2 * sin(now_ang[1]);
    b_dt_ddt(0) = -finger_weight_[1] * a1 * a2 * sin(now_ang[1]) * now_ang[1] * now_ang[1] /2;
    b_dt_ddt(1) = finger_weight_[1] * a1 * a2 * (sin(now_ang[1]) + cos(now_ang[1])) * now_ang[1] * now_ang[2] /2;
    double dx2 = a2 * sin(now_ang[0] + now_ang[1]);
    double dy2 = a2 * cos(now_ang[0] + now_ang[1]);
    u_jf(0) = this->torque[0] + (dx2 + a1 * sin(now_ang[0])) * contactForce.Fy
        + (dy2 + a1 * cos(now_ang[0])) * contactForce.Fx - 0.3 * ang_vel[0];
    u_jf(1) = this->torque[1] + dx2 * contactForce.Fy + dy2 * contactForce.Fx - 0.3 * ang_vel[1];
    
    dd_theta = m_theta.colPivHouseholderQr().solve(u_jf - b_dt_ddt - g_theta);
    // dd_theta = m_theta.colPivHouseholderQr().solve(u_jf - g_theta);

    ROS_INFO("accelerate is: %f %f", ang_acc[0], ang_acc[1]);
    ROS_INFO("angle is: %f %f", now_ang[0], now_ang[1]);
    ROS_INFO("velocity is: %f, %f", ang_vel[0], ang_vel[1]);
    
    //double new_pos[2], new_vel[2];
    this->ang_acc[0] = dd_theta(0);
    this->ang_acc[1] = dd_theta(1);
    
    integrate(now_ang, ang_vel, ang_acc, delta_t);
    this->now_pos[0] += this->tail_vel * delta_t;
    // ROS_INFO("accelerate is: %f %f", ang_acc[0], ang_acc[1]);
    // ROS_INFO("angle is: %f %f", now_ang[0], now_ang[1]);
    // ROS_INFO("velocity is: %f, %f", ang_vel[0], ang_vel[1]);
  }
  
  void PoseCal::torqueUpd(const std_msgs::Float64 &msg) {
    // this->torque[0] = msg.torque1;
    this->torque[1] = msg.data;
  }
  // void PoseCal::torqueUpd(const adaption_msgs::JointForce &msg) {
  //   this->torque[0] = msg.torque1;
  //   this->torque[1] = msg.torque2;
  // }
  
  void PoseCal::posePub() {
    adaption_msgs::FingerInfo info;
    info.stamp = ros::Time::now();
    info.angle.theta1 = this->now_ang[0];
    info.angle.theta2 = this->now_ang[1];
    info.position.x = this->now_pos[0];
    info.position.y = this->now_pos[1];
    
    this->finger_info_pub_.publish(info);
  }
  
  void PoseCal::forcePub() {
    this->force_pub_.publish(this->contactForce);
    this->force_pub_2.publish(this->contactForce.Fy);
  }
  
  void integrate(double* init_pos, double* init_vel, const double* acc,
      //double* res_pos, double* res_vel,
      double delta_t) {
    //double k11 = 0;  // d(theta)_0
    //double k21 = 0;  // theta_0
    //double k12 = acc[0] * delta_t/2;
    //double k22 = (init_vel[0] + k21 + acc[0] * delta_t / 4) * delta_t/2;
    //double k13 = k12;
    //double k22 =
    //res_vel[0] = init_vel[0] + acc[0] * delta_t;
    //res_vel[1] = init_vel[1] + acc[1] * delta_t;
    //res_pos[0] = init_pos[0] + (init_vel[0] + acc[0] * delta_t / 2) * delta_t;
    //res_pos[1] = init_pos[1] + (init_vel[1] + acc[1] * delta_t / 2) * delta_t;
    init_pos[0] += (init_vel[0] + acc[0] * delta_t / 2) * delta_t;
    init_pos[1] += (init_vel[1] + acc[1] * delta_t / 2) * delta_t;
    init_vel[0] += acc[0] * delta_t;
    init_vel[1] += acc[1] * delta_t;

    // while (init_pos[0] > M_PI) {
    //   init_pos[0] -= 2 * M_PI;
    // }
    // while (init_pos[0] < -M_PI) {
    //   init_pos[0] += 2 * M_PI;
    // }
    // while (init_pos[1] > M_PI) {
    //   init_pos[1] -= 2 * M_PI;
    // }
    // while (init_pos[1] < -M_PI) {
    //   init_pos[1] += 2 * M_PI;
    // }
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
  
  adaption_simulation::PoseCal calculator(&nh);
  ros::Rate r(pose_f);
  int report = pose_f / sensor_f;
  double time_interval = 1.0 / pose_f;
  int i = 0;
  while (ros::ok()) {
    ros::spinOnce();
    calculator.forceCal();
    calculator.poseUpd(time_interval);
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