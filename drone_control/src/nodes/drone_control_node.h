// Based on rotor_control/lee_position_controller_node.h

#ifndef DRONE_CONTROL_NODE_H
#define DRONE_CONTROL_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "rotors_control/common.h"

#include "drone_control/drone_control.h"

namespace drone_control {

static const double controller_publish_frequency = 100; //[messages/second]

class DroneControllerNode {
 public:
  DroneControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~DroneControllerNode();

  void InitializeParams();
  void Publish();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  DroneController drone_controller_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher rollPitchYawrateThrust_reference_pub_;

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;

  double last_published_controll_msg;

  void TimedCommandCallback(const ros::TimerEvent& e);

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
}

#endif // DRONE_CONTROL_NODE_H