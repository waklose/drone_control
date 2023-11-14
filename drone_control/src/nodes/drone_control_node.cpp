// Based on rotor_control/lee_position_controller_node.cpp

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "drone_control_node.h"

namespace drone_control {

DroneControllerNode::DroneControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh),
   last_published_controll_msg(0.0){
  InitializeParams();

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &DroneControllerNode::MultiDofJointTrajectoryCallback, this);


  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &DroneControllerNode::OdometryCallback, this);

  rollPitchYawrateThrust_reference_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &DroneControllerNode::TimedCommandCallback, this,
                                  true, false);
}

DroneControllerNode::~DroneControllerNode() { }

void DroneControllerNode::InitializeParams() {
  drone_controller_.InitializeParameters();
}

void DroneControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  drone_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void DroneControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  drone_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void DroneControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("DroneController got first odometry message.");

  double time_now = ros::Time::now().toSec();
  if (last_published_controll_msg == 0.0) {
    last_published_controll_msg = time_now; 
  } else if (time_now > last_published_controll_msg + 1/controller_publish_frequency) {
    drone_controller_.UpdateIntegral(time_now-last_published_controll_msg);
    last_published_controll_msg = time_now; 
  } else {
    return;
  }

  rotors_control::EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  drone_controller_.SetOdometry(odometry);
 
  mav_msgs::EigenRollPitchYawrateThrust msg_values;
  drone_controller_.ComputeDesiredRollPitchThrust(&(msg_values.roll), &(msg_values.pitch), &(msg_values.thrust));
  drone_controller_.ComputeDesiredYawrate(&(msg_values.yaw_rate));

  mav_msgs::RollPitchYawrateThrust* rollPitchYawrateThrust_msg(new mav_msgs::RollPitchYawrateThrust);

  rollPitchYawrateThrust_msg->header.stamp = odometry_msg->header.stamp;
  mav_msgs::msgRollPitchYawrateThrustFromEigen(msg_values, rollPitchYawrateThrust_msg);

  mav_msgs::RollPitchYawrateThrustPtr ready_rollPitchYawrateThrust_msg(rollPitchYawrateThrust_msg);

  rollPitchYawrateThrust_reference_pub_.publish(ready_rollPitchYawrateThrust_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "drone_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  drone_control::DroneControllerNode drone_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
