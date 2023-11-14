

#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace drone_control {

// Default values for the position PID controller (rmf_obelix)
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(1, 1, 5);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(0.6, 0.6, 2);
static const Eigen::Vector3d kDefaultIntegralGain = Eigen::Vector3d(0, 0, 0);

// Default values for the yaw rate PID controller (rmf_obelix)
static const double kDefaultYawAnguleGain = 4;
static const double kDefaultYawAngularRateGain = 1;


class DroneControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DroneControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        integral_gain_(kDefaultIntegralGain),
        yawAngle_gain_(kDefaultYawAnguleGain),
        yawAngularRate_gain_(kDefaultYawAngularRateGain) {}
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d integral_gain_;
  double yawAngle_gain_;
  double yawAngularRate_gain_;
};

class DroneController {
 public:
  DroneController();
  ~DroneController();
  void InitializeParameters();

  void SetOdometry(const rotors_control::EigenOdometry& odometry);
  void SetTrajectoryPoint(
      const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  void UpdateIntegral(const double delta_time);
  void ResetIntegral(){position_integral_=Eigen::Vector3d(0, 0, 0);}


  void ComputeDesiredRollPitchThrust(double* roll, double* pitch, Eigen::Vector3d* thrust) const;
  void ComputeDesiredYawrate(double* yawrate) const;

  DroneControllerParameters controller_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;

  Eigen::Vector3d position_integral_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  rotors_control::EigenOdometry odometry_;
};
}

#endif // DRONE_CONTROLLER_H