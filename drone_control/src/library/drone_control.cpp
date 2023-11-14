#include "drone_control/drone_control.h"
//#include <tf/tf.h>

namespace drone_control {

DroneController::DroneController()
    : initialized_params_(false),
        controller_active_(false),
        position_integral_(Eigen::Vector3d(0, 0, 0))
{
    InitializeParameters();
}
DroneController::~DroneController() {}

void DroneController::InitializeParameters(){
    initialized_params_ = true;
}

void DroneController::SetOdometry(const rotors_control::EigenOdometry& odometry){
    odometry_ = odometry;
}

void DroneController::UpdateIntegral(const double delta_time){
    Eigen::Vector3d position_error = odometry_.position - command_trajectory_.position_W;
    position_integral_ += position_error*delta_time;
}

void DroneController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory){
    command_trajectory_ = command_trajectory;
    controller_active_ = true;
    ResetIntegral();
}


void DroneController::ComputeDesiredRollPitchThrust(double* roll, double* pitch, Eigen::Vector3d* thrust) const {
    double g = 9.81;
    double mass = 0.5265; //[kg]
    Eigen::Vector3d position_error = odometry_.position - command_trajectory_.position_W;
    Eigen::Vector3d velocity_error = odometry_.velocity; // ref is zero
    Eigen::Vector3d a_feedback =  -controller_parameters_.position_gain_.cwiseProduct(position_error)
                            -controller_parameters_.velocity_gain_.cwiseProduct(velocity_error)
                            -controller_parameters_.integral_gain_.cwiseProduct(position_integral_);

    Eigen::Vector3d z_world = Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d a_desired = a_feedback + g*z_world;

    //find roll and pitch based on the desired acceleration
    //the convention used in RollPitchYawrateThrust message is intrinsic in the order yaw-pitch-roll (z, y', x'')
    //finding the roll/pitch angle is therefor dependent upon the current yaw angle.
    Eigen::Matrix3d current_R = odometry_.orientation.toRotationMatrix();
    double current_yaw = atan2(current_R(1, 0), current_R(0, 0));

    Eigen::Vector3d a_desired_norm = a_desired.normalized();
    Eigen::Vector3d y_dash = Eigen::Vector3d(-sin(current_yaw), cos(current_yaw), 0);
    Eigen::Vector3d x_dashdash = y_dash.cross(a_desired_norm);

    Eigen::Vector3d pitched = x_dashdash.cross(y_dash);

    double pitch_abs = atan2(z_world.cross(pitched).norm(), z_world.dot(pitched));
    *pitch = (z_world.cross(pitched).dot(y_dash) > 0) ? pitch_abs : -pitch_abs;

    double roll_abs = atan2(pitched.cross(a_desired_norm).norm(), pitched.dot(a_desired_norm));
    *roll = (pitched.cross(a_desired_norm).dot(x_dashdash)) > 0 ? roll_abs : -roll_abs;

    *thrust = Eigen::Vector3d(0, 0, a_desired.norm()*mass);
}

void DroneController::ComputeDesiredYawrate(double* yawrate) const {
    Eigen::Matrix3d current_R = odometry_.orientation.toRotationMatrix();
    double current_yaw = atan2(current_R(1, 0), current_R(0, 0));

    Eigen::Matrix3d desired_R = command_trajectory_.orientation_W_B.toRotationMatrix();
    double desired_yaw = atan2(desired_R(1, 0), desired_R(0, 0));

    double angle_error = current_yaw - desired_yaw;

    *yawrate =  -controller_parameters_.yawAngle_gain_*angle_error
                -controller_parameters_.yawAngularRate_gain_*odometry_.angular_velocity[2];


}

}