# drone_control
This repository contains a ROS package (with two helper files) for controlling position and yawrate for a drone simulation provided by Autonomous Robotics lab at NTNU, [link](https://github.com/ntnu-arl/gbplanner_ros).

## structure
- The package itself is contained inside of the drone_control directory.
- The launch file (rmf_sim_positionYawrateControl.launch) was added to the ./launch/rmf directory of the gbplanner package.
- The yaml file (roll_pitch_yawrate_thrust_controller_rmf_obelix.yaml) is a tuning file and it was added to the resource directory of the rotors_gazebo package.
