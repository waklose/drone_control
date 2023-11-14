# drone_control
This repository contains a ROS package (with two helper files) for controlling position and yawrate for a drone simulation provided by Autonomous Robotics Lab (ARL) at NTNU, [link](https://github.com/ntnu-arl/gbplanner_ros).

## structure
- The package itself is contained inside of the drone_control directory.
- The launch file (rmf_sim_positionYawrateControl.launch) was added to the ./launch/rmf directory of the gbplanner package.
- The yaml file (roll_pitch_yawrate_thrust_controller_rmf_obelix.yaml) is a tuning file and it was added to the resource directory of the rotors_gazebo package.

## running
If the [installation guide](https://github.com/ntnu-arl/gbplanner_ros/wiki/Installation) from ARL is followed and the structure above is used, the node can be run by: 
```
cd <path/to/gbplanner2_ws>
source devel/setup.bash
roslaunch gbplanner rmf_sim_positionYawrateControl.launch
```

