# sr_utilities
This package contains utility libraries.
There are examples of using python implementation of HandFinder utility in [HandCommander](../sr_robot_commander/doc/tutorial/HandCommander.md) and [RobotCommander](../sr_robot_commander/doc/tutorial/RobotCommander.md).
Check the [test file](test/test_hand_finder.cpp) for an example of using C++ implementation of HandFinder.

## ROS interface

### Controller Spawner

**controller_spawner.py** spawns the ROS controllers necessary for Shadow dexterous hands, based on a config file.

The default config file is in [sr_interface/sr_robot_launch/config/controller_spawner.yaml](https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_launch/config/controller_spawner.yaml). The config file contains all possible ROS controllers, organised into controller groups. You can specify a different config file using the `config_file_path` parameter.

When the controller spawner is run, it finds any running Shadow dexterous hands, and based on the `controller_group` parameter, launches the subset of controllers appropriate for the joints in the running hands. It also puts any relevant controller parameters defined in the `controller_configs` section of the config file on the parameter server.

Other controller groups defined in the config file are also stored on the ROS parameter server under `/controller_groups` for ease of controller group switching later.

Any controllers defined in the config file, relevant to the currently running hands, but not part of the requested controller group will be stopped.

How long the spawner waits for controller management services to come up is defined by the `service_timeout` parameter.

The `wait_for` parameter can be used to specify the name of a topic that the spawner will wait for before attempting to spawn controllers.

Joints can be excluded from the launched controllers using the `excluded_joints` parameter. If you want certain joint controllers to launch, even if other controllers relevant to those joints are excluded by `exclude_joints`, put the necessary controllers in a `necessary_if_joint_present` array. A historic example of this was when we wanted to (sometimes) exclude the wrist joints from e.g. `rh_trajectory_controller`, but needed the wrist joint position controllers to be running regardless. In this case, `rh_WRJ1` and `rh_WRJ2` were specified in `excluded joints`, which excluded them from `rh_trajectory_controller`, but `sh_rh_wrj1_position_controller` and `sh_rh_wrj2_position_controller` were included in a `necessary_if_joint_present` array, meaning they were launched anyway.

*Examples:*  
For trajectory and position controllers  
```bash
rosrun sr_utilities controller_spawner.py _controller_group:=trajectory
``` 
Spawns only the position controllers  
```bash
rosrun sr_utilities controller_spawner.py _controller_group:=position
```
In some cases it may be better to control the wrist joints with the arm trajectory. 
To remove the joints from the hand trajectory set a private parameter *~exclude_wrist* to true.
```bash
rosrun sr_utilities controller_spawner.py _exclude_wrist:=true
```
