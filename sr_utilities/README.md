#sr_utilities
This package contains utility libraries.
There are examples of using python implementation of HandFinder utility in [HandCommander](../sr_robot_commander/doc/tutorial/HandCommander.md) and [RobotCommander](../sr_robot_commander/doc/tutorial/RobotCommander.md).
Check the [test file](test/test_hand_finder.cpp) for an example of using C++ implementation of HandFinder.

## ROS interface
**trajectory_controller_spawner.py** checks the hands installed on the system (with HandFinder library) and finds the actual hand joints by parsing the URDF from parameter server (robot_description). It the position controllers for joints are not spawned the node spawn them. Afterwards it checks the parameter server for *~hand_trajectory* parameter and if it exists and its value is true it will spawn the trajectory controller for the hands.   
If only the position controllers are required, node can be run without the *~hand_trajectory* parameter or setting its value to false.  
*Examples:*  
For trajectory and position controllers  
```bash
rosrun sr_utilities trajectory_controller_spawner.py _hand_trajectory:=true
``` 
Spawns only the position controllers  
```bash
rosrun sr_utilities trajectory_controller_spawner.py _hand_trajectory:=false
```
In some cases it may be better to control the wrist joints with the arm trajectory. 
To remove the joints from the hand trajectory set a private parameter *~exclude_wrist* to true.
```bash
rosrun sr_utilities trajectory_controller_spawner.py _exclude_wrist:=true
