#sr_utilities
This package contains utility libraries.
There are examples of using python implementation of HandFinder utility in [HandCommander](../sr_robot_commander/doc/tutorial/HandCommander.md) and [RobotCommander](../sr_robot_commander/doc/tutorial/RobotCommander.md).
Check the [test file](test/test_hand_finder.cpp) for an example of using C++ implementation of HandFinder.

## ROS interface
**trajectory_controller_spawner.py** checks the hands installed on the system (with HandFinder library) and finds the actual hand joints by parsing the URDF from parameter server (robot_description). It then spawn the trajectory controller for the hands. If joints do not have position controllers running, trajectory_controller_spawner.py spawns the position controllers too.  
If only the position controllers are required, an argument *trajectory='false'* can be provided to the node which stops it from spawning the trajectory controller but spawns all the position controllers.  
*Examples:*
For trajectory and position controllers
'''bash
rosrun sr_utilities trajectory_controller_spawner.py
''' 
To just spawn the position controllers
'''bash
rosrun sr_utilities trajectory_controller_spawner.py trajectory:=false
'''