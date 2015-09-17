#sr_utilities
This package contains utility libraries.
There are examples of using python implementation of HandFinder utility in [HandCommander](../sr_robot_commander/doc/tutorial/HandCommander.md) and [RobotCommander](../sr_robot_commander/doc/tutorial/RobotCommander.md).
Check the [test file](test/test_hand_finder.cpp) for an example of using C++ implementation of HandFinder.

## ROS interface
**trajectory_controller_spawner** checks the hands installed on the system (with HandFinder library) and finds the actual hand joints. It then spawn the trajectory controller. If the joints do not have position controllers running, they get spawned too.  
If only the position controllers are required, an argument trajectory='false' can be provided to the node.  
  