TODO: Install ros2 package in workspace.


# Create workspace and source:
https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/

1. mkdir -p ~/ros2_ws/src
2. cd ~/ros2_ws
3. colcon build
4. source ~/ros2_ws/install/local_setup.bash
5. source ~/ros2_ws/install/setup.bash

# Create package from scratch:
https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/#createpkg

1. cd ~/ros2_ws/src
2. NOTE: Skip step 3 and COPY and PASTE robot_runner_native_test in ~/ros2_ws/src
3. ros2 pkg create --build-type ament_cmake robot_runner_native_test robot.launch.py

NOTE: after finishing package, source install/setup.bash AGAIN.
NOTE: chmod +x robot.launch.py
NOTE: 
-- ADD TO CMAKELISTS.txt

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

-- ADD TO CMAKELISTS.txt

# Kill a launched system:
1. set one node to be 'required': on_exit=launch.actions.Shutdown())