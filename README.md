# ROSMazeRunner


# Launching Gazebo, RVIZ and Navigation Stack without prior mapping:
**Either through the launch file I produced OR :**


1. export TURTLEBOT3_MODEL=burger
2. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
3. ros2 launch nav2_bringup navigation_launch.py
4. ros2 launch slam_toolbox online_async_launch.py
5. ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz



# Archives:
MUST READ
1. https://navigation.ros.org/commander_api/index.html#simple-commander-api
2. https://navigation.ros.org/configuration/packages/configuring-amcl.html

# Launch File:

1. Ensure that setup.py is properly setup with the naming of the launch folder and params
2. Ensure that source install/local_setup.bash is executed before launching
3. Ensure that no nodes are active "ros2 node list". If not, kill them or restart (experience)
4. Main focus now is tuning in of navigation parameters AND
5. Creating an algorithm that would increment/decrement, store waypoints(to go back if stuck) until robot exits the maze

ros2 launch <your_package_name> <your_launch_filename>.py


# Issue Remedy Archives:
Unknown Parameter to be set to true for costmaps with voxel or obstacle layer. Planner prefers free space to chart a path. without doing the necessary,
unknown areas in costmaps are treated as free spaces. Hence, set it for both global and local costmaps
1. https://answers.ros.org/question/215538/allow_unknown-parameter-ignored/

AMCL guarantees no change to map but SLAM does - can we incorporate AMCL instead?
3. https://answers.ros.org/question/246747/difference-between-localisation-with-amcl-and-slam/


