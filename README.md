# ROSMazeRunner


# Launching Gazebo, RVIZ and Navigation Stack without prior mapping:
1. export TURTLEBOT3_MODEL=burger
2. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
3. ros2 launch nav2_bringup navigation_launch.py
4. ros2 launch slam_toolbox online_async_launch.py
5. ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz



# Archives:
MUST READ
1. https://navigation.ros.org/commander_api/index.html#simple-commander-api

Reference for Creating a Launch File
1. Launching a launch file
   - ros2 launch turtlebot3_gazebo assmaze.launch.py
   - ros2 launch mark_1 nav_to_pose_example.launch.py
