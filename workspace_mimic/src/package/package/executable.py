#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

"""
Basic navigation demo to go to pose.
"""

#
class MARKCommanderInitiate(Node):
    def __init__(self):
        super().__init__('Listening_Commander')
        self.subscription = self.create_subscription(Odometry, 'odom', self.mark_callback, 10)
        self.subscription #Prevent unused warning
        self.startPose = PoseStamped()
    
    def mark_callback(self, msg):
        #...
        startPose.pose.pose.position.x = msg.pose.pose.position.x
        startPose.pose.pose.position.y = msg.pose.pose.position.y
        startPose.pose.pose.position.z = msg.pose.pose.position.z
        startPose.pose.pose.orientation.z = msg.pose.pose.orientation.z
        startPose.pose.pose.orientation.w = msg.pose.pose.orientation.w
    

    def mark_return_initial_pose(self):
        self.startPose.header.frame_id = 'map'
        return self.startPose

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.node_name = "SimpleCommander"
    firstCommand = MARKCommanderInitiate()
    n = 0
    while(n < 10):
        n = n + 1
    #return PoseStamped() object containing all details
    initial_pose = firstCommand.mark_return_initial_pose()
    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.38
    # initial_pose.pose.position.y = 5.85
    # initial_pose.pose.position.z = 0.0
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0

    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active(navigator="bt_navigator")
    # navigator.waitUntilNav2Active(navigator="bt_navigator", localizer="bt_navigator")
    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x =  2.00
    goal_pose.pose.position.y = -2.00
    goal_pose.pose.orientation.w =  1.0

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose)
    # smoothed_path = navigator.smoothPath(path)


    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                print("Navigation Timeout!!!")
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #     goal_pose.pose.position.x = -3.0
            #     navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()

    #exit(0)


if __name__ == '__main__':
    main()
