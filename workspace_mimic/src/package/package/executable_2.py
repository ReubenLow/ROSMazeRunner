from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


class MARK_Initial_Pose_State(Node):
    def __init__(self):
        super().__init__('Initiate MARK Pose')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.mark_callback, 10)
        self.subscription #Prevent unused warning
        self.startPose = PoseStamped()


    def mark_callback(self, msg):
        startPose.pose.pose.position.x = msg.pose.pose.position.x
        startPose.pose.pose.position.y = msg.pose.pose.position.y
        startPose.pose.pose.position.z = msg.pose.pose.position.z
        startPose.pose.pose.orientation.z = msg.pose.pose.orientation.z
        startPose.pose.pose.orientation.w = msg.pose.pose.orientation.w

    def mark_return_initial_pose(self):
        n = 0
        while(n < 2):
            n = n + 1
        self.startPose.header.frame_id = 'map'
        #shuts down this node subscription
        self.destroy_subscription(self.subscription)
        return self.startPose


def travel(path):
    followPath(path)
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




#stack to pass the stack across the traversal
#end_case variable to dictate when to stop recursion
def traverse(stack, prevDir, end_case):
    #retrieve previous goal
    prev_goal = stack.top()
    #copy previous goal
    new_goal = prev_goal
    #######################################################################################
    #TOP
    if(prevDir != 'top'):
        new_goal.pose.pose.position.x = new_goal.pose.pose.position.x + 1.0

        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            historical_poses.append(new_goal)
            travel(path)
            #recursion happens here
            traverse(stack, 'bottom', end_case)
        else:
            print("Path does not exist!")

        new_goal.pose.pose.position.x = new_goal.pose.pose.position.x - 1.0 #undo the increment
    #######################################################################################
    #bottom
    if(prevDir != 'bottom'):
        new_goal.pose.pose.position.x = new_goal.pose.pose.position.x - 1.0

        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            historical_poses.append(new_goal)
            travel(path)
            #recursion happens here
            traverse(stack, 'top', end_case)
        else:
            print("Path does not exist!")

        new_goal.pose.pose.position.x = new_goal.pose.pose.position.x + 1.0 #undo the increment
    #######################################################################################
    #right
    if(prevDir != 'right'):
        new_goal.pose.pose.position.x = new_goal.pose.pose.position.z - 1.0

        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            historical_poses.append(new_goal)
            travel(path)
            #recursion happens here
            traverse(stack, 'left', end_case)
        else:
            print("Path does not exist!")

        new_goal.pose.pose.position.x = new_goal.pose.pose.position.z + 1.0 #undo the increment
    #######################################################################################
    #left
    if(prevDir != 'left'):
        new_goal.pose.pose.position.x = new_goal.pose.pose.position.z + 1.0

        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            historical_poses.append(new_goal)
            travel(path)
            #recursion happens here
            traverse(stack, 'right', end_case)
        else:
            print("Path does not exist!")

        new_goal.pose.pose.position.x = new_goal.pose.pose.position.z - 1.0 #undo the increment
    #######################################################################################    
    #DeadEnd
        current_node = stack.top()
        previous_node_index = len(stack) - 2
        if(previous_node_index < 0):
            #do nothing
        else:
            previous_node = stack[previous_node_index]
            path = navigator.getPath(current_node, previous_node)
            travel(path)
            stack.pop()
        return
def main():
    rclpy.init()

    #node stack
    historical_poses = []
    #navigator node
    navigator = BasicNavigator()
    navigator.node_name = "SimpleCommander"

    #initial pose set
    initial_pose = firstCommand.mark_return_initial_pose()
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    #first node into stack
    historical_poses.append(initial_pose)

    status = traverse(historical_poses, 'centre', end_case)

    if(status == True):
        print("Traverse operation complete")
        return


if __name__ == '__main__':
    main()
