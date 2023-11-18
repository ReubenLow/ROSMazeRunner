from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node



count_traverse = 0

#mimics std::stack top()
def peek(stack):
    element = stack.pop()
    stack.append(element)
    return element


class MARK_Initial_Pose_State(Node):
    def __init__(self):
        super().__init__('PoseMark')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.mark_callback, 10)
        self.subscription #Prevent unused warning
        self.startPose = PoseStamped()


    def mark_callback(self, msg):
        self.startPose.pose.position.x = msg.pose.pose.position.x
        self.startPose.pose.position.y = msg.pose.pose.position.y
        self.startPose.pose.position.z = msg.pose.pose.position.z
        self.startPose.pose.orientation.z = msg.pose.pose.orientation.z
        self.startPose.pose.orientation.w = msg.pose.pose.orientation.w

    def mark_return_initial_pose(self):
        n = 0
        while(n < 2):
            n = n + 1
        self.startPose.header.frame_id = 'map'
        #shuts down this node subscription
        self.destroy_subscription(self.subscription)
        print("Initial pose:")
        print(self.startPose.pose.position.x)
        print(self.startPose.pose.position.y)
        return self.startPose


def travel(navigator, path):
    navigator.followPath(path)
    i = 0
    while not navigator.isTaskComplete():
        i = 1
        # ################################################
        # #
        # # Implement some code here for your application!
        # #
        # ################################################

        # # Do something with the feedback
        # i = i + 1
        # feedback = navigator.getFeedback()
        # if feedback and i % 5 == 0:
        #     print('Estimated time of arrival: ' + '{0:.0f}'.format(
        #           Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
        #           + ' seconds.')    

def endTraversal(prevDir, direction, end_case):
    if(prevDir == direction):
        end_case = end_case + 1
    else:
        direction = prevDir
        end_case = 0
        count_traverse = 0
    if(end_case >= 20):
        return True
    else:
        return None



#stack to pass the stack across the traversal
#end_case variable to dictate when to stop recursion
def traverse(stack, prevDir, direction, navigator, end_case):
    #retrieve previous goal
    prev_goal = peek(stack)
    #copy previous goal
    new_goal = prev_goal
    status = endTraversal(prevDir, direction, end_case)
    if(status == True):
        return True
    else:
        print("Traverse")
        #######################################################################################
        #TOP
        if(prevDir != 'top'):
            new_goal.pose.position.x = new_goal.pose.position.x + 1

            path = navigator.getPath(prev_goal, new_goal)
            if(path != None):
                print("Path exist!")
                stack.append(new_goal)
                travel(navigator, path)
                #recursion happens here
                traverse(stack, 'bottom', direction,navigator, end_case)
            else:
                print("Path does not exist!")

            new_goal.pose.position.x = new_goal.pose.position.x - 1 #undo the increment
        #######################################################################################
        #bottom
        if(prevDir != 'bottom'):
            new_goal.pose.position.x = new_goal.pose.position.x - 1

            path = navigator.getPath(prev_goal, new_goal)
            if(path != None):
                print("Path exist!")
                stack.append(new_goal)
                travel(navigator, path)
                #recursion happens here
                traverse(stack, 'top', direction,navigator, end_case)
            else:
                print("Path does not exist!")

            new_goal.pose.position.x = new_goal.pose.position.x + 1 #undo the increment
        #######################################################################################
        #right
        if(prevDir != 'right'):
            new_goal.pose.position.x = new_goal.pose.position.z - 1

            path = navigator.getPath(prev_goal, new_goal)
            if(path != None):
                print("Path exist!")
                stack.append(new_goal)
                travel(navigator, path)
                #recursion happens here
                traverse(stack, 'left',direction,navigator, end_case)
            else:
                print("Path does not exist!")

            new_goal.pose.position.x = new_goal.pose.position.z + 1 #undo the increment
        #######################################################################################
        #left
        if(prevDir != 'left'):
            new_goal.pose.position.x = new_goal.pose.position.z + 1

            path = navigator.getPath(prev_goal, new_goal)
            if(path != None):
                print("Path exist!")
                stack.append(new_goal)
                travel(navigator, path)
                #recursion happens here
                traverse(stack, 'right',direction,navigator, end_case)
            else:
                print("Path does not exist!")

            new_goal.pose.position.x = new_goal.pose.position.z - 1 #undo the increment
        #######################################################################################    
        #DeadEnd
            current_node = peek(stack)
            previous_node_index = len(stack) - 2
            if(previous_node_index < 0):
                #do nothing
                print("Well whatever")
            else:
                previous_node = stack[previous_node_index]
                path = navigator.getPath(current_node, previous_node)
                travel(navigator, path)
                stack.pop()
def main():
    rclpy.init()
    direction = ''
    #node stack
    historical_poses = []
    #navigator node
    navigator = BasicNavigator()
    navigator.node_name = "SimpleCommander"


    #initial pose set
    firstCommand = MARK_Initial_Pose_State()
    initial_pose = firstCommand.mark_return_initial_pose()
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    #first node into stack
    historical_poses.append(initial_pose)

    status = traverse(historical_poses, 'centre', direction, navigator, count_traverse)

    if(status == True):
        print("Traverse operation complete")
        return


if __name__ == '__main__':
    main()
