from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node



def printPose(pose_obj):
    print("Pose: ")
    print(pose_obj.pose.position.x)
    print(pose_obj.pose.position.y)



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




#stack to pass the stack across the traversal
#end_case variable to dictate when to stop recursion
#fromDir
#from left : -1
#from right : +1
#from top: -2
#from bottom : +2
#start at centre : -3
def traverse(stack, navigator, fromDir):
    #retrieve previous goal
    prev_goal = peek(stack)
    #copy previous goal
    new_goal = prev_goal
    print("Traverse")
    #top
    if(fromDir != -2):
        new_goal.pose.position.x = new_goal.pose.position.x + 1
        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            print("Go North")
            printPose(new_goal)
            stack.append(new_goal)
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            fromDir = 2
            traverse(stack, navigator, fromDir)
        else:
            print("Path does not exist!")

        new_goal.pose.position.x = new_goal.pose.position.x - 1 #undo the increment
    #right
    if(fromDir != 1):
        new_goal.pose.position.y = new_goal.pose.position.y - 1
        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            print("Go Right")
            printPose(new_goal)
            stack.append(new_goal)
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            fromDir = -1
            traverse(stack, navigator, -1)
        else:
            print("Path does not exist!")

        new_goal.pose.position.y = new_goal.pose.position.y + 1 #undo the increment
    #######################################################################################
    #left
    if(fromDir != -1):
        new_goal.pose.position.y = new_goal.pose.position.y + 1

        path = navigator.getPath(prev_goal, new_goal)
        if(path != None):
            print("Path exist!")
            print("Go Left")
            printPose(new_goal)
            stack.append(new_goal)
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            fromDir = 1
            traverse(stack, navigator, 1)
        else:
            print("Path does not exist!")

        new_goal.pose.position.y = new_goal.pose.position.y - 1 #undo the increment
    #######################################################################################
    #DEAD END
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

    status = traverse(historical_poses, navigator, -3)

    if(status == True):
        print("Traverse operation complete")
        return


if __name__ == '__main__':
    main()
