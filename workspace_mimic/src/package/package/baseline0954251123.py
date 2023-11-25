from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

import math

class MapPoseObj():
    def __init__(self, poseStampedObj):
        self.x = poseStampedObj.pose.position.x
        self.y = poseStampedObj.pose.position.y
        self.Orz = poseStampedObj.pose.orientation.z
        self.Orw = poseStampedObj.pose.orientation.w
    

    def convMapPosetoPoseStamp(self, prev_goal, navigator):
        new_goal = PoseStamped()
        new_goal.pose.position.x = prev_goal.pose.position.x
        new_goal.pose.position.y = prev_goal.pose.position.y
        new_goal.pose.orientation.z = prev_goal.pose.orientation.z
        new_goal.pose.orientation.w = prev_goal.pose.orientation.w
        new_goal.header.stamp = navigator.get_clock().now().to_msg()
        new_goal.header.frame_id = 'map'
        return new_goal


def check_duplicate_then_append(stack , static_map_set, posObj, navigator):
    for jkl in static_map_set:
        xStat = math.isclose(posObj.pose.position.x, jkl.x, rel_tol=1e-2)
        yStat = math.isclose(posObj.pose.position.y, jkl.y, rel_tol=1e-2)
        print("New x ", posObj.pose.position.x, "New y ", posObj.pose.position.y, "Old x ", jkl.x, "Old y ", jkl.y)
        if(xStat == True and yStat == True):
            print("Duplicate detected! Changing course!")
            return None

    print("No Duplicates!")
    mappedObj = MapPoseObj(posObj)
    static_map_set.add(mappedObj)
    stack.append(posObj)
    return True


#Debugging functions Pose() object print
def printPose(pose_obj):
    print("Pose: ")
    print(pose_obj.pose.position.x)
    print(pose_obj.pose.position.y)

#Debugging functions Pose() object print
def printStack(stack):
    for pose_obj in stack:
        print("::::::::::::::::::::::::")
        print("Pose x: ", pose_obj.pose.position.x)
        print("Pose y: ", pose_obj.pose.position.y)
        print("::::::::::::::::::::::::")


def printSet(set_map):
    for pose_obj in set_map:
        print("::::::::::::::::::::::::")
        print("Pose x: ", pose_obj.x)
        print("Pose y: ", pose_obj.y)
        print("::::::::::::::::::::::::")


#mimics std::stack top()
def peek(stack):
    element = stack.pop()
    stack.append(element)
    return element

def createPoseObj(prev_goal, navigator):
    new_goal = PoseStamped()
    new_goal.pose.position.x = prev_goal.pose.position.x
    new_goal.pose.position.y = prev_goal.pose.position.y
    new_goal.pose.orientation.z = prev_goal.pose.orientation.z
    new_goal.pose.orientation.w = prev_goal.pose.orientation.w
    new_goal.header.stamp = navigator.get_clock().now().to_msg()
    new_goal.header.frame_id = 'map'
    return new_goal

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





def traverse(stack, static_map_set, navigator, fromDir):
    print("===========")
    printSet(static_map_set)
    print("===|||====")
    #retrieve previous goal
    prev_goal = peek(stack)
    #copy previous goal
    new_goal = createPoseObj(prev_goal, navigator)
    print("Traverse")
    #top
    new_goal.pose.position.x = new_goal.pose.position.x + 1
    path = navigator.getPath(prev_goal, new_goal)
    if(path != None):
        print("Path exist!")
        status = check_duplicate_then_append(stack, static_map_set, new_goal, navigator)
        printPose(new_goal)
        print("Go North")
        if(status == True):
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            traverse(stack, static_map_set,  navigator, fromDir)
    else:
        print("Path does not exist!")
    new_goal.pose.position.x = new_goal.pose.position.x - 1 #undo the increment


    #left
    new_goal.pose.position.y = new_goal.pose.position.y + 1
    path = navigator.getPath(prev_goal, new_goal)
    if(path != None):
        print("Path exist!")
        status = check_duplicate_then_append(stack, static_map_set, new_goal, navigator)
        printPose(new_goal)
        print("Go Left")
        if(status == True):
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            traverse(stack, static_map_set,  navigator, fromDir)
    else:
        print("Path does not exist!")
    new_goal.pose.position.y = new_goal.pose.position.y - 1 #undo the increment


    #right
    new_goal.pose.position.y = new_goal.pose.position.y - 1
    path = navigator.getPath(prev_goal, new_goal)
    if(path != None):
        print("Path exist!")
        status = check_duplicate_then_append(stack, static_map_set, new_goal, navigator)
        printPose(new_goal)
        print("Go Right")
        if(status == True):
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            traverse(stack, static_map_set,  navigator, fromDir)
    else:
        print("Path does not exist!")
    new_goal.pose.position.y = new_goal.pose.position.y + 1 #undo the increment


    #bottom
    new_goal.pose.position.x = new_goal.pose.position.x - 1
    path = navigator.getPath(prev_goal, new_goal)
    if(path != None):
        print("Path exist!")
        status = check_duplicate_then_append(stack, static_map_set, new_goal, navigator)
        printPose(new_goal)
        print("Go South")
        if(status == True):
            travel(navigator, path)
            navigator.clearAllCostmaps()
            #recursion happens here
            traverse(stack, static_map_set,  navigator, fromDir)
    else:
        print("Path does not exist!")
    new_goal.pose.position.x = new_goal.pose.position.x + 1 #undo the increment

    #DEAD END
    print("Dead End")
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
        #does not remove set() nodes ; marks the nodes traversed as "visited" permanently


def main():
    rclpy.init()
    direction = ''
    #node stack
    historical_poses = []
    #set() containing all nodes visited
    map_nodes_set = set()
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

    #convert PoseStamped() into a python object
    mapPoseObj = MapPoseObj(initial_pose)
    map_nodes_set.add(mapPoseObj)


    status = traverse(historical_poses, map_nodes_set,  navigator, -3)

    if(status == True):
        print("Traverse operation complete")
        return


if __name__ == '__main__':
    main()
