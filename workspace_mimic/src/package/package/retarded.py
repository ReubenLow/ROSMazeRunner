from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

#performs deep copy of the Pose Objects
def cpyPoseObj(lhs, rhs):
    lhs.pose.position.x = rhs.pose.position.x
    lhs.pose.position.y = rhs.pose.position.y
#debugging purpose
def printPoseObject(pose_obj):
    print("Pose: ")
    print(pose_obj.pose.position.x)
    print(pose_obj.pose.position.y)
    print(pose_obj.pose.position.z)
    print(pose_obj.pose.orientation.w)

class map_node():
    def __init__(self, pose_object_prev, pose_object_current):
        #Store the previous Map Node in the current Node
        self.pose_obj_prev = PoseStamped()
        cpyPoseObj(self.pose_obj_prev, pose_object_prev)

        #stores the current map node into this object
        self.pose_obj_curr = PoseStamped()
        cpyPoseObj(self.pose_obj_curr, pose_object_current)
    
    #performs deep copy of the two classes
    def map_node_copy_cstr(self, rhs):
        cpyPoseObj(self.pose_obj_curr, rhs.pose_obj_curr)
        cpyPoseObj(self.pose_obj_prev, rhs.pose_object_prev)

    def prodNode(self, coord, change):
        #coord x or y
        #change +1 or -1
        #compares the change with the previous Map node to see if they matches
        #returns a new map_node()
            #a changed pose object that is verified
            #new map_node() contains the current object as a pose_obj_prev
        new_pose = PoseStamped()
        cpyPoseObj(new_pose, self.pose_obj_curr) 

        if(coord =='x'):
            new_pose.pose.position.x = new_pose.pose.position.x + change
            if(new_pose.pose.position.x != self.pose_obj_prev.pose.position.x):
                #do something
                new_node = map_node(self.pose_obj_curr, new_pose)
                return new_node
            else:
                print("Failed! Modified Node (H+1) and Previous Node (H-1) is the same!")
                return None
        elif(coord == 'y'):
            new_pose.pose.position.y = new_pose.pose.position.y + change
            if(new_pose.pose.position.y != self.pose_obj_prev.pose.position.y):
                #do something
                new_node = map_node(self.pose_obj_curr, new_pose)
                return new_node
            else:
                print("Failed! Modified Node (H+1) and Previous Node (H-1) is the same!")
                return None
        else:
            print("Parameters error! Returning...")
            return None
        



#mimics std::stack top()
def peek(stack):
    element = stack.pop()
    stack.append(element)
    return element


class MARK_Initial_Pose_State(Node):
    def __init__(self):
        super().__init__('PoseMark')
        self.subscription = self.create_subscription(Odometry, 'odom', self.mark_callback, 10)
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


#Blocking Travel Function
def travel(navigator, prev_pose, new_pose):
    navigator.goToPose(new_pose)
    #path = navigator.getPath(prev_pose, newPose)
    printPoseObject(prev_pose)
    printPoseObject(new_pose)
    i = 0
    while not navigator.isTaskComplete():
        i
    return True
    # if(path != None):
    #     print("Path exist!")
    #     print("Travel")
    #     navigator.followPath(path)
    #     i = 0
    #     while not navigator.isTaskComplete():
    #         i
    #     return True
    # else:
    #     print("Path does not exist!")
    #     return None






#stack to pass the stack across the traversal
#end_case variable to dictate when to stop recursion
def traverse(stack, navigator):
    mapNode = peek(stack)
    new_node = mapNode.prodNode('x', 1.0)
    if(new_node == None):
        # new_node = mapNode.prodNode('x', -0.2)
        new_node
    else:
        print("Trying out for +x")
        status = travel(navigator, new_node.pose_obj_prev, new_node.pose_obj_curr)
    #     if(status == None):
    #         print("Trying out for -x")
    #         new_node = mapNode.prodNode('x', -0.2)
    #     else:
    #         print("New location reached")
    #         stack.append(new_node)
    #         traverse(stack, navigator)

    # if(new_node == None):
    #     #Do nothing for now
    #     print("Do nothing")
    # else:
    #     print("Trying out for -x")
    #     status = travel(navigator, new_node.pose_obj_prev, new_node.pose_obj_curr)
    #     if(status == None):
    #         print("Y coord is not implemented yet")
    #         return
    #     else:
    #         print("New location reached")
    #         stack.append(new_node)
    #         traverse(stack, navigator)
    
    
    
def main():
    rclpy.init()

    #map_node stack
    historical_poses = []
    #navigator
    navigator = BasicNavigator()
    navigator.node_name = "SimpleCommander"


    #initial pose set
    firstCommand = MARK_Initial_Pose_State()
    initial_pose = firstCommand.mark_return_initial_pose()
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    #first map_node into stack
    initial_map_node = map_node(initial_pose, initial_pose)
    historical_poses.append(initial_map_node)
    print("First Operation Successful")
    print("Commencing Traversal")
    #status = traverse(historical_poses, navigator)
    lmao = PoseStamped()
    lmao.pose.position.x = 0.0
    lmao.pose.position.y = 1.0
    travel(navigator, initial_pose, lmao)

    # if(status == True):
    #     print("Traverse operation complete")
    #     return


if __name__ == '__main__':
    main()
