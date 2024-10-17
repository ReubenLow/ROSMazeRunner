from collections import deque

# Replace the DFS traversal function with BFS
def traverse_bfs(queue, static_map_set, navigator):
    print("===========")
    printSet(static_map_set)
    print("===|||====")
    
    # BFS works in a loop, processing each node level by level
    while queue:
        # Dequeue the current node
        current_goal = queue.popleft()
        
        # Attempt to move in all four directions (north, south, east, west)
        
        # North (increment x)
        new_goal = createPoseObj(current_goal, navigator)
        new_goal.pose.position.x += 1
        process_new_goal(new_goal, current_goal, static_map_set, queue, navigator, "North")
        
        # South (decrement x)
        new_goal.pose.position.x -= 2  # undo increment, then decrement
        process_new_goal(new_goal, current_goal, static_map_set, queue, navigator, "South")
        
        # East (increment y)
        new_goal.pose.position.x += 1  # reset x to original
        new_goal.pose.position.y += 1
        process_new_goal(new_goal, current_goal, static_map_set, queue, navigator, "East")
        
        # West (decrement y)
        new_goal.pose.position.y -= 2  # undo increment, then decrement
        process_new_goal(new_goal, current_goal, static_map_set, queue, navigator, "West")
        
        print("Finished processing current position.")

def process_new_goal(new_goal, current_goal, static_map_set, queue, navigator, direction):
    path = navigator.getPath(current_goal, new_goal)
    
    if path is not None:
        status = check_duplicate_then_append(queue, static_map_set, new_goal, navigator)
        if status:
            print(f"Moving {direction}")
            travel(navigator, path)
            navigator.clearAllCostmaps()
        else:
            print(f"Duplicate detected in {direction}, skipping.")
    else:
        print(f"No path exists {direction}, skipping.")

def check_duplicate_then_append(queue, static_map_set, posObj, navigator):
    for mappedObj in static_map_set:
        if math.isclose(posObj.pose.position.x, mappedObj.x, rel_tol=1e-2) and math.isclose(posObj.pose.position.y, mappedObj.y, rel_tol=1e-2):
            return None  # Duplicate detected

    # No duplicate, add the new node to the set and the queue
    mappedObj = MapPoseObj(posObj)
    static_map_set.add(mappedObj)
    queue.append(posObj)
    return True

def main():
    rclpy.init()
    
    # Queue for BFS
    historical_poses_queue = deque()
    
    # Set for visited nodes
    map_nodes_set = set()
    
    # Navigator
    navigator = BasicNavigator()
    navigator.node_name = "SimpleCommander"

    # Initial pose
    firstCommand = MARK_Initial_Pose_State()
    initial_pose = firstCommand.mark_return_initial_pose()
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    # Enqueue the initial position for BFS
    historical_poses_queue.append(initial_pose)

    # Add the initial pose to the set of visited nodes
    mapPoseObj = MapPoseObj(initial_pose)
    map_nodes_set.add(mapPoseObj)

    # Start BFS traversal
    traverse_bfs(historical_poses_queue, map_nodes_set, navigator)

if __name__ == '__main__':
    main()
