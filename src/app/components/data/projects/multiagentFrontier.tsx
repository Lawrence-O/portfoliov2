import { Project} from "@/app/components/project/interfaces";

export const multiRobotFrontierExploration: Project = {
    title: "Multi-Robot Frontier Exploration",
    subtitle: "Fall 2023",
    media: "/media/videos/frontier_grid_huge.mp4",
    tags: ["Robotics", "Multi-Agent Systems", "Path Planning", "A*", "Python"],
    section: [
        {
            title: "Project Overview",
            navName: "Overview",
            navRef: "overview",
            content: [
                {
                    type: "text",
                    content:
                        "This project focused on developing a multi-robot system for efficiently exploring an unknown environment. The project leverages a frontier-based exploration strategy, which allows multiple robots to work collaboratively to map an environment more quickly than a single robot. This multi-robot system is applicable in various fields, including search and rescue operations, environmental monitoring, and autonomous mapping.",
                },
                {
                    type: "image",
                    content: "/media/images/frontier_inprogress.png",
                    altContent: "A grid map being explored by multiple robots.",
                    subtitle: "A visual example of a map under exploration."
                },
            ],
        },
        {
            title: "Problem Statement and Objectives",
            navName: "Problem Statement",
            navRef: "problem-statement",
            content: [
                {
                    type: "text",
                    content:
                        "The core challenge of this project was to design a system that would enable multiple robots to efficiently explore an unknown 2D environment. Each robot is equipped with a limited sensing range and must collaborate to completely map the area, reducing exploration time while avoiding collisions with other robots. The primary objective is to achieve complete environmental coverage while maintaining an efficient and collision-free operation.",
                },
            ],
        },
        {
            title: "Exploration Approach and Design",
            navName: "Approach",
            navRef: "approach",
            content: [
                {
                    type: "text",
                    content:
                        "Our approach utilizes a decentralized, greedy frontier-based algorithm. Each robot independently identifies frontier cells (the boundaries between explored and unexplored areas), and then plans its path toward the nearest frontier using a modified A* search algorithm. This decentralized system promotes robustness and scalability. To prevent collisions, the robots take into account the planned paths of others, temporarily increasing the cost of areas near frontiers that are currently being explored.",
                },
                {
                    type: "text",
                    content:
                        "Key aspects of our approach include:",
                },
                {
                    type: "text",
                    content:
                       "Frontier Detection: The robot identifies the boundaries between explored free space and unknown space, focusing exploration on the most informative regions. Multi-Goal A* Path Planning: A modified A* algorithm is used to navigate robots towards frontier cells while also avoiding other robots.  Decentralized Control: Each robot operates independently, coordinating its actions based on local awareness and the dynamic adjustment of cost maps. Dynamic Obstacle Avoidance: The system incorporates dynamic obstacle avoidance where robots can detect a path collision with a dynamic obstacle, replan, and then move to a safe position.",
                },
            ],
        },
         {
            title: "Detailed Algorithm Implementation",
            navName: "Algorithm Details",
            navRef: "algorithm-details",
            content: [
                {
                    type: "text",
                    content:
                       "The core algorithm consists of several key steps. Each robot uses its sensors to update a local map of the environment. Then, they identify frontier cells based on the updated map, and finally, a multi-goal A* algorithm is used to navigate the robots toward the nearest frontier.",
                },
                {
                     type: "text",
                    content:
                    "The Multi-Goal A* Algorithm, a modification of the standard A* algorithm, allows robots to navigate to any number of frontier cells instead of just a single goal. It also allows the robot to consider the paths of other robots by increasing the cost of nearby cells. These cost maps help to prevent robots from choosing the same path and colliding with each other.",
                },
                {
                    type: "text",
                     content:
                    "Our dynamic obstacle handling system allows robots to detect a collision with a dynamic object, which forces a robot to replan its path, moving it to a safe region.",
                },
                 {
                    type: "text",
                     content:
                    "Robot Coordination was achieved through a decentralized control system where each robot is operating independently, while avoiding each other through a cost map. This cost map helps reduce the amount of collisions that occur while also allowing them to spread out.",
                },
            ]
        },
         {
            title: "Core Concepts",
            navName: "Key Concepts",
            navRef: "key-concepts",
            content: [
                {
                    type: "text",
                    content:
                        "Here are some of the core principles and ideas that drove this project:",
                },
                 {
                     type: "text",
                    content:
                    "Frontier-Based Exploration: This strategy directs robots to the boundaries between known and unknown areas, enabling efficient exploration.",
                },
                {
                     type: "text",
                    content:
                    "Greedy Approach: Robots choose the nearest frontier cell as their next goal. This simplifies decision-making and shows to work effectively in practice.",
                },
               {
                    type: "text",
                     content:
                    "Decentralized Approach: Each robot calculates its path independently. This simplifies implementation and allows robots to work even if there is a communication failure.",
                },
                 {
                    type: "text",
                     content:
                    "Cost Maps: Robots use cost maps to avoid each other, with increased costs in regions occupied or targeted by other robots to avoid collisions.",
                },
                 {
                    type: "text",
                     content:
                     "Multi-Goal A*: The A* algorithm was modified to allow for multiple goals, these goals are the frontier cells that are being explored. This avoids having to replan each time a robot reaches a single frontier. "
                },
            ]
        },
        {
            title: "Code Implementation",
            navName: "Code Implementation",
            navRef: "code",
            content: [
               {
                    type: "text",
                    content:
                        "The core logic of the project was implemented using Python and several standard libraries. Below are some key snippets from the project.",
                },
                {
                    type: "code",
                    codeLang: "python",
                    content: `
import heapq
import math
import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from vis import Visualize
from collections import defaultdict

# Constants
DRONE_Z = 1
UNKNOWN_SPACE = -1
FREE_SPACE = 0
OCCUPIED_SPACE = 100
NEIGHBOR_COST_RANGE = 5
NEIGHBOR_COST_INCREASE = 5
DRONE_START_LOC = 2
drone_ids = []
temp_costs = defaultdict(lambda : defaultdict(lambda : 0))

# Grid Node
class Node:
    '''
    Represents a node in the grid for path planning.
    '''
    def __init__(self, x=0, y=0, drone_id=0, g=0, h=0):
        '''
        Initializes a node.
        
        @param x: The x-coordinate of the node.
        @param y: The y-coordinate of the node.
        @param drone_id: The ID of the drone.
        @param g: The cost from start node to current node.
        @param h: The heuristic cost from current node to goal node.
        '''
        self.x = x
        self.y = y
        self.drone_id = drone_id
        self.g = g
        self.h = h
        self.f = g + h
    
    def __eq__(self, other):
        '''
        Checks if two nodes are equal.
        
        @param other: The other node to compare.
        @return: True if the nodes have the same x and y coordinates, False otherwise.
        '''
        return self.x == other.x and self.y == other.y
    def __lt__(self, other):
        '''
        Compares two nodes based on their f-value, g-value, and h-value.
        Used for priority queue ordering.
        
        @param other: The other node to compare.
        @return: True if the current node is less than the other node.
        '''
        
        my_val = self.f
        other_val = other.f
        for (curr_drone_id, cost_dict) in temp_costs.items():
            if (curr_drone_id != self.drone_id):
                my_val += cost_dict[(self.x, self.y)]
            if (curr_drone_id != other.drone_id):
                other_val += cost_dict[(other.x, other.y)]
        
        if my_val == other_val:
            if self.g != other.g:
                return self.g < other.g
            return self.h < other.h
        return my_val < other_val
    def __hash__(self):
        '''
        Calculates the hash value of the node.
        
        @return: The hash value of the node.
        '''
        return hash((self.x, self.y, self.drone_id))

#Class Robots
class Robot:
    '''
    Represents a robot in the environment.
    '''
    def __init__(self, start_x, start_y, drone_id):
        '''
        Initializes a robot.

        @param start_x: The starting x-coordinate of the robot.
        @param start_y: The starting y-coordinate of the robot.
        @param drone_id: The unique ID of the robot.
        '''
        self.x = start_x
        self.y = start_y
        self.id = drone_id
        self.path = []
    def __eq__(self,other):
        '''
        Checks if two robots are equal.
        
        @param other: The other robot to compare.
        @return: True if the robot IDs are the same, False otherwise.
        '''
        return self.id == other.id
    
class Planner:
    '''
    Manages the exploration of the environment.
    '''
    def __init__(self, truth_map):
        '''
        Initializes the planner with a truth map.
        
        @param truth_map: The true environment map.
        '''
        self.laser_scan_range = 1
        self.truth_map = truth_map
        self.grid_map = [[-1 for col in range(len(truth_map[row]))] for row in range(len(truth_map))]
        self.frontier_cells = []

    def is_valid_idx(self, x, y):
        '''
        Checks if the given indices are within the bounds of the grid.
        
        @param x: The x-coordinate.
        @param y: The y-coordinate.
        @return: True if the indices are valid, False otherwise.
        '''
        return 0 <= x < len(self.grid_map) and 0 <= y < len(self.grid_map[0])

    def update_map(self, robot):
        '''
        Updates the grid map based on the robot's current location and sensor range.
        
        @param robot: The robot instance.
        '''
        x, y = robot.x, robot.y
        for i in range(-self.laser_scan_range, self.laser_scan_range+1):
            for j in range(-self.laser_scan_range, self.laser_scan_range+1):
                if self.is_valid_idx(x+i, y+j):
                    self.grid_map[x+i][y+j] = self.truth_map[x+i][y+j]
    
    def get_frontier_nodes(self):
        '''
        Identifies frontier nodes (unexplored neighbors of explored free space).
        '''
        dirX = [-1, 0, 1, -1, 1, -1, 0, 1]
        dirY = [-1, -1, -1, 0, 0, 1, 1, 1]
        for x in range(len(self.grid_map)):
            for y in range(len(self.grid_map[x])):
                if self.grid_map[x][y] == 0:
                    is_frontier = False
                    for (dx,dy) in zip(dirX,dirY):
                        newX = x + dx
                        newY = y + dy
                        if self.is_valid_idx(newX,newY):
                            if self.grid_map[newX][newY] == -1:
                                is_frontier = True
                                break
                    if is_frontier:
                        self.frontier_cells.append(Node(x, y, 0, 0, 0))
    
    def refresh_frontier_list(self):
        '''
        Clears the frontier list and recalculates the frontier nodes.
        '''
        self.frontier_cells = []
        self.get_frontier_nodes()
    
    def is_goal(self,curr_node):
        '''
        Checks if a node is a goal (frontier) node.
        
        @param curr_node: The node to check.
        @return: True if the node is a frontier node, False otherwise.
        '''
        for node in self.frontier_cells:
            if node == curr_node:
                return True
        return False
    
    def heuristic(self,curr_node_x, curr_node_y, goal_nodes):
        '''
        Calculates the heuristic distance to the nearest frontier node.
        
        @param curr_node_x: The x-coordinate of the current node.
        @param curr_node_y: The y-coordinate of the current node.
        @param goal_nodes: The list of frontier nodes.
        @return: The minimum Euclidean distance to the frontier nodes.
        '''
        min_dist = float('inf')
        for goal in self.frontier_cells:
            dist = (curr_node_x - goal.x) ** 2 + (curr_node_y - goal.y) ** 2
            if dist < min_dist:
                min_dist = dist
        return min_dist
    
    def check_collision_drone(self,curr_node_x, curr_node_y, drones):
        '''
        Checks if a node is occupied by any of the robots.
        
        @param curr_node_x: The x-coordinate of the current node.
        @param curr_node_y: The y-coordinate of the current node.
        @param drones: The list of robots.
        @return: True if the node is occupied by another robot, False otherwise.
        '''
        for drone in drones:
            if drone.x == curr_node_x and drone.y == curr_node_y:
                return True
        return False
    
    def multi_goal_A(self,start_node, curr_drone_id, paths, waypoint_counters, drones):
        '''
        Implements the A* path-finding algorithm for multiple robots with goal frontiers.
        
        @param start_node: The start node.
        @param curr_drone_id: The ID of the current robot.
        @param paths: The paths of all the robots.
        @param waypoint_counters: The waypoint counter for each robot.
        @param drones: The list of robots.
        @return: The path to a frontier, or an empty list if no path is found.
        '''
        other_path_set = set()
        for other_drone_id in paths:
            for i in range(waypoint_counters[other_drone_id], len(paths[other_drone_id])):
                other_path_set.add(paths[other_drone_id][i])
        temp_costs[curr_drone_id].clear()
        
        #Define OpenSet
        open_set = [start_node]
        heapq.heapify(open_set)
        visited = set()
        g_score = dict()
        parent = dict()
        while open_set:
            curr_node = heapq.heappop(open_set)
            if self.is_goal(curr_node):
                path = []
                current_node = curr_node
                for x in range(curr_node.x - NEIGHBOR_COST_RANGE, curr_node.x + NEIGHBOR_COST_RANGE):
                    for y in range(curr_node.y - NEIGHBOR_COST_RANGE, curr_node.y + NEIGHBOR_COST_RANGE):
                        temp_costs[curr_drone_id][(x, y)] = NEIGHBOR_COST_INCREASE
                while current_node.x != start_node.x or current_node.y != start_node.y:
                    path.append(current_node)
                    current_node = parent[current_node]
                path.reverse()
                return path
            visited.add(curr_node)
            dirX = [-1, -1, -1, 0, 0, 1, 1, 1]
            dirY = [-1, 0, 1, -1, 1, -1, 0, 1]
            for (dx,dy) in zip(dirX,dirY):
                new_x = curr_node.x + dx
                new_y = curr_node.y + dy
                if self.is_valid_idx(new_x,new_y) and self.truth_map[new_x][new_y] == 0 and not self.check_collision_drone(new_x, new_y, drones):
                    new_cost = curr_node.g + 1
                    new_node = Node(new_x, new_y, curr_drone_id, new_cost, self.heuristic(new_x, new_y, self.frontier_cells))
                    if new_node not in g_score or new_cost < g_score[new_node]:
                        g_score[new_node] = new_cost
                        heapq.heappush(open_set, new_node)
                        parent[new_node] = curr_node
        return []
    
def main():
    '''
    Main function to run the multi-robot exploration simulation.
    '''
    global temp_costs, drone1_waypoint_counter, drone2_waypoint_counter, drone1_path, drone2_path, drone1_node_path, drone2_node_path, drone1_currentPos, drone2_currentPos
    drone1_waypoint_counter = 0
    drone2_waypoint_counter = 0
    drone1_path = []
    drone2_path = []
    
    plt.ion()
    vis = Visualize("grid.txt")
    truth_map = vis.grid
    start_x1 = None
    start_y1 = None
    start_x2 = None
    start_y2 = None
    drones = []
    drone_id = 0
    for x in range(len(truth_map)):
        for y in range(len(truth_map[x])):
            if (truth_map[x][y] == DRONE_START_LOC):
                print("Drone added")
                drones.append(Robot(x,y,drone_id))
                truth_map[x][y] = 0
                drone_id += 1
    
    drone_positions = [(drone.x, drone.y) for drone in drones]
    planner = Planner(truth_map)
    for drone in drones:
        planner.update_map(drone)
    planner.refresh_frontier_list()
    paths = {i: [] for i in range(len(drones))}
    waypoint_counters = {i: 0 for i in range(len(drones))}
    fully_explored = False
    while not fully_explored:
        for drone in drones:
            if  not paths[drone.id] or waypoint_counters[drone.id] >= len(paths[drone.id]):
                waypoint_counters[drone.id] = 0
                    
                droneStart = Node(drone.x, drone.y, drone.id, 0, 0)
                planner.refresh_frontier_list()
                print("drone about to a*")
                paths[drone.id] = planner.multi_goal_A(droneStart, drone.id, paths, waypoint_counters, drones)
            else:
                # tell drone1 to move
                curr_node = paths[drone.id][waypoint_counters[drone.id]]
                
                collision = False
                
                for other_drone in drones:
                    if (curr_node.x == other_drone.x) and (curr_node.y == other_drone.y):
                        #we are trying to move into the location a drone is
                        print("trying to collide with drone, will replan")
                        collision = True
                        break
                if collision:   
                    paths[drone.id] = []
                    waypoint_counters[drone.id] = 0
                else:
                    waypoint_counters[drone.id] += 1
                
                    drone.x = curr_node.x
                    drone.y = curr_node.y
                
                #reveal the area on the map
                planner.update_map(drone)
    
        drone_positions = [(drone.x, drone.y) for drone in drones]
        vis.plot_grid(np.array(planner.grid_map), drone_positions)
        
        if not planner.frontier_cells:
            planner.get_frontier_nodes()
            if not planner.frontier_cells:
                # we are done
                fully_explored = True
        time.sleep(0.5)
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show the final plot
    print("Environment explored")
                
main()
                    `,
                    subtitle: "A Python implementation of multi-robot exploration.",
                }
            ],
        },
         {
            title: "Experimental Results",
            navName: "Results",
            navRef: "results",
            content: [
               {
                    type: "text",
                    content:
                        "We conducted experiments to evaluate the performance of our algorithm in different scenarios. The primary metrics included the time required to fully explore the environment and the impact of varying the number of robots.",
                },
                {
                    type: "text",
                    content: "The results are summarized below:",
                },
                {
                    type: "text",
                    content:
                        "As expected, the exploration time increases with the size of the map. Our results showed a somewhat linear relationship between the number of cells and the time to complete the exploration, with larger maps taking more time but generally scaling well with respect to cell size.",
                },
                 {
                    type: "image",
                    content: "/media/images/frontier_multmap.png",
                    altContent: "Small, medium and large maps used for testing",
                    subtitle: "Maps of varying sizes were used for testing.",
                },
                 {
                    type: "text",
                    content:
                        "Increasing the number of robots significantly reduces exploration time, but with diminishing returns. Initially, each additional robot provides a substantial speedup, but as the number of robots increases, the benefits decrease, due to interference and multiple robots exploring the same area.",
                },
                 {
                     type: "image",
                    content: "/media/images/frontier_robotsgraph.png",
                    altContent: "A plot showing the relationship between run time and the number of robots",
                    subtitle: "Run time decreases as number of robots increases.",
                },
                {
                    type: "text",
                    content: "We found that an optimal range of five to seven robots was most efficient for the largest map tested, balancing coverage and redundancy."
                }
            ],
        },
         {
            title: "Future Work",
            navName: "Future Work",
            navRef: "future-work",
            content: [
               {
                    type: "text",
                    content:
                        "There are several avenues to expand upon this research. These include:",
                },
                 {
                    type: "text",
                    content:
                        "Testing the algorithm in more complex environments. More complex environments could include more dynamic obstacles or more complex map topologies.",
                },
                {
                     type: "text",
                    content:
                        "Testing with different sensing ranges. The current implementation only allows for a sensor range of 1. Expanding the sensor range could allow for faster exploration.",
                },
               {
                    type: "text",
                     content:
                        "Exploring different heuristic functions for the A* algorithm. A better heuristic might allow for more efficient exploration.",
                },
                  {
                    type: "text",
                     content:
                    "Implementing communication between robots. While the decentralized approach is robust, adding communication could increase efficiency.",
                },
            ]
        },
        {
            title: "Conclusion",
            navName: "Conclusion",
            navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content:
                        "This project demonstrates the effectiveness of a multi-robot system for efficient frontier-based exploration. Our results show that multi-robot systems can significantly reduce exploration time compared to single-robot systems. The algorithm is scalable to larger environments, but the number of robots should be selected carefully to avoid diminishing returns due to robot interference. This research contributes to the understanding and development of multi-agent exploration strategies for autonomous systems.",
                },
            ],
        },
    ],
};