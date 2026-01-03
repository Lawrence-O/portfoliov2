import { Project} from "@/app/components/project/interfaces";

export const multiRobotFrontierExploration: Project = {
    title: "Multi-Robot Frontier Exploration",
    date: "Fall 2023",
    media: "/media/videos/frontier_grid_huge.mp4",
    githubLink: "https://github.com/your-username/multiagent-frontier-exploration", // Added placeholder
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
                        "The core logic was implemented in Python. Key components include the `Node` class for A* search, the `Robot` class, and the `Planner` class which manages map updates, frontier detection, and pathfinding using a multi-goal A* algorithm. The A* algorithm was modified to consider temporary costs (`temp_costs`) associated with regions targeted by other robots, aiding in decentralized coordination.",
                },
                {
                    type: "code",
                    codeLang: "python",
                    content: `
from collections import defaultdict
import heapq

# temp_costs stores temporary cost increases for regions near other robots' targets
temp_costs = defaultdict(lambda: defaultdict(lambda: 0))
NEIGHBOR_COST_RANGE = 5
NEIGHBOR_COST_INCREASE = 5

class Node:
    def __init__(self, x=0, y=0, drone_id=0, g=0, h=0):
        self.x, self.y, self.drone_id = x, y, drone_id
        self.g, self.h = g, h
        self.f = g + h
    
    # Equality and hash based on x, y, drone_id
    # ... ( __eq__ and __hash__ ) ...

    def __lt__(self, other): # For priority queue in A*
        my_val = self.f
        other_val = other.f
        # Incorporate temporary costs from other drones for coordination
        for curr_drone_id, cost_dict in temp_costs.items():
            if curr_drone_id != self.drone_id:
                my_val += cost_dict.get((self.x, self.y), 0)
            if curr_drone_id != other.drone_id:
                other_val += cost_dict.get((other.x, other.y), 0)
        
        if my_val == other_val:
            return self.g < other.g if self.g != other.g else self.h < other.h
        return my_val < other_val

class Planner:
    def __init__(self, truth_map):
        self.truth_map = truth_map
        self.grid_map = [[-1 for _ in row] for row in truth_map] # -1: unknown
        self.frontier_cells = []
        # ... other initializations ...

    def get_frontier_nodes(self):
        # Identifies cells at the boundary of known free space and unknown space
        # Appends Node(x, y, ...) to self.frontier_cells
        # ...
        pass

    def heuristic(self, curr_x, curr_y): # Simplified: goal_nodes is self.frontier_cells
        # Calculates min Euclidean distance from (curr_x, curr_y) to any node in self.frontier_cells
        # ...
        pass

    def multi_goal_A(self, start_node, curr_drone_id, drones_paths, waypoint_counters, all_drones):
        # A* search implementation
        open_set = [start_node]
        # ... (g_score, parent dictionaries) ...
        
        temp_costs[curr_drone_id].clear() # Clear own temp costs before planning

        while open_set:
            curr_node = heapq.heappop(open_set)

            if self.is_goal(curr_node): # is_goal checks if curr_node is in self.frontier_cells
                # Path found, update temp_costs for this drone's chosen frontier region
                for x_offset in range(-NEIGHBOR_COST_RANGE, NEIGHBOR_COST_RANGE + 1):
                    for y_offset in range(-NEIGHBOR_COST_RANGE, NEIGHBOR_COST_RANGE + 1):
                        temp_costs[curr_drone_id][(curr_node.x + x_offset, curr_node.y + y_offset)] = NEIGHBOR_COST_INCREASE
                # Reconstruct and return path
                # ...
                return path 

            # Expand neighbors, considering self.truth_map, collisions with other_drones
            # For each valid neighbor:
            #   new_node = Node(...)
            #   Update g_score, push to open_set if better path
            # ...
        return [] # No path found

# Main simulation loop (conceptual)
# def main():
#   Initialize drones, planner, visualization
#   while not fully_explored:
#       for drone in drones:
#           if path_needed_for_drone:
#               planner.refresh_frontier_list()
#               start_node = Node(drone.x, drone.y, drone.id)
#               paths[drone.id] = planner.multi_goal_A(start_node, drone.id, ...)
#           
#           if paths[drone.id]:
#               Move drone along its path
#               planner.update_map(drone) # Based on new position and sensor range
#       
#       Check if fully_explored (e.g., no more frontiers)
#       Visualize current state
                    `,
                    subtitle: "Key Python snippets for multi-robot frontier exploration (simplified)",
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
