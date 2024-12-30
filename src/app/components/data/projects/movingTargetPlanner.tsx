import { Project } from "@/app/components/project/interfaces";

export const movingTargetPlanner: Project = {
  title: "Real-Time Target Interception with Multi-Goal A*",
  media: "/media/videos/movingTargetsVideo.mp4", // Add a URL or file path to media if available
  subtitle: "Spring 2024",
  section: [
    {
      title: "Introduction",
      content: [
        {
          type: "text",
          content:
            "The goal of this project was to develop a planner for a robot tasked with catching a moving target in a 2D grid world. The robot operates in an 8-connected grid, where it can move by at most one cell along the X, Y axes, or diagonally in each step. The planner minimizes the cost incurred by the robot (rather than the time it takes) while avoiding obstacles and high-cost areas in the grid.",
        },
      ],
      navRef: "Introduction",
    },
    {
      title: "Problem Statement",
      content: [
        {
          type: "text",
          content:
            "The problem imposes several constraints and inputs. The gridworld includes a costmap specifying the traversal cost for each cell, represented as positive integers. A collision threshold determines cells the robot cannot traverse; any cell with a cost greater than or equal to this threshold is treated as an obstacle. The planner is given the start position of the robot and the trajectory of the target as a sequence of positions, such as [(5,6), (5,7), (4,5)]. The planner operates under real-time constraints, with the target moving at a speed of one step per second. The largest grid size is approximately 2000x2000 for graduate students and 200x200 for undergraduates.",
        },
      ],
      navRef: "Problem Statement",
    },
    {
      title: "Algorithm Overview",
      content: [
        {
          type: "text",
          content:
            "The implemented planner utilizes a variant of Multi-Goal A*, treating the target’s trajectory as a series of dynamic goals. The algorithm operates in a 3D state space, defined as: <X,Y,T>. Where X and Y represent the robot’s position in the grid, and T represents the time step.",
        },
        {
            type: "text",
            content:
              "The planner uses a priority queue (open-set) to ensure nodes are processed in ascending order of their total cost, prioritizing nodes with the lowest cost.",
          },
          {
            type: "text",
            content:
              "When expanding nodes, the planner considers the eight possible moves in the 8-connected grid and a ninth option where the robot stays in its current position. This results in nine possible successor states for any node.",
          },{
            type: "code",
            codeLang: "cpp",
            subtitle: "Multi-Goal A*",
            content: `/**
 * @brief Implements the Multi-Goal A* pathfinding algorithm.
 *
 * This function finds a path for a robot navigating a 3D environment
 * (grid with X, Y coordinates and time steps) towards one of the provided
 * goal locations. It uses a priority queue (open set) to prioritize nodes
 * with the lowest total cost (f-score).
 *
 * @param map A pointer to the map data (represented as an array of ints).
 * @param x_size The width of the map.
 * @param y_size The height of the map.
 * @param collision_thresh The threshold for considering a cell a collision.
 * @param target_steps The total number of time steps allowed.
 * @param curr_time The current time step.
 * @param heuristicArray A pre-computed heuristic array for faster cost estimation.
 * @param startNode The starting node for the search.
 * @param goalNodes A vector of goal nodes representing target locations.
 * @return A vector of nodes representing the path from start to goal,
 *         or an empty vector if no path is found.
 */
std::vector<Node> multi_goal_A(const int* map, int x_size, int y_size,
                               int collision_thresh, int target_steps,
                               int curr_time, std::vector<int>& heuristicArray,
                               Node& startNode, std::vector<Node> goalNodes) {

  // Open set for prioritizing nodes based on f-score (g + heuristic)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
  openSet.push(startNode); // Add start node to open set

  // Visited set to avoid revisiting nodes
  std::unordered_map<Node, bool, NodeHash> visited;

  // g-score (movement cost from start) for each node (initially infinity)
  std::vector<int> gScore(x_size * y_size, INT_MAX);

  // Parent node for reconstructing the path
  std::unordered_map<Node, Node, NodeHash> parent;

  while (!openSet.empty()) {
    // Get the node with the lowest f-score from the open set
    Node currNode = openSet.top();
    openSet.pop();

    // Check if we reached a goal
    if (is_goal(currNode, x_size, y_size, curr_time, target_steps, goalNodes)) {
      // Reconstruct the path by backtracking from the goal node
      std::vector<Node> path;
      Node currentNode = currNode;
      while (currentNode.x != startNode.x || currentNode.y != startNode.y) {
        path.push_back(currentNode);
        currentNode = parent[currentNode];
      }
      path.push_back(startNode); // Add start node to the path
      std::reverse(path.begin(), path.end()); // Reverse for start -> goal order
      return path;
    }

    // Mark current node as visited
    visited[currNode] = true;

    // Explore neighboring nodes in 8 connected directions + staying put
    int dX[9] = {-1, -1, -1, 0, 0, 1, 1, 1, 0};
    int dY[9] = {-1, 0, 1, -1, 1, -1, 0, 1, 0};
    int dT[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
      for (int dir = 0; dir < 9; dir++) {
        int newX = currNode.x + dX[dir];
        int newY = currNode.y + dY[dir];
        int newT = currNode.t + dT[dir];
        int mapIndex = GETMAPINDEX(newX, newY, x_size, y_size);
        // Check if the new position is within the map bounds and time limit
        if (newX >= 1 && newX <= x_size && newY >= 1 && newY <= y_size &&
            newT < target_steps) { 
          int map_cost = (int)map[mapIndex];
          int new_cost = currNode.g + map_cost;
          Node newNode(newX, newY, newT, new_cost, heuristicArray[mapIndex]);
          // Check if the new node is valid and not visited
          if ((map_cost < collision_thresh) &&
              (visited.find(newNode) == visited.end()) &&
              (new_cost < gScore[mapIndex])) {
            openSet.push(newNode);
            parent[newNode] = currNode;
            gScore[mapIndex] = new_cost;
          }
        }
      }
    }
  
    // No path found
    return std::vector<Node>();
  }
                      `,
          },
          {
            type: "text",
            content:
              "The planner terminates when the current node matches one of the target’s positions at the corresponding time step. For computationally large maps, a relaxation is applied when less than of the remaining time is available. In such cases, the algorithm returns a suboptimal path to ensure the target is caught, albeit at a higher cost.",
          },
          {
            type: "code",
            codeLang: "cpp",
            content:`/**
 * @brief Checks if the current node is a goal node.
 *
 * This function iterates through a list of goal nodes and checks if the current
 * node matches any of them based on position (x, y) and time (t). A relaxation
 * is applied when the current time is greater than or equal to 2/3 of the
 * total target steps. In this case, reaching the goal position at any time
 * before or at the goal time is considered a success. Otherwise, the current
 * node's time must exactly match the goal node's time.
 *
 * @param currNode The current node being evaluated.
 * @param x_size The width of the map. (Not directly used in this function, but
 *               might be relevant in the broader context).
 * @param y_size The height of the map. (Not directly used in this function, but
 *               might be relevant in the broader context).
 * @param curr_time The current time step of the search.
 * @param target_steps The total number of time steps for the search.
 * @param goalNodes A vector of goal nodes.
 * @return True if the current node is a goal node, false otherwise.
 */
bool is_goal(Node& currNode, int x_size, int y_size, int curr_time,
             int target_steps, std::vector<Node> goalNodes) {
  for (Node goal : goalNodes) {
    // Check if the current node matches the goal node's position.
    if (goal.x == currNode.x && goal.y == currNode.y) {
      // Apply relaxation if current time is >= 2/3 of target steps.
      if (curr_time >= (2 * target_steps) / 3) {
        // In relaxed mode, any time before or at the goal time is a goal.
        if (currNode.t <= goal.t) {
          return true;
        }
      } else {
        // Otherwise, the time must match exactly.
        if (currNode.t == goal.t) {
          return true;
        }
      }
    }
  }
  return false;
}`,
            subtitle: "Check If Goal Reached",
          }
        
      ],
      navRef: "Algorithm Overview",
    },
    {
      title: "Heuristic",
      content: [
        {
          type: "text",
          content:
            "To create an admissible and accurate heuristic for the 3D Multi-Goal A* planner, the Backward A* algorithm was employed. The heuristic represents the cost-to-goal from any state:",
        },
        {
          type: "text",
          content:
            "Backward A* precomputes the heuristic values for all grid cells, storing them in a 1D array for efficient access. This heuristic ensures both optimality and computational efficiency.",
        },
        {
            type: "code",
            codeLang: "cpp",
            content: `/**
 * @brief Uses Backward A* to compute a heuristic map.
 *
 * This function calculates a heuristic map for a given environment using
 * the Backward A* algorithm. It starts from imaginary goal locations
 * (corresponding to the actual goals) and iteratively expands outwards
 * filling the heuristicArray with estimated distances to the imaginary goals.
 *
 * @param map A pointer to the map data (represented as an array of ints).
 * @param x_size The width of the map.
 * @param y_size The height of the map.
 * @param collision_thresh The threshold for considering a cell a collision.
 * @param heuristicArray A reference to the 1D array where the heuristic values will be stored.
 * @param startNode The starting node for the search (unused in Backward A*).
 * @param goalNodes A vector of goal nodes representing target locations.
 */
void heuristic(int* map, int x_size, int y_size, int collision_thresh,
               std::vector<int>& heuristicArray, Node& startNode,
               std::vector<Node>& goalNodes) {

  // Priority queue for open set (ordered by g-score)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
  openSet.push(startNode); // Add imaginary start node (unused)

  // Visited set as a flattened 1D grid for efficiency
  std::vector<bool> visited(x_size * y_size, false);

  while (!openSet.empty()) {
    Node currNode = openSet.top();
    openSet.pop();

    // If we encounter an imaginary goal location (reached by expanding outwards)
    if (currNode.isImaginary) {
      // Add all actual goal nodes to the open set to fill the heuristic map
      for (const Node& goal : goalNodes) {
        int goalIndex = GETMAPINDEX(goal.x, goal.y, x_size, y_size);
        if (goal.x >= 1 && goal.x <= x_size && goal.y >= 1 && goal.y <= y_size &&
            !visited[goalIndex]) {
          heuristicArray[goalIndex] = goal.g; // Set heuristic for actual goal
          openSet.push(Node(goal.x, goal.y, 0, goal.g, 0)); // Add actual goal to open set
          visited[goalIndex] = true; // Mark actual goal visited
        }
      }
      continue; // Skip to next iteration
    }

    // If not an imaginary node, mark current node as visited
    if (!currNode.isImaginary) {
      visited[GETMAPINDEX(currNode.x, currNode.y, x_size, y_size)] = true;
    }

    // Explore 8 connected directions (excluding diagonals for simplicity)
    int dX[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dY[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    for (int dir = 0; dir < NUMOFDIRS; dir++) {
      int newX = currNode.x + dX[dir];
      int newY = currNode.y + dY[dir];
      int newIndex = GETMAPINDEX(newX, newY, x_size, y_size);

      // Check if within the grid and not visited
      if (newX >= 1 && newX <= x_size && newY >= 1 && newY <= y_size &&
          !visited[newIndex]) {
        int map_cost = (int)map[GETMAPINDEX(newX, newY, x_size, y_size)];
        int new_cost = currNode.g + map_cost;

        // Update heuristic and open set if cost is lower and not a collision
        if ((map_cost < collision_thresh) && (new_cost < heuristicArray[newIndex])) {
          heuristicArray[newIndex] = new_cost;
          openSet.push(Node(newX, newY, 0, new_cost, 0));
        }
      }
    }
  }
}`,
            subtitle: "Backward A* Heuristic",
        }
      ],
      navRef: "Heuristic",
    },
    {
      title: "Data Structures",
      content: [
        {
          type: "text",
          content:
            "A custom node structure was created with attributes including , , and , which represent the coordinates and time of the node. The structure also includes , the cost from the start to the current node, , the heuristic cost from the current node to the goal, and , the total cost. Additionally, a boolean flag, is_imaginary, was introduced to represent goals during heuristic computation with Backward A*.",
        },
        {
          type: "text",
          content:
            "The node structure includes a comparison operator to resolve ties based on -value, -value, and -value, in that order. A hash function based on the state ensures unique identification of nodes.",
        },
        {
          type: "text",
          content:
            "Several other data structures were used to improve efficiency. The heuristic values were stored in a 1D vector accessed via a GETMAPINDEX function to save memory. An unordered map was used as the visited set, providing access to check if a node was visited. Similarly, the g-score values were stored in a 1D vector accessed through the GETMAPINDEX function. A parent map was used to track the parent of each node for path reconstruction, implemented as an unordered map for access. Finally, after a path was computed, it was placed into a global queue, allowing the robot to efficiently retrieve its next move at each iteration of the planner.",
        },
      ],
      navRef: "Problem Statement",
    },
    {
      title: "Implementation Challenges",
      content: [
        {
          type: "text",
          content:
            "For large grids, the planner initially attempts to compute the optimal path. This can result in higher costs when the robot waits at its current position while the path is computed. The relaxation mechanism mitigates this issue but introduces a trade-off between computational feasibility and path quality.",
        },
        {
          type: "text",
          content:
            "Efficient memory management was also critical, particularly for handling large grids. Storing heuristic and g-score values in 1D arrays, rather than multi-dimensional matrices, significantly reduced memory overhead.",
        },
      ],
      navRef: "Implementation Challenges",
    },
    {
      title: "Conclusion",
      content: [
        {
          type: "text",
          content:
            "The planner successfully utilizes a variant of Multi-Goal A* to minimize the robot’s traversal cost while catching a moving target in an 8-connected grid. By incorporating Backward A* for heuristic computation and employing efficient data structures, the implementation achieves a balance between optimality, computational efficiency, and memory usage. Although challenges remain for large grids, the planner demonstrates robust performance within the given constraints.",
        },
      ],
      navRef: "Conclusion",
    },
  ],
  tags: [],
};
