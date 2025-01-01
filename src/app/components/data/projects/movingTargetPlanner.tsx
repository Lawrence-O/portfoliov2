import { Project } from "@/app/components/project/interfaces";

export const movingTargetPlanner: Project = {
  title: "Real-Time Target Interception with Multi-Goal A*",
  media: "/media/videos/movingTargetsVideo.mp4",
  subtitle: "Spring 2024",
  section: [
    {
      title: "Project Introduction",
      navName: "Introduction",
      navRef: "Introduction",
      content: [
        {
          type: "text",
          content:
            "This project focused on creating a robust motion planner for a robot tasked with intercepting a moving target within a 2D grid environment. The robot navigates the grid, which allows for 8-connected movement, meaning one cell in the X, Y, or diagonal directions at a time. The primary objective of the planner is to minimize the traversal cost for the robot, as opposed to minimizing the time taken, while avoiding obstacles and high-cost regions in the grid.",
        },
      ],
    },
    {
      title: "Problem Definition and Constraints",
      navName: "Problem Statement",
      navRef: "Problem Statement",
      content: [
        {
          type: "text",
          content:
           "The problem involved several critical constraints and inputs. The grid environment includes a costmap, where each cell is assigned a positive integer that defines the traversal cost. A collision threshold was used to identify cells the robot cannot traverse. Cells with a cost equal to or higher than the collision threshold are treated as obstacles. The planner receives the starting position of the robot and the trajectory of the target which is a sequence of discrete positions such as `[(5,6), (5,7), (4,5)]`. Furthermore, the planner was constrained to operate in real-time, with the target moving one step per second. Finally, the size of the grid was limited to 2000x2000 for graduate students and 200x200 for undergraduate students.",
        },
      ],
    },
    {
      title: "Multi-Goal A* Algorithm",
      navName: "Algorithm Overview",
      navRef: "Algorithm Overview",
      content: [
        {
          type: "text",
          content:
            "The implemented solution is a variant of the Multi-Goal A* search algorithm that treats the target's trajectory as a dynamic series of goals. The planner operates within a 3D state space defined by `<X,Y,T>`, where X and Y represent the robot's coordinates in the grid, and T indicates the time step.",
        },
        {
            type: "text",
            content:
              "To prioritize exploration, the planner uses a priority queue (open-set) to ensure nodes are processed in ascending order of their total cost, which is the sum of the movement cost (`g-score`) and the estimated cost-to-goal (heuristic).",
          },
          {
            type: "text",
            content:
             "When expanding nodes, the planner considers nine potential successor states. These states include the eight possible moves in the 8-connected grid, along with the option for the robot to remain in its current position.",
          },
          {
            type: "code",
            codeLang: "cpp",
            subtitle: "Core Logic of the Multi-Goal A* Algorithm",
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
                "The algorithm terminates when the current node matches one of the target’s positions at the corresponding time step. When the remaining time to reach the target is less than a third of the total time, the planner enters a relaxed mode and will return a sub-optimal path, prioritizing reaching the target over minimizing cost. This helps ensure a path is found even when time is limited, at the cost of optimality.",
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
            subtitle: "C++ Implementation to Check If Goal Reached",
          }
      ],
    },
    {
      title: "Heuristic Implementation with Backward A*",
      navName: "Heuristic",
      navRef: "Heuristic",
      content: [
        {
          type: "text",
          content:
            "To develop an admissible and accurate heuristic for our 3D Multi-Goal A* planner, we used the Backward A* algorithm. The heuristic values were precomputed, and represent the cost-to-goal from any given state. The implementation of Backward A* involves exploring the space backwards from a series of imaginary goals.",
        },
        {
          type: "text",
          content:
            "Backward A* computes and stores heuristic values for all traversable cells in a 1D array for efficient access, ensuring both optimality and computational efficiency during the path planning.",
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
            subtitle: "C++ Implementation of the Backward A* Heuristic",
        }
      ],
    },
    {
      title: "Data Structures and Memory Management",
      navName: "Problem Statement",
      navRef: "Problem Statement",
      content: [
        {
          type: "text",
          content:
            "The project utilized a custom `Node` structure that included `x`, `y`, and `t` to represent the coordinates and time of the node, along with `g` (cost from start to the node), a heuristic estimate of cost-to-goal, and `f` (total cost). Also included was a flag, `is_imaginary` to represent goals during the heuristic process with Backward A*.",
        },
        {
          type: "text",
          content:
            "The `Node` structure also included a comparison operator to prioritize nodes based on f-value, then g-value, and finally the heuristic values.  A hash function was also developed based on the state to allow for efficient lookups in the unordered maps.",
        },
        {
          type: "text",
          content:
            "To improve efficiency, the implementation used several specialized data structures. A 1D vector accessed by the `GETMAPINDEX` function was used to store heuristic values, which helped conserve memory. A similar strategy was used for storing g-score values. An `unordered_map` was used as the visited set to quickly check if a node was already explored. The parent map was also an unordered map that stored the parent of each node for path reconstruction. Finally, the path was placed in a global queue to allow the robot to efficiently retrieve its next move.",
        },
      ],
        
    },
    {
      title: "Implementation Challenges and Tradeoffs",
      navName: "Implementation Challenges",
      navRef: "Implementation Challenges",
      content: [
        {
          type: "text",
          content:
            "For large grid environments, the planner initially aims to compute the optimal path, and in some cases that can result in the robot waiting while the path is computed. This issue is mitigated by the relaxation mechanism, where the planner will prioritize getting to the goal over the shortest path in time critical scenarios, which introduces a tradeoff between computational feasibility and path quality.",
        },
        {
          type: "text",
          content:
            "Memory management was another important aspect of the design. In particular, using 1D arrays for heuristic and g-score values significantly reduced memory use, especially for large grids, improving the algorithms practical usage in real world scenarios.",
        },
      ],
    },
    {
      title: "Conclusion",
      navName: "Conclusion",
      navRef: "Conclusion",
      content: [
        {
          type: "text",
          content:
            "The project successfully implemented a robust motion planner using a variant of Multi-Goal A* to minimize the robot’s traversal cost while capturing a moving target in a 2D grid. The planner incorporated Backward A* for heuristic computation along with efficient data structures. The implementation achieves an effective balance between optimality, computational efficiency, and memory usage, offering a practical solution for real-time target interception. Although challenges remain for scaling this to larger grids, the planner delivers reliable performance within the imposed constraints.",
        },
      ],
    },
  ],
  tags: ["Robotics", "Motion Planning", "A*", "Pathfinding", "Algorithms", "C++"],
};