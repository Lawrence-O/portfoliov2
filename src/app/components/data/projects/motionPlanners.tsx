import { Project, Section } from "@/app/components/project/interfaces";
const prmSection: Section = {
  title: "Probabilistic Roadmap (PRM)",
  navName: "Probabilistic Roadmap",
  navRef: "Probabilistic Roadmap",
  content: [
    {
      type: "text",
      content:
        "The Probabilistic Roadmap (PRM) algorithm is a motion planning technique for finding collision-free paths for robots in complex environments. It constructs a graph representation of the robot's configuration space (C-space), which encompasses all possible joint configurations. The key steps in PRM, based on the implementation, include initialization, roadmap generation, connecting nodes, and pathfinding. During initialization, the starting and goal positions (joint angles) of the robot are defined using startNode and goalNode. A specific number of samples (mapSamples) is chosen to represent potential robot configurations within the C-space, and a connection radius (radius) is set to define the maximum distance for considering two configurations as neighbors.",
    },
    {
      type: "text",
      content:
        "Roadmap generation involves an iterative process (generateRoadMapPRM) that continues until the desired number of samples (mapSamples) is reached. In each iteration, a random configuration (currNode) is generated within the robot's joint limits (getRandomNode). This configuration is then checked for collisions with obstacles in the environment using the map data (IsValidArmConfiguration). If the configuration is collision-free, it is added to the roadmap (roadMap).",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `/**
 * @brief Generates a probabilistic roadmap (PRM) for motion planning.
 *
 * This function generates a PRM by sampling random configurations
 * (joint angles) within the valid workspace and connecting them based
 * on certain criteria.
 *
 * @param map A pointer to the environment map data (as a double array).
 * @param x_size The width of the map.
 * @param y_size The height of the map.
 */
void generateRoadMapPRM(double* map, int x_size, int y_size) {
  // Target number of nodes in the PRM
  int numNodes = 0;

  // Loop until target number of nodes is reached
  while (numNodes < mapSamples) {
    // Sample a random configuration (joint angles)
    Node currNode = getRandomNode(numofDOFs);

    // Check if the configuration is collision-free
    if (IsValidArmConfiguration(currNode.jointAngles.data(), numofDOFs, map,
                               x_size, y_size)) {
      // Initialize an empty vector for neighbors of this node
      roadMap[currNode] = vector<Node>();
      numNodes += 1;

      // Get all neighboring configurations within a radius
      vector<Node> neighbors = getNeighborhoodNodes(currNode, roadMap, radius);

      // Iterate through neighbors
      for (const Node neighbor : neighbors) {
        // Check if current node has less than K connections AND
        // Check if connecting creates a collision-free path
        if (vertexDegreeLessThanK(currNode, roadMap) &&
            connect(neighbor, currNode, map, x_size, y_size)) {
          // Add connections between nodes if both conditions are met
          roadMap[currNode].push_back(neighbor);
          roadMap[neighbor].push_back(currNode);
        }
      }
    }
  }
}`,
      subtitle: "Joint Angle Sampling and Roadmap Generation",
    },
    {
      type: "text",
      content:
        "Once a set of collision-free configurations is generated, the algorithm connects them to form a graph structure. This is done through the addEdgeToRoadMap function. For each node (q) in the roadmap, the function retrieves neighboring nodes (neighbors) within the connection radius (getNeighborhoodNodes). Each neighbor is checked against two conditions: a vertex degree limit (vertexDegreeLessThanK), which ensures a balanced roadmap by comparing the number of existing connections to a predefined limit (K), and a collision-free connection check (connect) to verify a valid path between the neighbor and the current node. If both conditions are met, the neighbor is added as a connected node to both q and the neighbor in the roadmap.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `/**
 * @brief Adds edges to the roadmap for a given node.
 *
 * Finds neighbors of node 'q' and adds edges to the roadmap if certain
 * conditions (degree less than K and collision-free connection) are met.
 *
 * @param q The node to add edges for.
 * @param map A pointer to the environment map data.
 * @param x_size The width of the map.
 * @param y_size The height of the map.
 */
void addEdgeToRoadMap(Node& q, double* map, int x_size, int y_size) {
  roadMap[q] = vector<Node>();
  vector<Node> neighbors = getNeighborhoodNodes(q, roadMap, radius);

  for (const Node& neighbor : neighbors) {
    if (vertexDegreeLessThanK(q, roadMap) &&
        connect(neighbor, q, map, x_size, y_size)) {
      roadMap[q].push_back(neighbor);
      roadMap[neighbor].push_back(q);
    }
  }
}

/**
 * @brief Checks if the degree of a vertex (node) is less than K.
 *
 * This function checks if the number of connections (edges) of a given
 * node in the roadmap is less than a specified threshold K.
 *
 * @param q The node to check the degree of.
 * @param roadmap The roadmap data structure.
 * @param K The degree threshold (default is 10).
 * @return True if the degree of q is less than K, false otherwise.
 */
bool vertexDegreeLessThanK(
    const Node& q,
    const std::unordered_map<Node, std::vector<Node>, NodeHash>& roadmap,
    int K = 10) {
  auto it = roadmap.find(q);
  if (it != roadmap.end()) {
    return it->second.size() < K;
  }
  return false; // Node not found in roadmap
}`,
      subtitle: "Edge Addition and Vertex Degree Check",
    },
    {
      type: "text",
      content:
        "After the roadmap is constructed, a pathfinding algorithm like A* (AStar in the implementation) finds the shortest collision-free path between the start and goal configurations within the roadmap graph. The implementation uses an unordered_map to represent the roadmap, with each node (Node) acting as a key and storing a vector of its connected neighbors. The connect function likely performs a collision check by simulating movement between configurations using the map data. The code implicitly considers obstacles during random sample generation (IsValidArmConfiguration). Different strategies can be used for random sampling, such as prioritizing unexplored areas or biasing towards the goal region, and advanced PRM variants can further optimize roadmap construction.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `/**
 * @brief A container class for the Probabilistic Roadmap (PRM) planner.
 *
 * This class encapsulates the PRM data structures and algorithms,
 * including roadmap generation and path planning.
 */
class PRM_CONTAINER {
 public:
  /**
   * @brief Constructor for the PRM_CONTAINER.
   *
   * Initializes the start and goal nodes, number of DOFs, number of map
   * samples, and the connection radius.
   *
   * @param armstart_anglesV_rad Array of start joint angles (in radians).
   * @param armgoal_anglesV_rad Array of goal joint angles (in radians).
   * @param numofDOFs Number of degrees of freedom of the robot arm.
   * @param mapSamples Number of samples to generate for the roadmap.
   * @param radius Connection radius for neighborhood search.
   */
  PRM_CONTAINER(double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                int numofDOFs, int mapSamples, int radius)
      : startNode(armstart_anglesV_rad, numofDOFs),
        goalNode(armgoal_anglesV_rad, numofDOFs),
        numofDOFs(numofDOFs),
        mapSamples(mapSamples),
        radius(radius) {}

  // The roadmap data structure
  std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap;

  /**
   * @brief Runs the PRM algorithm to find a path.
   *
   * Generates the roadmap, connects the start and goal nodes to the
   * roadmap, and then runs the A* search algorithm to find a path.
   *
   * @param map A pointer to the environment map data.
   * @param x_size The width of the map.
   * @param y_size The height of the map.
   * @return A vector of nodes representing the path, or an empty vector
   *         if no path is found.
   */
  std::vector<Node> runPRM(double* map, int x_size, int y_size) {
    generateRoadMapPRM(map, x_size, y_size);

    // Connect Start Nodes to the roadmap
    addEdgeToRoadMap(startNode, map, x_size, y_size);

    // Connect Goal Nodes to the roadmap
    addEdgeToRoadMap(goalNode, map, x_size, y_size);

    // Run A* search on the roadmap
    std::vector<Node> path = AStar(startNode, goalNode, roadMap, isGoal);
    return path;
  }
};`,
      subtitle: "PRM Container Class and Run Function",
    },
  ],
};
const rrtSection: Section = {
  title: "Rapidly-exploring Random Tree (RRT)",
  navName: "Rapidly-exploring Random Tree",
  navRef: "Rapidly-exploring Random Tree",
  content: [
    {
      type: "text",
      content:
        "The Rapidly-exploring Random Tree (RRT) algorithm is an efficient motion planning technique used to find collision-free paths in complex environments. It iteratively builds a tree structure within the robot's configuration space (C-space). This section explains the RRT concept and implementation.",
    },
    {
      type: "code",
      codeLang: "cpp",
      subtitle: "RRT Container Class and Run Function",
      content: `/**
 * @brief A container class for the Rapidly-exploring Random Tree (RRT) planner.
 *
 * This class encapsulates the RRT algorithm and its data structures.
 */
class RRT_CONTAINER {
 public:
  /**
   * @brief Constructor for the RRT_CONTAINER.
   *
   * Initializes the start and goal nodes, number of DOFs, number of samples,
   * and the connection radius.
   *
   * @param armstart_anglesV_rad Array of start joint angles (in radians).
   * @param armgoal_anglesV_rad Array of goal joint angles (in radians).
   * @param numofDOFs Number of degrees of freedom of the robot arm.
   * @param numSamples Number of samples to generate for the RRT.
   * @param radius Connection radius for neighborhood search.
   */
  RRT_CONTAINER(double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                int numofDOFs, int numSamples, int radius)
      : startNode(armstart_anglesV_rad, numofDOFs),
        goalNode(armgoal_anglesV_rad, numofDOFs),
        numofDOFs(numofDOFs),
        numSamples(numSamples),
        radius(radius) {}

  // The roadmap data structure, representing the RRT.
  std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap;

  /**
   * @brief Runs the RRT algorithm to find a path.
   *
   * Generates the RRT, then uses A* to find a path from start to goal within
   * the generated tree.
   *
   * @param map A pointer to the environment map data.
   * @param x_size The width of the map.
   * @param y_size The height of the map.
   * @return A vector of nodes representing the path, or an empty vector if
   *         no path is found.
   */
  std::vector<Node> runRRT(double* map, int x_size, int y_size) {
    // Generate the RRT roadmap
    roadMap = generateRoadMapRRT(map, x_size, y_size);

    // Run A* pathfinding on the generated roadmap
    std::vector<Node> path = AStar(startNode, goalNode, roadMap, isGoal);
    return path;
  }
};`,
    },
    {
      type: "text",
      content:
        "The RRT algorithm's key concepts involve several steps. First, tree construction begins with a tree rooted at the startNode. Then, randomNode configurations are sampled within the C-space boundaries. For each randomNode, the algorithm identifies the nearestNode within the existing tree. Next, the algorithm attempts tree extension, extending the tree towards the randomNode using the radius parameter and the newConfig function. A collision check is then performed using the connect function and the environment map to ensure a collision-free path. If the extension is valid, the tree is updated by adding the new node. This process repeats until the tree reaches the goalNode or the maximum number of samples is generated.",
    },
    {
      type: "code",
      codeLang: "cpp",
      subtitle: "RRT Roadmap Generation and Tree Extension",
      content: `/**
 * @brief Generates a Rapidly-exploring Random Tree (RRT) roadmap.
 *
 * This function builds an RRT roadmap by iteratively sampling random
 * configurations, extending the tree towards them, and checking for
 * collision-free connections.
 *
 * @param map A pointer to the environment map data.
 * @param x_size The width of the map.
 * @param y_size The height of the map.
 * @return A map representing the RRT roadmap, where keys are nodes and
 *         values are their connected neighbors.
 */
std::unordered_map<Node, std::vector<Node>, NodeHash> generateRoadMapRRT(
    double* map, int x_size, int y_size) {
  // Initialize the roadmap with the start node as the root.
  std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap = {
      {startNode, std::vector<Node>()}};

  // Initialize random number generators.
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_real_distribution<double> distr(0.0, 1.0);

  // Sample numSamples random configurations.
  for (int i = 0; i < numSamples; ++i) {
    double prob = distr(generator);

    // Sample a random node with a 10% chance of being near the goal.
    Node randomNode;
    if (prob < 0.1) {
      randomNode = getRandomGoalNode(goalNode, numofDOFs, radius);
    } else {
      randomNode = getRandomNode(numofDOFs);
    }

    // Re-sample if the random node is not collision-free.
    while (!IsValidArmConfiguration(randomNode.jointAngles.data(), 
                                        numofDOFs, map, x_size, y_size)) 
    {
      randomNode = (prob < 0.1) ? getRandomGoalNode(goalNode, numofDOFs, radius) 
                                : getRandomNode(numofDOFs);
    }

    // Attempt to extend the tree towards the random node.
    auto [extendResult, advancedNode] = extendRRT(roadMap, randomNode, map, x_size, y_size);

    // Update the roadmap if the extension was successful and didn't reach
    // a trapped state.
    if (extendResult != Trapped) {
      // Check if the goal is reachable from the advanced node.
      if (connect(goalNode, advancedNode, map, x_size, y_size) &&
          distance(goalNode, advancedNode) <= radius) {
        roadMap[advancedNode].push_back(goalNode);
        roadMap[goalNode] = std::vector<Node>();
        roadMap[goalNode].push_back(advancedNode);
        return roadMap; // Early return if goal is reached.
      }
    }
  }
  // Return the final roadmap.
  return roadMap;
}`,
    },
    {
      type: "text",
      content:
        "The `runRRT` function orchestrates the overall RRT process. It generates the RRT by calling the `generateRoadMapRRT` function, connects the start and potentially the goal node to the tree using the `extendRRT` function, and then performs an A* search within the generated tree to find a path. The `generateRoadMapRRT` function iteratively samples configurations and attempts to extend the tree towards these sampled configurations using the `extendRRT` function. It also checks if the goal has been reached. This function uses a probability, `prob`, to occasionally bias the sampling towards the goal region, which can potentially improve the algorithm's convergence rate.",
    },
    {
      type: "code",
      codeLang: "cpp",
      subtitle: "RRT Roadmap Generation and Tree Extension",
      content: `/**
* @brief Creates a new configuration by extending from the nearest neighbor
*        towards the random node.
*
* This function attempts to create a new configuration by moving from
* the nearest neighbor towards the random node by a maximum distance of
* the radius. If the distance between the two nodes is less than the
* radius, the new configuration is simply the random node. Otherwise,
* the new configuration is created by interpolating between the nearest
* neighbor and the random node along each joint angle.
*
* @param nearestNeighborNode The nearest neighbor node in the RRT.
* @param randomNode The randomly sampled node.
* @return The new configuration node.
*/
Node newConfig(const Node& nearestNeighborNode, const Node& randomNode) {
double dist = distance(nearestNeighborNode, randomNode);
int numJoints = nearestNeighborNode.jointAngles.size();
Node newNode;
newNode.jointAngles.assign(numJoints, 0.0);

// If the random node is within the radius, just return it.
if (dist < radius) {
    newNode.jointAngles = randomNode.jointAngles;
    return newNode;
}

// Otherwise, interpolate along each joint angle.
for (int i = 0; i < numJoints; ++i) {
    double direction = randomNode.jointAngles[i] -
                        nearestNeighborNode.jointAngles[i];
    // Limit the extension to the radius.
    if (fabs(direction) > radius) {
    direction = copysign(radius, direction);
    }
    newNode.jointAngles[i] = nearestNeighborNode.jointAngles[i] + direction;
}
return newNode;
}

/**
 * @brief Extends the RRT towards a random node.
 *
 * This function finds the nearest node in the RRT to the given random
 * node, creates a new configuration by extending from the nearest node
 * towards the random node, and adds the new configuration to the RRT
 * if the connection is collision-free.
 *
 * @param roadMap The RRT roadmap.
 * @param randomNode The randomly sampled node.
 * @param map A pointer to the environment map data.
 * @param x_size The width of the map.
 * @param y_size The height of the map.
 * @return A pair containing the extension result (Reached, Advanced, or
 *         Trapped) and the advanced node.
 */
std::pair<ExtendResult, Node> extendRRT(
    std::unordered_map<Node, std::vector<Node>, NodeHash>& roadMap,
    Node& randomNode, double* map, int x_size, int y_size) {
Node nearestNode = getNearestNeighbor(roadMap, randomNode);
Node advancedNode = newConfig(nearestNode, randomNode);

// Check for collision-free connection.
if (connect(advancedNode, nearestNode, map, x_size, y_size)) {
    roadMap[advancedNode] = std::vector<Node>();
    roadMap[nearestNode].push_back(advancedNode);
    roadMap[advancedNode].push_back(nearestNode);

    // Check if the goal has been reached.
    if (advancedNode == randomNode) {
    return std::make_pair(ExtendResult(Reached), advancedNode);
    } else {
    return std::make_pair(ExtendResult(Advanced), advancedNode);
    }
}
return std::make_pair(ExtendResult(Trapped), advancedNode);
  }`,
    },
    {
      type: "text",
      content:
        "The `extendRRT` function is responsible for finding the nearest neighbor in the tree for a given random configuration. It then attempts to extend the tree towards this neighbor using the `newConfig` function and performs a collision-free connection check using the `connect` function. If the extension is successful, the function updates the tree structure by adding the new node and its connection. It returns a pair indicating the result of the extension operation: whether the goal was reached, the tree was advanced, or the extension was trapped due to a collision. The `newConfig` function creates a new configuration by moving from the nearest neighbor towards the random node by a maximum distance of the radius. If the distance between the two nodes is less than the radius, the new configuration is simply the random node. Otherwise, the new configuration is created by interpolating between the nearest neighbor and the random node along each joint angle, with a maximum step size of the radius.",
    },
    {
      type: "text",
      content:
        "The implementation uses a probability, `prob`, to bias sampling towards the goal. This means that with a certain probability, the algorithm will sample a configuration closer to the goal, which can help in finding a path faster. The `connect` function is assumed to perform a collision check between two configurations and returns true if the connection between them is collision-free. The roadmap, which represents the RRT, is stored as an unordered map where each node is a key, and the corresponding value is a vector of its connected neighbors.",
    },
  ],
};

const rrtConnectSection: Section = {
  title: "RRT-Connect Algorithm",
  navName: "RRT-Connect",
  navRef: "RRT-Connect",
  content: [
    {
      type: "text",
      content:
        "RRT-Connect enhances the basic RRT (Rapidly-exploring Random Tree) algorithm by growing two trees simultaneously. This bidirectional search strategy allows for faster exploration of the configuration space and can potentially lead to shorter paths compared to the traditional single-tree RRT approach.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `/**
 * @brief Generates a path using the RRT-Connect algorithm.
 *
 * This function grows two trees simultaneously from the start and goal
 * configurations until they connect.
 *
 * @param map Environment map data.
 * @param x_size Map width.
 * @param y_size Map height.
 * @return Path as a vector of Nodes, or empty vector if no path is found.
 */
vector<Node> generateRoadMapRRTConnect(double *map, int x_size, int y_size) {
  // Initialize roadmaps and pointers
  unordered_map<Node, vector<Node>, NodeHash> roadMap_A = {
      {startNode, vector<Node>()}};
  unordered_map<Node, vector<Node>, NodeHash> roadMap_B = {
      {goalNode, vector<Node>()}};
  unordered_map<Node, vector<Node>, NodeHash> *leadingMap = &roadMap_A;
  unordered_map<Node, vector<Node>, NodeHash> *laggingMap = &roadMap_B;

  // Expand tree
  for (int i = 0; i < K; ++i) {
    // Generate a random Node
    Node randomNode = getRandomNode(numofDOFs);
    while (!IsValidArmConfiguration(randomNode.jointAngles.data(), numofDOFs,
                                     map, x_size, y_size)) {
      randomNode = getRandomNode(numofDOFs);
    }

    // Extend the tree from randomNode to the current leadingMap
    auto [extendResult, advancedNode] = extendRRT(
        *leadingMap, randomNode, map, x_size, y_size);

    // Check if the node is not trapped and has advanced
    if (extendResult != Trapped) {
      // Extend the lagging map towards the extended node
      auto [extendResult_connect, extendedNodeLag] = connectRRTCONNECT(
          advancedNode, *laggingMap, map, x_size, y_size);

      // We have connected the trees
      if (extendResult_connect == Reached) {
        vector<Node> pathA;
        vector<Node> pathB;
        if (leadingMap == &roadMap_A) {
          pathA = AStar(startNode, advancedNode, *leadingMap, isGoal);
          pathB = AStar(goalNode, extendedNodeLag, *laggingMap, isGoal);
        } else {
          pathB = AStar(goalNode, advancedNode, *leadingMap, isGoal);
          pathA = AStar(startNode, extendedNodeLag, *laggingMap, isGoal);
        }
        vector<Node> finalPath;
        for (size_t i = 0; i < pathA.size(); ++i) {
          finalPath.push_back(pathA[i]);
        }
        std::reverse(pathB.begin(), pathB.end());
        for (size_t i = 1; i < pathB.size(); ++i) {
          finalPath.push_back(pathB[i]);
        }
        return finalPath;
      }
    }
    unordered_map<Node, vector<Node>, NodeHash> *tmp = leadingMap;
    leadingMap = laggingMap;
    laggingMap = tmp;
  }
  return vector<Node>();
}`,
      subtitle: "RRT-Connect Path Generation",
    },
    {
      type: "text",
      content:
        "RRT-Connect begins by creating two empty roadmaps. One roadmap is initialized with the initial configuration, `startNode`, and the other with the goal configuration, `goalNode`.",
    },
    {
      type: "text",
      content:
        "The algorithm then enters an iterative loop that runs for a predefined number of iterations, `K`. In each iteration, a random configuration, `randomNode`, is sampled within the valid joint angle space. The algorithm maintains two roadmaps: a 'leading' roadmap, which is extended towards the `randomNode` using the `extendRRT` function, and a 'lagging' roadmap. The `extendRRT` function attempts to connect the nearest neighbor in the leading roadmap to the `randomNode`, ensuring collision-free motion. The result of this extension is then checked.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `/**
 * @brief Creates a new configuration by extending from the nearest neighbor
 *        towards the random node.
 *
 * This function attempts to create a new configuration by moving from the
 * nearest neighbor towards the random node by a maximum distance of the radius.
 * If the distance is less than the radius, the random node is returned.
 * Otherwise, a new node is created by interpolating along each joint angle.
 *
 * @param nearestNeighborNode The nearest neighbor node in the RRT.
 * @param randomNode The randomly sampled node.
 * @return The new configuration node.
 */
Node newConfig(const Node &nearestNeighborNode, const Node &randomNode) {
  double dist = distance(nearestNeighborNode, randomNode);
  int numJoints = nearestNeighborNode.jointAngles.size();
  Node newNode;

  if (dist < radius) {
    newNode.jointAngles = randomNode.jointAngles;
    return newNode;
  }

  for (int i = 0; i < numJoints; ++i) {
    double direction = randomNode.jointAngles[i] -
                       nearestNeighborNode.jointAngles[i];
    if (fabs(direction) > radius) {
      direction = copysign(radius, direction);
    }
    newNode.jointAngles[i] = nearestNeighborNode.jointAngles[i] + direction;
  }
  return newNode;
}
/**
 * @brief Extends the RRT tree towards a random node.
 *
 * Finds the nearest node in the roadmap to the random node, creates a new
 * configuration by extending towards the random node, and adds it to the
 * roadmap if the connection is collision-free.
 *
 * @param roadMap The RRT roadmap.
 * @param randomNode The randomly sampled node.
 * @param map Environment map data.
 * @param x_size Map width.
 * @param y_size Map height.
 * @return Pair of ExtendResult and the advanced Node.
 */
std::pair<ExtendResult, Node> extendRRT(
    std::unordered_map<Node, std::vector<Node>, NodeHash> &roadMap,
    Node &randomNode, double *map, int x_size, int y_size) {
  Node nearestNode = getNearestNeighbor(roadMap, randomNode);
  Node advancedNode = newConfig(nearestNode, randomNode);

  if (connect(advancedNode, nearestNode, map, x_size, y_size)) {
    roadMap[advancedNode] = std::vector<Node>();
    roadMap[nearestNode].push_back(advancedNode);
    roadMap[advancedNode].push_back(nearestNode);
    if (advancedNode == randomNode) {
      return std::make_pair(ExtendResult(Reached), advancedNode);
    } else {
      return std::make_pair(ExtendResult(Advanced), advancedNode);
    }
  }
  return std::make_pair(ExtendResult(Trapped), advancedNode);
}

/**
 * @brief Connects a node to the RRT roadmap by repeatedly extending towards it.
 *
 * This function repeatedly calls extendRRT to connect a given node to the
 * roadmap. It continues extending until the extension is trapped (cannot
 * proceed due to collisions).
 *
 * @param node The node to connect to the roadmap.
 * @param roadMap The RRT roadmap.
 * @param map Environment map data.
 * @param x_size Map width.
 * @param y_size Map height.
 * @return Pair of ExtendResult and the advanced Node.
 */
std::pair<ExtendResult, Node> connectRRTCONNECT(
    Node &node, unordered_map<Node, vector<Node>, NodeHash> &roadMap,
    double *map, int x_size, int y_size) {
  auto [extendResult, advancedNode] = extendRRT(roadMap, node, map, x_size, y_size);
  while (extendResult == Advanced) {
    auto [extendResult, advancedNode] = extendRRT(
        roadMap, node, map, x_size, y_size);
  }
  return std::make_pair(extendResult, advancedNode);
}`,
      subtitle: "Extend and Connect Functions",
    },
    {
      type: "text",
      content:
        "If the extension in the leading tree gets trapped (cannot find a collision-free path), the loop continues to the next iteration. If the extension successfully reaches the `randomNode`, that node is added to the leading roadmap. If the extension advances towards the `randomNode` but does not reach it, the advanced node, `advancedNode`, is stored. Then, the `connectRRTCONNECT` function attempts to connect the lagging roadmap towards the `advancedNode`. This connection attempt involves repeatedly calling `extendRRT` on the lagging roadmap until it connects to the `advancedNode`, gets trapped, or encounters an error. If `connectRRTCONNECT` successfully reaches `advancedNode`, a path connecting the two trees has been found.",
    },
    {
      type: "text",
      content:
        "Once the two trees are connected, an A* search algorithm (not shown here) can be employed to find a smooth path through both roadmaps, connecting the `startNode` to the `goalNode`. After each iteration of extending and connecting, the roles of the leading and lagging roadmaps are swapped. This ensures balanced exploration of the configuration space from both the start and goal configurations. If a path is not found within `K` iterations, the algorithm returns an empty vector. Otherwise, the combined path found by the A* search is returned.",
    },
    {
      type: "text",
      content:
        "RRT-Connect offers faster exploration compared to single-tree RRT algorithms due to its bidirectional growth and can potentially find shorter paths. However, it requires more memory to store two roadmaps, and selecting an appropriate value for `K` is important for good performance.",
    },
  ],
};

const rrtStarSection: Section = {
  title: "RRT* Algorithm",
  navName: "RRT*",
  navRef: "RRT*",
  content: [
    {
      type: "text",
      content:
        "The RRT* (RRT Star) algorithm is an advanced path planning technique that builds upon the Rapidly-exploring Random Tree (RRT) method. While RRT excels at quickly exploring a configuration space and finding a feasible path, it doesn't guarantee the most efficient or optimal solution. RRT* addresses this limitation by incorporating path cost considerations and a 'rewiring' process, which allows it to converge towards an optimal solution, such as the shortest or least costly path. This makes RRT* especially valuable in scenarios where path efficiency is critical, such as robotics, autonomous navigation, and game AI.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `class RTTSTAR_CONTAINER {
public:
    /**
     * Constructor for the RTTSTAR_CONTAINER.
     * @param armstart_anglesV_rad Start joint angles in radians.
     * @param armgoal_anglesV_rad Goal joint angles in radians.
     * @param numofDOFs Number of degrees of freedom of the robot.
     * @param numSamples Number of samples to take when constructing the roadmap.
     * @param radius Radius to consider while checking for neighbor nodes.
     */
    RTTSTAR_CONTAINER(double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                      int numofDOFs, int numSamples, int radius)
        : startNode(armstart_anglesV_rad, numofDOFs),
          goalNode(armgoal_anglesV_rad, numofDOFs),
          numofDOFs(numofDOFs),
          numSamples(numSamples),
          radius(radius) {}

    // The roadmap which contains the graph of nodes and edges of the RRT*
    // Algorithm
    std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap;

    /**
     * Runs the RRT* algorithm to find a path from start to goal
     * @param map A pointer to the map of the workspace, which is used for
     * collision checking
     * @param x_size The x dimension of the map
     * @param y_size The y dimension of the map
     * @return A vector of nodes representing the path.
     */
    std::vector<Node> runRRTSTAR(double* map, int x_size, int y_size) {
        roadMap = generateRoadMapRRTSTAR(map, x_size, y_size);
        // Run A*
        std::vector<Node> path = AStar(startNode, goalNode, roadMap, isGoal);
        return path;
    }
};`,
      subtitle: "RRT* Container Class and Execution",
    },
    {
      type: "text",
      content:
        "The core distinction between RRT* and basic RRT lies in two key concepts: Path Cost and Rewiring. Unlike RRT, which focuses primarily on finding *any* feasible path, RRT* explicitly considers the cost of traversing a path. This cost could represent various factors such as Euclidean distance, travel time, fuel consumption, or even a measure of risk. The algorithm aims to minimize the total cost from the start configuration to the goal. The Rewiring mechanism is the heart of RRT*'s optimization. As new nodes are added to the tree, the algorithm searches for nearby nodes within a defined radius. It then checks if connecting to the new node through an alternative parent would result in a lower cost path from the start. If a lower cost path is found, the parent of the existing node is updated (the tree is 'rewired') to reflect this improvement, iteratively leading to increasingly optimal paths.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `/**
 * @brief Generates the RRT* roadmap.
 *
 * This function constructs a roadmap using the RRT* algorithm. It starts with
 * the start node and iteratively adds new nodes by random sampling and
 * extension, until a path to the goal is found.
 *
 * @param map The environment map data.
 * @param x_size Map width.
 * @param y_size Map height.
 * @return The generated roadmap.
 */
std::unordered_map<Node, std::vector<Node>, NodeHash>
RTTSTAR_CONTAINER::generateRoadMapRRTSTAR(double* map, int x_size, int y_size) {
  // Initialize the roadmap with the start node.
  std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap = {
      {startNode, std::vector<Node>()}};
  costMap[startNode] = 0;

  // Setup random number generation.
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_real_distribution<double> distr(0.0, 1.0);

  // Generate nodes for the roadmap.
  for (int i = 0; i < numSamples; ++i) {
    double prob = distr(generator);
    Node randomNode;
    // Bias the random sampling towards the goal with a probability of 10%.
    if (prob < 0.1) {
      randomNode = getRandomGoalNode(goalNode, numofDOFs, radius);
      // Ensure the random node is not in collision
      while (!IsValidArmConfiguration(randomNode.jointAngles.data(), numofDOFs,
                                      map, x_size, y_size)) {
        randomNode = getRandomGoalNode(goalNode, numofDOFs, radius);
      }
    } else {
      randomNode = getRandomNode(numofDOFs);
      // Ensure the random node is not in collision
      while (!IsValidArmConfiguration(randomNode.jointAngles.data(), numofDOFs,
                                      map, x_size, y_size)) {
        randomNode = getRandomNode(numofDOFs);
      }
    }

    // Attempt to extend the roadmap with the new random node.
    auto [extendResult, advancedNode] =
        extendRRTSTAR(roadMap, randomNode, map, x_size, y_size);

    // If the roadmap was extended successfully, attempt to connect to the goal.
    if (extendResult != Trapped) {
      if (connect(goalNode, advancedNode, map, x_size, y_size) &&
          distance(goalNode, advancedNode) <= radius) {
        roadMap[advancedNode].push_back(goalNode);
        roadMap[goalNode] = std::vector<Node>();
        roadMap[goalNode].push_back(randomNode);
        return roadMap;
      }
    }
  }
  return roadMap;
}`,
      subtitle: "RRT* Roadmap Construction",
    },
    {
      type: "text",
      content:
        "The RRT* algorithm iteratively works as follows: Initialization: The algorithm starts with a tree containing only the start configuration (the initial node), which has a path cost of zero. Sampling: A random configuration, or random node, is sampled from the configuration space. Nearest Neighbor Search: The algorithm identifies the node in the existing tree that is closest to the newly sampled random node, based on a defined distance metric. Steering: A new node is generated by 'steering' from the nearest neighbor towards the random node. This involves taking a small step in the direction of the random node, typically limited by a predefined maximum step size. Collision Checking: The path between the nearest neighbor and the new node is checked for collisions with any obstacles in the environment. If a collision is detected, the new node is discarded. Rewiring: The algorithm searches for nearby nodes to the newly added node. For each of these nearby nodes, it calculates the cost of reaching it through the new node. If this cost is lower than the existing cost of reaching the nearby node, the parent of the nearby node is changed to the new node, effectively rewiring the tree. Iteration: Steps 2-6 are repeated until a path connecting the start and goal configurations is found, or a maximum number of iterations is reached.",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `
      /**
 * @brief Extends the RRT* roadmap towards the given random node.
 *
 * This function attempts to add a new node to the roadmap by connecting it to
 * the nearest existing node and then rewires the tree based on the new node's
 * cost.
 *
 * @param roadMap The RRT* roadmap.
 * @param randomNode The random node to extend towards.
 * @param map The environment map data.
 * @param x_size Map width.
 * @param y_size Map height.
 * @return Pair of ExtendResult and the advanced Node.
 */
std::pair<ExtendResult, Node> RTTSTAR_CONTAINER::extendRRTSTAR(
    std::unordered_map<Node, std::vector<Node>, NodeHash>& roadMap,
    Node& randomNode, double* map, int x_size, int y_size) {
  Node nearestNode = getNearestNeighbor(roadMap, randomNode);
  Node newNode = newConfig(nearestNode, randomNode);

  if (connect(newNode, nearestNode, map, x_size, y_size)) {
    // Connect along the least cost path
    std::vector<Node> nearestNeighbors =
        getNeighborhoodNodes(newNode, roadMap, radius);
    Node minNode = nearestNode;
    double minCost = costMap[nearestNode] + distance(newNode, nearestNode);

    for (const Node& neighbor : nearestNeighbors) {
      if (connect(neighbor, newNode, map, x_size, y_size)) {
        double neighborCost = distance(newNode, neighbor) + costMap[neighbor];
        if (neighborCost < minCost) {
          minNode = neighbor;
          minCost = neighborCost;
        }
      }
    }

    // Add edges
    roadMap[newNode] = std::vector<Node>();
    roadMap[newNode].push_back(minNode);
    roadMap[minNode].push_back(newNode);
    costMap[newNode] = minCost;

    // Rewire Tree
    for (const Node& neighbor : nearestNeighbors) {
      if (connect(neighbor, newNode, map, x_size, y_size)) {
        double tentativeCost = distance(newNode, neighbor) + costMap[newNode];
        if (tentativeCost < costMap[neighbor]) {
          costMap[neighbor] = tentativeCost;
          Node oldParent = getParent(roadMap, neighbor);
          roadMap[oldParent].erase(
              std::remove(roadMap[oldParent].begin(), roadMap[oldParent].end(),
                          neighbor),
              roadMap[oldParent].end());
          roadMap[neighbor].erase(
              std::remove(roadMap[neighbor].begin(), roadMap[neighbor].end(),
                          oldParent),
              roadMap[neighbor].end());
          roadMap[newNode].push_back(neighbor);
          roadMap[neighbor].push_back(newNode);
        }
      }
    }

    // Remove Any Nodes that are invalid
    std::vector<Node> nodesToRemove;
    for (const auto& node : roadMap) {
      if (node.first.jointAngles.size() == 0) {
        nodesToRemove.push_back(node.first);
      }
    }
    // Remove nodes from the roadMap
    for (const auto& node : nodesToRemove) {
      roadMap.erase(node);
    }
    if (newNode == randomNode) {
      return std::make_pair(ExtendResult::Reached, newNode);
    } else {
      return std::make_pair(ExtendResult::Advanced, newNode);
    }
  }
  return std::make_pair(ExtendResult::Trapped, newNode);
}`,
      subtitle: "RRT* Tree Extension and Optimization",
    },
    {
      type: "text",
      content:
        "RRT* offers several advantages: Asymptotic Optimality:  As the number of iterations increases, the path found by RRT* is guaranteed to converge towards the true optimal path. High-Dimensional Suitability: RRT* is well-suited for planning in high-dimensional configuration spaces, such as those encountered in robotics with many degrees of freedom. Complex Environment Handling: RRT* can effectively navigate complex environments with obstacles, making it applicable to real-world scenarios. However, there are some disadvantages: Computational Cost: The rewiring step, which involves searching for nearby nodes and recalculating path costs, can be computationally demanding, especially in dense or high-dimensional spaces. This can affect the algorithm's real-time performance. Convergence Speed: Although asymptotically optimal, achieving a truly optimal path can require a large number of iterations, resulting in slow convergence in some cases.",
    },
    {
      type: "text",
      content:
        "It's important to understand how RRT* differs from other pathfinding methods, like A* search. A* is a graph search algorithm, which operates on a discrete representation of the environment. RRT*, on the other hand, is a sampling-based algorithm that works in a continuous configuration space. A* is generally more efficient in low-dimensional, discrete spaces where a complete graph can be constructed efficiently. However, creating such a graph in high-dimensional or continuous spaces becomes prohibitively expensive computationally, making RRT* a more suitable option. The C++ code you've provided implements the RRT* algorithm within the `RTTSTAR_CONTAINER` class. The `generateRoadMapRRTSTAR` function builds the roadmap by iteratively sampling random configurations and extending the tree. The `extendRRTSTAR` function is crucial, as it handles the rewiring process, optimizing connections within the tree. The `newConfig` function manages the 'steering' process, extending from existing nodes to create new configurations, and the `getParent` function helps find the parent of a node in the roadmap. Finally, the `runRRTSTAR` function orchestrates the RRT* algorithm. It calls `generateRoadMapRRTSTAR` to create the roadmap and then uses A* search to find the final path. A cost map is maintained to keep track of the cost to reach each node and guide the rewiring towards the most optimal path.",
    },
  ],
};

export const motionPlanner: Project = {
  title: "Sampling Based Motion Planners",
  subtitle: "Spring, 2024",
  media: "/media/videos/motionPlanner.mp4",
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "This project evaluated the performance of various motion planning algorithms for robotic systems. The analysis compared Probabilistic Roadmap (PRM), Rapidly-exploring Random Tree (RRT), RRT-Connect, RRT*, and A* Graph Search across key metrics. These metrics included computational efficiency, path cost, and success rate to provide insights into their practical applicability.",
        },
      ],
    },
    prmSection,
    rrtSection,
    rrtConnectSection,
    rrtStarSection,
    {
      title: "Hyperparameter Configuration",
      navName: "Hyperparameters",
      navRef: "hyperparameters",
      content: [
        {
          type: "text",
          content:
            "To ensure robust performance across all algorithms, specific hyperparameters were configured. PRM utilized 1,000 samples for roadmap generation. RRT, RRT*, and RRT-Connect employed 10,000 samples to navigate potentially complex environments. A connection radius of 10 was used for all algorithms to ensure each node had at least one neighbor. To improve goal reachability in RRT and RRT*, the goal configuration was sampled 10% of the time. All algorithms terminated upon reaching the goal configuration, prioritizing computational efficiency.",
        },
      ],
    },
    {
      title: "Results and Analysis",
      navName: "Results",
      navRef: "results",
      content: [
        {
          type: "text",
          content:
            "The planners exhibited varying cost efficiency. RRT achieved an average planning cost of 17.633 with a standard deviation of 8.663. RRT-Connect showed higher variability, averaging 20.818 with a standard deviation of 14.362. RRT* demonstrated the lowest average cost at 12.775 with a standard deviation of 4.579, reflecting its optimization capabilities. PRM achieved a balanced performance with an average cost of 15.438 and a standard deviation of 6.106. PRM generated dense and systematic roadmaps, averaging 1,002 nodes without variability. In contrast, RRT and RRT* exhibited more dynamic behavior, with average node counts of 96.230 and 100.307, respectively, reflecting their exploration-driven approach. RRT and RRT-Connect were the fastest planners, averaging 0.157 and 0.139 seconds respectively, while maintaining 100% success rates. RRT* took longer, averaging 1.348 seconds due to its optimization focus. PRM was the most computationally intensive, averaging 4.215 seconds, but achieved a 100% success rate, underscoring its reliability.",
        },
      ],
    },
    {
      title: "Conclusion",
      navName: "Conclusion",
      navRef: "conclusion",
      content: [
        {
          type: "text",
          content:
            "This evaluation demonstrated the trade-offs inherent in different motion planning algorithms. PRM excels in generating reliable roadmaps but at a higher computational cost. RRT-Connect provides rapid pathfinding through bidirectional exploration, making it well-suited for time-critical scenarios. RRT* delivers exceptional path cost optimization, albeit with increased computation time. The choice of planner should be based on the specific application requirements, balancing speed, reliability, and path quality.",
        },
      ],
    },
  ],
};
