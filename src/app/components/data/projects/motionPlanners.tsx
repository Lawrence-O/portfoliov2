import { Project, Section } from "@/app/components/project/interfaces";

const prmSection: Section = {
  title: "Probabilistic Roadmap (PRM)",
  navName: "PRM",
  navRef: "probabilistic-roadmap",
  content: [
    {
      type: "text",
      content: [
        "The Probabilistic Roadmap (PRM) algorithm constructs a graph, known as a roadmap, within the robot's free configuration space (C-space). This is achieved by randomly sampling configurations and connecting nearby, collision-free samples. Our implementation, encapsulated in the 'PRM_CONTAINER' class, initializes with start and goal configurations, the number of Degrees of Freedom (DOFs), a specified number of samples ('mapSamples'), and a connection 'radius'.",
        "The 'generateRoadMapPRM' function iteratively samples random nodes, validates them for collision-free placement using 'IsValidArmConfiguration', and adds valid nodes to the 'roadMap'. It then attempts to connect each new node to existing neighbors within the defined 'radius', provided the connection itself is collision-free (checked by the 'connect' function) and the node's degree (number of connections) is less than a threshold 'K' ('vertexDegreeLessThanK'). This threshold 'K' helps maintain a balanced and computationally manageable roadmap. Finally, the 'runPRM' method orchestrates the roadmap generation, connects the start and goal nodes to this roadmap using 'addEdgeToRoadMap', and employs an A* search algorithm ('AStar' function) to find the optimal path through the constructed graph."
      ]
    },
    {
      type: "text",
      content: "Key aspects of our PRM implementation include the 'generateRoadMapPRM' method for iterative sample generation and connection, and the 'addEdgeToRoadMap' method for establishing valid connections based on proximity and collision checks. The 'PRM_CONTAINER' class manages the overall process, as illustrated below."
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `class PRM_CONTAINER {
 public:
  PRM_CONTAINER(double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                int numofDOFs, int mapSamples, int radius)
      : startNode(armstart_anglesV_rad, numofDOFs), /* ... other initializations ... */ {}

  std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap;

  // Generates roadmap by sampling and connecting nodes
  void generateRoadMapPRM(double* map, int x_size, int y_size) {
    while (/* numNodes < mapSamples */) {
      Node currNode = getRandomNode(numofDOFs);
      if (IsValidArmConfiguration(/* ... */)) {
        roadMap[currNode] = vector<Node>();
        // ... connect to neighbors within radius if collision-free and degree < K ...
      }
    }
  }
  
  // Connects a given node to existing roadmap nodes
  void addEdgeToRoadMap(Node& q, double* map, int x_size, int y_size) {
    // ... find neighbors within radius, connect if collision-free and degree < K ...
  }

  std::vector<Node> runPRM(double* map, int x_size, int y_size) {
    generateRoadMapPRM(map, x_size, y_size);
    addEdgeToRoadMap(startNode, map, x_size, y_size);
    addEdgeToRoadMap(goalNode, map, x_size, y_size);
    return AStar(startNode, goalNode, roadMap, isGoal);
  }
};`,
      subtitle: "Simplified PRM Container and Core Logic",
    },
    {
      type: "text",
      content: "The 'IsValidArmConfiguration' function is crucial for ensuring that all sampled configurations reside in free space, while the 'connect' function verifies that the path segment between any two nodes is also collision-free. The A* algorithm then efficiently searches the constructed graph. Our choice of 'mapSamples' and 'radius' (detailed in the Hyperparameters section) aimed to strike a balance between roadmap density, coverage, and computational cost."
    }
  ],
};

const rrtSection: Section = {
  title: "Rapidly-exploring Random Tree (RRT)",
  navName: "RRT",
  navRef: "rapidly-exploring-random-tree",
  content: [
    {
      type: "text",
      content: [
        "The Rapidly-exploring Random Tree (RRT) algorithm incrementally builds a tree structure from a starting configuration towards randomly sampled points within the C-space. Our 'RRT_CONTAINER' class manages this process. The core 'generateRoadMapRRT' function iteratively samples 'numSamples' configurations.",
        "A key feature of our RRT implementation is goal biasing: there is a 10% probability ('prob < 0.1') of sampling near the goal configuration ('getRandomGoalNode') to direct exploration more efficiently; otherwise, a general random sample ('getRandomNode') is taken. Each sampled configuration is validated using 'IsValidArmConfiguration'. The tree is then extended towards the valid random sample using the 'extendRRT' function. If this extension successfully connects to the goal node (checked via 'connect' and ensuring the 'distance' is within the 'radius'), the roadmap (tree) is returned early. If not, the process continues until all samples are processed. Finally, 'runRRT' calls 'generateRoadMapRRT' and then uses A* to find a path in the resulting tree."
      ]
    },
    {
      type: "text",
      content: "The 'extendRRT' function identifies the 'getNearestNeighbor' in the current tree to the 'randomNode'. It then attempts to create a 'newConfig' by steering from this nearest neighbor towards the 'randomNode', limited by the step size 'radius'. If this new segment is collision-free (verified by 'connect'), the 'newNode' is added to the tree. The 'newConfig' function handles this step-limited extension: if the 'randomNode' is already within the 'radius' of the 'nearestNeighbor', it's used directly; otherwise, interpolation occurs up to the 'radius' limit."
    },
    {
      type: "code",
      codeLang: "cpp",
      subtitle: "RRT Core Logic: Roadmap Generation with Goal Biasing",
      content: `// Within RRT_CONTAINER class
std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap;

std::unordered_map<Node, std::vector<Node>, NodeHash> generateRoadMapRRT(
    double* map, int x_size, int y_size) {
  roadMap = {{startNode, std::vector<Node>()}}; // Initialize with start
  // ... random number setup ...

  for (int i = 0; i < numSamples; ++i) {
    Node randomNode;
    // Goal biasing (10% chance)
    if (/* prob < 0.1 */) { 
      randomNode = getRandomGoalNode(goalNode, numofDOFs, radius);
    } else {
      randomNode = getRandomNode(numofDOFs);
    }
    // Ensure randomNode is valid (collision-free)
    while (!IsValidArmConfiguration(/* ... */)) { /* re-sample */ }

    auto [extendResult, advancedNode] = extendRRT(roadMap, randomNode, map, x_size, y_size);

    if (extendResult != Trapped) {
      // Check for direct connection to goal
      if (connect(goalNode, advancedNode, map, x_size, y_size) &&
          distance(goalNode, advancedNode) <= radius) {
        // ... connect to goal and return ...
        return roadMap; 
      }
    }
  }
  return roadMap;
}

// extendRRT: Finds nearest, calls newConfig, checks collision, adds to tree.
// newConfig: Extends towards randomNode up to 'radius'.
// runRRT: Calls generateRoadMapRRT then AStar.
`,
    },
    {
        type: "text",
        content: "This structure allows RRT to efficiently explore the C-space, with the goal biasing strategy often accelerating convergence towards the goal in many practical scenarios. The 'radius' parameter critically controls the maximum extension length per step, influencing the tree's growth characteristics."
    }
  ],
};

const rrtConnectSection: Section = {
  title: "RRT-Connect Algorithm",
  navName: "RRT-Connect",
  navRef: "rrt-connect",
  content: [
    {
      type: "text",
      content: [
        "RRT-Connect accelerates the pathfinding process by simultaneously growing two RRTs: one from the start configuration ('roadMap_A') and another from the goal configuration ('roadMap_B'). The core of our 'generateRoadMapRRTConnect' function iterates 'K' times (a predefined maximum). In each iteration, a 'randomNode' is sampled from the C-space.",
        "The 'extendRRT' function (similar in principle to the one in the basic RRT) attempts to grow one tree (designated as the 'leadingMap') towards this 'randomNode'. If this extension is successful (i.e., not 'Trapped'), the 'connectRRTCONNECT' function then tries to extend the other tree (the 'laggingMap') towards the newly added node ('advancedNode') from the first tree. The 'connectRRTCONNECT' function repeatedly calls 'extendRRT' in a greedy manner until the 'advancedNode' is 'Reached' by the 'laggingMap' or the extension becomes 'Trapped'. If the trees successfully connect ('extendResult_connect == Reached'), paths from the start to the connection point in 'roadMap_A' and from the goal to the connection point in 'roadMap_B' are found using A*. These paths are then combined to form the final solution. The roles of 'leadingMap' and 'laggingMap' are swapped in each iteration to ensure balanced growth from both ends."
      ]
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `// Simplified RRT-Connect Core Logic
vector<Node> generateRoadMapRRTConnect(double *map, int x_size, int y_size) {
  unordered_map<Node, vector<Node>, NodeHash> roadMap_A = {{startNode, {}}};
  unordered_map<Node, vector<Node>, NodeHash> roadMap_B = {{goalNode, {}}};
  auto *leadingMap = &roadMap_A, *laggingMap = &roadMap_B;

  for (int i = 0; i < K; ++i) { // K is max iterations
    Node randomNode = getRandomNode(numofDOFs);
    // ... ensure randomNode is valid ...

    auto [extendResult, advancedNode] = extendRRT(*leadingMap, randomNode, /*...*/);

    if (extendResult != Trapped) {
      auto [connectResult, extendedNodeLag] = connectRRTCONNECT(advancedNode, *laggingMap, /*...*/);
      if (connectResult == Reached) {
        // Paths found in leadingMap (e.g., pathA) and laggingMap (e.g., pathB) via A*
        // vector<Node> pathA = AStar(startNode, advancedNode, *leadingMap, isGoal);
        // vector<Node> pathB = AStar(goalNode, extendedNodeLag, *laggingMap, isGoal);
        // ... combine pathA and reversed pathB ...
        return finalPath;
      }
    }
    std::swap(leadingMap, laggingMap); // Swap tree roles
  }
  return {}; // No path found
}

// connectRRTCONNECT: Repeatedly calls extendRRT for the lagging tree 
// towards a node from the leading tree.
std::pair<ExtendResult, Node> connectRRTCONNECT(
    Node &targetNode, unordered_map<Node, vector<Node>, NodeHash> &treeToExtend,
    /*...*/) {
  auto [res, advancedN] = extendRRT(treeToExtend, targetNode, /*...*/);
  while (res == Advanced) { // Keep extending if progress is made
    std::tie(res, advancedN) = extendRRT(treeToExtend, targetNode, /*...*/);
  }
  return {res, advancedN};
}`,
      subtitle: "RRT-Connect Bidirectional Growth and Connection Logic",
    },
    {
      type: "text",
      content: "The 'extendRRT' and 'newConfig' functions are fundamentally similar to those used in the basic RRT implementation, focusing on step-limited, collision-free extension of the tree. The key distinction and power of RRT-Connect lie in its strategy of growing two trees concurrently and attempting to meet in the middle of the C-space. This bidirectional approach often significantly reduces planning time, especially for complex problems with narrow passages. The parameter 'K' dictates the maximum number of expansion attempts before termination if no connection is found."
    }
  ],
};

const rrtStarSection: Section = {
  title: "RRT* (RRT-Star) Algorithm",
  navName: "RRT*",
  navRef: "rrt-star",
  content: [
    {
      type: "text",
      content: "RRT* (RRT-Star) enhances the basic RRT algorithm by incorporating path cost optimization and a tree rewiring process. This allows it to converge towards an asymptotically optimal path, rather than just the first feasible path found. Our 'RTTSTAR_CONTAINER' class implements this advanced planner. The 'generateRoadMapRRTSTAR' function, much like RRT, samples configurations (employing the same 10% goal-biasing strategy) and extends the tree. However, the 'extendRRTSTAR' function contains the core logic specific to RRT*."
    },
    {
      type: "text",
      content: "After a 'newNode' is potentially added via 'newConfig' (steering from 'getNearestNeighbor' towards 'randomNode') and a collision-free connection is verified, RRT* performs two critical operations:"
    },
    {
        type: "text",
        displayAs: "subtitle",
        content: "1. Choosing Best Parent"
    },
    {
        type: "text",
        content: "Instead of merely connecting 'newNode' to the 'nearestNode' found, 'extendRRTSTAR' considers all nodes in the 'getNeighborhoodNodes' of 'newNode' (i.e., existing tree nodes within the 'radius'). It then selects the neighbor ('minNode') from this set that results in the 'minCost' to reach 'newNode' from the start configuration. Costs to reach each node are tracked in a 'costMap'. The 'newNode' is then connected to this 'minNode'."
    },
    {
        type: "text",
        displayAs: "subtitle",
        content: "2. Rewiring the Tree"
    },
    {
        type: "text",
        content: "For all other neighbors of 'newNode' within the 'radius' (those not chosen as the 'minNode'), the algorithm checks if reaching them through the newly added 'newNode' would offer a lower path cost than their current path from the start. If a shorter path is found, their parent connection is updated (rewired) to 'newNode', and their costs (and potentially the costs of their descendants) are updated accordingly. This rewiring step is crucial for RRT*'s ability to progressively improve path quality over iterations."
    },
    {
        type: "text",
        content: "The 'runRRTSTAR' function then uses A* on this optimized tree to extract the final path."
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `// Within RTTSTAR_CONTAINER class
std::unordered_map<Node, std::vector<Node>, NodeHash> roadMap;
std::unordered_map<Node, double, NodeHash> costMap; // Tracks cost to reach each node

std::pair<ExtendResult, Node> extendRRTSTAR(
    std::unordered_map<Node, std::vector<Node>, NodeHash>& currentRoadMap,
    Node& randomNode, double* map, int x_size, int y_size) {
  
  Node nearestInTree = getNearestNeighbor(currentRoadMap, randomNode);
  Node newNode = newConfig(nearestInTree, randomNode); // Steer towards randomNode

  if (connect(newNode, nearestInTree, map, x_size, y_size)) { // Initial connection is valid
    // 1. Choose Best Parent for newNode
    std::vector<Node> neighbors = getNeighborhoodNodes(newNode, currentRoadMap, radius);
    Node bestParent = nearestInTree;
    double costViaBestParent = costMap[nearestInTree] + distance(newNode, nearestInTree);

    for (const Node& potentialParent : neighbors) {
      if (connect(potentialParent, newNode, map, x_size, y_size)) {
        double costViaPotentialParent = costMap[potentialParent] + distance(newNode, potentialParent);
        if (costViaPotentialParent < costViaBestParent) {
          bestParent = potentialParent;
          costViaPotentialParent = costViaPotentialParent;
        }
      }
    }
    // Add newNode to tree with bestParent
    currentRoadMap[newNode] = {bestParent};
    currentRoadMap[bestParent].push_back(newNode);
    costMap[newNode] = costViaBestParent;

    // 2. Rewire the Tree
    for (const Node& neighborToRewire : neighbors) {
      if (neighborToRewire == bestParent) continue; // Already handled

      if (connect(neighborToRewire, newNode, map, x_size, y_size)) {
        double costToReachNeighborViaNewNode = costMap[newNode] + distance(newNode, neighborToRewire);
        if (costToReachNeighborViaNewNode < costMap[neighborToRewire]) {
          // Rewire: Update parent of neighborToRewire to newNode
          Node oldParent = getParent(currentRoadMap, neighborToRewire); // Assumes getParent exists
          // Remove old edge, add new edge (newNode, neighborToRewire)
          // Update costMap[neighborToRewire] and potentially its descendants
        }
      }
    }
    // ... (handle return status: Reached, Advanced) ...
  }
  // ... (handle return status: Trapped) ...
  return {ExtendResult::Trapped, newNode}; // Placeholder
}

// generateRoadMapRRTSTAR: Iteratively calls extendRRTSTAR, includes goal biasing.
// runRRTSTAR: Calls generateRoadMapRRTSTAR then AStar.
`,
      subtitle: "RRT* Core: Extension with Cost Optimization and Rewiring",
    },
    {
      type: "text",
      content: "This dual process of choosing the locally optimal parent and then rewiring nearby nodes within the tree allows RRT* to converge towards a globally optimal path as more samples are added. The 'costMap' is essential for these informed decisions, distinguishing RRT* from the basic RRT which only guarantees a feasible path."
    }
  ],
};

export const motionPlanner: Project = {
  title: "Sampling Based Motion Planners",
  date: "Spring, 2024",
  media: "/media/videos/motionPlanner.mp4",
  githubLink: "https://github.com/your-username/motion-planners",
  tags: ["Robotics", "Motion Planning", "C++", "Algorithms", "PRM", "RRT", "RRT*", "RRT-Connect", "A* Search"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content: [
            "This project involved the implementation and comparative evaluation of several prominent sampling-based motion planning algorithms for robotic systems. The primary goal was to understand their performance characteristics and trade-offs in practical scenarios.",
            "The analysis focused on comparing Probabilistic Roadmap (PRM), Rapidly-exploring Random Tree (RRT), RRT-Connect, and RRT* (RRT-Star), alongside the A* Graph Search algorithm (used as a pathfinding component within these planners). Key performance metrics included computational efficiency (planning time), path cost (solution quality), and success rate in finding a valid path. This evaluation aimed to provide insights into the practical applicability of each algorithm for different robotic tasks and environments."
          ]
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
          content: [
            "To ensure a fair comparison and robust performance across all implemented algorithms, specific hyperparameters were carefully configured. For the Probabilistic Roadmap (PRM) algorithm, 1,000 samples were utilized for roadmap generation. In contrast, RRT, RRT*, and RRT-Connect employed a higher count of 10,000 samples to adequately explore and navigate potentially complex environments.",
            "A uniform connection 'radius' of 10 units was used for all algorithms; this value was chosen to ensure that, on average, each node would have at least one neighbor, facilitating graph connectivity. To improve goal reachability and convergence speed in RRT and RRT*, the goal configuration was explicitly sampled 10% of the time (goal biasing). All algorithms were designed to terminate upon successfully reaching the goal configuration, prioritizing computational efficiency once a solution was found."
          ]
        },
      ],
    },
    {
      title: "Comparative Results and Analysis",
      navName: "Results",
      navRef: "results",
      content: [
        {
          type: "text",
          content: "The implemented motion planners exhibited distinct characteristics in terms of cost efficiency, roadmap/tree structure, and computational speed, while all achieved a 100% success rate in the tested scenarios."
        },
        {
            type: "text",
            displayAs: "subtitle",
            content: "Path Cost Efficiency"
        },
        {
            type: "text",
            displayAs: "list",
            orderedList: false, // Changed to false for bullet points
            content: [
                "RRT* demonstrated the lowest average path cost at 12.775 (std. dev. 4.579), reflecting its inherent optimization capabilities.",
                "PRM achieved a balanced performance with an average cost of 15.438 (std. dev. 6.106).",
                "RRT resulted in an average planning cost of 17.633 (std. dev. 8.663).",
                "RRT-Connect showed the highest average cost and variability, averaging 20.818 (std. dev. 14.362), as its primary focus is speed rather than path optimality."
            ]
        },
        {
            type: "text",
            displayAs: "subtitle",
            content: "Roadmap/Tree Complexity"
        },
        {
            type: "text",
            content: [ // Made this a paragraph array for consistency
                "PRM generated dense and systematic roadmaps, consistently averaging 1,002 nodes without variability, due to its fixed sampling approach.",
                "In contrast, RRT and RRT* exhibited more dynamic behavior in tree growth, with average node counts of 96.230 and 100.307, respectively. This reflects their exploration-driven approach where the tree size depends on when the goal is found."
            ],
        },
        {
            type: "text",
            displayAs: "subtitle",
            content: "Computational Speed"
        },
        {
            type: "text",
            displayAs: "list",
            orderedList: false, // Changed to false for bullet points
            content: [
                "RRT-Connect was the fastest planner, averaging 0.139 seconds for path computation.",
                "RRT was also very efficient, averaging 0.157 seconds.",
                "RRT* took significantly longer, averaging 1.348 seconds, due to its cost optimization and tree rewiring processes.",
                "PRM was the most computationally intensive, averaging 4.215 seconds for roadmap construction and path extraction."
            ]
        }
      ],
    },
    {
      title: "Conclusion and Future Directions",
      navName: "Conclusion",
      navRef: "conclusion",
      content: [
        {
          type: "text",
          content: [
            "This comparative evaluation highlighted the inherent trade-offs among different sampling-based motion planning algorithms. PRM excels in generating reliable and comprehensive roadmaps but at a higher computational cost. RRT-Connect provides rapid pathfinding through its efficient bidirectional exploration, making it well-suited for scenarios where planning speed is critical.",
            "RRT* consistently delivers paths with lower costs due to its optimization focus, albeit with increased computation time. The basic RRT offers a good balance between speed and feasibility. The choice of an appropriate planner should therefore be guided by the specific application requirements, carefully balancing the need for speed, reliability, path quality, and available computational resources.",
            "Future work could involve exploring adaptive sampling strategies, integrating dynamic obstacle avoidance, or extending these planners to higher-dimensional C-spaces and more complex robotic systems."
          ]
        },
      ],
    },
  ],
};
