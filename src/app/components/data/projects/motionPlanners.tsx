import { Project, Section } from "@/app/components/project/interfaces";

const prmSection: Section = {
  title: "Probabilistic Roadmap (PRM)",
  navName: "PRM",
  navRef: "probabilistic-roadmap",
  content: [
    {
      type: "text",
      content:
        "How do you navigate a robot arm through a cluttered space? **PRM** builds a 'roadmap' by scattering random configurations throughout the free space and connecting nearby valid positions. Think of it like dropping breadcrumbs in the valid regions, then connecting them into a network of paths.",
    },
    {
      type: "text",
      displayAs: "subtitle",
      content: "Algorithm Overview",
    },
    {
      type: "text",
      displayAs: "list",
      content: [
        "**Sample generation** — randomly place nodes, keep only collision-free ones",
        "**Connection** — link nearby nodes if the path between them is clear",
        "**Degree limiting** — cap connections per node to avoid explosion",
        "**Query phase** — connect start/goal to the roadmap, search for a path",
      ],
    },
    {
      type: "text",
      content:
        "The roadmap is built once, then reused for many queries—great when you need to plan multiple paths in the same environment:",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `while (numNodes < mapSamples) {
  Node curr = getRandomNode(numofDOFs);
  if (IsValidArmConfiguration(curr)) {
    roadMap[curr] = {};
    // Connect to neighbors within radius
  }
}`,
    },
    {
      type: "text",
      content:
        "Once the roadmap exists, finding a path is just graph search. The tricky part is choosing parameters: too few samples leaves gaps in coverage, too many wastes computation. The connection radius balances local detail against global connectivity.",
    },
  ],
};

const rrtSection: Section = {
  title: "Rapidly-exploring Random Tree (RRT)",
  navName: "RRT",
  navRef: "rapidly-exploring-random-tree",
  content: [
    {
      type: "text",
      content:
        "Unlike PRM's roadmap, **RRT** grows a tree from the start toward the goal. It randomly picks a point in space, finds the nearest node in the current tree, and extends toward that point. The tree rapidly spreads to explore the space—hence the name.",
    },
    {
      type: "text",
      displayAs: "subtitle",
      content: "Key Features",
    },
    {
      type: "text",
      displayAs: "list",
      content: [
        "**Goal biasing** — 10% of the time, sample near the goal to speed up",
        "**Step-limited extension** — extend by at most `radius` to control growth",
        "**Early termination** — stop as soon as we can reach the goal",
      ],
    },
    {
      type: "text",
      content:
        "The clever trick is **goal biasing**: occasionally sample near the goal instead of purely randomly. This gives the tree direction without sacrificing exploration:",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `if (prob < 0.1)
  randomNode = getRandomGoalNode(goal, radius);
else
  randomNode = getRandomNode(numofDOFs);`,
    },
    {
      type: "text",
      content:
        "The tree extends one step at a time: find the closest existing node, step toward the sample (but not too far), and add the new node if the path is clear. When any node can directly reach the goal, we're done—no need to keep exploring.",
    },
  ],
};

const rrtConnectSection: Section = {
  title: "RRT-Connect Algorithm",
  navName: "RRT-Connect",
  navRef: "rrt-connect",
  content: [
    {
      type: "text",
      content:
        "Why grow just one tree? **RRT-Connect** grows two trees simultaneously—one from the start, one from the goal—trying to meet in the middle. This bidirectional approach often finds paths much faster, especially through narrow passages.",
    },
    {
      type: "text",
      displayAs: "subtitle",
      content: "Bidirectional Strategy",
    },
    {
      type: "text",
      displayAs: "list",
      content: [
        "**Dual trees** — one grows from start, one from goal",
        "**Alternating extension** — one tree takes a step toward a random point",
        "**Greedy connection** — the other tree rushes toward the new node",
        "**Role swapping** — trees alternate who leads each round",
      ],
    },
    {
      type: "text",
      content:
        "Here's the magic: while the 'leading' tree takes one careful step, the 'lagging' tree greedily extends as far as it can toward the leader's new node. If they connect—success!",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `auto [res, adv] = extendRRT(*leading, random);
if (res != Trapped) {
  // Greedily extend lagging toward adv
  auto conn = connectRRTCONNECT(adv, *lagging);
  if (conn == Reached) return finalPath;
}
swap(leading, lagging);  // Swap roles`,
    },
    {
      type: "text",
      content:
        "By swapping roles each iteration, both trees grow at similar rates and meet somewhere in the middle. This bidirectional approach shines in problems with tight corridors—a single tree might wander aimlessly, but two trees searching from opposite ends find each other much faster.",
    },
  ],
};

const rrtStarSection: Section = {
  title: "RRT* (RRT-Star) Algorithm",
  navName: "RRT*",
  navRef: "rrt-star",
  content: [
    {
      type: "text",
      content:
        "Basic RRT finds *a* path, but not necessarily a *good* one. **RRT*** adds path optimization: as the tree grows, it continuously improves by rewiring connections. Given enough time, it converges to the optimal path—not just any feasible one.",
    },
    {
      type: "text",
      displayAs: "subtitle",
      content: "1. Choosing Best Parent",
    },
    {
      type: "text",
      content:
        "When adding a new node, don't just connect to the nearest neighbor. Look at *all* nearby nodes and pick the one that gives the lowest total cost from start.",
    },
    {
      type: "text",
      displayAs: "subtitle",
      content: "2. Rewiring the Tree",
    },
    {
      type: "text",
      content:
        "Here's the key innovation: for each nearby node, check if routing *through* the new node would be cheaper. If so, rewire—change that node's parent to the new node:",
    },
    {
      type: "code",
      codeLang: "cpp",
      content: `for (const Node& nb : neighbors) {
  double costViaNw = costMap[newNode] + dist(newNode, nb);
  if (costViaNw < costMap[nb]) {
    updateParent(tree, nb, newNode);  // Rewire
    costMap[nb] = costViaNw;
  }
}`,
    },
    {
      type: "text",
      content:
        "As more nodes are added, the tree keeps improving. Paths that looked good early on get rewired when better routes appear. This is what makes RRT* **asymptotically optimal**: given enough samples, it finds the true shortest path.",
    },
  ],
};

export const motionPlanner: Project = {
  title: "Sampling-Based Motion Planners",
  subtitle:
    "Comparative implementation of PRM, RRT, RRT-Connect, and RRT* algorithms",
  date: "Spring 2024",
  media: "/media/videos/motionPlanner.mp4",
  tags: [
    "Robotics",
    "Motion Planning",
    "C++",
    "PRM",
    "RRT",
    "RRT*",
    "RRT-Connect",
  ],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "How does a robot arm find a path through a cluttered workspace? Unlike grid-based planning, robot arms live in **configuration space**—the space of all possible joint angles. A 5-joint arm has a 5-dimensional C-space! **Sampling-based planners** handle this complexity by randomly exploring the space rather than discretizing it. This project implements and compares four popular algorithms.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Algorithms Implemented",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**PRM** — Probabilistic Roadmap for multi-query planning",
            "**RRT** — Rapidly-exploring Random Tree with goal biasing",
            "**RRT-Connect** — Bidirectional RRT for fast convergence",
            "**RRT*** — Asymptotically optimal variant with rewiring",
          ],
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
            "Every planner has knobs to tune. I chose parameters that give each algorithm a fair shot while keeping computation reasonable.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Configuration",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**PRM samples** — 1,000 nodes for roadmap generation",
            "**RRT/RRT*/RRT-Connect samples** — 10,000 nodes for exploration",
            "**Connection radius** — 10 units (ensures average ≥1 neighbor)",
            "**Goal bias** — 10% sampling probability toward goal (RRT, RRT*)",
            "**Termination** — stop immediately upon reaching goal",
          ],
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
            "Good news: all four planners successfully found paths in every test case. But they differ dramatically in speed and path quality.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Path Cost (lower is better)",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**RRT*** — 12.78 ± 4.58 (best, due to optimization)",
            "**PRM** — 15.44 ± 6.11 (balanced performance)",
            "**RRT** — 17.63 ± 8.66 (first feasible path)",
            "**RRT-Connect** — 20.82 ± 14.36 (speed over optimality)",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Tree/Roadmap Size",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**PRM** — 1,002 nodes (fixed sampling)",
            "**RRT*** — 100.3 nodes average (exploration-driven)",
            "**RRT** — 96.2 nodes average (early termination)",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Computation Time",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**RRT-Connect** — 0.14s (fastest, bidirectional)",
            "**RRT** — 0.16s (efficient exploration)",
            "**RRT*** — 1.35s (rewiring overhead)",
            "**PRM** — 4.22s (full roadmap construction)",
          ],
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
            "There's no single 'best' planner—each excels at different things. The right choice depends on what matters for your application:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**PRM** — build once, query many times; great for repeated planning in the same environment",
            "**RRT-Connect** — fastest to find *a* path; use when speed trumps optimality",
            "**RRT*** — best path quality; worth the extra time when efficiency matters",
            "**RRT** — simple and reliable; good baseline for single-query problems",
          ],
        },
        {
          type: "text",
          content:
            "Match the planner to your needs: **speed** (RRT-Connect), **optimality** (RRT*), or **multi-query efficiency** (PRM).",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Future Work",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "Adaptive sampling strategies",
            "Dynamic obstacle avoidance",
            "Extension to higher-dimensional C-spaces",
          ],
        },
      ],
    },
  ],
};
