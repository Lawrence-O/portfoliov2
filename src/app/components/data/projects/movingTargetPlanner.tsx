import { Project } from "@/app/components/project/interfaces";

export const movingTargetPlanner: Project = {
  title: "Real-Time Target Interception with Multi-Goal A*",
  subtitle: "Motion planning for intercepting moving targets in grid environments",
  media: "/media/videos/movingTargetsVideo.mp4",
  date: "Spring 2024",
  tags: ["Robotics", "Motion Planning", "A*", "Pathfinding", "C++"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content:
            "Imagine a robot trying to catch a moving target—like a drone intercepting a package in mid-air. The target follows a known path, but the robot must navigate around obstacles to reach it in time. This project builds a **real-time motion planner** that figures out where to intercept the target while minimizing travel cost (think: energy or distance). The twist? The robot can move in 8 directions (including diagonals), and must plan fast enough to keep up with the target's movement.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Problem Constraints",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Costmap** — each cell has a positive integer traversal cost",
            "**Collision threshold** — cells at or above threshold are obstacles",
            "**Target trajectory** — discrete positions like `[(5,6), (5,7), (4,5)]`",
            "**Real-time** — target moves one step per second",
            "**Grid size** — up to 2000×2000 cells",
          ],
        },
      ],
    },
    {
      title: "Multi-Goal A* Algorithm",
      navName: "Algorithm",
      navRef: "algorithm-overview",
      content: [
        {
          type: "text",
          content:
            "The key insight: we don't just search in 2D space—we search in **space and time** together. The target is at position (5,6) at time 0, then (5,7) at time 1, and so on. Our planner finds a path where the robot arrives at the same place as the target at the same time. This creates a 3D search space: `⟨X, Y, T⟩` where T is the time step.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Search Strategy",
        },
        {
          type: "text",
          content:
            "The algorithm explores states in order of estimated total cost. At each state, the robot can move in any of 8 directions, or stay put (useful when waiting is cheaper than a detour):",
        },
        {
          type: "code",
          codeLang: "cpp",
          content: `int dX[9] = {-1, -1, -1, 0, 0, 1, 1, 1, 0};
int dY[9] = {-1,  0,  1,-1, 1,-1, 0, 1, 0};
for (int dir = 0; dir < 9; dir++) {
  int newX = curr.x + dX[dir];
  int newY = curr.y + dY[dir];
  int newT = curr.t + 1;
  // ... validate and expand
}`,
        },
        {
          type: "text",
          content:
            "Success! The algorithm finds the robot at the same position as the target at the right time. We then trace back through parent pointers to recover the full path.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Relaxed Mode",
        },
        {
          type: "text",
          content:
            "Sometimes finding the *perfect* intercept takes too long. When we're running low on planning time (less than ⅓ remaining), the planner enters **relaxed mode**—it accepts catching the target early rather than exactly on schedule. Better to catch it a bit early than miss entirely:",
        },
        {
          type: "code",
          codeLang: "cpp",
          content: `if (curr_time >= (2 * target_steps) / 3) {
  if (currNode.t <= goal.t) return true;
} else {
  if (currNode.t == goal.t) return true;
}`,
        },
        {
          type: "text",
          content:
            "This is a practical trade-off: we sacrifice guaranteed optimality for reliability. The planner aims for the best solution but gracefully degrades when time is critical.",
        },
      ],
    },
    {
      title: "Backward A* Heuristic",
      navName: "Heuristic",
      navRef: "heuristic",
      content: [
        {
          type: "text",
          content:
            "A-Star is only as fast as its heuristic is good. The heuristic estimates 'how far is this state from the goal?'—if the estimate is too low, we waste time exploring bad paths; if it's too high, we might miss the optimal path. We use **Backward A-Star**: starting from the goal positions, we compute the actual minimum cost to reach them from every cell.",
        },
        {
          type: "text",
          content:
            "Think of it as flooding outward from the goals, recording the cost to reach each cell. This precomputation happens once, then every A-Star query gets perfect heuristic estimates in O(1) time:",
        },
        {
          type: "code",
          codeLang: "cpp",
          content: `if (new_cost < heuristicArray[newIndex]) {
  heuristicArray[newIndex] = new_cost;
  openSet.push(Node(newX, newY, 0, new_cost, 0));
}`,
        },
        {
          type: "text",
          content:
            "Storing heuristics in a **1D array** provides O(1) lookup while keeping memory manageable for large grids.",
        },
      ],
    },
    {
      title: "Data Structures",
      navName: "Data Structures",
      navRef: "data-structures",
      content: [
        {
          type: "text",
          content:
            "With grids up to 2000×2000 cells (4 million cells!), naive data structures would be too slow or consume too much memory. Every lookup and storage operation needed careful optimization.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Node Structure",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "`x, y, t` — position and time step",
            "`g` — cost from start",
            "`h` — heuristic estimate to goal",
            "`f = g + h` — total estimated cost",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Memory Optimizations",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**1D arrays** for heuristic and g-scores — reduced memory overhead",
            "**Flattened indexing** via `GETMAPINDEX` — O(1) access",
            "**Global path queue** — efficient next-move retrieval",
          ],
        },
      ],
    },
    {
      title: "Challenges and Trade-offs",
      navName: "Challenges",
      navRef: "challenges",
      content: [
        {
          type: "text",
          content:
            "The fundamental tension: finding the *best* path takes time, but the target won't wait. Several techniques helped balance this:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Relaxation mechanism** — accept 'good enough' when time runs short",
            "**Precomputed heuristics** — do expensive work once, reuse for every query",
            "**1D storage** — flatten 2D arrays to squeeze out every bit of performance",
          ],
        },
        {
          type: "text",
          content:
            "The result: a planner that delivers optimal paths when possible, but always delivers *some* valid path even under pressure.",
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
            "This project tackled a challenging real-time planning problem: intercepting a moving target while navigating obstacles. The solution combines several techniques:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**3D state space** — reason about position *and* time together",
            "**Backward A-Star** — precompute perfect heuristics for fast search",
            "**Relaxed goal checking** — graceful degradation under time pressure",
            "**Memory-efficient storage** — scale to massive 2000×2000 grids",
          ],
        },
        {
          type: "text",
          content:
            "The implementation shows how to balance competing demands: optimality, speed, and memory—delivering reliable interception even for large, complex environments.",
        },
      ],
    },
  ],
};