import { Project} from "@/app/components/project/interfaces";

export const multiRobotFrontierExploration: Project = {
    title: "Multi-Robot Frontier Exploration",
    date: "Fall 2023",
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
            "Imagine sending robots into an unknown building after a disaster—how do they efficiently search every room without bumping into each other? This project tackles exactly that: **multi-robot exploration** of unknown environments. Instead of one robot slowly mapping everything, multiple robots work together, each exploring different areas simultaneously. The challenge is coordinating them without a central 'brain'—each robot makes its own decisions based on what it knows.",
        },
        {
          type: "image",
          content: "/media/images/frontier_inprogress.png",
          altContent: "A grid map being explored by multiple robots.",
          subtitle: "Multiple robots exploring a grid environment",
        },
      ],
    },
    {
      title: "Problem Statement",
      navName: "Problem",
      navRef: "problem-statement",
      content: [
        {
          type: "text",
          content:
            "Each robot can only 'see' a small area around itself (like a flashlight in the dark). The goal: explore the entire environment as quickly as possible, without robots colliding or wasting time exploring the same areas twice.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Objectives",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Complete coverage** — map the entire environment",
            "**Minimize time** — leverage parallelism across robots",
            "**Collision avoidance** — prevent robot-robot interference",
            "**Decentralized** — no central coordinator required",
          ],
        },
      ],
    },
    {
      title: "Approach",
      navName: "Approach",
      navRef: "approach",
      content: [
        {
          type: "text",
          content:
            "The core idea is **frontier-based exploration**: robots seek out 'frontiers'—the boundaries between explored space and the unknown. It's like always walking toward the edge of your flashlight's beam. Each robot independently finds the nearest frontier and plans a path there using a modified **A-Star** algorithm. To avoid crowding, robots make areas near other robots' targets more 'expensive' to visit—naturally spreading them across the environment.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Components",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Frontier Detection** — identify boundaries between explored and unknown space",
            "**Multi-Goal A-Star** — navigate toward any frontier while avoiding others",
            "**Cost Maps** — dynamically penalize regions targeted by other robots",
            "**Dynamic Replanning** — detect collisions and replan to safe positions",
          ],
        },
      ],
    },
    {
      title: "Algorithm Details",
      navName: "Algorithm",
      navRef: "algorithm-details",
      content: [
        {
          type: "text",
          content:
            "Each robot repeats a simple loop: look around and update its map, pick the best frontier to explore, move toward it, and repeat. The magic is in how robots avoid getting in each other's way.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Sense** — update local map from sensor readings",
            "**Plan** — identify frontiers, run Multi-Goal A-Star to nearest one",
            "**Act** — move along planned path, replan if collision detected",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Multi-Goal A*",
        },
        {
          type: "text",
          content:
            "Standard A-Star pathfinding finds the shortest path to *one* destination. But we have many possible frontiers! **Multi-Goal A-Star** terminates as soon as it reaches *any* frontier—whichever is easiest to get to. The clever part: when comparing paths, we add 'penalty costs' for areas near where other robots are heading:",
        },
        {
          type: "code",
          codeLang: "python",
          content: `def __lt__(self, other):
    my_val = self.f
    other_val = other.f
    # Add costs from other robots' targets
    for drone_id, costs in temp_costs.items():
        if drone_id != self.drone_id:
            my_val += costs.get((self.x, self.y), 0)
    return my_val < other_val`,
        },
        {
          type: "text",
          content:
            "When a robot claims a frontier, it inflates costs in a radius around that cell—discouraging other robots from targeting the same area.",
        },
      ],
    },
    {
      title: "Core Concepts",
      navName: "Concepts",
      navRef: "key-concepts",
      content: [
        {
          type: "text",
          content:
            "Why does this work? Several design choices work together:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Frontier-Based Exploration** — always move toward the unknown; stop when nothing's left",
            "**Greedy Selection** — pick the nearest frontier; simple but surprisingly effective",
            "**Decentralized Control** — no central computer needed; if one robot fails, others keep going",
            "**Cost Map Coordination** — robots implicitly 'claim' areas by making them expensive for others",
            "**Multi-Goal A-Star** — efficiently find paths when any of many destinations works",
          ],
        },
      ],
    },
    {
      title: "Implementation",
      navName: "Implementation",
      navRef: "code",
      content: [
        {
          type: "text",
          content:
            "The Python implementation has three main pieces: tracking search progress (`Node`), managing individual robots (`Robot`), and coordinating the overall exploration (`Planner`).",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "`Node` — A-Star search state with position, costs, and drone ID",
            "`Robot` — individual robot state and movement",
            "`Planner` — map updates, frontier detection, pathfinding",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Cost Inflation",
        },
        {
          type: "text",
          content:
            "Here's the key coordination trick: when a robot picks a frontier target, it 'claims' that area by inflating costs in a square neighborhood around it. Other robots see these costs and naturally steer toward different frontiers:",
        },
        {
          type: "code",
          codeLang: "python",
          content: `# When path found to frontier
for dx in range(-COST_RANGE, COST_RANGE + 1):
    for dy in range(-COST_RANGE, COST_RANGE + 1):
        temp_costs[drone_id][(goal.x + dx, goal.y + dy)] = COST_INCREASE`,
        },
        {
          type: "text",
          content:
            "Other robots incorporate these costs during planning, naturally spreading out across the environment.",
        },
      ],
    },
    {
      title: "Results",
      navName: "Results",
      navRef: "results",
      content: [
        {
          type: "text",
          content:
            "Does adding more robots actually help? I tested on maps of different sizes with varying robot counts to find out.",
        },
        {
          type: "image",
          content: "/media/images/frontier_multmap.png",
          altContent: "Small, medium and large maps used for testing",
          subtitle: "Test environments of varying sizes",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Findings",
        },
        {
          type: "text",
          content:
            "The results show a sweet spot: adding robots helps up to a point, then they start getting in each other's way.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Map scaling** — exploration time scales roughly linearly with cell count",
            "**Robot scaling** — more robots reduce time, but with diminishing returns",
            "**Optimal count** — 5-7 robots balanced coverage vs. interference on largest maps",
          ],
        },
        {
          type: "image",
          content: "/media/images/frontier_robotsgraph.png",
          altContent: "A plot showing the relationship between run time and the number of robots",
          subtitle: "Exploration time vs. number of robots",
        },
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
            "Several directions for extending this work:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Complex environments** — dynamic obstacles, complex topologies",
            "**Variable sensing range** — current implementation uses range of 1",
            "**Better heuristics** — informed estimates could improve efficiency",
            "**Robot communication** — explicit coordination vs. implicit cost maps",
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
            "This project shows that multiple robots can efficiently explore unknown spaces without needing a central coordinator—just local information and clever cost-sharing. The big takeaways:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "Multiple robots **dramatically speed up** exploration",
            "**No central brain needed** — robots coordinate through shared cost maps",
            "There's an **optimal team size** that balances speed vs. crowding",
            "The approach **scales naturally** to bigger environments",
          ],
        },
      ],
    },
  ],
};
