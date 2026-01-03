import { ProjectBlurb } from "@/app/components/project/ProjectCard";

export const projects: ProjectBlurb[] = [
  {
    title: "Personal Portfolio Website",
    description:
      "A modern, responsive portfolio built with Next.js, TypeScript, and Tailwind CSS. Features dynamic project pages, type-safe content management, and optimized performance.",
    media: "/media/images/landing-page-author.jpg",
    tags: ["Software Development"],
    status: "Completed",
    date: "2025-03-25",
  },
  {
    title: "Autonomous Garden Maintenance Robot",
    description:
      "An autonomous robot for garden maintenance featuring navigation, plant monitoring through soil sensing and imaging, and precise water and nutrient delivery. Built with computer vision, LiDAR, SLAM, and soil sensors.",
    media: "/media/images/garden-robot.jpg",
    tags: ["Robotics & Intelligent Systems", "Software Development"],
    status: "Under Construction",
    date: "2024-05-01",
  },
  {
    title: "Distributed Quadrotors Transporting a Load",
    description:
      "A hybrid control approach for cable-suspended load transport using multiple quadrotors. The system manages cable slack and enables dynamic reconfigurations to maintain stable formation during payload transport.",
    media: "/media/images/cableSus_4_agents.png",
    tags: ["Robotics & Intelligent Systems", "Control & Optimization"],
    status: "Completed",
    date: "2024-04-15",
  },
  {
    title: "Trajectory Generation with Iterative Learning and Hybrid Control",
    description:
      "Iterative Learning Control (ILC) for a car performing a moose test maneuver, combined with hybrid trajectory optimization for bipedal walking. Implemented in Julia to demonstrate advanced trajectory control techniques.",
    media: "/media/videos/biped_walking.mp4",
    tags: ["Control & Optimization", "Software Development"],
    status: "Completed",
    date: "2023-12-01",
  },
  {
    title: "Trajectory Optimization with DIRCOL, iLQR, and TVLQR",
    description:
      "Trajectory optimization for dynamic systems using Direct Collocation (DIRCOL) and iterative LQR (iLQR), with Time-Varying LQR (TVLQR) for trajectory tracking. Applied to cart-pole and quadrotor systems in Julia.",
    media: "/media/images/quadrotor_reorient.gif",
    tags: ["Control & Optimization", "Software Development"],
    status: "Completed",
    date: "2023-11-01",
  },
  {
    title: "Optimal Control of Linear Systems: LQR, TVLQR, and MPC",
    description:
      "Optimal control techniques including Finite-Horizon LQR, Infinite-Horizon LQR, and Model Predictive Control (MPC) applied to linear systems such as a double integrator, cart-pole, and spacecraft rendezvous. Implemented in Julia.",
    media: "/media/videos/mpc_rendezvous.mp4",
    tags: ["Control & Optimization", "Software Development"],
    status: "Completed",
    date: "2023-08-15",
  },
  {
    title: "Multi-Robot Frontier Exploration",
    description:
      "A decentralized multi-robot frontier exploration algorithm enabling coordinated exploration of unknown environments. Features a modified A* algorithm with cost maps for efficient frontier allocation.",
    media: "/media/videos/frontier_grid_small.mp4",
    tags: ["Robotics & Intelligent Systems", "Software Development"],
    status: "Completed",
    date: "2023-05-20",
  },
  {
    title: "Symbolic Planner",
    description:
      "A general-purpose symbolic planner for solving planning problems described using symbolic representations, including PDDL-style domain and problem definitions.",
    media: "/media/images/symbolicPlanner.png",
    tags: ["Robotics & Intelligent Systems", "Software Development"],
    status: "Completed",
    date: "2023-03-10",
  },
  {
    title: "Sampling-Based Motion Planners",
    description:
      "A comparative analysis of PRM, RRT, RRT-Connect, and RRT* motion planning algorithms, evaluating computational efficiency, path cost, and success rate for robotic path planning.",
    media: "/media/videos/motionPlanner.mp4",
    tags: ["Robotics & Intelligent Systems", "Software Development"],
    status: "Completed",
    date: "2022-12-05",
  },
  {
    title: "Real-Time Target Interception with Multi-Goal A*",
    description:
      "A Multi-Goal A* path planner with Backward A* heuristic for real-time interception of moving targets in 2D costmap environments. Implemented in C++ for efficient real-time performance.",
    media: "/media/videos/movingTargetsVideo.mp4",
    tags: ["Robotics & Intelligent Systems", "Software Development"],
    status: "Completed",
    date: "2022-10-01",
  },
  {
    title: "Jenga Building Robotic Arm",
    description:
      "A robotic arm system that autonomously builds Jenga towers using object detection, path planning, and precise manipulation control.",
    media: "/media/videos/rkd_jenga.mp4",
    tags: ["Robotics & Intelligent Systems", "Software Development"],
    status: "Completed",
    date: "2022-08-15",
  },
  {
    title: "Robotic Sentry: Safeguard Against Pests",
    description:
      "An autonomous robotic system for pest detection and elimination using computer vision and dynamic path planning. Features adaptive difficulty adjustment via custom linear regression for user training.",
    media: "/media/videos/sap_front_view.mp4",
    tags: ["Robotics & Intelligent Systems", "Mechanical Engineering", "Software Development"],
    status: "Completed",
    date: "2022-05-01",
  },
  {
    title: "Grip Assistive Glove",
    description:
      "A motor-driven assistive glove that enhances grip strength for individuals with conditions such as Cerebral Palsy and Tendonitis. Designed using CAD, FEA, and iterative prototyping.",
    media: "/media/videos/gripAssitiveGlove.mp4",
    tags: ["Mechanical Engineering"],
    status: "Completed",
    date: "2021-12-10",
  },
  {
    title: "Habitat Airlock Linkage System",
    description:
      "A mechanical linkage system designed to actuate an airlock button within precise time constraints. Optimized through iterative design, stress analysis, and testing using CAD and FEA.",
    media: "/media/videos/habitat_linkage.mp4",
    tags: ["Mechanical Engineering"],
    status: "Completed",
    date: "2021-08-20",
  },
  {
    title: "The Ultimate Quiz: Ruler Edition",
    description:
      "A dynamic quiz game featuring web-scraped question generation and a personalized mastery system based on spaced repetition. Built entirely in Python.",
    media: "/media/videos/quizGame.mp4",
    tags: ["Software Development"],
    status: "Completed",
    date: "2020-12-15",
  },
];
