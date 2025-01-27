import { ProjectBlurb } from "@/app/components/project/ProjectCard";

export const projects: ProjectBlurb[] = [
  {
    title: "Autonomous Garden Maintenance Robot (*Under Construction*)",
    description:
      "This project developed an autonomous robot for garden maintenance, focusing on navigation, plant monitoring through soil sensing and imaging, and precise water and nutrient delivery. The robot uses a combination of computer vision, LiDAR, SLAM, and soil sensors.",
    media: "/media/images/garden-robot.jpg",
    tags: [
      "Robotics",
      "Autonomous Navigation",
      "Computer Vision",
      "Sensors",
      "Embedded Systems",
      "ROS",
    ],
  },
  {
    title: "Distributed Quadrotors Transporting a Load (*Under Construction*)",
    description:
      "This project focused on using a hybrid control approach to manage slack in the cables while also allowing for dynamic reconfigurations of the quadrotors to maintain a stable formation.",
    media: "/media/images/iterativeControl.png",
    tags: [
      "Optimal Control",
      "IPOPT",
      "ALTRO",
      "UAVs",
      "Hybrid Approach",
      "Quaternions",
    ],
  },
  {
    title: "Trajectory Generation with Iterative Learning and Hybrid Control",
    description:
      "This project implemented Iterative Learning Control (ILC) for a car performing a moose test, and hybrid trajectory optimization for bipedal walking, demonstrating advanced trajectory control techniques in Julia.",
    media: "/media/videos/biped_walking.mp4",
    tags: [
      "Iterative Learning Control",
      "Hybrid Dynamics",
      "Trajectory Optimization",
      "Bipedal Walking",
      "Julia",
    ],
  },
  {
    title: "Trajectory Optimization with DIRCOL, iLQR, and TVLQR (*Under Construction*)",
    description:
      "This project explored trajectory optimization for complex systems using Direct Collocation (DIRCOL) and iterative Linear Quadratic Regulator (iLQR). It also used Time-Varying LQR (TVLQR) for tracking solutions on systems such as a cart-pole and a quadrotor using Julia.",
    media: "/media/images/optimalControl.png",
    tags: [
      "Optimization",
      "Control Theory",
      "iLQR",
      "DIRCOL",
      "TVLQR",
      "Julia",
    ],
  },
  {
    title: "Optimal Control of Linear Systems: LQR, TVLQR, and MPC",
    description:
      "This project explored optimal control techniques such as Finite-Horizon LQR, Infinite-Horizon LQR, and Model Predictive Control (MPC) on various linear systems, including a double integrator, a cartpole, and a rendezvous model using Julia.",
    media: "/media/videos/mpc_rendezvous.mp4",
    tags: [
      "Optimal Control",
      "LQR",
      "Convex Optimization",
      "MPC",
      "Julia",
      "State Space Control",
    ],
  },
  {
    title: "Multi-Robot Frontier Exploration",
    description:
      "This project developed a decentralized multi-robot frontier exploration algorithm, allowing a team of robots to explore unknown environments by coordinating their efforts and sharing information. The project implemented a modified A* algorithm and used cost maps to aid in exploration.",
    media: "/media/videos/frontier_grid_small.mp4",
    tags: ["Robotics", "Multi-Agent Systems", "Path Planning", "A*", "Python"],
  },
  {
    title: "Symbolic Planner",
    description:
      "This project involved the development of a symbolic planner, a general-purpose tool designed to solve problems described using symbolic representations.",
    media: "/media/images/symbolicPlanner.png",
    tags: ["Symbolic Planning", "Logic", "Algorithms", "Planning"],
  },
  {
    title: "Sampling Based Motion Planners",
    description:
      "This project compared Probabilistic Roadmap (PRM), Rapidly-exploring Random Tree (RRT), RRT-Connect, and RRT* motion planning algorithms based on computational efficiency, path cost, and success rate for robotic systems.",
    media: "/media/videos/motionPlanner.mp4",
    tags: ["Robotics", "Motion Planning", "PRM", "RRT", "RRT*", "Algorithms"],
  },
  {
    title: "Real-Time Target Interception with Multi-Goal A*",
    description:
      "A Multi-Goal A* path planner, enhanced with a Backward A* heuristic, was developed to enable a robot to efficiently intercept a moving target in a 2D costmap environment under real-time constraints. This project used C++",
    media: "/media/videos/movingTargetsVideo.mp4",
    tags: [
      "Robotics",
      "Motion Planning",
      "A*",
      "Pathfinding",
      "Algorithms",
      "C++",
    ],
  },
  {
    title: "Jenga Building Robotic Arm",
    description:
      "This project developed a robotic arm that used object detection, path planning and manipulation to build a Jenga tower autonomously.",
    media: "/media/videos/rkd_jenga.mp4",
    tags: [
      "Robotics",
      "Object Detection",
      "Path Planning",
      "Manipulation",
      "Algorithms",
      "Python",
    ],
  },
  {
    title: "Robotic Sentry: Safe Guard Against Pests",
    description:
      "This project developed a robotic system with object detection, and dynamic path planning to autonomously find and eliminate pests. The robot uses computer vision, embedded systems, and a custom linear regression algorithm to adjust difficulty as the user learns.",
    media: "/media/videos/sap_front_view.mp4",
    tags: [
      "Robotics",
      "Pest Control",
      "Computer Vision",
      "Embedded Systems",
      "CAD",
      "Machine Learning",
    ],
  },
  {
    title: "Grip Assistive Glove",
    description:
      "This project aimed to create an assistive glove with a motor-driven mechanism that enhances grip strength, using CAD, FEA, and prototyping for people with conditions such as Cerebral Palsy and Tendonitis.",
    media: "/media/videos/gripAssitiveGlove.mp4",
    tags: [
      "Assistive Technology",
      "Mechanical Design",
      "CAD",
      "FEA",
      "Prototyping",
      "Product Design",
    ],
  },
  {
    title: "Habitat Airlock Linkage System",
    description:
      "This project designed and optimized a mechanical linkage system to actuate a button within a specific time frame, using iterative design, stress analysis, and testing for an airlock system, and using CAD and FEA to determine the best design.",
    media: "/media/videos/habitat_linkage.mp4",
    tags: [
      "Mechanical Design",
      "Linkage System",
      "CAD",
      "FEA",
      "Prototyping",
      "Optimization",
    ],
  },
  {
    title: "The Ultimate Quiz: Ruler Edition",
    description:
      "This project developed a dynamic quiz game that generates questions using web scraping and a personalized mastery system based on spaced repetition. This project used python to create both the game and the algorithms.",
    media: "/media/videos/quizGame.mp4",
    tags: [
      "Web Scraping",
      "Game Development",
      "Algorithms",
      "Python",
      "Object-Oriented Design",
    ],
  },
];
