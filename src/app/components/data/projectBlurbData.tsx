import { ProjectBlurb } from "@/app/components/project/ProjectCard";

export const projects: ProjectBlurb[] = [
  {
    title: "Portfolio Website",
    description:
      "The site you're looking at. A data-driven architecture where each project is a TypeScript file, making it easy to add new content without touching layout code.",
    media: "/media/images/landing-page-author.jpg",
    tags: ["Software"],
    status: "Completed",
    date: "2025-03-25",
  },
  {
    title: "Autonomous Garden Maintenance Robot",
    description:
      "A robot that navigates a garden autonomously, monitors plant health through sensors and imaging, and delivers water and nutrients where needed.",
    media: "/media/videos/garden_robot_vid.mp4",
    tags: ["Robotics"],
    status: "Under Construction",
    date: "2024-05-01",
  },
  {
    title: "Distributed Quadrotors Transporting a Load",
    description:
      "Multiple drones working together to carry a heavy load suspended by cables. The system handles cable slack and formation changes to keep the payload stable.",
    media: "/media/images/cableSus_4_agents.png",
    tags: ["Robotics", "Controls"],
    status: "Completed",
    date: "2024-04-15",
  },
  {
    title: "Trajectory Generation with Iterative Learning and Hybrid Control",
    description:
      "A car learns to perform emergency lane changes through repeated practice, plus a walking robot that switches between different leg phases.",
    media: "/media/videos/biped_walking.mp4",
    tags: ["Controls"],
    status: "Completed",
    date: "2023-12-01",
  },
  {
    title: "Trajectory Optimization with DIRCOL, iLQR, and TVLQR",
    description:
      "Finding optimal paths for dynamic systems like swinging up a cart-pole or flipping a quadrotor, then tracking those trajectories with feedback control.",
    media: "/media/images/quadrotor_reorient.gif",
    tags: ["Controls"],
    status: "Completed",
    date: "2023-11-01",
  },
  {
    title: "Optimal Control of Linear Systems: LQR, TVLQR, and MPC",
    description:
      "Classic control techniques applied to spacecraft docking, cart-pole balancing, and other linear systems. Compares LQR and Model Predictive Control approaches.",
    media: "/media/videos/mpc_rendezvous.mp4",
    tags: ["Controls"],
    status: "Completed",
    date: "2023-08-15",
  },
  {
    title: "Multi-Robot Frontier Exploration",
    description:
      "Multiple robots coordinate to explore unknown environments efficiently. Each robot claims frontier regions and avoids duplicating work with teammates.",
    media: "/media/videos/frontier_grid_small.mp4",
    tags: ["Robotics", "AI"],
    status: "Completed",
    date: "2023-05-20",
  },
  {
    title: "Symbolic Planner",
    description:
      "An AI planner that figures out action sequences to achieve goals. Given a start state, goal, and available actions, it finds a plan using A-Star search.",
    media: "/media/images/symbolicPlanner.png",
    tags: ["AI"],
    status: "Completed",
    date: "2023-03-10",
  },
  {
    title: "Sampling-Based Motion Planners",
    description:
      "Comparing different approaches to robot path planning: PRM builds a roadmap, RRT grows a tree, and RRT* optimizes for shorter paths.",
    media: "/media/videos/motionPlanner.mp4",
    tags: ["AI", "Robotics"],
    status: "Completed",
    date: "2022-12-05",
  },
  {
    title: "Real-Time Target Interception with Multi-Goal A*",
    description:
      "Catching moving targets in real-time. The planner considers multiple possible interception points and picks the best one as the target moves.",
    media: "/media/videos/movingTargetsVideo.mp4",
    tags: ["AI", "Robotics"],
    status: "Completed",
    date: "2022-10-01",
  },
  {
    title: "Jenga Building Robotic Arm",
    description:
      "A robotic arm that detects Jenga blocks, plans collision-free paths, and stacks them into a tower autonomously.",
    media: "/media/videos/rkd_jenga.mp4",
    tags: ["Robotics"],
    status: "Completed",
    date: "2022-08-15",
  },
  {
    title: "Safeguard Against Pests (SAP)",
    description:
      "An autonomous robot that detects lanternflies using computer vision and sprays them with targeted pesticide, reducing chemical use compared to broad spraying.",
    media: "/media/videos/sap_front_view.mp4",
    tags: ["Robotics", "Mechanical"],
    status: "Completed",
    date: "2022-05-01",
  },
  {
    title: "Grip Assistive Glove",
    description:
      "A motorized glove that helps people with limited hand strength grip objects. Designed for conditions like Cerebral Palsy and Tendonitis.",
    media: "/media/videos/gripAssitiveGlove.mp4",
    tags: ["Mechanical"],
    status: "Completed",
    date: "2021-12-10",
  },
  {
    title: "Habitat Airlock Linkage System",
    description:
      "A mechanical linkage that presses a button at a precise time using only gravity and carefully designed geometry. No motors or electronics.",
    media: "/media/videos/habitat_linkage.mp4",
    tags: ["Mechanical"],
    status: "Completed",
    date: "2021-08-20",
  },
  {
    title: "The Ultimate Quiz",
    description:
      "A trivia game about historical rulers that generates questions by scraping Wikipedia. Tracks your progress and revisits questions you miss more often.",
    media: "/media/videos/quizGame.mp4",
    tags: ["Software"],
    status: "Completed",
    date: "2020-12-15",
  },
];
