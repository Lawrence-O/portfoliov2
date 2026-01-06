import { Project } from "@/app/components/project/interfaces";

export const autonomousGardenRobot: Project = {
  title: "Autonomous Garden Maintenance Robot",
  date: "Fall 2023",
  media: "/media/videos/garden_robot_vid.mp4",
  tags: ["Robotics"],
  section: [
    {
      title: "Project Introduction",
      navName: "Introduction",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content: "We built a robot that takes care of garden plants on its own. It drives around the garden, checks each plant's soil, and delivers water or nutrients when needed—all without human intervention.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "How It Works",
        },
        {
          type: "text",
          displayAs: "list",
          content: "**Navigates autonomously:** Uses a LiDAR sensor and camera to map the garden and find its way around obstacles.\n**Checks soil health:** A probe measures moisture, pH, and temperature at each plant.\n**Delivers resources:** Based on readings, it dispenses precise amounts of water or liquid nutrients.\n**Logs everything:** Sends photos and sensor data to a web server so you can monitor plant health remotely.",
        },
        {
          type: "text",
          content: "The system runs on **ROS** (Robot Operating System) for high-level control and **FreeRTOS** for real-time motor and sensor operations, all powered by a Jetson Nano.",
        }
      ],
    },
    {
      title: "Challenges and Motivation",
      navName: "Motivation",
      navRef: "challenges-motivation",
      content: [
        {
          type: "text",
          content: "Gardening requires consistent attention—checking soil conditions, watering at the right times, and tracking plant health over weeks or months. For large gardens or farms, this becomes labor-intensive and easy to get wrong.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Problems We Addressed",
        },
        {
          type: "text",
          displayAs: "list",
          content: "**Inconsistent watering:** Plants often get too much or too little water based on guesswork.\n**Soil variability:** Different areas of a garden can have very different moisture and nutrient levels.\n**Time-consuming monitoring:** Manually checking each plant takes hours and is easy to neglect.\n**No historical data:** Without records, it's hard to know what's working and what isn't.",
        },
        {
          type: "text",
          content: "Our robot automates these tasks, providing consistent care and building a data log that helps gardeners make informed decisions.",
        },
      ],
    },
    {
      title: "Robot Function and Operation",
      navName: "Operation",
      navRef: "robot-function",
      content: [
        {
          type: "text",
          content: "The robot follows a simple loop: navigate to a plant, check its soil, take action if needed, and move on.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Operational Cycle",
        },
        {
          type: "text",
          displayAs: "list",
          content: "**Navigate:** The robot uses its LiDAR-built map and fiducial markers (like QR codes) to find each plant location.\n**Sense:** A motorized probe extends into the soil and measures moisture, pH, and temperature.\n**Decide:** Software compares readings against thresholds for that plant type.\n**Act:** If soil is too dry or nutrients are low, the robot dispenses the right amount.\n**Record:** A camera captures the plant's appearance, and all data uploads to a web server.",
        },
        {
          type: "text",
          content: "Users can set how often the robot visits each plant and customize the thresholds that trigger watering or feeding.",
        },
      ],
    },
    {
      title: "System Architecture",
      navName: "Architecture",
      navRef: "system-architecture",
      content: [
          {
            type: "text",
            content: "The robot uses a two-layer software architecture. **ROS** handles high-level decisions like where to go and what to do, while **FreeRTOS** handles precise, time-sensitive tasks like motor control and sensor reading.",
          },
          {
            type: "text",
            displayAs: "subtitle",
            content: "Main Subsystems",
          },
          {
            type: "text",
            displayAs: "list",
            content: "**Movement:** Tread-based drivetrain for navigating over soil, grass, and uneven terrain.\n**Dispensing:** Reservoirs and a spray nozzle deliver measured amounts of water or liquid nutrients.\n**Plant Monitoring:** Linear actuator extends a soil probe; stereo camera captures plant images.\n**Navigation:** LiDAR-based SLAM builds a map; fiducial markers help identify specific plant locations.",
          },
        {
           type: "text",
           content: "The web interface runs on **Django** hosted on AWS EC2, letting users view plant data and photos from anywhere.",
        },
          {
            type: "text",
            displayAs: "subtitle",
            content: "Decision Logic",
          },
          {
            type: "text",
            content: "The robot follows straightforward logic at each plant:"
          },
          {
            type: "code",
            codeLang: "plaintext",
            content:
`1. Navigate to plant using map
2. Deploy soil sensor probe
3. Read moisture, pH, temperature
4. IF reading < threshold: dispense resource
5. Capture photo, log data to server
6. Move to next plant or return to base`
          },
          {
            type: "text",
            content: "The robot's state machine visualizes this flow:"
          },
          {
            type: "code",
            codeLang: "mermaid",
            content:
`graph TD
    A[Idle] --> B(Navigate to Plant);
    B --> C{Plant Reached?};
    C -- Yes --> D[Sense Soil Conditions];
    D --> E[Analyze Sensor Data];
    E --> F{Action Required?};
    F -- Yes --> G[Dispense Resources];
    G --> H[Log Data & Image];
    F -- No --> H; % If no action, proceed to logging
    H --> I{More Plants to Visit?};
    I -- Yes --> B; % Loop to next plant
    I -- No --> A; % Return to Idle
    C -- No --> B; % Continue navigation
`
          }
      ],
    },
        {
             title: "Hardware and Components",
             navName: "Hardware",
             navRef: "components",
            content: [
                {
                    type: "text",
                    content: "The robot is built around a **Jetson Nano** computer mounted on a custom tread-based chassis (15″ × 12″) for stable movement over uneven ground.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Key Components",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: "**LiDAR sensor:** Scans surroundings to build a navigation map.\n**Stereo camera:** Captures plant images and helps with localization.\n**Soil probe:** Custom sensor on a linear actuator measures moisture, pH, and temperature.\n**Dispensing system:** 3D-printed reservoirs and a 60° cone nozzle for precise water/nutrient delivery.\n**Tread drivetrain:** Provides traction on soil, grass, and gravel.",
                },
            ]
        },
       {
        title: "Testing and Validation",
        navName: "Testing",
        navRef: "testing",
        content: [
             {
                    type: "text",
                    content: "We tested each subsystem individually across different surfaces—tiles, lawn, sandbox, and garden soil.",
                  },
                  {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Test Results",
                  },
                  {
                    type: "text",
                    displayAs: "list",
                    content: "**Terrain traversal:** Robot moved stably over all surfaces with no drivetrain issues.\n**Fiducial mapping:** Successfully detected markers and built an accurate map using Hector SLAM.\n**Soil sensing:** Measurements matched expected ranges; temperature accurate within ±2°C.\n**Water dispensing:** Covered a 1′×1′ area accurately with adjustable spray duration.\n**Photo upload:** Images successfully transmitted to the web server (tested via teleoperation).",
                  },
                  {
                    type: "text",
                    content: "Due to time constraints, the photo capture test used teleoperation rather than full autonomy, but all other tests ran autonomously.",
                  },
         ],
        },
         {
             title: "Fault Recovery and Operational Modes",
             navName: "Safety",
             navRef: "fault-recovery",
            content: [
                {
                    type: "text",
                    content: "The robot includes safety features to handle unexpected situations gracefully.",
                 },
                 {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Safety Features",
                 },
                 {
                    type: "text",
                    displayAs: "list",
                    content: "**Emergency stop:** Physical button immediately halts all motors.\n**Sensor error handling:** Ignores or flags inconsistent readings rather than acting on bad data.\n**Communication failure mode:** Continues basic operation locally if server connection drops.\n**Low power mode:** Returns to base when battery runs low.",
                 },
            ]
        },
         {
             title: "Future Improvements",
              navName: "Future Work",
             navRef: "future-work",
            content: [
                {
                     type: "text",
                     content: "With more time, we'd focus on:",
                 },
                 {
                     type: "text",
                     displayAs: "list",
                     content: "**Better sensor calibration:** More rigorous testing against lab-grade instruments.\n**Full autonomous navigation:** Completing the SLAM pipeline for truly hands-off operation.\n**Weather adaptation:** Adjusting behavior based on rain forecasts or soil saturation.\n**Plant identification:** Using computer vision to automatically identify plant species and their needs.",
                 },
            ]
         },
    {
      title: "Conclusion",
      navName: "Conclusion",
      navRef: "conclusion",
      content: [
        {
          type: "text",
          content: "We built a working garden robot that navigates autonomously, monitors soil conditions, and delivers water and nutrients where needed. While we had to simplify some features (like fully autonomous photo capture), the core system works and demonstrates how robotics can make plant care easier and more consistent.",
        },
        {
          type: "text",
          content: "The modular design means future teams can extend it—adding new sensors, improving navigation, or integrating weather data—without rebuilding from scratch.",
        },
      ],
    },
  ],
};
