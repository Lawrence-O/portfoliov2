import { Project } from "@/app/components/project/interfaces";

export const autonomousGardenRobot: Project = {
  title: "Autonomous Garden Maintenance Robot",
  date: "Fall 2023",
  media: "/media/images/safeguardAgainstPests.png",
  githubLink: "https://github.com/your-username/autonomous-garden-robot", // Added placeholder
  tags: ["Robotics", "Autonomous Navigation", "Computer Vision", "Sensors", "Embedded Systems", "ROS"],
  section: [
    {
      title: "Project Introduction",
      navName: "Introduction",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content:
            "This project focused on developing an autonomous garden maintenance robot that assists in resource management and plant care. The robot integrates computer vision (for tasks such as plant identification or fiducial marker detection), LiDAR, and SLAM techniques (specifically Hector SLAM) to autonomously navigate a garden, analyze plants using a soil sensor probe, and precisely administer water and nutrients based on its analysis. The system was developed primarily using ROS (Robot Operating System) for high-level control and sensor integration, with C++ and Python for its nodes. The goal was to create a self-sustaining, intelligent gardener that could revolutionize horticulture and agriculture practices.",
        },
        {
          type: "image",
          content: "/media/images/placeholder_garden_robot_overview.png", // Suggested: Replace with a high-level diagram of the garden robot system
          altContent: "Conceptual overview diagram of the Autonomous Garden Robot system",
          subtitle: "High-Level System Overview of the Autonomous Garden Robot",
        }
      ],
    },
    {
      title: "Challenges and Motivation",
      navName: "Challenges and Motivation",
      navRef: "challenges-motivation",
      content: [
        {
          type: "text",
          content:
           "Traditional gardening and farming often involve labor-intensive processes that can be inefficient. This project addresses challenges such as varied soil conditions, the need for precise plant identification, inconsistent plant monitoring, and the complexities of resource management. Our motivation stems from the need to automate repetitive tasks, reduce labor dependency, and enhance overall efficiency in plant care while also providing a convenient data logging framework.",
        },
        {
          type: "text",
            content: " Key issues addressed include:"
          },
        {
            type: "text",
            content:
                "**Soil Monitoring:** Measuring and mapping variations in soil quality, moisture, and pH levels. **Plant Identification:** Accurately identifying different plant species and their needs. **Resource Management:** Managing water, nutrients, and energy efficiently. **Data Collection:** Organizing and utilizing collected data for informed gardening.",
        },
      ],
    },
    {
      title: "Robot Function and Operation",
      navName: "Robot Function",
      navRef: "robot-function",
      content: [
        {
          type: "text",
          content:
            "The robot is designed to autonomously navigate a garden using a combination of camera, LiDAR, and fiducial markers, creating a map and identifying plants within. The robot uses a multi-parameter soil sensor probe (e.g., measuring moisture, pH) to analyze each plant's needs; this data directly informs the decision-making logic for dispensing precise amounts of water and nutrients at user-specified frequencies. Additionally, it collects plant images which are available to the user through an online interface. The robot's design prioritizes efficiency, precision, and consistency in plant care.",
        },
      ],
    },
    {
      title: "System Architecture",
      navName: "System Architecture",
      navRef: "system-architecture",
      content: [
          {
            type: "text",
            content:
               "The system architecture is designed to be robust and modular, handling various aspects of the robot. A combination of ROS and FreeRTOS was a strategic choice: ROS provided a flexible framework for high-level tasks like navigation, perception, and inter-process communication, while FreeRTOS offered the real-time determinism required for precise low-level control of motors and sensor data acquisition. The system is structured into several key subsystems:",
          },
         {
            type: "image",
            content: "/media/images/functional_decomposition.png",
            altContent: "Functional Decomposition Diagram",
            subtitle: "Functional decomposition tree diagram."
         },
         {
           type: "text",
           content: "The main subsystems are: **Movement**: To navigate and traverse diverse soil conditions using treads. **Water/Nutrient Dispensing**: To precisely administer water and nutrients through a nozzle system. **Plant Monitoring**: To capture images and measure soil conditions using a probe sensor. **Navigation**:  To map and localize within the garden using SLAM and fiducials.",
          },
        {
           type: "image",
           content: "/media/images/hardware-firmware-layer.png",
           altContent: "Diagram of Software and Hardware Architecture",
           subtitle: "Diagram of the software and hardware architecture.",
        },
         {
            type: "image",
            content: "/media/images/JetsonNanoIO.png",
            altContent: "Diagram of the Jetson Nano's IO ports",
            subtitle:"The Jetson Nano's IO ports used for connecting the robot's subsystems.",
         },
        {
           type: "text",
           content: "Additionaly, the software implementation uses Django on the EC2 server for web application, and ROS and FREERTOS on the robot. ",
        },
        {
             type: "image",
             content: "/media/images/middleware.png",
             altContent: "Diagram showcasing the middleware and data flow",
             subtitle: "Diagram of the middleware and data flow through the robot.",
          },
          {
            type: "text",
            content: "**Core Operational Logic (Pseudocode):** The robot's decision-making for plant care follows a defined sequence:"
          },
          {
            type: "code",
            codeLang: "plaintext",
            content:
`Algorithm: Autonomous Plant Care Logic
1. Robot navigates to designated plant/area using SLAM-generated map.
2. Deploy soil sensor probe into the soil.
3. Read sensor values (e.g., moisture, pH, nutrient levels).
4. FOR EACH monitored_parameter (e.g., moisture, specific nutrient):
5.     IF current_value < plant_profile.threshold[parameter] THEN
6.         Calculate required_amount based on deficit and plant needs.
7.         Activate_dispenser(parameter, required_amount).
8.     ENDIF
9. ENDFOR
10. Capture image of the plant/area for visual record.
11. Log sensor_data, actions_taken, and image to the remote server.
12. Proceed to next plant/area or return to base.`
          },
          {
            type: "text",
            content: "**Simplified State Machine:** The robot's operational states can be visualized as follows:"
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
             navName: "Components",
             navRef: "components",
            content: [
                {
                    type: "text",
                     content:
                        "The robot's physical form is designed with various components working together. The chassis uses a tread-style drivetrain with a 15-inch width and 12-inch length to ensure stability. The nutrient and water dispensing system has custom 3D-printed reservoirs and a custom 60-degree cone nozzle. The plant monitoring subsystem uses a linear actuator, a custom soil sensor, and a stereo camera.",
                },
                {
                    type: "image",
                     content: "/media/images/robot_components.png",
                     altContent: "CAD model of the robot design",
                     subtitle: "Isometric view of the fully assembled robot.",
                 },
                {
                     type: "text",
                    content:
                        "The robot utilizes a Jetson Nano as the central computing platform, and various sensors to acquire information and ensure proper control and navigation. The key components include a LiDAR sensor, a stereo camera, a soil sensor, a linear actuator, various motors and drivers, as well as a power supply and communication system for data logging and robot control.",
                },
                 {
                     type: "image",
                      content: "/media/images/parts_and_components.png",
                     altContent: "Bill of Materials",
                     subtitle: "Bill of materials for the core components of the robot.",
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
                    content:
                        "Our testing focused on validating key aspects of the system. We conducted a traversable soil test to ensure mobility over various surfaces. A mapping fiducial test was also performed to verify that the robot could map the garden using fiducials. We also tested our sensor subsystem to see if it could accurately measure the pH, moisture, and temperature of soil, as well as water dispensing for the ability to precisely deliver water to the plant. Lastly a photo capture test was performed to verify the robot's ability to send data to a server. For enhanced academic review, future iterations could detail specific quantitative metrics from these tests (e.g., 'soil sensor accuracy was ±X% for moisture readings compared to calibrated instruments,' or 'fiducial detection success rate was Y% under Z lighting conditions').",
                  },
                  {
                    type: "text",
                      content:
                        "We conducted the tests on various areas such as on tiles, in the lawn, in a sandbox, and in a garden environment. Due to time constraints, we were only able to implement the photo capture system via teleoperation rather than autonomously as we did with all other tests.",
                  },
                {
                   type: "text",
                    content:"Here are some of the results from the testing:",
                  },
                  {
                    type: "text",
                    content:
                         "**Traversable Soil Test:** The robot successfully traversed various soil types on a straight path. The chassis was stable without any sagging, and all motors and drive train components performed as expected.",
                 },
                 {
                    type: "text",
                     content:
                         "**Fiducial Mapping Test:** The robot successfully detected fiducials while moving around the environment, and is able to map it's position using hector slam as well.",
                 },
                  {
                      type: "text",
                    content:
                         "**Soil Sampling Test**:  The robot was tested in open air and water and the results indicate that the sensor can measure soil properties within the specified tolerances. The sensor also showed the ability to measure temperature with a +-2 degree error on a series of tests.",
                     },
                    {
                      type: "text",
                        content:
                         "**Water Dispensing Test:** The robot was able to accurately dispense water within a 3 foot wide area, and we can adjust the dispensing time to create coverage over the 1'x1' area."
                    },
                   {
                     type: "text",
                      content:
                        "**Photo Capture Test:** Although we were unable to test the camera autonomously, we were able to test uploading an image to a server, indicating that the transmission of data is functional.",
                    },
              {
                type: "image",
                 content: "/media/images/soil_sensor.png",
                 altContent: "A soil sensor being used to probe a soil sample",
                   subtitle: "Testing the soil sensor on a soil sample.",
                },
                {
                 type: "image",
                  content: "/media/images/soil_upload.png",
                  altContent: "An image of data being uploaded to the server.",
                    subtitle: "Data collected by the sensor being uploaded to a web server",
                },

         ],
        },
         {
             title: "Fault Recovery and Operational Modes",
             navName: "Fault Recovery",
             navRef: "fault-recovery",
            content: [
                {
                    type: "text",
                     content:
                         "The robot incorporates robust fault recovery mechanisms to ensure consistent performance. Key safety features include a hardware emergency stop (ESTOP) button for immediate shutdown, comprehensive sensor error handling to address inconsistent data from the soil sensor, and active detection of actuator and communication failures. The robot uses different operational modes to manage different failures such as a Safe Navigation Mode, Communication Failure Mode, Manual Control Mode and a Lower Power Mode.",
                 },
            ]
        },
         {
             title: "Future Improvements",
              navName: "Future Improvements",
             navRef: "future-work",
            content: [
                {
                     type: "text",
                     content:
                        "The team recommends several improvements for future iterations of the project. These include exploring more diverse methods for sensor calibration, focusing on improved autonomy through a functional SLAM pipeline, and more in depth testing of the various subsystems within different environmental parameters.",
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
          content:
            "This project created an autonomous gardening robot that can successfully navigate a garden, analyze plant health, and dispense water and nutrients as needed. While the project faced challenges and had to descope some parts of the project (such as the fully autonomous navigation), the project successfully showcased the systems capacity to help automate and optimize gardening tasks.  Through a detailed design process, testing, and validation, the project produced a system that is both functional and has a sound basis for future expansion.",
        },
      ],
    },
  ],
};
