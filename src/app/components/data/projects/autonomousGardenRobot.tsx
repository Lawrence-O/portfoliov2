import { Project } from "@/app/components/project/interfaces";

export const autonomousGardenRobot: Project = {
  title: "Autonomous Garden Maintenance Robot",
  subtitle: "Fall 2023",
  media: "/media/images/safeguardAgainstPests.png",
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
            "This project focused on developing an autonomous garden maintenance robot that assists in resource management and plant care. The robot integrates computer vision, LiDAR, and SLAM techniques to autonomously navigate a garden, analyze plants using a soil sensor probe, and precisely administer water and nutrients based on its analysis. The goal was to create a self-sustaining, intelligent gardener that could revolutionize horticulture and agriculture practices.",
        },
         {
              type: "image",
              content: "/media/images/optimalControlIntro.png",
              altContent: "Diagram illustrating the ILC and hybrid trajectory optimization concepts",
              subtitle: "General Overview of Project Concepts",
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
            "The robot is designed to autonomously navigate a garden using a combination of camera, LiDAR, and fiducial markers, creating a map and identifying plants within. The robot uses its soil sensor to analyze each plant's needs, dispensing water and nutrients at user-specified frequencies. Additionally, it collects plant images which are available to the user through an online interface. The robot's design prioritizes efficiency, precision, and consistency in plant care.",
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
               "The system architecture is designed to be robust and modular, handling various aspects of the robot.  A combination of ROS and FREE RTOS were used to ensure interoperability between the low and high level modules. The system is structured into several key subsystems:",
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
                        "Our testing focused on validating key aspects of the system. We conducted a traversable soil test to ensure mobility over various surfaces. A mapping fiducial test was also performed to verify that the robot could map the garden using fiducials.  We also tested our sensor subsystem to see if it could accurately measure the pH, moisture, and temperature of soil, as well as water dispensing for the ability to precisely deliver water to the plant. Lastly a photo capture test was performed to verify the robots ability to send data to a server.",
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