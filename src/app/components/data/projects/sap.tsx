import { Project} from "@/app/components/project/interfaces";

export const safeguardAgainstPests: Project = {
    title: "Safeguard Against Pests (SAP)",
    subtitle: "Electromechanical Systems Design Project - Fall 2023",
    media: "/media/videos/sap_front_view.mp4",
    tags: ["Robotics", "Pest Control", "Computer Vision", "Embedded Systems", "CAD", "Machine Learning"],
    section: [
        {
            title: "Project Overview",
            navName: "Project Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content:
                        "The Safeguard Against Pests (SAP) project was initiated to address the increasing threat posed by pest infestations, with a particular focus on the destructive lantern fly. These invasive pests pose a significant risk to agriculture and home gardens. Traditional pest control methods often rely on harmful chemicals, which have detrimental impacts on both the environment and human health. The SAP project aimed to develop an effective, sustainable, and autonomous pest control solution that combines robotics, computer vision, and embedded systems to create an intelligent pest control robot."
                },
                {
                    type: "image",
                    content: "/media/images/sap_prototype.png",
                    altContent: "Image of the SAP prototype",
                    subtitle: "The completed SAP prototype, ready for testing."
                },
            ]
        },
        {
            title: "Problem Definition and Market Analysis",
            navName: "Problem Definition & Market Analysis",
            navRef: "problem-definition",
            content: [
                {
                    type: "text",
                    content:
                        "Our project began with a clear understanding of the need for an effective and environmentally conscious pest control solution. Existing market options were found to be limited, with many still in the prototype stage. Competitors included CO2-emitting and suction-based mosquito control robots, large-scale agricultural pest control robots, and laser deterrents with constrained computational power. SAP is designed to stand out by using targeted pesticide application, advanced computer vision, and fully autonomous operation, making it ideal for home garden use. Our survey data confirmed that current methods lack accuracy and range, which our design addresses, making it a competitive alternative to conventional methods."
                },
                  {
                    type: "text",
                     content: "Key stakeholders for this project include homeowners, gardeners, local communities, and environmental agencies. The team conducted surveys to identify customer needs, including the need for autonomy, environmental friendliness, accuracy, and sufficient range. These needs were then classified as either basic or performance needs to prioritize our design goals."
                },
                  {
                    type: "text",
                     content: "Target specifications were established based on these identified needs and survey results. These specifications included minimum liquid storage, maximum liquid used per insect, minimum battery life, minimum accuracy rate, and minimum shooting range. A competitive analysis was performed comparing our target specifications to both conventional pest control methods and the ideal values determined through comparing retail products and customer feedback."
                 },

            ]
        },
         {
            title: "Concept Generation and Design",
            navName: "Concept Generation",
            navRef: "concept-generation",
             content: [
               {
                    type: "text",
                    content: "Our design process began with functional decomposition, identifying core subsystems: material storage, target identification, target tracking, and liquid shooting. Each subsystem was further broken down into its main functions. The system uses an electric pump for liquid dispensing, a rotating base combined with pitch controls for tracking, a force sensor to determine liquid levels, and a simple screw cap for refilling."
                },
                {
                   type: "image",
                   content: "/media/images/sap_functional_decomposition.png",
                   altContent: "Functional Decomposition Diagram",
                   subtitle: "Diagram illustrating the functional decomposition of the system."
                },
                 {
                    type: "text",
                    content:
                         "For target identification, the team explored several methods, including Convolutional Neural Networks (CNN), a Bag of Words approach with K-Nearest Neighbors, template matching, and color matching. A CNN was initially chosen for its superior performance and robustness to environmental variables, but this was later replaced by a more efficient YOLO model. For tracking, we considered a rotating cylinder with tilt control, and a rotating base with a linear actuator. A combination of both was selected, using a rotating platform combined with pitch controls. The shooting mechanisms explored included an electric pump and a pressurized air system, with the electric pump being selected for its consistent shooting power and ease of implementation."
                },
            ]
        },
        {
            title: "Detailed Design and Performance Analysis",
            navName: "Detailed Design and Analysis",
            navRef: "detailed-design",
            content: [
                {
                    type: "text",
                    content:
                        "The storage subsystem uses a force sensor to measure liquid levels, with an equation fitted to observed values. Finite Element Analysis (FEA) was conducted on the weighing platform and main weighing plate to ensure structural integrity, and confirmed that the design can withstand the expected loads without excessive deformation. A shooting analysis was performed on different nozzle configurations, with a sprayer nozzle being selected for the final design. This configuration provided an average flow rate of 40mL per second, with a burst volume of 48mL."
                },
                {
                    type: "text",
                    content:
                        "The target identification subsystem initially used a CNN, but the team switched to a YOLO model to improve execution time. While the CNN model had a sub 90 millisecond execution time, its implementation using a selective search algorithm had an unacceptable execution time of 15 seconds. The YOLO model provided a better execution time of 500 milliseconds and also proved to be spatially more accurate. The tracking subsystem analysis involved calculating the minimum torque required for the stepper motors and the electrical power requirements. A factor of safety of 6 was achieved with a 60 Ncm torque. The power requirements were determined to be 12V and 1.5A, guided by IEEE 1100-2005 standards to ensure proper power and grounding."
                },
                 {
                    type: "image",
                   content: "/media/images/sap_yolo_output.png",
                   altContent: "YOLO model output",
                   subtitle: "Output from the YOLO model."
                 },
                 {
                    type: "text",
                    content:
                         "A Failure Mode and Effects Analysis (FMEA) was conducted to identify potential failure points and their impacts. Key failure points included the tracking, rotation, and shooting mechanisms. Mitigation strategies involved adding a feedback loop to the tracking system, implementing waterproofing for electrical components, and adding filters to the pump and hoses to prevent blockages."
                }
            ]
        },
        {
          title: "Manufacturing and Assembly",
          navName: "Manufacturing and Assembly",
           navRef: "manufacturing-assembly",
            content: [
                {
                    type: "text",
                    content:
                        "The robot prototype was constructed using laser-cut acrylic plates and 3D-printed components. A 'dovetail joint' method was used to assemble the acrylic plates, eliminating the need for screws or nails. A detailed bill of materials (BOM) was created to keep track of all components and their associated costs. In line with Design for Manufacturability (DFM) principles, mass-produced versions of the robot would benefit from a metal mold with injection molding, which would reduce both material and installation costs. This mass-produced version would also integrate printed circuit boards (PCBs) for improved reliability and simplified assembly."
                },
                  {
                    type: "image",
                    content: "/media/images/sap_dovetail_joint.png",
                    altContent: "Dovetail joint method",
                     subtitle: "Dovetail joint method used in the construction of the prototype."
                 },
                {
                    type: "text",
                    content:
                        "Challenges in mass production included initial costs, ensuring consistency in manufacturing, and managing the supply chain. A key challenge is the high upfront costs associated with injection molding and any changes made to the mold would also be difficult and costly. The team recognized the importance of securing and potentially self-producing the supply of non-self produced components to maintain competitive pricing."
                },
            ]
        },
          {
            title: "Prototype Demonstration and Testing",
            navName: "Final Prototype and Demonstration",
            navRef: "final-prototype",
            content: [
                 {
                    type: "text",
                    content:
                        "Our design took into account environmental, public safety, and ethical considerations. The robot was programmed to use a penalty-based approach, reducing scores for targets near humans or electronics, to avoid unintended actions. The team also tested to determine the minimum amount of pesticide needed. The design was built with an understanding of the negative impact lantern-flies have on the community and sought to address this problem."
                },
                  {
                    type: "image",
                    content: "/media/images/sap_penalty_demo.png",
                    altContent: "Penalty scoring demo",
                     subtitle: "Demonstration of the penalty system for unwanted targets."
                 },
                {
                   type: "text",
                   content: "The final prototype was tested sequentially for its weight sensing, tracking, and shooting capabilities. Initial tests confirmed that the weight sensing was functional simultaneously with the camera. Tracking tests, conducted with different target locations and multiple targets, showed that the robot was able to mostly track its targets, though with some horizontal offsets. The right side of the camera showed reduced accuracy, resulting in only a 20% success rate compared to 60% overall. Shooting tests, performed after successful target tracking, required a manual offset to achieve consistent hits. The average hit rate during testing was 4 out of 5 targets."
                }
            ]
        },
        {
            title: "Conclusion and Lessons Learned",
             navName: "Conclusion and Lessons Learned",
             navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content:
                         "The development of the SAP project was a valuable learning experience. We realized the importance of starting with a well-defined design, as the team faced challenges throughout the development cycle due to not having an initially concise design. Further initial research would have been beneficial in selecting hardware such as the Raspberry Pi and stereo cameras. Additionally, we learned the importance of sticking closely to the original Gantt chart. When integrating the motor drivers, the team struggled to identify why the motors weren't functioning, which was later found to be due to insufficient amperage. Through this project, the team gained a deeper understanding of the need for meticulous design, thorough research, and preemptive technical understanding for smoother project execution."
                }
            ]
        }
    ]
};