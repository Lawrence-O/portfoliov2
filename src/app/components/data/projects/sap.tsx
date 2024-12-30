import { Project} from "@/app/components/project/interfaces";

export const safeguardAgainstPests: Project = {
    title: "Safeguard Against Pests (SAP)",
    subtitle: "Electromechanical Systems Design Project - Fall 2023",
    media: "/media/images/safeguardAgainstPests.png",
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
                        "The Safeguard Against Pests (SAP) project addresses the growing concern of pest infestations, particularly focusing on lantern flies. These pests pose a threat to agriculture and home gardens. Traditional methods often rely on harmful chemicals, leading to environmental and health risks. The SAP project aims to create an efficient, sustainable, and autonomous pest control solution. This project combines robotics, computer vision, and embedded systems to create a pest control robot."
                },
                {
                    type: "image",
                    content: "/media/images/sap_prototype.png",
                    altContent: "Image of the SAP prototype",
                    subtitle: "The final SAP prototype."
                },
            ]
        },
        {
            title: "Problem Definition & Market Analysis",
            navName: "Problem Definition & Market Analysis",
            navRef: "problem-definition",
            content: [
                {
                    type: "text",
                    content:
                        "The project identified a clear need for a pest control solution that is both effective and environmentally conscious. Current market options are limited, with many still in the prototype phase. Competitors include mosquito control robots using CO2 emissions and suction, agricultural robots designed for large-scale pest control, and laser deterrents with limited computational power. SAP stands out by focusing on targeted pesticide application, utilizing advanced computer vision and autonomous operation suited for home garden use. Our survey data confirmed that current methods are not as accurate and have a limited range. Our design was built to overcome these limitations, ensuring it was a product that could compete with conventional methods. "
                },
                 {
                    type: "image",
                    content: "/media/images/market_competitors.png",
                    altContent: "Competitor products",
                    subtitle: "A few of the competitors analyzed."
                 },
                  {
                    type: "text",
                     content: "Stakeholders for this project include homeowners, gardeners, local communities, and environmental agencies. The team conducted surveys to gather customer needs, which included autonomy, environmental friendliness, accuracy, and range. The needs were prioritized as either basic needs or performance needs."
                },
                  {
                    type: "text",
                     content: "Target specifications were established based on these needs and survey results. The specifications included minimum liquid storage, maximum liquid used per insect, minimum battery life, minimum accuracy rate, and minimum shooting range. A competitive analysis was performed comparing our target specifications to conventional methods of pest control and ideal values. The ideal values were found through comparing retail products and customer feedback."
                 },
                 
            ]
        },
         {
            title: "Concept Generation",
            navName: "Concept Generation",
            navRef: "concept-generation",
             content: [
               {
                    type: "text",
                    content: "The design process began with a functional decomposition, identifying key subsystems: Store Materials, Identify Targets, Track Targets, and Shooting Liquids. Each subsystem was further broken down into its main functions. The system was designed with an electric pump for dispensing liquid. For the tracking mechanism, a rotating base is used along with pitch controls. For the storage subsystem, we use a force sensor to determine the liquid levels in the container and a simple screw cap is used for refilling."
                },
                {
                   type: "image",
                   content: "/media/images/functional_decomposition.png",
                   altContent: "Functional Decomposition Diagram",
                   subtitle: "Functional decomposition tree diagram."
                },
                 {
                    type: "text",
                    content:
                        "For identifying targets, the team explored various methods, including Convolutional Neural Networks (CNN), Bag of Words approach with K-Nearest Neighbors, template matching, and color matching. Ultimately, a CNN was chosen for its superior performance and robustness to environmental variables, and later switched to a more efficient YOLO model. For tracking targets, concepts included a rotating cylinder with tilt control and a rotating base with a linear actuator. A combination of these two concepts was used where there was a rotating platform with pitch controls. Shooting mechanisms explored included an electric pump and a pressurized air system; the electric pump was selected for its consistent shooting power and ease of implementation."
                },
                {
                   type: "image",
                   content: "/media/images/concept_selection.png",
                   altContent: "Concept Selection Table",
                   subtitle: "The final concept selection table."
                },

            ]
        },
        {
            title: "Detailed Design and Analysis",
            navName: "Detailed Design and Analysis",
            navRef: "detailed-design",
            content: [
                {
                    type: "text",
                    content:
                        "The storage subsystem uses a force sensor to measure liquid levels, combined with an equation fitted using observed values.  Finite Element Analysis (FEA) was conducted on the weighing platform and main weighing plate to ensure structural integrity, confirming that the design could withstand the loads without excessive deformation. A shooting analysis was performed with different nozzle configurations, with the final design using a sprayer nozzle. The average flow rate was 40mL per second, with a burst volume of 48mL."
                },
                 {
                    type: "image",
                   content: "/media/images/shooting_analysis.png",
                   altContent: "Shooting analysis of different nozzles",
                   subtitle: "Shooting analysis with different nozzles."
                },
                {
                    type: "text",
                    content:
                        "The identification subsystem initially used a CNN for object detection, but transitioned to a YOLO model to improve execution time. The CNN model had a sub 90 millisecond execution time, but the selective search algorithm had an execution time of 15 seconds making it inadequate. The YOLO model provided a better execution time of 500 milliseconds, along with being more accurate spatially. The tracking subsystem analysis involved calculations to determine the minimum torque needed for the stepper motors and electrical power requirements. A factor of safety of 6 was achieved with a 60 Ncm torque. The power requirements were determined to be 12V and 1.5A. The analysis was guided by IEEE 1100-2005 standards to ensure proper power and grounding. "
                },
                 {
                    type: "image",
                   content: "/media/images/yolo_output.png",
                   altContent: "YOLO model output",
                   subtitle: "Output from the YOLO Model"
                 },
                {
                   type: "code",
                   codeLang: "java",
                   content: `
                   /**
                    * Calculates the required torque for the stepper motor.
                    * @param weight The weight of the components being moved in kilograms.
                    * @param distance The distance from the center of rotation in meters.
                    * @return The torque required in Newton-centimeters (Ncm).
                    */
                    public double calculateTorque(double weight, double distance) {
                        double gravity = 9.81; // Acceleration due to gravity in m/s^2
                        double moment = weight * gravity * distance; // Newton-meters
                        double torque = moment * 100; // Convert to Ncm
                        return torque;
                    }
                   `
                  ,
                  subtitle: "Example of calculations"
                },

                {
                    type: "text",
                    content:
                        "A Failure Mode and Effects Analysis (FMEA) was conducted to identify potential failure points and their impacts. The main failure points included tracking, rotation, and shooting. Mitigation strategies involved adding a feedback loop to the tracking system to prevent the robot from continuing to track the same target.  Waterproofing around the electrical components was added to mitigate issues due to water damage. Filters were also added to the pump and hoses to prevent blockage."
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
                         "The robot prototype was constructed using laser-cut acrylic plates and 3D-printed components. A “dovetail joint” method was used to connect the acrylic plates without the need for screws or nails. A bill of materials (BOM) was created to record the different parts and their associated costs.  Design for Manufacturability (DFM) principles were considered, suggesting that a metal mold with injection molding should be used for the mass produced version.  This method would help lower material and installation costs.  The mass produced version would also have printed circuit boards (PCBs) to better integrate the electronics, which improves reliability and simplifies assembly."
                },
                   {
                    type: "image",
                    content: "/media/images/dovetail_joint.png",
                    altContent: "Dovetail joint method",
                     subtitle: "Installation of the dovetail joints."
                 },
                {
                    type: "text",
                    content:
                         "Challenges in mass production included initial costs, ensuring consistency, and managing supply chains. A key challenge is the high upfront costs of injection molding, and any changes to the mold are difficult and costly. The team acknowledged the importance of securing and potentially self-producing the supply of non-self produced components to maintain competitive pricing."
                },
            ]
        },
          {
            title: "Final Prototype and Demonstration",
            navName: "Final Prototype and Demonstration",
            navRef: "final-prototype",
            content: [
                  {
                    type: "text",
                    content:
                         "The design took into account environmental, public safety, and ethical considerations. The robot was programmed to use a penalty-based approach, that would reduce scores of targets near humans or electronics, to avoid shooting at unwanted objects. The minimal amount of pesticide to be dispersed was also tested. The design seeks to address the negative impacts of lantern-flies in the community."
                },
                {
                    type: "image",
                    content: "/media/images/target_demo.png",
                    altContent: "Target scoring demo",
                     subtitle: "Demonstration of the scoring for target selection."
                 },
                  {
                    type: "image",
                    content: "/media/images/penalty_demo.png",
                    altContent: "Penalty scoring demo",
                     subtitle: "Demonstration of the penalty system for unwanted targets"
                 },
                  {
                     type: "video",
                     content: "/media/videos/sap_demo.mp4",
                     altContent: "Video of the SAP robot working",
                     subtitle: "Video of the prototype in action"
                  },
                {
                   type: "text",
                   content: "The final prototype was tested for weight sensing, tracking, and shooting. All of these tests were performed sequentially. Initial tests confirmed that the weight sensing worked simultaneously with the camera. The tracking tests varied by different locations of the target and having multiple targets. The robot was able to mostly follow the targets with some horizontal offsets. The right side of the camera was less accurate, resulting in only 20% of the attempts being successful compared to 60% of attempts overall. The shooting tests, which were done only on targets that were successfully tracked, required a manual offset for consistent hits. The average hit rate was 4/5 targets."
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
                         "The development of the SAP project was a learning experience. The team learned the importance of a concise design from the beginning, which resulted in the team having to catch up throughout the development cycle. More initial research would have been beneficial when choosing certain hardware, such as the Raspberry Pi and stereo cameras. The team would have also benefitted from sticking more closely to the original Gantt chart. When integrating the motor drivers, the team had a difficult time identifying why the motors weren't functioning, which was later found to be due to not having sufficient amperage. Through this project, the team found the importance of meticulous design, thorough research, and preemptive technical understanding for smoother project execution."
                }
            ]
        }
    ]
};