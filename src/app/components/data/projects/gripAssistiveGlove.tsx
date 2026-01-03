import { Project } from "@/app/components/project/interfaces";

export const gripAssistiveGlove: Project = {
    title: "Grip Assistive Glove",
    date: "Design 1 Project - Spring 2023",
    media: "/media/videos/gripAssitiveGlove.mp4",
    githubLink: "https://github.com/your-username/grip-assistive-glove",
    tags: ["Assistive Technology", "Mechanical Design", "CAD", "FEA", "Prototyping", "Product Design"],
    section: [
        {
            title: "Project Overview",
            navName: "Project Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content: [
                        "The Grip Assistive Glove project was undertaken to address the challenges faced by individuals with grip-related conditions, such as Cerebral Palsy and Tendonitis. Our primary goal was to create an affordable and effective assistive device designed to enhance grip strength, thereby enabling users to perform daily tasks with greater ease and independence.",
                        "The project involved the thoughtful modification of a standard leather glove, integrating a carefully designed mechanism. This mechanism, developed using CAD software (e.g., SolidWorks/Fusion 360), is powered by a motor and utilizes fishing lines to mimic the natural gripping motion of the hand. By focusing on accessibility, functionality, and user-centric design, we aimed to significantly improve the quality of life for our target user population."
                    ]
                },
                {
                    type: "image",
                    content: "/media/images/gripglove_assembly.png",
                    altContent: "Assembly of the Grip Assistive Glove",
                    subtitle: "The final assembly and bill of materials for the Grip Assistive Glove."
                }
            ]
        },
        {
            title: "Early Design Concepts and Ideation",
            navName: "Early Concepts",
            navRef: "early-ideation",
            content: [
                {
                    type: "text",
                    content: [
                        "The initial phase of the project was dedicated to exploring a variety of mechanisms to achieve effective assisted grip. We brainstormed numerous concepts, including designs featuring rings at the finger joints connected by fishing lines to a central motor.",
                        "Early iterations explored configurations where all lines converged at the palm, as well as designs that connected lines in distinct sections for potentially enhanced grip functionality and control. The team initially considered a two-motor system for independent finger articulation but ultimately opted for a single motor. This decision aimed to reduce complexity, weight, and cost, striking a balance between advanced functionality and crucial factors like user accessibility and ease of manufacturing."
                    ]
                },
                {
                   type: "image",
                   content: "/media/images/gripGlove_initial_sketches.png",
                    altContent: "Initial sketches of the Grip Assistive Glove",
                    subtitle: "Early sketches exploring different design approaches and mechanisms."
                 },
                 {
                    type: "text",
                    content: "During this ideation phase, we identified the critical need for a mechanism to assist in opening the hand after a grip was performed. While maintaining tension on the hand via the fishing line during use was important for grip strength, the team explored using elastic elements to aid in hand release. However, this idea was ultimately rejected to ensure that the glove did not impede normal, unassisted finger movement when the assistive feature was not active."
                }
            ]
        },
        {
            title: "Refined Design and Mechanical Analysis",
            navName: "Refined Design",
            navRef: "developed-ideation",
             content: [
               {
                    type: "text",
                     content: [
                        "The developed ideation phase focused on refining our initial concepts into a more robust and user-friendly design. The team converged on a design utilizing a single motor to actuate all fingers collectively, with the thumb potentially having a separate or integrated line for opposition. This approach simplified the design, enhancing ease of user interaction.",
                        "A key refinement was the removal of the elastic component previously considered for the back of the glove. Instead, the design allows the user to manually open their hand, which in turn unwinds the string from the spool, ensuring no interference with natural hand movement."
                    ]
                },
                 {
                    type: "text",
                    content: [
                         "To validate the mechanical integrity of the design, a stress analysis was performed. Assuming a stall torque of 25 Nm for the motor (representing a worst-case scenario for force exerted on the components), the analysis focused on the reaction forces experienced at the finger nodes.",
                         "This ensured that the chosen fishing line, rated for 40 pounds (approximately 178 N), possessed a sufficient factor of safety against the maximum forces exerted by the motor during a full grip. A simplified governing principle for calculating line tension can be expressed as `F_line = (\\tau_motor / r_spool) * \\eta`, where `\\tau_motor` is the motor torque, `r_spool` is the radius of the motor spool, and `\\eta` represents an efficiency factor for the system."
                    ]
                }
            ]
        },
        {
             title: "CAD Model Development and Control System",
              navName: "CAD & Controls",
             navRef: "cad-prototype",
            content: [
                {
                    type: "text",
                    content: [
                         "The refined design was meticulously modeled using CAD software (e.g., SolidWorks), resulting in a detailed product assembly. This included subassemblies for the finger nodes and the motor spool. The fishing lines are routed through nodes on each individual finger, terminating at the fingertips.",
                         "At the palm, all lines converge at a single collection node, which is then connected to the motor spool. A geared DC motor, controlled by simple 'wind' and 'unwind' buttons, rotates the spool to pull and release the strings, thereby actuating the grip. The entire motor and spool assembly is designed to be mounted on the user's forearm for ergonomic use."
                    ]
                },
                {
                    type: "text",
                    content: "Control System Overview",
                    displayAs: "subtitle"
                },
                {
                    type: "code",
                    codeLang: "plaintext",
                    content:
`Algorithm: Grip Assistive Glove Control
1. Initialize motor_state = IDLE
2. LOOP indefinitely:
3.     IF wind_button_pressed AND motor_state != WINDING:
4.         motor_state = WINDING
5.         Activate motor to pull fishing lines (tense fingers)
6.     ELSE IF unwind_button_pressed AND motor_state != UNWINDING:
7.         motor_state = UNWINDING
8.         Activate motor to release fishing lines (relax fingers)
9.     ELSE IF no_button_pressed OR (wind_button_pressed AND motor_state == WINDING) OR (unwind_button_pressed AND motor_state == UNWINDING):
10.        IF motor_state != IDLE: // (and not at limit if limit switches are used)
11.            Continue current motor action (or hold if at limit)
12.        ELSE:
13.            motor_state = IDLE
14.            Deactivate motor
15.    ENDIF
16.    // Optional: Add limit switch checks to stop motor at full grip/release
17. ENDLOOP`,
                    subtitle: "Control Logic Pseudocode"
                },
                   {
                     type: "image",
                    content: "/media/images/gripGlove_glove_node.png",
                    altContent: "CAD model of the finger node",
                     subtitle: "CAD model illustrating the design of the finger nodes."
                   },
                    {
                     type: "image",
                     content: "/media/images/gripGlove_glove_spool.png",
                     altContent: "CAD model of the motor spool",
                    subtitle: "CAD model illustrating the motor spool assembly."
                    }
            ]
        },
        {
           title: "Finite Element Analysis (FEA) Validation",
           navName: "FEA",
            navRef: "testing-fea",
            content: [
                {
                    type: "text",
                    content: [
                        "Finite Element Analysis (FEA) was employed as a crucial step to digitally simulate and validate how the 3D-printed finger nodes would behave under operational loads. By applying a simulated force of 25 N (representative of typical grip assistance requirements) to the CAD models of the nodes, the FEA software calculated detailed stress distributions and potential deformations.",
                        "The analysis results were highly encouraging, indicating minimal stress levels—well below the yield strength of PLA material—and negligible displacement. This computationally validated our initial hand calculations and provided a more detailed understanding of stress concentrations, ultimately confirming the structural robustness of the 3D-printed PLA nodes for the intended application."
                    ]
                },
                 {
                    type: "image",
                     content: "/media/images/gripGlove_fea_analysis.png",
                    altContent: "FEA Stress Analysis of Finger Node",
                     subtitle: "FEA results visualizing stress distribution on a finger node."
                },
                {
                    type: "image",
                     content: "/media/images/gripGlove_fea_displacement.png",
                      altContent: "FEA displacement analysis of Finger Node",
                     subtitle: "FEA results visualizing displacement on a finger node under load."
                }
            ]
        },
        {
            title: "Manufacturing Strategy and Cost Analysis",
             navName: "Manufacturing & Cost",
            navRef: "manufacturing-drawing",
            content: [
                {
                    type: "text",
                    content: [
                        "A comprehensive manufacturing plan was developed, specifying materials and processes for each component. We determined that PLA plastic was an ideal material for the finger nodes and the palm node due to its low cost, ease of 3D printing, and sufficient yield strength for this application.",
                        "These components would be manufactured using Fused Deposition Modeling (FDM) 3D printing for initial prototyping and small-batch production, allowing for rapid iteration and customization. The motor spool was also designed for 3D printing using PLA. For potential mass production scenarios, transitioning to injection molding for these plastic parts was considered as a means to significantly reduce per-unit costs."
                    ]
                },
                {
                    type: "text",
                    content: "Component Costs", // Shortened subtitle
                    displayAs: "subtitle"
                },
                {
                    type: "text",
                    displayAs: "list",
                    orderedList: false,
                    content: [
                        "Leather glove: $11.00 (or $6.50 per glove if sourced in bulk)",
                        "Fishing Wire: $0.11",
                        "Palm node (3D printed PLA): $0.50",
                        "Motor spool (3D printed PLA): $2.50",
                        "Electronic components (motor, buttons, wiring, etc.): $26.00"
                    ]
                },
                {
                    type: "text",
                    content: "Adding a 30% markup to the total manufacturing cost (approximately $40.11), the final estimated retail price for the Grip Assistive Glove is around $52.14. This positions the device as a comparatively affordable solution in the assistive technology market."
                }
            ]
        },
        {
            title: "Project Budget Summary",
            navName: "Budget",
            navRef: "budget",
            content: [
                {
                    type: "text",
                    content: ["The project budget primarily covered the cost of materials such as leather gloves, push button switches, and fishing line, all of which were procured through Amazon. The total expenditure on these essential materials amounted to $43.94. This left a remaining budget of $156.06 from the initial allocation, demonstrating cost-effective resource management throughout the project's development."]
                 }
            ]
        }
    ]
};
