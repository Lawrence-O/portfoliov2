import { Project } from "@/app/components/project/interfaces";

export const gripAssistiveGlove: Project = {
    title: "Grip Assistive Glove",
    subtitle: "Design 1 Project - Spring 2023",
    media: "/media/videos/gripAssitiveGlove.mp4",
    tags: ["Assistive Technology", "Mechanical Design", "CAD", "FEA", "Prototyping", "Product Design"],
    section: [
        {
            title: "Project Overview",
            navName: "Project Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content:
                        "The Grip Assistive Glove project was undertaken to address the challenges faced by individuals with grip-related conditions, such as Cerebral Palsy and Tendonitis. Our goal was to create an affordable and effective assistive device that enhances grip strength, enabling users to perform daily tasks with greater ease. The project involved modifying a standard leather glove, integrating a mechanism powered by a motor and fishing lines to mimic the natural gripping motion. By focusing on accessibility and functionality, we aimed to significantly improve the quality of life for our target user population."
                },
                {
                    type: "image",
                    content: "/media/images/gripglove_assembly.png",
                    altContent: "Assembly of the Grip Assistive Glove",
                    subtitle: "The final assembly and bill of materials for the Grip Assistive Glove."
                },
            ]
        },
        {
            title: "Early Design Concepts",
            navName: "Early Design Concepts",
            navRef: "early-ideation",
            content: [
                {
                    type: "text",
                    content:
                        "The initial phase of the project focused on exploring different mechanisms to achieve assisted grip. We brainstormed various concepts, including designs with rings at the finger joints connected by fishing lines to a motor. Early iterations included designs where all lines converged at the palm, as well as designs that connected the lines in sections for enhanced grip functionality. The team initially considered a two-motor system, but ultimately decided to use a single motor to reduce complexity and bulk."
                },
                {
                   type: "image",
                   content: "/media/images/gripGlove_initial_sketches.png",
                    altContent: "Initial sketches of the Grip Assistive Glove",
                    subtitle: "Early sketches exploring different design approaches."
                 },
                 {
                    type: "text",
                    content:
                        "During this ideation phase, we realized the critical need for a mechanism to help open the hand after gripping. We identified that maintaining tension on the hand via the fishing line during use would be important. The team explored the possibility of using elastic to assist with hand release, but ultimately rejected this idea to ensure normal finger movement while wearing the glove."
                }
            ]
        },
        {
            title: "Refined Design and Stress Analysis",
            navName: "Refined Design and Stress Analysis",
            navRef: "developed-ideation",
             content: [
               {
                    type: "text",
                     content:
                        "The developed ideation phase focused on refining our initial concepts. The team decided on a design that used a single motor to pull all the fingers, and the thumb separately, as we wanted to reduce the complexity of the design for ease of user interaction. We removed the elastic component on the back of the glove, allowing the user to manually open their hand and unwind the string, ensuring no interference with natural movement."
                },
                 {
                    type: "text",
                    content:
                         "To validate the design, we performed a stress analysis assuming a stall torque of 25 Nm for the motor. The analysis focused on the nodes attached to the fingers and identified the reaction forces opposing the string tension. Given the fishing line was rated for 40 pounds, the team confirmed it was sufficient to withstand the forces exerted by the motor."
                }
            ]
        },
        {
             title: "CAD Model Development",
              navName: "CAD Prototype",
             navRef: "cad-prototype",
            content: [
                {
                    type: "text",
                    content:
                         "The refined design was meticulously modeled using CAD software, creating a detailed product assembly, finger node subassembly, and motor spool subassembly. The fishing lines run through each node on each individual finger, terminating at the fingertips. At the palm, all lines converge at a single node, connected to the motor spool. The motor, controlled by 'wind' and 'unwind' buttons, rotates to pull and release the strings. The motor and spool assembly are mounted on the forearm."
                },
                   {
                     type: "image",
                    content: "/media/images/gripGlove_glove_node.png",
                    altContent: "CAD model of the finger node",
                     subtitle: "CAD model illustrating the finger nodes."
                   },
                    {
                     type: "image",
                     content: "/media/images/gripGlove_glove_spool.png",
                     altContent: "CAD model of the motor spool",
                    subtitle: "CAD model illustrating the motor spool."
                    }
            ]
        },
        {
           title: "Finite Element Analysis (FEA)",
           navName: "Finite Element Analysis (FEA)",
            navRef: "testing-fea",
            content: [
                {
                    type: "text",
                    content:
                        "To assess the structural integrity of our design, we performed Finite Element Analysis (FEA) on the node subassembly. We simulated the stress and displacement experienced by the nodes with a force of 25 N. The analysis indicated minimal stress and negligible displacement, thus confirming our hand calculations and providing a more precise representation of stress concentrations. "
                },
                 {
                    type: "image",
                     content: "/media/images/gripGlove_fea_analysis.png",
                    altContent: "FEA Stress Analysis",
                     subtitle: "FEA analysis visualizing the stress distribution."
                },
                {
                    type: "image",
                     content: "/media/images/gripGlove_fea_displacement.png",
                      altContent: "FEA displacement analysis",
                     subtitle: "FEA analysis visualizing the displacement."
                },
            ]
        },
        {
            title: "Manufacturing and Cost Analysis",
             navName: "Manufacturing and Cost Analysis",
            navRef: "manufacturing-drawing",
            content: [
                {
                    type: "text",
                    content:
                        "A comprehensive manufacturing plan was developed, specifying materials for each component. We determined that PLA plastic was ideal for the finger nodes and the palm node due to its low cost and sufficient yield strength. These components would be 3D printed, with a per-unit cost estimate. The motor spool was also designed for 3D printing using PLA. For mass production, injection molding was considered to further reduce costs. Based on our estimates, the total manufacturing cost per glove is $35.61."
                },
                {
                    type: "text",
                    content:
                        "The cost breakdown includes $11 for the leather glove ($6.50 per glove), $0.11 for the wire, $0.50 for the palm node, $2.50 for the motor spool, and $26 for the electronic components, including the motor. Adding a 30% markup to the manufacturing cost, the final estimated retail price for the grip assistive glove would be around $46.30, positioning it as an affordable solution."
                },
            ]
        },
        {
            title: "Project Budget",
            navName: "Project Budget",
            navRef: "budget",
            content: [
                {
                    type: "text",
                    content: "The project budget included the cost of leather gloves, push button switches, and fishing line, all purchased through Amazon. The total expenditure on these materials was $43.94, leaving a remaining budget of $156.06."
                 },
            ]
        }
    ]
};