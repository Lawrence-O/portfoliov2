import { Project} from "@/app/components/project/interfaces";

export const safeguardAgainstPests: Project = {
    title: "Safeguard Against Pests (SAP)",
    subtitle: "Electromechanical Systems Design Project",
    date: "Fall 2023",
    media: "/media/videos/sap_front_view.mp4",
    tags: ["Robotics", "Computer Vision", "Embedded Systems", "CAD"],
    section: [
        {
            title: "Project Overview",
            navName: "Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content:
                        "SAP is an autonomous pest control robot designed to detect and spray lanternflies—an invasive species that threatens agriculture and gardens. The robot uses computer vision to identify pests, a motorized turret to aim, and a pump system to spray targeted amounts of pesticide.",
                },
                {
                    type: "text",
                    content:
                        "Traditional pest control relies on broad chemical application, which affects beneficial insects and the environment. SAP addresses this by only spraying when a pest is detected, reducing pesticide use while maintaining effectiveness.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Key Features",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**YOLO-based detection** — real-time pest identification using machine learning",
                        "**Motorized turret** — pan/tilt tracking to aim at detected targets",
                        "**Targeted spray** — electric pump delivers precise amounts of liquid",
                        "**Safety penalties** — avoids spraying near humans or electronics",
                    ],
                },
                {
                    type: "image",
                    content: "/media/images/sap_prototype.png",
                    altContent: "SAP prototype",
                    subtitle: "The completed SAP prototype.",
                },
            ],
        },
        {
            title: "Problem Definition",
            navName: "Problem Definition",
            navRef: "problem-definition",
            content: [
                {
                    type: "text",
                    content:
                        "Existing pest control robots are mostly in prototype stages, with limited options for home garden use. Competitors include CO2-emitting mosquito traps, suction-based devices, and laser deterrents—each with limitations in accuracy, range, or computational power.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Design Requirements",
                },
                {
                    type: "text",
                    content:
                        "Customer surveys identified key needs: autonomous operation, environmental friendliness, accuracy, and sufficient range. These translated into target specifications:",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Liquid storage** — minimum capacity for extended operation",
                        "**Efficiency** — minimize liquid used per insect",
                        "**Battery life** — sufficient for typical use sessions",
                        "**Accuracy** — minimum hit rate for detected pests",
                        "**Range** — effective shooting distance for garden coverage",
                    ],
                },
            ],
        },
        {
            title: "System Design",
            navName: "System Design",
            navRef: "concept-generation",
            content: [
                {
                    type: "text",
                    content:
                        "The system breaks down into four core subsystems: liquid storage, target identification, target tracking, and shooting mechanism. Each was designed and tested independently before integration.",
                },
                {
                    type: "image",
                    content: "/media/images/sap_functional_decomposition.png",
                    altContent: "Functional decomposition diagram",
                    subtitle: "Functional decomposition of the SAP system.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Design Decisions",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Detection** — YOLO model chosen over CNN for faster execution (500ms vs 15s with selective search)",
                        "**Tracking** — rotating platform with pitch control for pan/tilt aiming",
                        "**Shooting** — electric pump selected for consistent power and simpler implementation than pressurized air",
                        "**Level sensing** — force sensor measures liquid weight to track remaining capacity",
                    ],
                },
            ],
        },
        {
            title: "Technical Analysis",
            navName: "Analysis",
            navRef: "detailed-design",
            content: [
                {
                    type: "text",
                    content:
                        "Each subsystem underwent detailed analysis. The storage system uses a fitted equation to convert force sensor readings to liquid volume. FEA confirmed the weighing platform can handle expected loads without excessive deformation.",
                },
                {
                    type: "text",
                    content:
                        "The YOLO model achieved 500ms execution time—a significant improvement over the initial CNN approach which took 15 seconds with selective search. Tracking motors were sized with a factor of safety of 6, requiring 60 Ncm torque at 12V/1.5A.",
                },
                {
                    type: "image",
                    content: "/media/images/sap_yolo_output.png",
                    altContent: "YOLO detection output",
                    subtitle: "YOLO model detecting lanternflies in real-time.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Failure Analysis (FMEA)",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Tracking failure** — mitigated with feedback loop for closed-loop control",
                        "**Water damage** — waterproofing added for electrical components",
                        "**Pump blockage** — filters added to pump and hoses",
                    ],
                },
            ],
        },
        {
            title: "Manufacturing",
            navName: "Manufacturing",
            navRef: "manufacturing-assembly",
            content: [
                {
                    type: "text",
                    content:
                        "The prototype was built using laser-cut acrylic plates and 3D-printed components. Acrylic pieces connect using dovetail joints, eliminating the need for screws or adhesives.",
                },
                {
                    type: "image",
                    content: "/media/images/sap_dovetail_joint.png",
                    altContent: "Dovetail joint assembly",
                    subtitle: "Dovetail joint method used for assembly.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Design for Manufacturing",
                },
                {
                    type: "text",
                    content:
                        "For mass production, the design would transition to injection-molded parts and integrated PCBs. This reduces per-unit cost but requires significant upfront tooling investment. Mold changes are expensive, so the design must be finalized before tooling.",
                },
            ],
        },
        {
            title: "Testing Results",
            navName: "Testing",
            navRef: "final-prototype",
            content: [
                {
                    type: "text",
                    content:
                        "The robot uses a penalty-based targeting system that reduces scores for targets near humans or electronics, preventing unintended spray. Testing determined the minimum pesticide amount needed for effectiveness.",
                },
                {
                    type: "image",
                    content: "/media/images/sap_penalty_demo.png",
                    altContent: "Penalty scoring demonstration",
                    subtitle: "Penalty system reduces priority for targets near protected areas.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Test Results",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Weight sensing** — functional and operates simultaneously with camera",
                        "**Tracking** — 60% overall accuracy, reduced to 20% on right side of camera view",
                        "**Shooting** — 4 out of 5 targets hit after manual offset calibration",
                    ],
                },
            ],
        },
        {
            title: "Conclusion",
            navName: "Conclusion",
            navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content:
                        "The project demonstrated a working prototype for autonomous pest detection and targeted spraying. Key lessons included the importance of early hardware research (the team encountered issues with Raspberry Pi power requirements and motor driver amperage) and maintaining the project schedule.",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Design iteration** — initial CNN replaced with YOLO for performance",
                        "**Hardware selection** — insufficient amperage caused motor issues, resolved by proper driver selection",
                        "**Calibration needs** — tracking offset required manual adjustment for accurate shooting",
                    ],
                },
            ],
        },
    ],
};