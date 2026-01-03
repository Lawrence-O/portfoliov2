import { Project } from "@/app/components/project/interfaces";

export const habitatAirlockLinkage: Project = {
    title: "Habitat's Airlock Linkage System",
    date: "Engineering Design I: Skills and Methods - Spring 2023",
    media: "/media/videos/habitat_linkage.mp4",
    githubLink: "https://github.com/your-username/habitat-airlock-linkage",
    tags: ["Mechanical Design", "Linkage System", "CAD", "FEA", "Prototyping", "Optimization"],
    section: [
        {
            title: "Project Overview",
            navName: "Project Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content: [
                        "The Habitat's Airlock Linkage System project, developed using CAD software (e.g., SolidWorks), aimed to design and optimize a mechanical linkage system, such as a modified four-bar linkage, capable of actuating a button within a specified timeframe.",
                        "This project challenged us to integrate iterative design methodologies, rigorous engineering analysis, and comprehensive testing protocols to enhance the system's overall performance. The primary focus areas included minimizing mass, reducing stress concentrations through careful design and FEA, and ensuring reliable, repeatable operation, with the ultimate goal of creating a robust and effective mechanical system."
                    ]
                },
                {
                    type: "image",
                     content: "/media/images/airlock_final_testing.png",
                     altContent: "Final testing of the linkage system.",
                     subtitle: "The linkage system undergoing its final performance testing."
                 },
            ]
        },
        {
            title: "Performance Results and Key Improvements",
            navName: "Results & Improvements",
            navRef: "summary-results",
            content: [
                {
                    type: "text",
                    content: [
                         "Throughout the project, we conducted both intermediate and final performance tests, each requiring the linkage system to actuate a target button within 30 seconds. In the intermediate test, our system achieved an experimental button press time of 27.55 seconds. The final test yielded an improved time of 25.91 seconds, with a slight variance attributed to a minor assembly imprecision.",
                         "Key improvements implemented during the design process included filleting the edges of the hexagonal hole on a critical link to significantly reduce stress concentrations. Furthermore, we focused on mass optimization, successfully reducing the weight of the links by an impressive 72.4% (e.g., from an initial X grams to a final Y grams). This not only improved material efficiency but also contributed to faster actuation times due to reduced inertia."
                    ]
                },
                {
                     type: "image",
                     content: "/media/images/airlock_intermediate.png",
                    altContent: "Airlock Linkage System Intermediate Testing",
                     subtitle: "The linkage system during intermediate performance testing."
                  },
            ]
        },
        {
             title: "Initial Design and Brainstorming Phase",
             navName: "Initial Design",
            navRef: "brainstorming-design",
             content: [
               {
                    type: "text",
                     content: [
                         "Our initial design process commenced with a simplified single-hole linkage concept. We explored six distinct linkage designs, ultimately selecting the top three based on their geometric advantages and their potential to effectively and reliably actuate the target button.",
                         "To rapidly explore and visualize the kinematics of different configurations, we utilized an online linkage calculator. This tool enabled us to input various link lengths and pivot point locations, observing the resulting motion paths in real-time. This iterative visualization helped select promising candidates for more detailed analysis and refinement in CAD. The final chosen design was a synthesis of the top two concepts, combining their strong geometric properties and predicting a button press time of approximately 30 seconds based on kinematic simulations."
                    ]
                },
                {
                     type: "image",
                     content: "/media/images/airlock_linkage_design.png",
                      altContent: "Linkage calculator designs and outputs",
                     subtitle:"Various linkage designs explored using an online kinematic calculator."
                   },
            ]
        },
        {
           title: "Stress Analysis and FEA Verification",
           navName: "Stress & FEA",
            navRef: "stress-analysis-fea",
            content: [
                {
                    type: "text",
                    content:
                         ["We conducted stress analysis on three critical orientations: maximized plate height, bending stress on the crank and coupler, and the vertical crank position. The maximized plate height configuration was selected for detailed study. Basic stress was estimated using `\\sigma = F/A` (Force/Area). Finite Element Analysis (FEA) simulations in our CAD software were used to digitally verify these manual stress calculations and to gain a more detailed understanding of stress distributions across the links under load.",
                         "This allowed us to pinpoint specific areas of high stress (stress concentrations), particularly around holes or sharp corners, which then informed design modifications like adding fillets. Based on these analyses, we chose a conservative quarter-inch thick acrylic and a width of two inches for each link to ensure robustness for the initial tests. The final analysis indicated a high safety factor of 16 (calculated as `FoS = \\text{Material Yield Strength} / \\text{Actual Max Stress}`), with minimal displacement."]
                },
                {
                    type: "text",
                    content: "Conceptual FBD of a Link",
                    displayAs: "subtitle"
                },
                 {
                    type: "image",
                    content: "/media/images/airlock_fea_stress.png",
                     altContent: "FEA stress analysis of linkage component",
                    subtitle: "FEA results illustrating stress concentrations on a linkage component."
                   },
                {
                    type: "image",
                     content: "/media/images/airlock_fea_displacement.png",
                      altContent: "FEA displacement analysis of linkage component",
                    subtitle: "FEA results illustrating displacement under load for a linkage component."
                   },
            ]
        },
        {
             title: "Fabrication, Assembly, and Initial Testing",
            navName: "Fabrication & Assembly",
            navRef: "fabrication-assembly",
             content: [
                {
                    type: "text",
                    content: [
                         "The fabrication process strictly adhered to a detailed assembly diagram, which provided specific instructions for assembling the crank, rocker, coupler links, and associated washers. During the initial practice assembly and testing, we achieved a button press time of 28 seconds.",
                         "Based on these initial runs and observations, we identified key areas for improvement. Subsequent design iterations focused on reducing the width of the links to decrease mass and inertia, and adding fillets to critical edges to further reduce stress and enhance the overall reliability and longevity of the system."
                    ]
                 },
                 {
                     type: "image",
                     content: "/media/images/airlock_assembly_diagram.png",
                      altContent: "Assembly Diagram for Airlock Linkage",
                    subtitle: "Detailed assembly diagram guiding the construction of the optimized linkage system."
                   },
            ]
        },
         {
           title: "Iterative Design Refinements and Learnings",
           navName: "Iterative Design",
           navRef: "iterative-design",
           content: [
               {
                    type: "text",
                    content: [
                         "Following initial testing, our team conducted a comprehensive evaluation of the system's performance, leading to targeted design refinements. Key changes included decreasing the width of the links, which successfully reduced mass and rotational inertia, and adding fillets to the edges of the hexagonal hole to mitigate stress concentrations previously identified through FEA.",
                         "These improvements, guided by an iterative design methodology, allowed the system to achieve a lower button press time and a significant reduction in overall weight while maintaining, and in some cases improving, structural integrity. This phase underscored the value of systematically applying learnings from testing and analysis back into the design cycle."
                    ]
                },
                {
                    type: "image",
                     content: "/media/images/airlock_iterative_designs.png",
                     altContent: "Sketches showing iterative design changes for the linkage",
                     subtitle: "Sketches and CAD modifications illustrating the iterative design process."
                },
            ]
        }
    ]
};
