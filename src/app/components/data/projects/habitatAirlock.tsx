import { Project } from "@/app/components/project/interfaces";

export const habitatAirlockLinkage: Project = {
    title: "Habitat's Airlock Linkage System",
    subtitle: "Engineering Design I: Skills and Methods - Spring 2023",
    media: "/media/videos/habitat_linkage.mp4",
    tags: ["Mechanical Design", "Linkage System", "CAD", "FEA", "Prototyping", "Optimization"],
    section: [
        {
            title: "Project Overview",
            navName: "Project Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content:
                        "The Habitat's Airlock Linkage System project was undertaken to design and optimize a mechanical linkage system capable of actuating a button within a specified timeframe. This project challenged us to integrate iterative design, rigorous analysis, and comprehensive testing to enhance the system's overall performance. The primary focus included minimizing mass, reducing stress concentrations, and ensuring reliable operation, with the ultimate goal of creating a robust and effective system."
                },
                {
                    type: "image",
                     content: "/media/images/airlock_final_testing.png",
                     altContent: "Final testing of the linkage system.",
                     subtitle: "The linkage system during its final performance testing."
                 },
            ]
        },
        {
            title: "Performance Results",
            navName: "Performance Results",
            navRef: "summary-results",
            content: [
                {
                    type: "text",
                    content:
                         "During the project, we conducted both intermediate and final performance tests, which required the linkage system to actuate a button. The target time for the button press was 30 seconds for both tests. In the intermediate test, we achieved an experimental button press time of 27.55 seconds, while the final test yielded a time of 25.91 seconds. This slight difference in performance was attributed to a minor assembly error. Key improvements made throughout the project included filleting the edges of the hexagonal hole to reduce stress concentrations and optimizing mass by reducing the weight of the links by an impressive 72.4%."
                },
                {
                     type: "image",
                     content: "/media/images/airlock_intermediate.png",
                    altContent: "Airlock Linkage System Intermediate Testing",
                     subtitle: "The linkage system undergoing intermediate performance testing."
                  },
            ]
        },
        {
             title: "Initial Design and Brainstorming",
             navName: "Initial Design and Brainstorming",
            navRef: "brainstorming-design",
             content: [
               {
                    type: "text",
                     content:
                         "Our initial design process started with a simplified single-hole design. We explored six different linkage designs, selecting the top three based on their geometry and their potential to effectively actuate the button. To further analyze the selected designs, we used an online linkage calculator, allowing us to refine our understanding of their motion and performance. The final design was a synthesis of the top two concepts, combining their strong geometric properties and predicting a button press time of approximately 30 seconds."
                },
                {
                     type: "image",
                     content: "/media/images/airlock_linkage_design.png",
                      altContent: "Linkage calculator designs",
                     subtitle:"Different linkage designs derived from the online calculator."
                   },
            ]
        },
        {
           title: "Stress Analysis and FEA Verification",
           navName: "Stress Analysis and FEA",
            navRef: "stress-analysis-fea",
            content: [
                {
                    type: "text",
                    content:
                         "We conducted stress analysis on three critical orientations: maximized plate height, bending stress on the crank and coupler, and the vertical crank position. The maximized plate height configuration was selected for detailed study. Finite Element Analysis (FEA) simulations confirmed the hand calculations and revealed areas of stress concentration within the design. Based on these analyses, we chose a conservative quarter-inch thick acrylic and a width of two inches for each link to ensure robustness for the initial tests. The final analysis indicated a high safety factor of 16, with minimal displacement."
                },
                 {
                    type: "image",
                    content: "/media/images/airlock_fea_stress.png",
                     altContent: "FEA stress analysis",
                    subtitle: "FEA analysis illustrating the stress concentrations."
                   },
                {
                    type: "image",
                     content: "/media/images/airlock_fea_displacement.png",
                      altContent: "FEA displacement analysis",
                    subtitle: "FEA analysis illustrating the displacement."
                   },
            ]
        },
        {
             title: "Fabrication and Assembly Process",
            navName: "Fabrication and Assembly",
            navRef: "fabrication-assembly",
             content: [
                {
                    type: "text",
                    content:
                         "The fabrication process adhered to a detailed assembly diagram, with specific instructions for assembling the crank, rocker, coupler, and washers. During the initial practice, we achieved a button press time of 28 seconds. Based on these initial runs, we made key improvements to our design by reducing the width of the links and adding fillets to reduce stress and allow for a better and more reliable system."
                 },
                 {
                     type: "image",
                     content: "/media/images/airlock_assembly_diagram.png",
                      altContent: "Assembly Diagram",
                    subtitle: "Detailed assembly diagram of the optimized linkage system."
                   },
            ]
        },
         {
           title: "Iterative Design Refinements",
           navName: "Iterative Design Refinements",
           navRef: "iterative-design",
           content: [
               {
                    type: "text",
                    content:
                         "Post initial testing, our team conducted a comprehensive evaluation of the system's performance. We made key refinements, such as decreasing the width of the links, adding fillets to the edges of the hexagonal hole, and minimizing out-of-plane bending. These improvements allowed the system to achieve a lower button press time and a significant reduction in weight. These improvements were based on a comprehensive evaluation of the geometry and stress analysis of the design."
                },
                {
                    type: "image",
                     content: "/media/images/airlock_iterative_designs.png",
                     altContent: "Iterative Design sketches",
                     subtitle: "Sketches illustrating the iterative design process."
                },
            ]
        },
        {
            title: "Recommendations for Future Iterations",
             navName: "Recommendations for Future Iterations",
             navRef: "continuous-improvement",
            content: [
                 {
                    type: "text",
                    content:
                        "Based on our experience, we recommend several enhancements for future iterations of the project. These include incorporating an intermediate testing day to monitor progress, developing a more detailed project description, increasing the group size to four members to better manage workload, exploring more diverse material options, having students determine geometric constraints independently, and adding a dynamic button that can increase the project's score."
                 },
            ]
        }
    ]
};