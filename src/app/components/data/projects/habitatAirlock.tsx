import { Project } from "@/app/components/project/interfaces";

export const habitatAirlockLinkage: Project = {
  title: "Habitat Airlock Linkage System",
  subtitle: "Optimized four-bar linkage for rapid button actuation",
  date: "Spring 2023",
  media: "/media/videos/habitat_linkage.mp4",
  tags: ["Mechanical Design", "Linkage Systems", "CAD", "FEA", "Optimization"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "project-overview",
      content: [
        {
          type: "text",
          content:
            "Imagine a mechanical device that needs to press a button as fast as possible—like an airlock mechanism on a space habitat. This project designed and optimized a **four-bar linkage** (a classic mechanism with four connected bars that converts rotational motion into complex paths) to actuate a button reliably and quickly. The challenge: minimize weight to maximize speed, while keeping the mechanism strong enough not to break.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Design Goals",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Minimize mass** — reduce weight for faster actuation",
            "**Reduce stress concentrations** — FEA-guided filleting",
            "**Ensure reliability** — repeatable, robust operation",
          ],
        },
        {
          type: "image",
          content: "/media/images/airlock_final_testing.png",
          altContent: "Final testing of the linkage system.",
          subtitle: "The linkage system during final performance testing.",
        },
      ],
    },
    {
      title: "Performance Results",
      navName: "Results",
      navRef: "summary-results",
      content: [
        {
          type: "text",
          content:
            "The target: press the button within **30 seconds**. We beat that in both tests, and the final design weighed less than a third of our initial attempt. Here's how the numbers stacked up:",
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
            "**Intermediate test** — 27.55 seconds",
            "**Final test** — 25.91 seconds",
            "**Mass reduction** — 72.4% from initial design",
          ],
        },
        {
          type: "text",
          content:
            "Key improvements included **filleting hexagonal hole edges** to reduce stress concentrations and **optimizing link geometry** to reduce mass and inertia.",
        },
        {
          type: "image",
          content: "/media/images/airlock_intermediate.png",
          altContent: "Airlock Linkage System Intermediate Testing",
          subtitle: "Linkage system during intermediate performance testing.",
        },
      ],
    },
    {
      title: "Initial Design Phase",
      navName: "Initial Design",
      navRef: "brainstorming-design",
      content: [
        {
          type: "text",
          content:
            "We didn't just pick one design—we explored **six different linkage configurations** using an online kinematic simulator. Each had different trade-offs in reach, speed, and mechanical advantage. We narrowed down to three finalists, then combined the best features into our final design.",
        },
        {
          type: "text",
          content:
            "The selected design predicted ~30 second actuation time based on simulations—right at our target. Good enough to start building and testing.",
        },
        {
          type: "image",
          content: "/media/images/airlock_linkage_design.png",
          altContent: "Linkage calculator designs and outputs",
          subtitle: "Linkage designs explored using an online kinematic calculator.",
        },
      ],
    },
    {
      title: "Stress Analysis and FEA",
      navName: "Stress & FEA",
      navRef: "stress-analysis-fea",
      content: [
        {
          type: "text",
          content:
            "Light is fast, but too light means it breaks. We analyzed three critical positions where the linkage experiences maximum stress—when it's most likely to fail. **Finite Element Analysis (FEA)** simulations showed exactly where stress concentrated: around holes and sharp corners. That's where cracks start, so we added fillets (rounded edges) to spread the load.",
        },
        {
          type: "math",
          block: true,
          content: String.raw`\sigma = \frac{F}{A}`,
          subtitle: "Basic stress calculation (Force / Area)",
        },
        {
          type: "text",
          content:
            "**FEA simulations** verified manual calculations and identified stress concentrations around holes and sharp corners, informing fillet additions.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Material Selection",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Material** — 1/4\" thick acrylic",
            "**Link width** — 2 inches",
            "**Safety factor** — 16× (yield strength / max stress)",
          ],
        },
        {
          type: "image",
          content: "/media/images/airlock_fea_stress.png",
          altContent: "FEA stress analysis of linkage component",
          subtitle: "FEA stress distribution on linkage component.",
        },
        {
          type: "image",
          content: "/media/images/airlock_fea_displacement.png",
          altContent: "FEA displacement analysis of linkage component",
          subtitle: "FEA displacement under load.",
        },
      ],
    },
    {
      title: "Fabrication and Assembly",
      navName: "Fabrication",
      navRef: "fabrication-assembly",
      content: [
        {
          type: "text",
          content:
            "With the design validated in simulation, we laser-cut the acrylic links and assembled the mechanism. First test: **28 seconds**—faster than predicted! But we saw opportunities to go even faster.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Identified Improvements",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Reduce link width** — decrease mass and inertia",
            "**Add fillets** — reduce stress at critical edges",
            "**Optimize clearances** — improve assembly precision",
          ],
        },
      ],
    },
    {
      title: "Iterative Design Refinements",
      navName: "Iterations",
      navRef: "iterative-design",
      content: [
        {
          type: "text",
          content:
            "Engineering is iterative: test, learn, improve, repeat. After the first build, we made targeted changes based on real-world performance and FEA insights. The result? Faster actuation, lower mass, and the same structural safety margin.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Learnings",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Iterative methodology** — systematic application of test learnings",
            "**FEA-guided design** — data-driven stress reduction",
            "**Mass optimization** — improved speed without sacrificing strength",
          ],
        },
        {
          type: "image",
          content: "/media/images/airlock_iterative_designs.png",
          altContent: "Sketches showing iterative design changes for the linkage",
          subtitle: "CAD modifications illustrating the iterative design process.",
        },
      ],
    },
  ],
};
