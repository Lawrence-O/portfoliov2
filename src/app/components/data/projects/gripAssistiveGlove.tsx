import { Project } from "@/app/components/project/interfaces";

export const gripAssistiveGlove: Project = {
  title: "Grip Assistive Glove",
  subtitle: "Affordable assistive technology for enhanced grip strength",
  date: "Spring 2023",
  media: "/media/videos/gripAssitiveGlove.mp4",
  tags: ["Assistive Technology", "Mechanical Design", "CAD", "FEA", "Prototyping"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "project-overview",
      content: [
        {
          type: "text",
          content:
            "For people with conditions like Cerebral Palsy or Tendonitis, simply gripping a cup or opening a door can be exhausting or impossible. Existing assistive devices often cost thousands of dollars. We set out to build something **affordable and effective**: a motorized glove that helps close the fingers, restoring grip strength for everyday tasks.",
        },
        {
          type: "text",
          content:
            "The concept is simple: fishing lines run from each fingertip to a motor-driven spool on the forearm. Press a button, the motor winds the lines, and the fingers curl into a grip. Release the button, and the user can manually open their hand. Total estimated cost: around **$50**.",
        },
        {
          type: "image",
          content: "/media/images/gripglove_assembly.png",
          altContent: "Assembly of the Grip Assistive Glove",
          subtitle: "Final assembly and bill of materials for the Grip Assistive Glove.",
        },
      ],
    },
    {
      title: "Early Design Concepts",
      navName: "Early Concepts",
      navRef: "early-ideation",
      content: [
        {
          type: "text",
          content:
            "We explored several mechanisms before settling on our approach. Some ideas used springs for automatic release; others had separate motors per finger. Each added complexity, weight, or cost. The winning concept: **one motor, simple lines, manual release**—the user's own muscle handles opening.",
        },
        {
          type: "image",
          content: "/media/images/gripGlove_initial_sketches.png",
          altContent: "Initial sketches of the Grip Assistive Glove",
          subtitle: "Early sketches exploring different design approaches and mechanisms.",
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
            "**Single motor** chosen over dual-motor system to reduce complexity, weight, and cost",
            "**Elastic release mechanism** rejected to avoid impeding natural finger movement",
            "**Palm convergence** design selected for simplified line routing",
          ],
        },
      ],
    },
    {
      title: "Refined Design and Analysis",
      navName: "Refined Design",
      navRef: "developed-ideation",
      content: [
        {
          type: "text",
          content:
            "The final mechanism uses **one motor** to pull all four fingers at once, with a separate line for the thumb (which moves differently during gripping). When not powered, the lines are slack—they don't fight against the user trying to open their hand.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Stress Analysis",
        },
        {
          type: "text",
          content:
            "Will the fishing line snap? We checked by assuming the motor stalls at maximum torque (worst case). The 40 lb rated fishing line has plenty of margin against the calculated forces:",
        },
        {
          type: "math",
          block: true,
          content: String.raw`F_{\text{line}} = \frac{\tau_{\text{motor}}}{r_{\text{spool}}} \cdot \eta`,
          subtitle: "Where τ is motor torque, r is spool radius, and η is system efficiency.",
        },
      ],
    },
    {
      title: "CAD Model and Control System",
      navName: "CAD & Controls",
      navRef: "cad-prototype",
      content: [
        {
          type: "text",
          content:
            "We modeled every component in **SolidWorks** before building anything. Lines route through small 3D-printed nodes on each finger, converge at a palm node, and connect to a spool on the forearm. Two buttons control the motor: *wind* to grip, *unwind* to release.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Control Logic",
        },
        {
          type: "code",
          codeLang: "plaintext",
          content:
`GRIP CONTROL ALGORITHM
======================

STATE: idle | winding | unwinding

LOOP:
  IF wind_button pressed:
    state = winding
    Motor pulls lines (grip)
    
  ELSE IF unwind_button pressed:
    state = unwinding  
    Motor releases lines (open)
    
  ELSE:
    state = idle
    Motor stops

  Check limit switches for safety
ENDLOOP`,
        },
        {
          type: "image",
          content: "/media/images/gripGlove_glove_node.png",
          altContent: "CAD model of the finger node",
          subtitle: "CAD model of the finger node design.",
        },
        {
          type: "image",
          content: "/media/images/gripGlove_glove_spool.png",
          altContent: "CAD model of the motor spool",
          subtitle: "CAD model of the motor spool assembly.",
        },
      ],
    },
    {
      title: "Finite Element Analysis",
      navName: "FEA",
      navRef: "testing-fea",
      content: [
        {
          type: "text",
          content:
            "The 3D-printed finger nodes are the weakest links—will they break under load? **FEA simulation** applied realistic forces and checked whether the PLA plastic would hold up. Good news: stress levels stayed well below failure, with essentially no bending.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Results",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Stress levels** well below PLA yield strength",
            "**Negligible displacement** under load",
            "**Validated** structural robustness for intended application",
          ],
        },
        {
          type: "image",
          content: "/media/images/gripGlove_fea_analysis.png",
          altContent: "FEA Stress Analysis of Finger Node",
          subtitle: "FEA stress distribution on a finger node.",
        },
        {
          type: "image",
          content: "/media/images/gripGlove_fea_displacement.png",
          altContent: "FEA displacement analysis of Finger Node",
          subtitle: "FEA displacement analysis under load.",
        },
      ],
    },
    {
      title: "Manufacturing and Cost Analysis",
      navName: "Manufacturing",
      navRef: "manufacturing-drawing",
      content: [
        {
          type: "text",
          content:
            "We chose **PLA plastic** for the 3D-printed parts—cheap, easy to print, strong enough. For production at scale, injection molding would drop costs further. Here's what goes into one glove:",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Bill of Materials",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Leather glove** — $11.00 ($6.50 bulk)",
            "**Fishing wire** — $0.11",
            "**Palm node** (3D printed) — $0.50",
            "**Motor spool** (3D printed) — $2.50",
            "**Electronics** (motor, buttons, wiring) — $26.00",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Pricing",
        },
        {
          type: "text",
          content:
            "With a **30% markup** on manufacturing cost (~$40), the estimated retail price is **$52.14** — positioning this as an *affordable solution* in the assistive technology market.",
        },
      ],
    },
    {
      title: "Budget Summary",
      navName: "Budget",
      navRef: "budget",
      content: [
        {
          type: "text",
          content:
            "We built a working prototype for under $44—well under our budget. This validates the core premise: grip assistance doesn't have to cost thousands of dollars. With some refinement, this could become an accessible option for people who need it.",
        },
      ],
    },
  ],
};
