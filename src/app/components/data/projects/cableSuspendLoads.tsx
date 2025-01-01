import { Project } from "@/app/components/project/interfaces";

export const cableSuspendedLoads: Project = {
  title: "Hybrid Control for Cable-Suspended Loads with Quadrotors",
  subtitle: "Spring 2024",
  media: "/media/images/iterativeControl.png",
  tags: ["Optimal Control", "IPOPT", "ALTRO", "UAVs", "Hybrid Approach", "Quaternions"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "This project extends the framework from 'Scalable Cooperative Transport of Cable-Suspended Loads with UAVs using Distributed Trajectory Optimization' to address the challenges presented by slack in suspension cables. We introduce a hybrid control strategy that enables quadrotors to dynamically reconfigure when a quadrotor becomes inactive and causes slack. The project explores the use of IPOPT and compares it against ALTRO in solving complex trajectory optimization problems. The core objective was to maintain safe and efficient transport of a payload under varying conditions by leveraging a novel hybrid approach.",
        },
        {
           type: "image",
           content: "/media/images/iterativeControlIntro.png",
           altContent: "Diagram illustrating the ILC and hybrid trajectory optimization concepts",
           subtitle: "General Overview of Project Concepts",
        }
      ],
    },
    {
      title: "Background and Motivation",
      navName: "Background",
      navRef: "background",
      content: [
          {
            type: "text",
            content:
              "Previous research has shown the benefits and challenges of using quadrotors for transporting heavy loads, emphasizing the cost-effectiveness, versatility, and ease of deployment of such systems. Existing trajectory optimization solutions, like the one implemented with ALTRO in the 'Scalable Cooperative Transport' paper, have difficulty in handling scenarios with slack cables, which can occur if a quadrotor becomes inactive, or due to workspace constraints. This project seeks to improve upon this, by creating a solution for scenarios where slack is introduced. The paper models the cables as massless rigid links, and we do the same, and only introduce slack to the system through zeroing out the tension constraints."
          },
        {
          type: "text",
          content:
             "Our work builds on this existing framework by introducing a hybrid control strategy to manage slack. The approach also uses an active set method to switch the constraints of a quadrotor when it has slack to properly model the physics. Similar to the original paper, quaternions were used to allow for aggressive maneuvers and more complex reconfigurations.  The objective was to transport a payload through dynamic environments while exploring the performance limitations of IPOPT in large batch problems.",
        },
      ],
    },
      {
          title: "Problem Formulation and Hybrid Approach",
          navName: "Problem Formulation",
          navRef: "problem-formulation",
          content: [
            {
              type: "text",
              content:
                "The project formulated a trajectory optimization problem for a cable-suspended load with multiple quadrotors, modeling the cables as massless rigid links. These cables transmit forces but do not have dynamics of their own. We aimed to solve a system where some of the quadrotors go slack. Our hybrid approach uses the concept of active sets to define two states for the quadrotors, the state where they are actively supporting the load, and the state when they are slack.  This allows us to maintain one dynamics function and dynamically change the constraints of the system.",
            },
            {
               type: "text",
               content: "The quadrotor dynamics are represented by:",
            },
            {
            type: "text",
            content: `
                \dot{x} = \begin{bmatrix}
                    \dot{r} \\
                    \dot{q} \\
                    \dot{v} \\
                    \dot{\omega} \\
                \end{bmatrix} = \begin{bmatrix}
                v \\
                \frac{1}{2} q \otimes \hat{\omega} \\
                g + \frac{1}{m} \left( R(q) F(u) + F_c(u_5, x, x') \right) \\
                J^{-1} \left( \tau(u) - \omega \times J\omega \right)
                \end{bmatrix}
            `,
          },
            {
                type: "text",
                content:
                     "Where `r` is the position, `q` is a unit quaternion, `R(q)` is a quaternion-dependent rotation matrix, `v` is the linear velocity, `ω` is the angular velocity, `x` is the state vector, `u` is the control vector, `x’` is the load state vector, `g` is gravity and `m` is mass.  The forces and torques on the quadrotor are defined as  `F(u)` and `τ(u)`. The cable forces are modeled as `F_{c}(\gamma, x, x’)`.",
            },
              {
                 type: "text",
                content: "The load dynamics are modeled as: ",
              },
             {
               type: "text",
                content: `
                    \dot{x}^{\ell} = \begin{bmatrix}
                        \dot{r}^{\ell} \\
                        \dot{v}^{\ell} \\
                    \end{bmatrix} = \begin{bmatrix}
                        v^{\ell} \\
                        g + \frac{1}{m^{\ell}} F^{\ell}(x^{\ell}, u^{\ell}, x^{1:L})
                    \end{bmatrix}
                `,
              },
            {
               type:"text",
               content: "Where `r^{\ell}` is the load position, `v^{\ell}` is the load velocity, and `m^{\ell}` is the load mass."
            },
            {
                type: "text",
                content: "The approach uses an active set method to dynamically manage when a quadrotor is actively supporting the load (`Q_i`) or when it is slack (`S_i`), effectively zeroing out the tension constraints from the slack quadrotor. This allows a hybrid approach without the need for changing dynamics functions."
            },
            {
               type: "text",
                content: "The optimization problem seeks to minimize a cost function with constraints for the discrete quadrotor dynamics, discrete load dynamics, initial conditions, final load conditions, workspace bounds, quadrotor motor limits, positive cable tensions, equal cable forces, fixed cable length, and collision avoidance."
             }
        ]
    },
     {
        title: "Simulation Results and Observations",
        navName: "Simulation Results",
        navRef: "simulation-results",
        content: [
          {
            type: "text",
            content:
                "We conducted several simulations to evaluate the performance of our algorithm under various scenarios. These simulations were performed on an Apple MacBook using an M1 Pro processor and 16 GB RAM. The code was implemented in the Julia programming language. Trim conditions were used to seed IPOPT with valid initial states and controls.",
            },
             {
                 type: "text",
                 content: "The simulations produced several key results:"
             },
                {
                type: "text",
                    content:
                    "The solver, IPOPT, was able to provide trajectories that abide by our expectations of the problem. However, due to the nonconvexity and nonlinearity of the problem the solver did not converge well. While our simulations produced trajectories that meet our expectations of how the system should behave, the dual infeasibility did not converge to 0."
                },
                 {
                     type: "image",
                    content: "/media/images/4 agents.png",
                     altContent: "4 agents moving in simulation.",
                      subtitle: "An example of 4 agents mid trajectory.",
                    },
                   {
                    type: "image",
                    content: "/media/images/9 agents.png",
                    altContent: "8 agents moving in simulation",
                      subtitle: "An example of 8 agents mid trajectory.",
                    },
              {
                type: "image",
                 content: "/media/images/position plot 6 agents.png",
                 altContent: "Position plot for a 6 agent configuration",
                subtitle: "Position plot for a 6-agent configuration from start to goal.",
              },
               {
                 type: "text",
                 content: " The time it took for IPOPT to produce a trajectory varied greatly depending on the number of agents, and other complexities in the problem. The solver showed to converge well in the early parts of the simulation, but struggled to reach convergence in later iterations."
                 },
                  {
                    type: "image",
                    content: "/media/images/time.png",
                     altContent: "Time required for IPOPT solve for different number of agents",
                    subtitle: "Time required for IPOPT solve for different number of agents.",
                 },
                    {
                    type: "image",
                    content: "/media/images/knot.png",
                     altContent: "Graph showing runtime vs number of knot points",
                    subtitle: "Graph of runtime vs number of knot points.",
                 },
                   {
                    type: "image",
                     content: "/media/images/constr.png",
                     altContent: "Constraint violations across iterations",
                      subtitle: "Constraint violations across iterations of IPOPT.",
                     },
                      {
                    type: "image",
                     content: "/media/images/dual.png",
                     altContent: "Dual infeasibility across iterations",
                      subtitle: "Dual infeasibility across iterations of IPOPT.",
                     },
             {
               type: "text",
                content: "Doubling the knot points led to an 8-fold increase in the solver time which highlights the computational complexity of this problem. Due to this limitation, a 100-knot problem formulation was not able to be solved due to the jacobian's size of 54 million."
              },

        ]
    },
       {
        title: "IPOPT vs ALTRO and Limitations",
        navName: "ALTRO vs IPOPT",
        navRef: "altro-vs-ipopt",
        content: [
          {
            type: "text",
            content:
                "ALTRO is a trajectory optimization method that combines an augmented Lagrangian approach with an active-set method to achieve fast convergence for constrained problems. It has shown to perform competitively with DIRCOL methods such as IPOPT. A key strength of ALTRO is its ability to be initialized with infeasible state trajectories, which can be a major challenge for interior-point methods like IPOPT which requires a feasible starting point for convergence. This makes ALTRO a more promising alternative to IPOPT for complex trajectory optimization problems like ours.  Furthermore, the way in which ALTRO handles constraints, particularly in problems involving obstacle avoidance, has shown to be more effective than the approach utilized by IPOPT.",
          },
            {
              type: "text",
                content: "During this project, we faced challenges with IPOPT's convergence, primarily due to the complexity of the problem, numerical instabilities, and the initial condition not being tuned perfectly. The lack of convergence could be due to the problem size, numerical instability, or poor initial conditions."
            },
          ],
        },
        {
            title: "Conclusion and Future Work",
            navName: "Conclusion",
            navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content:
                        "This project successfully built upon previous work, outlining a methodology to handle slack in a multi-agent cable-suspended system, and explored the implementation of several techniques for this type of problem. Through this work we observed some challenges with IPOPT. A distributed approach to solving the problem using IPOPT could allow for increased scalability by reducing overall problem size and potential numerical instabilities. An implementation of the slack variable in a distributed manner could also improve adaptability in dynamic environments. Lastly, exploring ALTRO might address the initialization and convergence issues we faced in this work.",
                },
            ],
        },
    ],
};