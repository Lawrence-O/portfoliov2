import { Project } from "@/app/components/project/interfaces";

export const cableSuspendedLoads: Project = {
  title: "Hybrid Control for Cable-Suspended Loads with Quadrotors",
  date: "Spring 2024",
  media: "/media/images/cableSus_4_agents.png",
  tags: ["Optimal Control", "IPOPT", "Trajectory Optimization", "UAVs", "Hybrid Approach", "Quaternions"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "This project extends the framework from 'Scalable Cooperative Transport of Cable-Suspended Loads with UAVs using Distributed Trajectory Optimization' to address the challenges presented by slack in suspension cables. We introduce a hybrid control strategy that enables quadrotors to dynamically reconfigure when a quadrotor becomes inactive and causes slack. The project uses IPOPT for trajectory optimization and analyzes its performance limitations compared to alternative solvers like ALTRO. The implementation was done in Julia, leveraging its strong numerical computation capabilities and packages like `Ipopt.jl`. The core objective was to maintain safe and efficient transport of a payload under varying conditions using a novel hybrid approach.",
        },
        {
           type: "image",
           content: "/media/images/cableSuspension9_agents.png", 
           altContent: "Diagram of multi-quadrotor cable-suspended load transport with hybrid control",
           subtitle: "Hybrid Control for Cable-Suspended Load System",
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
             "Our work builds on this existing framework by introducing a hybrid control strategy to manage slack. The approach also uses an active set method to switch the constraints of a quadrotor when it has slack to properly model the physics. An active set method allows the system to dynamically switch between different sets of active constraints, effectively changing the model based on whether a cable is taut or slack, without needing entirely separate dynamic models. Similar to the original paper, quaternions were used to allow for aggressive maneuvers and more complex reconfigurations. The objective was to transport a payload through dynamic environments while exploring the performance limitations of IPOPT in large batch problems.",
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
                "The project formulated a trajectory optimization problem for a cable-suspended load with multiple quadrotors, modeling the cables as massless rigid links. These cables transmit forces but do not have dynamics of their own. We aimed to solve a system where some of the quadrotors go slack. Our hybrid approach uses the concept of active sets to define two states for the quadrotors, the state where they are actively supporting the load, and the state when they are slack. This allows us to maintain one dynamics function and dynamically change the constraints of the system."
            },
            {
               type: "text",
               content: "The dynamics of each quadrotor `i` are modeled as a 13-state system (position `r`, orientation (quaternion `q`), linear velocity `v`, angular velocity `\omega`):",
            },
            {
            type: "text",
            content: `
                \\dot{x} = \\begin{bmatrix}
                    \\dot{r} \\\\
                    \\dot{q} \\\\
                    \\dot{v} \\\\
                    \\dot{\\omega} \\\\
                \\end{bmatrix} = \\begin{bmatrix}
                v \\\\
                \\frac{1}{2} q \\otimes \\hat{\\omega} \\\\
                g + \\frac{1}{m} \\left( R(q) F(u) + F_c(u_5, x, x') \\right) \\\\
                J^{-1} \\left( \\tau(u) - \\omega \\times J\\omega \\right)
                \\end{bmatrix}
            `,
          },
            {
                type: "text",
                content:
                     "Where `r` is the position, `q` is a unit quaternion representing orientation, `R(q)` is a quaternion-dependent rotation matrix, `v` is the linear velocity, `ω` is the angular velocity (with `\hat{\omega}` being its skew-symmetric matrix form for quaternion kinematics `q \\otimes \hat{\omega}`), `x` is the 13-element state vector `[r, q, v, \omega]`, `u` is the control vector (typically motor thrusts/torques), `x’` is the load state vector, `g` is gravity and `m` is mass. The forces and torques generated by the quadrotor motors are `F(u)` and `τ(u)`. The cable force exerted by the specific cable on this quadrotor is `F_{c}` which depends on cable state (e.g. tension, represented by `u_5` if it's a control variable or derived) and the states of the quadrotor `x` and load `x'`.",
            },
              {
                 type: "text",
                content: "The load dynamics are modeled as: ",
              },
             {
               type: "text",
                content: `
                    \\dot{x}^{\\ell} = \\begin{bmatrix}
                        \\dot{r}^{\\ell} \\\\
                        \\dot{v}^{\\ell} \\\\
                    \\end{bmatrix} = \\begin{bmatrix}
                        v^{\\ell} \\\\
                        g + \\frac{1}{m^{\\ell}} F^{\\ell}(x^{\\ell}, u^{\\ell}, x^{1:L})
                    \\end{bmatrix}
                `,
              },
            {
               type:"text",
               content: "Where `r^{\ell}` is the load position, `v^{\ell}` is the load velocity, `m^{\ell}` is the load mass, and `F^{\ell}` is the net force on the load, including tensions from all active cables and any other external forces (potentially dependent on load control `u^{\ell}` and states of all `L` quadrotors `x^{1:L}`)."
            },
            {
                type: "text",
                content: "The optimization problem seeks to minimize a cost function `J`, typically penalizing deviations from a desired trajectory (`x_d`), control effort (`u`), and ensuring the final state `x(T)` meets objectives via a terminal cost `\\phi(x(T))`. A general form is:"
            },
            {
                type: "text",
                content: "`J = \\int_{0}^{T} ( (x - x_d)^T Q (x - x_d) + u^T R u ) dt + \\phi(x(T))`"
            },
            {
                type: "text",
                content: "This is subject to constraints including: discrete quadrotor dynamics (ensuring physics are obeyed at each time step), discrete load dynamics, initial conditions (starting state of quadrotors and load), final load conditions (desired end position/velocity of the load), workspace bounds (keeping quadrotors within safe operational areas), quadrotor motor limits (respecting physical capabilities of motors), positive cable tensions (cables can only pull, not push – this is modified by the hybrid logic), equal cable forces (if distributing load evenly), fixed cable length (geometric constraint between quadrotor and load), and collision avoidance (maintaining safe distances between quadrotors and any obstacles)."
            },
            {
                type: "text",
                content: "**Hybrid Control Logic (Pseudocode):** The active set method manages the switch between 'active' and 'slack' cable states:"
            },
            {
                type: "code",
                codeLang: "plaintext",
                content:
`Algorithm: Hybrid Control for Cable Slack Management (Conceptual)

1. Initialize: Define all quadrotors (Q_i) as initially ACTIVE (cable taut).
2. Iterative Optimization Loop (e.g., at each control update or replanning cycle):
    a. Solve Trajectory Optimization: Based on the current set of ACTIVE/SLACK states for each cable, formulate and solve the trajectory optimization problem. This involves:
        - Applying full constraints (e.g., positive tension, fixed cable length) for ACTIVE cables.
        - Applying modified constraints (e.g., zero tension, relaxed geometric constraints) for SLACK cables.
    b. Evaluate Cable States:
        i. For each ACTIVE quadrotor Q_i:
           - If calculated_tension_i < slack_threshold:
             - Transition Q_i to SLACK state.
             - Mark system configuration as changed.
        ii. For each SLACK quadrotor Q_i:
           - If reattachment_conditions_met (e.g., cable path clear, quadrotor in position to re-tension):
             - Transition Q_i to ACTIVE state.
             - Mark system configuration as changed.
    c. Re-solve if Configuration Changed:
        - If the state of any cable (ACTIVE/SLACK) has changed in step 2b, re-formulate and re-solve the optimization problem (back to step 2a) with the updated constraint set.
        - Otherwise, proceed with the current trajectory.
3. Apply Control: Implement the first part of the optimized trajectory. Repeat from step 2.
`
            },
            {
                type: "text",
                content: "**Conceptual Julia Code for IPOPT Problem Definition:**"
            },
            {
                type: "code",
                codeLang: "julia",
                content:
`# Conceptual Julia Snippet for IPOPT Problem Definition using Ipopt.jl
using Ipopt

# 1. Define Problem Dimensions (Illustrative)
num_knot_points = 50
num_states_per_quad = 13
num_controls_per_quad = 4 # (e.g., thrust and 3 torques) or 5 (if tension is a control)
num_states_load = 6
num_quads = 4

total_vars = num_knot_points * (num_quads * (num_states_per_quad + num_controls_per_quad) + num_states_load)
# total_constraints = ... (based on dynamics, path constraints, etc.)

# 2. Define Objective Function (eval_f)
#    - Takes a vector x (all decision variables flattened)
#    - Returns a scalar cost (e.g., sum of squared errors from ref_traj + control effort)
function eval_f(x)
    # cost = 0.0
    # for k = 1:num_knot_points
    #   current_states_quads = ... extract from x ...
    #   current_state_load = ... extract from x ...
    #   current_controls_quads = ... extract from x ...
    #   cost += stage_cost(current_states_quads, current_state_load, current_controls_quads, ref_traj[k])
    # end
    # cost += terminal_cost(...)
    return cost_placeholder
end

# 3. Define Constraint Functions (eval_g)
#    - Takes x and populates a vector g with constraint violations.
#    - Constraints include: dynamics, initial/final conditions, cable length, tension limits (hybrid), etc.
function eval_g(x, g)
    # idx = 1
    # For each knot point k:
    #   g[idx:idx_end] = dynamics_constraint_quad_i(x_k, u_k, x_{k+1}) for each quad i
    #   g[...] = dynamics_constraint_load(...)
    #   g[...] = cable_length_constraint_quad_i(...)
    #   g[...] = tension_constraint_quad_i(...) (handles ACTIVE/SLACK logic)
    # ... and other constraints ...
    return constraint_violations_placeholder
end

# 4. Define Gradients (eval_grad_f, eval_jac_g)
#    - (Often using Automatic Differentiation, e.g., ForwardDiff.jl, or finite differences)
# eval_grad_f(x, grad_f) = ...
# eval_jac_g(x, rows, cols, values) = ... (sparse Jacobian structure)

# 5. Create IPOPT Problem
# prob = Ipopt.CreateProblem(...)
# Set options (e.g., tolerance, max_iterations)
# Add_Str_Option(prob, "mu_strategy", "adaptive")
# Add_Num_Option(prob, "tol", 1e-4)

# 6. Provide Initial Guess
# prob.x = very_good_initial_guess_vector

# 7. Solve
# status = Ipopt.Solve_Problem(prob)
# solution = prob.x
`
            },
            {
                type: "text",
                content: "**Flowchart of Hybrid Control Logic:** The hybrid control logic can be visualized as an iterative process. It starts with an initial assumption of all quadrotors being active. A trajectory optimization is solved. Then, for each quadrotor, its cable tension is checked. If an active quadrotor's cable goes slack, its state is changed, and constraints are modified (e.g., removing positive tension requirement). If a slack quadrotor meets re-attachment criteria, its state is changed back to active, and original constraints are re-applied. If any configuration change occurs, the optimization is re-initialized and re-solved. This loop continues, adapting the trajectory to the dynamic cable states."
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
                    "The solver, IPOPT, was able to provide trajectories that qualitatively met our expectations of the problem. However, due to the high non-convexity and non-linearity of the problem, the solver exhibited challenges in achieving full numerical convergence (specifically, the dual infeasibility did not reduce to machine precision). This is a common issue in highly complex, non-convex optimization problems and can be attributed to factors like problem scale, numerical conditioning, or the proximity of the initial guess to a good local minimum. While our simulations produced trajectories that meet our expectations of how the system should behave, the dual infeasibility did not converge to 0."
                },
              {
                type: "image",
                 content: "/media/images/cableSus_6_agent_plot.png",
                 altContent: "Position plot for a 6 agent configuration",
                subtitle: "Position plot for a 6-agent configuration from start to goal.",
              },
               {
                 type: "text",
                 content: "The time required for IPOPT to produce a trajectory varied significantly depending on the number of agents and other problem complexities. The solver converged well in the early iterations but struggled to reach full convergence in later iterations."
                 },
                  {
                    type: "image",
                    content: "/media/images/cableSus_time.png",
                     altContent: "Time required for IPOPT solve for different number of agents",
                    subtitle: "Time required for IPOPT solve for different number of agents.",
                 },
                    {
                    type: "image",
                    content: "/media/images/cableSus_knot.png",
                     altContent: "Graph showing runtime vs number of knot points",
                    subtitle: "Graph of runtime vs number of knot points.",
                 },
                   {
                    type: "image",
                     content: "/media/images/cableSus_constraint.png",
                     altContent: "Constraint violations across iterations",
                      subtitle: "Constraint violations across iterations of IPOPT.",
                     },
             {
               type: "text",
                content: "Doubling the knot points led to an 8-fold increase in solver time, highlighting the computational complexity of this problem. Due to this limitation, a 100-knot problem formulation could not be solved as the Jacobian matrix grew to approximately 54 million elements."
              },

        ]
    },
       {
        title: "IPOPT Limitations and ALTRO as an Alternative",
        navName: "Limitations & Alternatives",
        navRef: "limitations-alternatives",
        content: [
          {
            type: "text",
            content:
                "ALTRO (Augmented Lagrangian Trajectory Optimizer) is a trajectory optimization method that combines an augmented Lagrangian approach with an iterative LQR solver and an active-set method to achieve fast convergence for constrained problems. Research has shown that ALTRO performs competitively with direct collocation (DIRCOL) methods such as those using IPOPT. A key strength of ALTRO is its ability to be initialized with infeasible state trajectories, which is a major challenge for interior-point methods like IPOPT that typically require a feasible (or nearly feasible) starting point for robust convergence. This makes ALTRO a promising alternative for complex trajectory optimization problems like ours, especially when good initial guesses are difficult to obtain. Furthermore, ALTRO's constraint handling, particularly for problems involving obstacle avoidance or state-triggered constraints, can be more numerically stable and efficient than the barrier methods used in IPOPT.",
          },
            {
              type: "text",
                content: "During this project, we faced challenges with IPOPT's convergence, primarily due to the problem's high dimensionality, non-convexity, potential numerical instabilities from the dynamics and constraints, and the sensitivity to the initial guess. The lack of full convergence could be attributed to these factors, making exploration of solvers like ALTRO particularly relevant for future work."
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
