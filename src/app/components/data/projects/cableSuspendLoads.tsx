import { Project } from "@/app/components/project/interfaces";

export const cableSuspendedLoads: Project = {
  title: "Hybrid Control for Cable-Suspended Loads with Quadrotors",
  subtitle: "Multi-agent trajectory optimization with dynamic constraint handling",
  date: "Spring 2024",
  media: "/media/images/cableSus_4_agents.png",
  tags: ["Optimal Control", "IPOPT", "Trajectory Optimization", "UAVs", "Hybrid Systems", "Julia"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "Picture a heavy package being carried by multiple drones, each connected by cables. What happens if one drone fails? The cables go slack, and the system needs to adapt instantly. This project tackles that challenge: **how do you plan trajectories when cables can transition between taut and slack?** It's a hybrid control problem—the physics changes depending on cable state.",
        },
        {
          type: "text",
          content:
            "We used **trajectory optimization** (finding the best path through space and time) with a solver called IPOPT. The key innovation: an **active set method** that dynamically switches constraints when cables change state. Implementation was done in **Julia**.",
        },
        {
          type: "image",
          content: "/media/images/cableSuspension9_agents.png",
          altContent: "Diagram of multi-quadrotor cable-suspended load transport with hybrid control",
          subtitle: "Nine-agent cable-suspended load transport system",
        },
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
            "Using multiple drones to carry loads is attractive: they're flexible, easy to deploy, and can lift more together than alone. But previous work assumed cables always stay tight. In reality, cables go slack—when a drone fails, when there's turbulence, or when maneuvering aggressively. Standard trajectory optimizers like ALTRO struggle with this discontinuity.",
        },
        {
          type: "text",
          content:
            "Our solution: treat slack as a **mode switch**. Instead of separate physics models for 'all cables taut' vs 'some cables slack', we use one model with **switchable constraints**. The optimizer figures out when to enforce cable tension constraints and when to relax them.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Innovations",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Active set method** for dynamic constraint switching",
            "**Quaternion-based orientation** for aggressive maneuvers",
            "**Hybrid state machine** for taut/slack cable transitions",
            "Analysis of `IPOPT` performance in large-scale non-convex problems",
          ],
        },
      ],
    },
    {
      title: "Problem Formulation",
      navName: "Formulation",
      navRef: "problem-formulation",
      content: [
        {
          type: "text",
          content:
            "The math gets involved, but here's the intuition: each quadrotor has 13 states (position, orientation as a quaternion, linear and angular velocity), while the payload is just a point mass (6 states). Cables are modeled as massless rigid links—they transmit force but have no dynamics of their own.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Quadrotor Dynamics",
        },
        {
          type: "text",
          content: "Each drone's motion follows Newton's laws plus the cable force pulling on it:",
        },
        {
          type: "math",
          block: true,
          content: String.raw`\dot{x} = \begin{bmatrix} \dot{r} \\ \dot{q} \\ \dot{v} \\ \dot{\omega} \end{bmatrix} = \begin{bmatrix} v \\ \frac{1}{2} q \otimes \hat{\omega} \\ g + \frac{1}{m} \left( R(q) F(u) + F_c \right) \\ J^{-1} \left( \tau(u) - \omega \times J\omega \right) \end{bmatrix}`,
          subtitle: "Quadrotor state dynamics with cable force Fc",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Load Dynamics",
        },
        {
          type: "text",
          content: "The payload is modeled as a point mass with 6 states:",
        },
        {
          type: "math",
          block: true,
          content: String.raw`\dot{x}^{\ell} = \begin{bmatrix} \dot{r}^{\ell} \\ \dot{v}^{\ell} \end{bmatrix} = \begin{bmatrix} v^{\ell} \\ g + \frac{1}{m^{\ell}} \sum_{i=1}^{L} F_c^i \end{bmatrix}`,
          subtitle: "Load dynamics with cable forces from L active quadrotors",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Optimization Objective",
        },
        {
          type: "text",
          content: "The trajectory optimization minimizes a cost function penalizing state deviation, control effort, and terminal error:",
        },
        {
          type: "math",
          block: true,
          content: String.raw`J = \int_{0}^{T} \left[ (x - x_d)^T Q (x - x_d) + u^T R u \right] dt + \phi(x(T))`,
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Constraints",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Dynamics constraints** — discretized quadrotor and load dynamics",
            "**Boundary conditions** — initial and final state requirements", 
            "**Cable constraints** — fixed length geometric constraint",
            "**Tension constraints** — cables can only pull (modified by hybrid logic)",
            "**Motor limits** — respecting physical actuator capabilities",
            "**Collision avoidance** — safe distances between agents",
          ],
        },
      ],
    },
    {
      title: "Hybrid Control Strategy",
      navName: "Hybrid Control",
      navRef: "hybrid-control",
      content: [
        {
          type: "text",
          content:
            "Here's the heart of the project: the **active set method**. Think of it as a traffic cop for constraints. When a cable is taut, we enforce that it stays at its fixed length and can only pull (not push). When it goes slack, we drop those constraints—the cable is effectively disconnected. The optimizer solves the trajectory, checks each cable's tension, and updates which constraints are 'active' for the next solve.",
        },
        {
          type: "code",
          codeLang: "plaintext",
          content:
`HYBRID CONTROL ALGORITHM
========================

INITIALIZE:
  Set all cables to ACTIVE (taut)

OPTIMIZATION LOOP:
  1. Solve trajectory optimization
     - Apply full constraints for ACTIVE cables
     - Apply relaxed constraints for SLACK cables
  
  2. Evaluate each cable state:
     IF cable is ACTIVE:
       IF tension < threshold → set SLACK
     IF cable is SLACK:
       IF reattach conditions met → set ACTIVE
  
  3. IF any state changed:
       Re-solve with updated constraints
     ELSE:
       Apply control, advance to next step

REPEAT until trajectory complete`,
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Julia Implementation",
        },
        {
          type: "text",
          content:
            "The **conceptual Julia code** below shows how to define the IPOPT problem for this trajectory optimization:",
        },
        {
          type: "code",
          codeLang: "julia",
          content:
`using Ipopt

# Problem dimensions
num_knots = 50
num_quads = 4
nx_quad = 13    # states per quadrotor
nu_quad = 4     # controls per quadrotor
nx_load = 6     # load states

# Total decision variables
total_vars = num_knots * (
    num_quads * (nx_quad + nu_quad) + nx_load
)

# Objective: tracking error + control effort
function eval_f(x)
    cost = 0.0
    for k = 1:num_knots
        xq = get_quad_states(x, k)
        xl = get_load_state(x, k)
        u  = get_controls(x, k)
        cost += stage_cost(xq, xl, u)
    end
    return cost + terminal_cost(x)
end

# Constraints: dynamics, cables, bounds
function eval_g(x, g)
    idx = 1
    for k = 1:(num_knots - 1)
        # Quadrotor dynamics
        for i = 1:num_quads
            g[idx] = quad_dynamics(x, k, i)
            idx += 1
        end
        
        # Load dynamics
        g[idx] = load_dynamics(x, k)
        idx += 1
        
        # Cable constraints (hybrid)
        for i = 1:num_quads
            g[idx] = cable_length(x, k, i)
            g[idx+1] = tension(x, k, i)
            idx += 2
        end
    end
end

# Configure solver
prob = CreateProblem(total_vars, bounds...)
AddOption(prob, "mu_strategy", "adaptive")
AddOption(prob, "tol", 1e-4)
AddOption(prob, "max_iter", 1000)

# Solve
prob.x = initial_guess
status = IpoptSolve(prob)
solution = prob.x`
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Hybrid Control Flow",
        },
        {
          type: "text",
          content:
            "The hybrid control logic is an **iterative process**: all quadrotors start as *active*, trajectory optimization is solved, cable tensions are evaluated, state transitions occur as needed (active→slack or slack→active), and constraints are updated accordingly. If any configuration changes, the optimization re-solves with the new constraint set.",
        },
      ],
    },
     {
      title: "Simulation Results and Observations",
      navName: "Simulation Results",
      navRef: "simulation-results",
      content: [
        {
          type: "text",
          content:
            "We tested the framework on configurations from 4 to 9 quadrotors. The good news: IPOPT produces physically reasonable trajectories. The challenge: these problems are **highly non-convex** (many local minima), and the solver sometimes struggles to find the true optimum. Real-world robotics often means 'good enough' rather than mathematically perfect.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Findings",
        },
        {
          type: "text",
          content:
            "Qualitatively, the trajectories look right—drones spread out, maintain formation, and transport the load smoothly. Quantitatively, numerical convergence was difficult; the solver found feasible solutions but couldn't always reduce constraint violations to machine precision.",
        },
        {
          type: "image",
          content: "/media/images/cableSus_6_agent_plot.png",
          altContent: "Position plot for a 6 agent configuration",
          subtitle: "Position plot for a 6-agent configuration from start to goal.",
        },
        {
          type: "text",
          content:
            "Solver time varied significantly with agent count and problem complexity. Early iterations converged well, but later iterations struggled to reach full convergence.",
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
          displayAs: "subtitle",
          content: "Computational Scaling",
        },
        {
          type: "text",
          content:
            "Doubling the knot points led to an **8-fold increase** in solver time, highlighting the computational complexity. A 100-knot formulation was infeasible—the Jacobian matrix grew to approximately **54 million elements**.",
        },
      ],
    },
    {
      title: "IPOPT Limitations and ALTRO as an Alternative",
      navName: "Limitations & Alternatives",
      navRef: "limitations-alternatives",
      content: [
        {
          type: "text",
          content:
            "IPOPT is a general-purpose nonlinear optimizer—powerful but not specialized for robotics. **ALTRO** (Augmented Lagrangian Trajectory Optimizer) is designed specifically for trajectory problems and handles certain challenges better. If I were to continue this work, ALTRO would be the next thing to try.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "ALTRO Advantages",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Infeasible initialization** — can start from infeasible trajectories (vs. IPOPT needing near-feasible)",
            "**Constraint handling** — more numerically stable for obstacle avoidance and state-triggered constraints",
            "**Convergence** — less sensitive to initial guess quality",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Challenges with IPOPT",
        },
        {
          type: "text",
          content:
            "Our IPOPT implementation faced convergence challenges due to high dimensionality, non-convexity, numerical instabilities from dynamics/constraints, and sensitivity to initial guesses. These factors make ALTRO a promising alternative for future work.",
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
            "This project shows that cable slack—a real-world problem often ignored—can be handled systematically through hybrid control. The active set method provides a clean way to switch constraints based on physical state. While IPOPT has limitations for these highly non-convex problems, the framework itself is sound and could benefit from better solvers.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Future Directions",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Distributed optimization** — reduce problem size and improve scalability",
            "**Distributed slack handling** — improve adaptability in dynamic environments",
            "**ALTRO exploration** — address initialization and convergence challenges",
          ],
        },
      ],
    },
    ],
};
