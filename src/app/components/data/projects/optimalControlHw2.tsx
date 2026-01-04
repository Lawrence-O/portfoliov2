import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW2: Project = {
  title: "Optimal Control of Linear Systems",
  subtitle: "LQR, TVLQR, and MPC for trajectory optimization and tracking",
  date: "Spring 2024",
  media: "/media/videos/mpc_rendezvous.mp4",
  tags: ["Optimal Control", "LQR", "MPC", "Julia", "Robotics"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "How do you make a robot move 'optimally'—minimizing energy, time, or error? **Optimal control** provides the math. This project implements several foundational techniques: **LQR** (Linear Quadratic Regulator) finds the best feedback controller for linear systems, **TVLQR** extends this to track time-varying trajectories, and **MPC** (Model Predictive Control) handles constraints by replanning in real-time.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Systems Studied",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Double integrator** — position/velocity with acceleration control",
            "**Cartpole** — stabilization and swing-up trajectory tracking",
            "**Spacecraft rendezvous** — Clohessy-Wiltshire relative motion",
          ],
        },
      ],
    },
    {
      title: "Finite-Horizon LQR",
      navName: "Finite LQR",
      navRef: "finite-horizon-lqr",
      content: [
        {
          type: "text",
          content:
            "Given a fixed time window, what's the best way to steer a system to a goal? **Finite-Horizon LQR** answers this by minimizing a 'cost' that penalizes both being far from the goal (state error) and using too much control effort. The math is elegant: the optimal solution is a linear feedback law, and we can compute it by solving either a **convex optimization** problem or a **Riccati recursion** (working backwards from the final time). I tested this on a simple double integrator (think: a mass you can push).",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `cost = sum(quadform(X[:,k], Q) + quadform(U[:,k], R) for k=1:N-1)
cost += quadform(X[:,N], Qf)  # terminal cost

problem.constraints += X[:,1] == x_ic
for k=1:N-1
    problem.constraints += X[:,k+1] == A*X[:,k] + B*U[:,k]
end`,
        },
        {
          type: "image",
          content: "/media/images/finite_lqr_trajectory.png",
          altContent: "Finite Horizon LQR Trajectory",
          subtitle: "Double integrator trajectory using Finite-Horizon LQR",
        },
      ],
    },
    {
      title: "Bellman's Principle",
      navName: "Bellman",
      navRef: "bellmans-principle",
      content: [
        {
          type: "text",
          content:
            "Here's a beautiful insight: if you've found the optimal path from A to C (passing through B), then the portion from B to C is *also* optimal for that sub-problem. This is **Bellman's Principle of Optimality**—it's why dynamic programming works. I verified this experimentally: starting a new optimization from an intermediate state along the optimal trajectory gives the same remaining path.",
        },
        {
          type: "image",
          content: "/media/images/bellman_trajectory.png",
          altContent: "Bellman optimal trajectory validation",
          subtitle: "Trajectory comparison validating Bellman's Principle",
        },
      ],
    },
    {
      title: "Infinite-Horizon LQR",
      navName: "Infinite LQR",
      navRef: "infinite-horizon-lqr",
      content: [
        {
          type: "text",
          content:
            "What if there's no fixed end time—you just want to stay at a goal forever? **Infinite-Horizon LQR** handles this by finding a single, constant feedback gain that works for all time. The math converges to a steady-state solution of the Riccati equation (called DARE). I used this to balance a cart-pole at its unstable upright position—the controller constantly corrects small deviations.",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `for _ = 1:max_iters
    K = (R + B'*P*B) \\ (B'*P*A)
    P_new = Q + K'*R*K + (A-B*K)'*P*(A-B*K)
    norm(P_new - P) <= tol && return P_new, K
    P = P_new
end`,
        },
        {
          type: "text",
          content:
            "Not every starting position can be stabilized—if the pole starts too far from upright, no controller can save it. The **basin of attraction** shows which initial conditions lead to success. I also tuned the cost matrices Q and R to keep control forces within realistic motor limits.",
        },
        {
          type: "image",
          content: "/media/images/basin_of_attraction.png",
          altContent: "Basin of attraction for Cartpole LQR",
          subtitle: "Basin of attraction for the cartpole LQR controller",
        },
      ],
    },
    {
      title: "Time-Varying LQR (TVLQR)",
      navName: "TVLQR",
      navRef: "tvlqr",
      content: [
        {
          type: "text",
          content:
            "Regular LQR stabilizes around a single point. But what if you want to follow a moving reference—like a swing-up trajectory? **Time-Varying LQR (TVLQR)** computes different feedback gains for each moment in time by linearizing the system along the reference path. The controller combines the planned control (feedforward) with corrections for any deviations (feedback):",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `for k = N-1:-1:1
    K[k] = (R + B[k]'*P[k+1]*B[k]) \\ (B[k]'*P[k+1]*A[k])
    P[k] = Q + A[k]'*P[k+1]*(A[k] - B[k]*K[k])
end
# Control: u[k] = Uref[k] - K[k]*(x[k] - Xref[k])`,
        },
        {
          type: "image",
          content: "/media/images/tvlqr_trajectory.png",
          altContent: "TVLQR Trajectory for Cartpole Swing-up",
          subtitle: "Cartpole swing-up tracking using TVLQR",
        },
      ],
    },
    {
      title: "Spacecraft Rendezvous",
      navName: "Rendezvous",
      navRef: "rendezvous",
      content: [
        {
          type: "text",
          content:
            "How does a spacecraft approach another in orbit? The **Clohessy-Wiltshire equations** describe relative motion between two orbiting objects—they're linear, making them perfect for LQR. The goal is to reach the target spacecraft while using minimal fuel (thrust).",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**State** — relative position and velocity (6D)",
            "**Control** — thrust accelerations in each axis",
            "**Objective** — reach target while minimizing fuel",
          ],
        },
      ],
    },
    {
      title: "Convex Trajectory Optimization",
      navName: "Convex TrajOpt",
      navRef: "convex-trajopt",
      content: [
        {
          type: "text",
          content:
            "Real systems have limits: motors can only push so hard, spacecraft must avoid obstacles. **Convex optimization** lets us encode these as explicit constraints. The solver finds the best trajectory that respects *all* the rules—something basic LQR can't guarantee:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `problem.constraints += X[:,1] == x0
problem.constraints += X[:,N] == xg
for k=1:N-1
    problem.constraints += U[:,k] >= u_min
    problem.constraints += U[:,k] <= u_max
end`,
        },
        {
          type: "video",
          content: "/media/videos/convex_rendezvous.mp4",
          altContent: "Rendezvous using convex trajectory optimization",
          subtitle: "Spacecraft rendezvous with convex optimization",
        },
      ],
    },
    {
      title: "Model Predictive Control (MPC)",
      navName: "MPC",
      navRef: "mpc",
      content: [
        {
          type: "text",
          content:
            "What if conditions change? **Model Predictive Control (MPC)** replans at every timestep. It solves a finite-horizon optimization starting from wherever you currently are, applies only the first control action, then immediately replans with updated information. This 'receding horizon' approach handles disturbances and model errors gracefully—if something unexpected happens, you just replan:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `# At each timestep:
problem.constraints += X_pred[:,1] == x_current
solve!(problem)
u_apply = U_pred.value[:,1]  # apply first control only`,
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "MPC Advantages",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Constraint satisfaction** — bounds respected at each step",
            "**Disturbance rejection** — replanning corrects errors",
            "**Receding horizon** — computational tractability",
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
            "This project built up from simple feedback control to sophisticated constrained optimization. The progression shows how each technique addresses different challenges:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**LQR** — the foundation: optimal feedback for staying at a goal",
            "**TVLQR** — follow a planned trajectory with time-varying corrections",
            "**MPC** — handle constraints and adapt to real-time changes",
            "**Convex optimization** — the unifying framework that makes it all tractable",
          ],
        },
      ],
    },
  ],
};
