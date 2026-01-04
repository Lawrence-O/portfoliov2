import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW3: Project = {
  title: "Trajectory Optimization: DIRCOL and iLQR",
  subtitle: "Direct collocation and iterative LQR for nonlinear systems",
  media: "/media/images/quadrotor_reorient.gif",
  date: "Spring 2024",
  tags: ["Optimal Control", "iLQR", "DIRCOL", "Julia"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "overview",
      content: [
        {
          type: "text",
          content:
            "How do you compute the best path for a robot to follow? This project explores **trajectory optimization**—algorithms that find optimal motion plans by minimizing cost (like energy or time) while respecting physical constraints. I implemented two approaches: **Direct Collocation (DIRCOL)**, which converts the problem into a large system of equations solved all at once, and **iterative LQR (iLQR)**, which refines a trajectory through repeated local improvements. Both generate open-loop plans, which I then track using **Time-Varying LQR (TVLQR)** feedback control.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Systems and Methods",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Cart-pole swing-up** — DIRCOL with Hermite-Simpson integration",
            "**Quadrotor aerobatics** — iLQR with RK4 discretization",
            "**Multi-quadrotor** — collision avoidance via inequality constraints",
            "**TVLQR tracking** — closed-loop control with model mismatch",
          ],
        },
      ],
    },
    {
      title: "DIRCOL for Cart-Pole",
      navName: "DIRCOL",
      navRef: "dircol-cartpole",
      content: [
        {
          type: "text",
          content:
            "**Direct Collocation** transforms the continuous 'find the best trajectory' problem into a finite set of decision variables that standard optimization solvers can handle. Instead of solving differential equations directly, we represent the trajectory as a sequence of states and controls at discrete time points. **Hermite-Simpson integration** connects these points with smooth curves that approximate the true physics. The solver (IPOPT) then finds values that minimize cost while satisfying all constraints.",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `# Stage cost: quadratic in state error and control
for i = 1:(N-1)
    J += 0.5 * (x[i] - xg)' * Q * (x[i] - xg)
    J += 0.5 * u[i]' * R * u[i]
end
J += 0.5 * (x[N] - xg)' * Qf * (x[N] - xg)`,
        },
        {
          type: "text",
          content:
            "Equality constraints enforce dynamics (via collocation) plus boundary conditions:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `constraints = [x[1] - x_ic;      # initial state
               x[N] - x_goal;    # terminal state  
               dynamics_residuals]  # Hermite-Simpson`,
        },
      ],
    },
    {
      title: "Cart-Pole Swing-Up",
      navName: "Swing-Up",
      navRef: "cartpole-swingup",
      content: [
        {
          type: "text",
          content:
            "The cart-pole is a classic control problem: balance a pole on a moving cart by pushing the cart left or right. The challenge is swinging the pole from hanging down to standing upright—a maneuver requiring carefully timed forces. **Hermite-Simpson integration** approximates the physics between time steps using cubic polynomials, giving much better accuracy than simple Euler steps:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `function hermite_simpson(p, x1, x2, u, dt)
    xm = 0.5*(x1+x2) + (dt/8)*(f(x1,u) - f(x2,u))
    return x1 + (dt/6)*(f(x1,u) + 4*f(xm,u) + f(x2,u)) - x2
end`,
        },
        {
          type: "image",
          content: "/media/images/cartpole_states.png",
          altContent: "State trajectory of the cartpole swingup",
          subtitle: "Cart-pole state trajectory from DIRCOL",
        },
        {
          type: "video",
          content: "/media/videos/cartpole_video.mp4",
          altContent: "Animation of the cartpole swingup maneuver",
          subtitle: "Cart-pole swing-up animation",
        },
      ],
    },
    {
      title: "TVLQR Tracking",
      navName: "TVLQR",
      navRef: "tvlqr-tracking",
      content: [
        {
          type: "text",
          content:
            "The DIRCOL solution gives us an open-loop plan—a pre-computed sequence of controls. But real systems drift from the plan due to disturbances and modeling errors. **TVLQR** adds feedback: it computes correction gains at each time step that push the system back toward the planned trajectory. The controller applies both the planned control (feedforward) and error corrections (feedback). I simulate the closed-loop system using **RK4** (Runge-Kutta 4th order), a standard numerical integration method:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `function rk4(p, x, u, dt)
    k1 = dt * f(x, u)
    k2 = dt * f(x + k1/2, u)
    k3 = dt * f(x + k2/2, u)
    k4 = dt * f(x + k3, u)
    return x + (k1 + 2*k2 + 2*k3 + k4)/6
end`,
        },
        {
          type: "image",
          content: "/media/images/cartpole_tvlqr_states.png",
          altContent: "State trajectory of the cartpole with TVLQR tracking",
          subtitle: "TVLQR tracking with model mismatch",
        },
      ],
    },
    {
      title: "iLQR for Quadrotor",
      navName: "iLQR",
      navRef: "ilqr-quadrotor",
      content: [
        {
          type: "text",
          content:
            "While DIRCOL solves everything at once, **iterative LQR (iLQR)** takes a different approach: start with an initial guess trajectory, then repeatedly improve it. Each iteration approximates the nonlinear system as locally linear around the current trajectory, solves the simpler linear problem (standard LQR), then updates the trajectory. This process converges to a locally optimal solution. Here I apply it to a 6-DOF quadrotor performing aerobatic maneuvers.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Cost Expansion",
        },
        {
          type: "text",
          content:
            "iLQR needs to know how the cost changes as we adjust states and controls. We compute these **gradients** (first derivatives) and **Hessians** (second derivatives) using automatic differentiation—the computer calculates exact derivatives by tracking operations:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `Qxx = FD.hessian(dx -> stage_cost(dx, u, k), x)
Qx  = FD.gradient(dx -> stage_cost(dx, u, k), x)
Quu = FD.hessian(du -> stage_cost(x, du, k), u)
Qu  = FD.gradient(du -> stage_cost(x, du, k), u)`,
        },
        {
          type: "image",
          content: "/media/images/quadrotor_trajectories.png",
          altContent: "Trajectory of quadrotor from iLQR",
          subtitle: "Quadrotor aerobatic trajectory via iLQR",
        },
      ],
    },
    {
      title: "Quadrotor Tracking",
      navName: "Quad Tracking",
      navRef: "quad-tvlqr",
      content: [
        {
          type: "text",
          content:
            "Just like with the cart-pole, the iLQR trajectory is open-loop and needs feedback for robustness. I apply TVLQR to track the aerobatic maneuver, testing with intentional parameter mismatches (e.g., wrong mass estimates) to verify the controller corrects for modeling errors.",
        },
        {
          type: "image",
          content: "/media/images/quadrotor_orientations.png",
          altContent: "Orientation of the quadrotor with TVLQR",
          subtitle: "Quadrotor orientation during TVLQR tracking",
        },
      ],
    },
    {
      title: "Multi-Quadrotor Collision Avoidance",
      navName: "Collision Avoidance",
      navRef: "collision-avoidance",
      content: [
        {
          type: "text",
          content:
            "The final challenge: coordinate three quadrotors swapping positions without crashing into each other. I extend DIRCOL with **inequality constraints** that require each pair of quadrotors to stay at least a minimum distance apart at every time step. The optimizer finds trajectories that complete the reorientation while respecting these safety margins:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `# Distance constraints (squared to avoid sqrt)
for i = 1:N-1
    px1, px2, px3 = x[i][1:2], x[i][7:8], x[i][13:14]
    c[i*3-2] = norm(px1-px2)^2 - d_min^2  # >= 0
    c[i*3-1] = norm(px2-px3)^2 - d_min^2
    c[i*3]   = norm(px1-px3)^2 - d_min^2
end`,
        },
        {
          type: "image",
          content: "/media/images/quadrotor_distances.png",
          altContent: "Distance between the 3 quadrotors",
          subtitle: "Inter-quadrotor distances during maneuver",
        },
        {
          type: "image",
          content: "/media/images/quadrotor_reorient.gif",
          altContent: "Animation of the three quadrotors reorienting",
          subtitle: "Three quadrotors reorienting with collision avoidance",
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
            "This project built up a toolkit for robot motion planning, from computing optimal trajectories to tracking them reliably:",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**DIRCOL** converts continuous motion planning into a solvable optimization problem",
            "**iLQR** finds optimal trajectories through iterative refinement",
            "**TVLQR** adds feedback control for robust real-world tracking",
            "**Inequality constraints** enforce safety requirements like collision avoidance",
          ],
        },
      ],
    },
  ],
};