import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW4: Project = {
  title: "Iterative Learning and Hybrid Control",
  subtitle: "Learning from repetition and handling discontinuous dynamics",
  media: "/media/videos/biped_walking.mp4",
  date: "Spring 2024",
  tags: ["Optimal Control", "ILC", "Hybrid Systems", "Julia"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content:
            "Some problems are best solved by practice: try something, see what went wrong, adjust, and try again. **Iterative Learning Control (ILC)** formalizes this intuition—the controller learns from each attempt to do better next time. Meanwhile, **hybrid systems** have dynamics that suddenly change (like a foot hitting the ground while walking). This project tackles both: ILC for a car's evasive maneuver, and hybrid trajectory optimization for bipedal walking.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Topics Explored",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Iterative Learning Control** — refine trajectories by learning from previous runs",
            "**Hybrid dynamics** — handle discrete mode switches (foot strikes)",
            "**Bipedal walking** — optimize gait with contact constraints",
            "**Nonlinear bicycle model** — realistic car dynamics",
          ],
        },
      ],
    },
    {
      title: "Iterative Learning Control for Car Maneuver",
      navName: "ILC Car",
      navRef: "ilc-car",
      content: [
        {
          type: "text",
          content:
            "Picture a driver practicing a 'moose test'—that sudden swerve to avoid an obstacle. Each run, they get a little better. **ILC** does the same thing mathematically: simulate the car, compare to the desired path, compute corrections, and repeat. After a few iterations, the controller has learned to nail the maneuver.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Bicycle Model Dynamics",
        },
        {
          type: "text",
          content:
            "We model the car using a **nonlinear bicycle model**—a simplification that captures steering and velocity dynamics without modeling each wheel individually:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `function car_dynamics(model, x, u)
    px, py, θ, δ, v = x  # position, heading, steering, velocity
    a, δdot = u          # acceleration, steering rate
    
    β = atan(model.lr * δ, model.L)  # slip angle
    xdot = [v*cos(θ+β), v*sin(θ+β),  # velocity components
            v*cos(β)*tan(δ)/model.L,  # yaw rate
            δdot, a]                   # steering/accel
    return xdot
end`,
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "ILC Update Step",
        },
        {
          type: "text",
          content:
            "Each iteration, we linearize around the current trajectory and solve a convex optimization to find control corrections that reduce tracking error:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `# ILC update: minimize tracking error via convex optimization
ΔX, ΔU = cvx.Variable(nx, N), cvx.Variable(nu, N-1)
cost = sum(quadform(X[k] + ΔX[:,k] - Xref[k], Q) for k=1:N)
constraints = [ΔX[:,1] == 0]  # start from actual state
for k = 1:N-1
    constraints += ΔX[:,k+1] == A[k]*ΔX[:,k] + B[k]*ΔU[:,k]
end
solve!(minimize(cost), constraints)`,
        },
        {
          type: "image",
          content: "/media/images/ilc_trajectory.png",
          altContent: "Trajectory of the car with ILC",
          subtitle: "Car trajectory converging to reference over ILC iterations",
        },
        {
          type: "video",
          content: "/media/videos/ilc_demo.mp4",
          altContent: "ILC demonstration video",
          subtitle: "Moose test maneuver learned via ILC",
        },
      ],
    },
    {
      title: "Hybrid Trajectory Optimization for Bipedal Walking",
      navName: "Bipedal Walking",
      navRef: "hybrid-trajectory-optimization",
      content: [
        {
          type: "text",
          content:
            "Walking is fundamentally **hybrid**: smooth leg swings interrupted by sudden foot strikes. The physics changes instantly when a foot hits the ground—velocities reset, forces redistribute. Standard trajectory optimization assumes smooth dynamics, so we need special handling for these **discrete jumps** between modes.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Three-Mass Model",
        },
        {
          type: "text",
          content:
            "The biped is simplified to three point masses: one body, two feet. Each leg acts like a telescoping rod that can push but not pull. The dynamics switch depending on which foot is planted:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `# Stance dynamics depend on which foot is grounded
if k in stance1_phase
    xdot = stance1_dynamics(model, x, u)  # foot 1 planted
elseif k in stance2_phase
    xdot = stance2_dynamics(model, x, u)  # foot 2 planted
end`,
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Jump Maps (Foot Strikes)",
        },
        {
          type: "text",
          content:
            "When a swing foot hits the ground, it's an **inelastic collision**—the foot's velocity instantly goes to zero while other states continue. This discrete reset is the 'jump map':",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `function jump1_map(x)
    # Foot 1 hits ground: zero out its velocity
    return [x[1:8]; 0.0; 0.0; x[11:12]]
end`,
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Trajectory Optimization",
        },
        {
          type: "text",
          content:
            "We use IPOPT to find a trajectory that tracks a reference gait while respecting hybrid dynamics. The optimizer enforces that stance foot velocity stays zero and leg lengths stay within bounds:",
        },
        {
          type: "code",
          codeLang: "julia",
          content: `# Dynamics constraints switch based on gait phase
for k = 1:N-1
    if k in M1 && !(k in J1)      # foot 1 stance
        c[k] = x[k+1] - rk4(stance1_dynamics, x[k], u[k])
    elseif k in J1                 # foot 1 strike
        c[k] = x[k+1] - jump2_map(rk4(stance1_dynamics, ...))
    end
end`,
        },
        {
          type: "video",
          content: "/media/videos/biped_walking.mp4",
          altContent: "Bipedal walking animation",
          subtitle: "Optimized bipedal walking gait",
        },
        {
          type: "image",
          content: "/media/images/biped_walking_positions.png",
          altContent: "Body positions during walking",
          subtitle: "Body and foot trajectories during one gait cycle",
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
            "This project explored two powerful ideas: learning from repetition and handling systems that suddenly change behavior. Both are essential for real-world robotics.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**ILC** improves control by learning from each trial—no perfect model needed",
            "**Hybrid dynamics** capture the reality of contact: smooth motion interrupted by impacts",
            "**Jump maps** handle the instantaneous velocity resets at foot strikes",
            "**Mode-dependent constraints** let the optimizer know which foot is planted when",
          ],
        },
        {
          type: "text",
          content:
            "These techniques extend optimal control to messy real-world problems where models are imperfect and physics has discontinuities.",
        },
      ],
    },
  ],
};