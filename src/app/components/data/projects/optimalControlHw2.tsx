import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW2: Project = {
    title: "Optimal Control of Linear Systems: LQR, TVLQR, and MPC",
    date: "Spring 2024",
    media: "/media/videos/mpc_rendezvous.mp4",
    githubLink: "https://github.com/your-username/optimal-control-hw2",
    tags: ["Optimal Control", "LQR", "TVLQR", "Convex Optimization", "MPC", "Julia", "State Space Control", "Robotics"],
    section: [
        {
            title: "Project Overview: Optimal Control Techniques",
            navName: "Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content: [
                        "This project explores a variety of optimal control techniques applied to different linear and linearized systems. Key methods covered include Finite-Horizon Linear Quadratic Regulator (LQR), Infinite-Horizon LQR, Time-Varying LQR (TVLQR), and Model Predictive Control (MPC).",
                        "The project employs several dynamic models: a double integrator, a cartpole (for stabilization and swing-up trajectory tracking), and a spacecraft rendezvous model (Clohessy-Wiltshire equations). Implementations are primarily in Julia, applied to both continuous and discrete-time systems. The core objective is to design and simulate controllers achieving specific performance criteria through state-space methodologies."
                    ]
                },
            ],
        },
        {
            title: "Finite-Horizon LQR Implementation",
            navName: "Finite-Horizon LQR",
            navRef: "finite-horizon-lqr",
            content: [
                {
                    type: "text",
                    content: [
                         "The project starts with a Finite-Horizon LQR problem, solved via convex optimization and Riccati recursion. A double integrator model (position/velocity states, acceleration controls) was used. The continuous-time dynamics ('Ac', 'Bc') were discretized using the matrix exponential method to get discrete-time matrices ('A', 'B') for controller design."
                    ]
                },
                {
                    type: "code",
                    codeLang: "julia",
                    content: `
# Solves finite-horizon LQR using convex optimization (Convex.jl)
function convex_lqr_trajopt(A, B, Q, R, Qf, N, x_ic)
    nx, nu = size(B)
    X = Convex.Variable(nx, N) # State trajectory
    U = Convex.Variable(nu, N-1) # Control trajectory

    # Cost: sum of quadratic state/control costs + terminal state cost
    cost = sum(0.5*quadform(X[:,k], Q) + 0.5*quadform(U[:,k], R) for k=1:N-1)
    cost += 0.5*quadform(X[:,N], Qf)

    problem = minimize(cost)
    problem.constraints += X[:,1] == x_ic # Initial condition
    for k=1:N-1 # Dynamics constraints
        problem.constraints += X[:,k+1] == A*X[:,k] + B*U[:,k]
    end
    
    Convex.solve!(problem, ECOS.Optimizer; silent_solver=true)
    return Matrix(X.value), Matrix(U.value)
end
                    `,
                    subtitle: "Julia: LQR via convex optimization (simplified).",
                },
                  {
                     type: "image",
                    content: "/media/images/finite_lqr_trajectory.png",
                     altContent: "Finite Horizon LQR Trajectory",
                    subtitle: "Double integrator trajectory using Finite Horizon LQR."
                 },
            ]
        },
        {
             title: "Validation of Bellman's Principle of Optimality",
             navName: "Bellman's Principle",
            navRef: "bellmans-principle",
             content: [
                {
                    type: "text",
                     content: "Bellman’s Principle of Optimality was validated by re-solving the LQR problem with initial conditions from an intermediate point of a previously computed optimal trajectory. The new trajectory matched the corresponding subsection of the original, confirming the principle."
                   },
                 {
                    type: "image",
                     content: "/media/images/bellman_trajectory.png",
                     altContent: "Bellman optimal trajectory validation",
                     subtitle: "Trajectory comparison validating Bellman's Principle."
                   },
             ]
        },
        {
           title: "Infinite-Horizon LQR for Cartpole Stabilization",
           navName: "Infinite-Horizon LQR",
            navRef: "infinite-horizon-lqr",
            content: [
               {
                    type: "text",
                     content: [
                        "Infinite-Horizon LQR was used to find a time-invariant state feedback `u = -Kx`. This involved solving the Discrete Algebraic Riccati Equation (DARE) via iteration until the cost-to-go matrix 'P' converged. The resulting gain 'K' stabilized a cartpole system around its unstable upward equilibrium. The analysis included using different estimated parameters to simulate model uncertainty."
                     ]
                   },
                {
                    type: "code",
                    codeLang: "julia",
                    content: `
# Solves infinite-horizon LQR using Riccati recursion (DARE)
function infinite_horizon_lqr(A, B, Q, R; tol=1e-5, max_iters=1000)
    P_prev = Q 
    P_curr = Q
    K_curr = zeros(size(B,2), size(A,1))

    for _ = 1:max_iters
        K_curr = (R + B' * P_prev * B) \\ (B' * P_prev * A) # Calculate gain
        P_curr = Q + K_curr'*R*K_curr + (A-B*K_curr)'*P_prev*(A-B*K_curr) # Update P

        if norm(P_curr - P_prev) <= tol
            return P_curr, K_curr # Converged
        end
        P_prev = P_curr
    end
    error("Infinite-horizon LQR did not converge")
end
    `               ,
                     subtitle: "Julia: Solving infinite-horizon LQR via DARE iteration."
                },
                {
                    type: "text",
                     content: "The basin of attraction was explored by varying initial conditions to find the limits of the controller, showing how the linearized model's accuracy affects stability. Cost tuning (Q and R matrices) was also analyzed to respect actuator limits."
                },
                {
                    type: "image",
                    content: "/media/images/basin_of_attraction.png",
                    altContent: "Basin of attraction for Cartpole LQR",
                    subtitle: "Basin of attraction for the cartpole LQR controller."
                },
            ]
        },
        {
            title: "Trajectory Tracking with Time-Varying LQR (TVLQR)",
            navName: "TVLQR Tracking",
            navRef: "trajectory-tracking-tvlqr",
            content: [
                  {
                    type: "text",
                    content: [
                         "Time-Varying LQR (TVLQR) was implemented for cartpole swing-up trajectory tracking. This involved linearizing system dynamics ('A_k', 'B_k') around a reference state ('Xbar') and control ('Ubar') trajectory at each time step 'k'. A backward Riccati recursion computed time-varying gains 'K_k' and cost-to-go matrices 'P_k'. The controller `u_k = Ubar_k - K_k * (X_k - Xbar_k)` was tested with mismatched true and estimated parameters."
                    ]
                },
                {
                     type: "code",
                    codeLang: "julia",
                    content: `
# Backward Riccati recursion for TVLQR gains (simplified)
# P_N = Qf (terminal cost-to-go)
# A_k, B_k are Jacobians at Xbar[k], Ubar[k]

for k = N-1:-1:1 # Iterate backwards from N-1 to 1
    # K_k = (R + B_k' * P[k+1] * B_k)^-1 * (B_k' * P[k+1] * A_k)
    K_tvlqr[k] = (R + B_k' * P[k+1] * B_k) \\ (B_k' * P[k+1] * A_k)
    
    # P_k = Q + K_tvlqr[k]'*R*K_tvlqr[k] + (A_k - B_k*K_tvlqr[k])'*P[k+1]*(A_k-B_k*K_tvlqr[k])
    P[k] = Q + A_k' * P[k+1] * (A_k - B_k * K_tvlqr[k]) // Simpler form for illustration
end

# Forward simulation uses u_k = Ubar[k] - K_tvlqr[k] * (X_sim[k] - Xbar[k])
# ... (simulation described in text)
                        `,
                    subtitle: "Julia: TVLQR backward Riccati recursion (core logic)."
                },
                {
                    type: "image",
                    content: "/media/images/tvlqr_trajectory.png",
                    altContent: "TVLQR Trajectory for Cartpole Swing-up",
                    subtitle: "Cartpole swing-up tracking using TVLQR."
                }
             ]
        },
        {
            title: "Optimal Spacecraft Rendezvous and Docking with LQR",
            navName: "Rendezvous LQR",
            navRef: "rendezvous-lqr",
            content: [
               {
                    type: "text",
                    content: [
                         "This section addresses a spacecraft rendezvous problem using optimal control. The system dynamics are modeled by the Clohessy-Wiltshire (CW) equations, describing relative motion between two orbiting objects (e.g., chaser and target like ISS). These continuous-time linear equations (defined by mean motion 'n') are first discretized.",
                         "The CW state matrix 'Ac' includes terms like `3n^2` and `2n`, and the input matrix 'Bc' typically allows control over relative accelerations. After discretization to 'Ad' and 'Bd', a Finite-Horizon LQR controller is designed to achieve rendezvous while respecting control input limits."
                    ]
                    },
            ]
        },
        {
             title: "Rendezvous with Convex Trajectory Optimization",
             navName: "Convex TrajOpt",
            navRef: "convex-trajectory-optimization-rendezvous",
             content: [
                {
                    type: "text",
                    content: [
                         "The rendezvous problem was also addressed using convex trajectory optimization. This allowed explicit inclusion of constraints such as control input bounds (`u_min`, `u_max`) and collision avoidance. The optimization minimized an LQR-like cost subject to initial/goal states, system dynamics, and these path constraints."
                    ]
                    },
                    {
                        type: "code",
                        codeLang: "julia",
                        content: `
# Solves rendezvous trajectory optimization with constraints (Convex.jl)
function constrained_convex_trajopt(Ad, Bd, Q, R, X_ref, x0, xg, u_min, u_max, N)
    # ... (Variable setup for X_cvx, U_cvx) ...
    X_cvx = Convex.Variable(size(Ad,1), N) 
    U_cvx = Convex.Variable(size(Bd,2), N-1)

    cost = sum(0.5*quadform(X_cvx[:,k]-X_ref[:,k],Q) + 0.5*quadform(U_cvx[:,k],R) for k=1:N-1)
    cost += 0.5*quadform(X_cvx[:,N]-X_ref[:,N], Q) # Terminal cost

    problem = minimize(cost)
    problem.constraints += X_cvx[:,1] == x0   # Initial state
    problem.constraints += X_cvx[:,N] == xg   # Terminal state
    
    for k=1:N-1 # Dynamics and control bounds
        problem.constraints += X_cvx[:,k+1] == Ad*X_cvx[:,k] + Bd*U_cvx[:,k]
        problem.constraints += U_cvx[:,k] >= u_min 
        problem.constraints += U_cvx[:,k] <= u_max
        # ... (Other path constraints e.g. keep-out zones) ...
    end
    
    Convex.solve!(problem, ECOS.Optimizer; silent_solver=true)
    return Matrix(X_cvx.value), Matrix(U_cvx.value)
end
                        `,
                        subtitle: "Julia: Convex trajectory optimization for constrained rendezvous."
                      },
                     {
                     type: "video",
                    content: "/media/videos/convex_rendezvous.mp4", // This video is distinct from the main project video
                    altContent: "Rendezvous using convex trajectory optimization",
                     subtitle: "Spacecraft rendezvous using convex trajectory optimization."
                   },
            ]
        },
        {
            title: "Trajectory Tracking with Convex Model Predictive Control (MPC)",
            navName: "Convex MPC",
            navRef: "convex-mpc-rendezvous",
            content: [
                 {
                    type: "text",
                    content: [
                        "Convex MPC was implemented for robust trajectory tracking in the rendezvous scenario. At each time step, MPC solves a finite-horizon optimal control problem over a prediction window ('N_mpc'), using the current state ('x_current') as the initial condition to track a segment of the reference ('X_ref_window'). Only the first control input from the optimized sequence is applied. This replanning at each step allows MPC to react to disturbances and model mismatch."
                    ]
                    },
                   {
                       type: "code",
                        codeLang: "julia",
                       content: `
# Solves one step of Convex MPC for trajectory tracking (Convex.jl)
function convex_mpc_step(Ad, Bd, Q, R, X_ref_window, x_current, u_min, u_max, N_mpc)
    # ... (Variable setup for X_pred, U_pred over N_mpc horizon) ...
    X_pred = Convex.Variable(size(Ad,1), N_mpc)
    U_pred = Convex.Variable(size(Bd,2), N_mpc-1)

    cost = sum(0.5*quadform(X_pred[:,k]-X_ref_window[:,k],Q) + 0.5*quadform(U_pred[:,k],R) for k=1:N_mpc-1)
    cost += 0.5*quadform(X_pred[:,N_mpc]-X_ref_window[:,N_mpc], Q) # Window terminal cost

    problem = minimize(cost)
    problem.constraints += X_pred[:,1] == x_current # Initial condition for this window
    
    for k=1:N_mpc-1 # Dynamics and control bounds for the window
        problem.constraints += X_pred[:,k+1] == Ad*X_pred[:,k] + Bd*U_pred[:,k]
        problem.constraints += U_pred[:,k] >= u_min
        problem.constraints += U_pred[:,k] <= u_max
    end

    Convex.solve!(problem, ECOS.Optimizer; silent_solver=true)
    # ... (Error handling and return U_pred.value[:,1]) ...
    if problem.status == Convex.OPTIMAL || problem.status == Convex.ALMOST_OPTIMAL
        return U_pred.value[:,1] 
    else
        return zeros(size(Bd,2)) # Fallback
    end
end
                        `,
                      subtitle: "Julia: Single step of Convex MPC for constrained tracking."
                     }
                     // Video block for mpc_rendezvous.mp4 was here, removed as it's the main project video.
             ]
         },
        {
            title: "Conclusion and Key Learnings",
            navName: "Conclusion",
            navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content: [
                        "This project demonstrated various optimal control techniques, including LQR, TVLQR, and MPC, applied to linear and linearized systems like double integrators, cartpoles, and spacecraft rendezvous models. Key learnings include the importance of system modeling, discretization, the trade-offs in controller complexity, the utility of convex optimization for constraints, and MPC's robustness."
                    ]
                }
            ]
        }
    ]
};
