import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW2: Project = {
    title: "Optimal Control of Linear Systems: LQR, TVLQR, and MPC",
    subtitle: "Coursework - Spring 2024",
    media: "/media/images/optimalControlHW.png",
    tags: ["Optimal Control", "LQR", "Convex Optimization", "MPC", "Julia", "State Space Control"],
    section: [
        {
            title: "Project Overview",
            navName: "Project Overview",
            navRef: "project-overview",
            content: [
                {
                    type: "text",
                    content:
                        "This project explores various optimal control techniques applied to different linear systems, including Finite-Horizon Linear Quadratic Regulator (LQR), Infinite-Horizon LQR, and Model Predictive Control (MPC). The project employs several models to demonstrate these techniques, including a double integrator model, a cartpole model, and a rendezvous docking model. The use of the Julia programming language allows for the implementation of these techniques on both continuous and discrete time linear systems. The core objective of this project is to design and simulate controllers that meet specific performance criteria through the use of state-space control methodologies.",
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
                    content:
                         "The project starts with a Finite-Horizon LQR problem, which is solved using both Convex Optimization and Riccati recursion. The control system was developed using a double integrator model, where the position and velocity of an object are the states, and the acceleration of the object are the controls. We then compared the performance of these two approaches, noting that the implementation of the controller required discretizing the continuous time dynamics of the double integrator.",
                },
                  {
                    type: "code",
                    codeLang: "julia",
                    content: `
# double integrator dynamics
"""
Discretizes the continuous-time double integrator dynamics using the matrix exponential.
# Arguments
- \`dt\`: The time step.
# Returns
A tuple containing the discrete-time state matrix (A) and input matrix (B).
"""
function double_integrator_AB(dt::Real):: Tuple{Matrix, Matrix}
    # Continuous-time state matrix
    Ac = [0 0 1 0;
            0 0 0 1;
            0 0 0 0;
            0 0 0 0.]
    # Continuous-time input matrix
    Bc = [0 0;
            0 0;
            1 0;
            0 1]
    nx, nu = size(Bc)
    # Augment the matrices to apply the matrix exponential
    AB_square = [Ac Bc; zeros(nu, nx + nu)]
    # Apply the matrix exponential
    AB_discrete = exp(AB_square*dt)
    # Extract the discrete-time A and B matrices
    A = AB_discrete[1:nx, 1:nx]
    B = AB_discrete[1:nx, nx+1:end]
    return A, B
end
                    `
                     ,
                    subtitle: "Julia code for discretizing the double integrator system."
                  },
                  {
                     type: "image",
                    content: "/media/images/finite_lqr_trajectory.png",
                     altContent: "Finite Horizon LQR Trajectory",
                    subtitle: "Trajectory of the double integrator system using Finite Horizon LQR."
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
                     content:
                         "Bellman’s Principle of Optimality was explored by solving the Finite-Horizon LQR problem again, but with updated initial conditions. The results of this test showed that subsections of the resulting optimal trajectories are also optimal, which validated Bellman's Principle of Optimality. "
                   },
                   {
                       type: "code",
                        codeLang: "julia",
                        content: `
"""
    Solves a finite-horizon LQR problem using convex optimization.
"""
function convex_trajopt(A:: Matrix, B:: Matrix, Q::Matrix, R::Matrix, Qf:: Matrix, N:: Int64, x_ic:: Vector; verbose = false)::Tuple{Vector{Vector{Float64}},Vector{Vector{Float64}}}
    nx, nu = size(B)
    X = cvx.Variable(nx, N)
    U = cvx.Variable(nu, N-1)
    cost = 0
    for k = 1:(N-1)
        x_k = X[:,k]
        u_k = U[:,k]
        cost += 0.5*cvx.quadform(x_k,Q)
        cost += 0.5*cvx.quadform(u_k, R)
    end
    cost += 0.5*cvx.quadform(X[:,N],Qf)
    prob = cvx.minimize(cost)
    prob.constraints += X[:,1] == x_ic
    for k = 1: (N-1)
        x_k = X[:,k]
            u_k = U[:,k]
        prob.constraints += A*x_k + B*u_k == X[:, k+1]
    end
    cvx.solve!(prob, ECOS.Optimizer; silent_solver = !verbose)
    X = vec_from_mat(X.value)
    U = vec_from_mat(U.value)
    return X,U
end
                         `
                        ,
                        subtitle: "Implementation of the Convex Trajectory Optimization algorithm."
                   },
                 {
                    type: "image",
                     content: "/media/images/bellman_trajectory.png",
                     altContent: "Bellman optimal trajectory",
                     subtitle: "Trajectory of the double integrator system, validating Bellman's Principle of Optimality."
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
                     content:
                        "The project then explores Infinite-Horizon LQR, where a Riccati recursion was used to achieve a steady-state gain, which was then applied to the cartpole system. This controller was designed to stabilize the cartpole around its unstable equilibrium point at the origin, with the pole pointing straight up. The analysis was performed using different estimated parameter sets to simulate real-world uncertainty. The goal was to stabilize the cartpole while respecting the physical limitations of the system.",
                   },
                {
                    type: "code",
                    codeLang: "julia",
                    content: `
"""
Solves the infinite-horizon LQR problem using Riccati recursion.
    """
function ihlqr (A:: Matrix, B:: Matrix, Q::Matrix, R::Matrix, tol = 1e-5, max_iters=1000)
        P_prev = deepcopy (Q)
        for i = 1:max_iters
        K = (R + B'*P_prev*B) \\ B' *P_prev*A
        P = Q + A'*P_prev* (A - B*K)
            if norm(P - P_prev) <= tol
            return P, K
        end
        P_prev = P
    end
        error("ihlqr did not converge")
    end
    `
                     ,
                     subtitle: "Julia code for solving infinite horizon LQR."
                },
                    {
                    type: "text",
                     content:
                        "The concept of the basin of attraction was explored by varying the initial conditions to determine the limits of the controller. The results show how the linearized model is an approximation that is inaccurate at points far from the linearization point. It also demonstrated that the controller will not work if initial conditions are too far from the equilibrium.",
                   },
                     {
                    type: "image",
                     content: "/media/images/basin_of_attraction.png",
                     altContent: "Basin of attraction of the Infinite Horizon LQR",
                      subtitle: "Visual representation of the basin of attraction for the Infinite Horizon LQR controller."
                     },
                  {
                   type: "text",
                   content: "The effects of cost tuning were also analyzed by adjusting the Q and R matrices to control the cartpole while also respecting the actuator limits of the system."
                  },
            ]
        },
        {
            title: "Trajectory Tracking with Time-Varying LQR",
             navName: "Trajectory Tracking",
            navRef: "trajectory-tracking",
            content: [
                  {
                    type: "text",
                    content:
                         "The project then explored Time-Varying LQR (TVLQR) for trajectory tracking. A swing-up trajectory for the cartpole system was used as a reference trajectory for the controller to follow. The controller was designed based on estimated system parameters, and was tested by simulating the system with a different set of true parameters to simulate real-world uncertainty. The simulations showed the system's ability to track the reference trajectory while meeting performance requirements."
                },
                {
                     type: "code",
                    codeLang: "julia",
                    content: `
#Time Varying Ricatti Recursion to Calculate Gains
for k = N-1:-1:1
        # Linearize dynamics about reference state
        A = FD.jacobian(_x -> rk4 (params_est, _x, Ubar [k], dt), Xbar[k])
        B = FD.jacobian (_u -> rk4 (params_est, Xbar[k], _u, dt), Ubar[k])
        # Compute the time varying gain matrix
        K[k] = (R + B'*P[k+1]*B) \\ B'*P[k+1]*A
        # Update the cost to go matrix
        P[k] = Q + A'*P[k+1]*(A - B*K[k])
    end
                        `,
                    subtitle: "Julia implementation of the Time-Varying LQR algorithm."
                },
                  {
                    type: "image",
                    content: "/media/images/tvlqr_cartpole.png",
                     altContent: "Trajectory Tracking with TVLQR",
                     subtitle: "The cartpole's trajectory while tracking a reference with Time-Varying LQR."
                    },
             ]
        },
        {
            title: "Optimal Rendezvous and Docking with LQR",
            navName: "Rendezvous with LQR",
            navRef: "rendezvous-lqr",
            content: [
               {
                    type: "text",
                    content:
                         "The project uses optimal control theory to solve a rendezvous and docking problem, specifically a SpaceX Dragon spacecraft docking with the International Space Station (ISS). The dynamics of the system are defined by the Clohessy-Wiltshire equations. A Finite Horizon LQR controller was then used to bring the system to the desired states while maintaining the controls within a specified limit. The results showed the systems ability to successfully meet the required performance metrics.",
                    },
                     {
                         type: "code",
                         codeLang: "julia",
                       content: `
"""
Creates a discrete time model of the Clohessy-Wiltshire equations
"""
    function create_dynamics(dt:: Real)::Tuple{Matrix,Matrix}
    mu = 3.986004418e14
        a = 6971100.0
    n = sqrt(mu/a^3)
    A = [0 0 0 1 0 0;
            0 0 0 0 1 0;
            0 0 0 0 0 1;
            3*n^2 0 0 0 2*n 0;
            0 0 0 -2*n 0 0;
            0 0 -n^2 0 0 0]
    B = Matrix([zeros(3,3);0.1*I(3)])
    nx, nu = size(B)
    AB_square = [A B; zeros(nu, nx + nu)]
    AB_discrete = exp(AB_square*dt)
    Ad = AB_discrete[1:nx, 1:nx]
    Bd = AB_discrete[1:nx, nx+1:end]
    return Ad, Bd
    end
                        `,
                     subtitle: "Julia code for the Clohessy-Wiltshire Equations."
                   },
                    {
                     type: "image",
                     content: "/media/images/lqr_rendezvous.png",
                      altContent: "LQR control for rendezvous",
                      subtitle: "Rendezvous and Docking using Finite Horizon LQR."
                    },
            ]
        },
        {
             title: "Rendezvous and Docking with Convex Trajectory Optimization",
             navName: "Convex Trajectory Optimization",
            navRef: "convex-trajectory-opt",
             content: [
                {
                    type: "text",
                    content:
                         "Convex trajectory optimization was used to solve the rendezvous and docking problem, using the same parameters as in the previous section. The trajectory optimization problem was formulated with an LQR cost function, an initial condition constraint, dynamics constraints, bound constraints on the control, and a collision constraint. The optimization produced a trajectory that brought the system to the desired state while avoiding a collision in the y-axis, and the control input remained within the specified bounds."
                    },
                    {
                        type: "code",
                        codeLang: "julia",
                        content: `
"""
Solves a trajectory optimization problem using convex optimization.
"""
function convex_trajopt(A:: Matrix, B::Matrix, X_ref:: Vector{Vector{Float64}}, x0:: Vector, xg:: Vector, u_min:: Vector, u_max:: Vector, N::Int64)::Tuple{Vector{Vector{Float64}}, Vector{Vector{Float64}}}
    nx, nu = size(B)
    X = cvx.Variable(nx, N)
    U = cvx.Variable(nu, N-1)
    obj = 0
    for i=1:N-1
        x_k = X[:,i]
        u_k = U[:,i]
        obj += 0.5*cvx.quadform(x_k - X_ref[i],Q)
        obj += 0.5*cvx.quadform(u_k, R)
        end
    obj += 0.5*cvx.quadform (X[:,N] - X_ref[N],Q)
    prob = cvx.minimize(obj)
    prob.constraints += X[:,1] == x0
    prob.constraints += X[:,end] == xg
    prob.constraints += X[:,end] [2] <= xg[2]
    for k = 1: (N-1)
            x_k = X[:,k]
        u_k = U[:,k]
            prob.constraints += (u_k <= u_max)
            prob.constraints += (u_min <= u_k)
            prob.constraints += A*x_k + B*u_k == X[:,k+1]
            prob.constraints += x_k[2] <= xg[2]
    end
    cvx.solve!(prob, ECOS.Optimizer; silent_solver = true)
    X = vec_from_mat(X.value)
    U = vec_from_mat(U.value)
    return X, U
end
                        `,
                        subtitle: "Julia implementation of convex trajectory optimization."
                      },
                     {
                     type: "image",
                    content: "/media/images/convex_rendezvous.png",
                    altContent: "Rendezvous using convex trajectory optimization",
                     subtitle: "A trajectory of a spacecraft undergoing rendezvous using convex trajectory optimization."
                   },
            ]
        },
        {
            title: "Trajectory Tracking with Convex MPC",
            navName: "Convex MPC",
            navRef: "convex-mpc",
            content: [
                 {
                    type: "text",
                    content:
                        "The final section of this project implements Convex Model Predictive Control (MPC). This technique provides a solution more robust to system changes than LQR alone. This was accomplished by combining feedback control with the convex trajectory optimization from the previous section. By using MPC, the controller could replan the trajectory in real time. The controller performed well by tracking a reference trajectory while maintaining all defined requirements and constraints.",
                    },
                   {
                       type: "code",
                        codeLang: "julia",
                       content: `
"""
Solves a trajectory optimization problem using convex optimization and model predictive control.
"""
function convex_mpc (A:: Matrix, B::Matrix, X_ref_window:: Vector{Vector{Float64}}, 
                        xic:: Vector, xg::Vector, u_min::Vector, u_max:: Vector, 
                        N_mpc::Int64)::Vector{Float64}
    nx, nu = size(B)
    X = cvx.Variable(nx, N_mpc)
    U = cvx.Variable(nu, N_mpc-1)
    obj = 0
    for k = 1: (N_mpc-1)
        x_k = X[:,k]
        u_k = U[:,k]
        obj += 0.5*cvx.quadform(x_k - X_ref_window[k],Q)
        obj += 0.5*cvx.quadform(u_k, R)
        end
        obj += 0.5*cvx.quadform (X[:, N_mpc] - X_ref_window [N_mpc],Q)
    prob = cvx.minimize(obj)
    prob.constraints += X[:,1] == xic
    prob.constraints += X[:,end] == xg
        prob.constraints += X[:,end] [2] <= xg[2]
    for k = 1: (N_mpc-1)
            x_k = X[:,k]
            u_k = U[:,k]
        prob.constraints += (u_k <= u_max)
            prob.constraints += (u_min <= u_k)
            prob.constraints += (A*x_k + B*u_k == X[:,k+1])
            prob.constraints += (x_k[2] <= xg[2])
    end
    cvx.solve!(prob, ECOS.Optimizer; silent_solver = true)
    return U[:,1]
end
                        `,
                      subtitle: "Julia implementation of convex MPC."
                     },
                     {
                         type: "image",
                       content: "/media/images/mpc_rendezvous.png",
                       altContent: "Rendezvous using Convex MPC",
                      subtitle: "Trajectory of a spacecraft using convex MPC for rendezvous and docking.",
                      },
             ]
         },
        {
            title: "Conclusion",
            navName: "Conclusion",
            navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content:
                        "This project demonstrated the implementation and analysis of multiple optimal control techniques. The project covers both linear and nonlinear systems, and both discrete and continuous time. Through the use of various methods like Convex Optimization, Ricatti recursion, and LQR, it provided a comprehensive understanding of how different control methodologies apply to distinct scenarios."
                }
            ]
        }
    ]
};