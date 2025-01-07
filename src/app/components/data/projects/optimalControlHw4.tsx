import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW4: Project = {
  title: "Trajectory Generation with Iterative Learning and Hybrid Control",
  subtitle: "Spring 2024",
  media: "/media/videos/biped_walking.mp4",
  tags: ["Iterative Learning Control", "Hybrid Dynamics", "Trajectory Optimization", "Bipedal Walking", "MATLAB"],
  section: [
    {
      title: "Project Introduction",
      navName: "Introduction",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content:
            "This project explores two advanced control techniques: Iterative Learning Control (ILC) and Hybrid Trajectory Optimization. The ILC algorithm is applied to generate a control trajectory for a car performing a moose test maneuver, while hybrid trajectory optimization is used to generate a walking gait for a simple bipedal model. Both algorithms leverage model-based control techniques to generate smooth and optimal behaviors. This project was implemented in Julia.",
        },
      ],
    },
    {
      title: "Iterative Learning Control for Car Maneuver",
      navName: "Iterative Learning Control (ILC) for a Car",
      navRef: "ilc-car",
      content: [
        {
          type: "text",
          content:
            "In this section, the Iterative Learning Control (ILC) algorithm is implemented to generate a control trajectory for a car performing a moose test maneuver. This involves iteratively refining the control inputs by learning from previous trajectories and a defined reference. The car's behavior is modeled using a nonlinear bicycle model, and the ILC aims to reconcile the differences between the simulated model and real-world dynamics.",
        },
         {
            type: "code",
            content: `
             /**
               * Calculates the continuous-time dynamics of a car using a nonlinear bicycle model.
               *
               * This function takes the system parameters, the current state (x)
               * , and control input (u) as arguments and returns the derivative of the state (xdot).
               *
               * @param model NamedTuple containing model parameters including length L and rear length lr.
               * @param x Current state vector [px, py, θ, δ, v].
               * @param u Control input vector [a, δdot].
               * @return The derivative of the state (xdot).
               */
            function estimated_car_dynamics(model::NamedTuple, x::Vector, u::Vector)::Vector
                # nonlinear bicycle model continuous time dynamics
                px, py, θ, δ, v = x
                a, δdot = u

                β = atan(model.lr * δ, model.L)
                s, c = sincos(θ + β)
                ω = v * cos(β) * tan(δ) / model.L
                vx = v * c
                vy = v * s

                xdot = [
                    vx,
                    vy,
                    ω,
                    δdot,
                    a
                ]
                return xdot
            end
            `,
            codeLang: "julia",
            subtitle: "Julia implementation of the Estimated Car Dynamics Function",
          },
          {
            type: "code",
            content: `
# feel free to use/not use any of these

/**
 * Calculates the cost for a given trajectory.
 *
 * This function computes the total cost of a trajectory based on
 * simulated states, simulated controls, reference states, and
 * reference controls, using LQR cost matrices.
 *
 * @param Xsim Vector of simulated states at each time step.
 * @param Ubar Vector of simulated controls at each time step.
 * @param Xref Vector of reference states at each time step.
 * @param Uref Vector of reference controls at each time step.
 * @param Q State cost matrix in LQR.
 * @param R Control cost matrix in LQR.
 * @param Qf Terminal cost matrix in LQR.
 * @return The total trajectory cost.
 */
function trajectory_cost(
    Xsim::Vector{Vector{Float64}}, # Simulated states
    Ubar::Vector{Vector{Float64}}, # Simulated controls
    Xref::Vector{Vector{Float64}}, # Reference states
    Uref::Vector{Vector{Float64}}, # Reference controls
    Q::Matrix,                     # State cost matrix
    R::Matrix,                     # Control cost matrix
    Qf::Matrix                     # Terminal state cost matrix
)::Float64                     # Return cost J
    J = 0 # Initialize the total cost
     N = length(Xsim) # Get the number of time steps
   # Loop through all time steps except the last one
    for k = 1:(N-1)
        # Calculate stage cost with LQR terms
        J += 0.5 * (Xsim[k] - Xref[k])' * Q * (Xsim[k] - Xref[k])
        J += 0.5 * (Ubar[k] - Uref[k])' * R * (Ubar[k] - Uref[k])
    end
     # Calculate the terminal cost
    J += 0.5 * (Xsim[N] - Xref[N])' * Qf * (Xsim[N] - Xref[N])
    return J # Return the total cost
end

/**
 * Converts a matrix to a vector of vectors.
 *
 * This utility function transforms a matrix into a vector where each
 * element is a vector representing a column of the input matrix.
 *
 * @param Xm The input matrix to be converted.
 * @return A vector of vectors representing the matrix columns.
 */
function vec_from_mat(Xm::Matrix)::Vector{Vector{Float64}}
    # Convert a matrix to a vector of vectors
    X = [Xm[:, i] for i = 1:size(Xm, 2)]
    return X # Return the converted vector
end

/**
 * Computes the control update for the ILC algorithm.
 *
 * This function solves an optimization problem to calculate the
 * control update for the ILC algorithm. It minimizes a tracking
 * cost subject to linearized dynamics constraints using cvx.
 *
 * @param Xsim Vector of simulated states.
 * @param Ubar Vector of simulated control inputs.
 * @param Xref Vector of reference states.
 * @param Uref Vector of reference control inputs.
 * @param As Vector of A Jacobian matrices at each time step.
 * @param Bs Vector of B Jacobian matrices at each time step.
 * @param Q State cost matrix in LQR.
 * @param R Control cost matrix in LQR.
 * @param Qf Terminal cost matrix in LQR.
 * @return Vector of control input updates for each step.
 */
function ilc_update(
    Xsim::Vector{Vector{Float64}}, # Simulated states
    Ubar::Vector{Vector{Float64}}, # Simulated controls
    Xref::Vector{Vector{Float64}}, # Reference states
    Uref::Vector{Vector{Float64}}, # Reference controls
    As::Vector{Matrix{Float64}},   # A Jacobians at each time step
    Bs::Vector{Matrix{Float64}},   # B Jacobians at each time step
    Q::Matrix,                     # State cost matrix
    R::Matrix,                     # Control cost matrix
    Qf::Matrix                     # Terminal state cost matrix
)::Vector{Vector{Float64}}        # Return vector of control updates

    N = length(Xsim) # Get number of time steps
    nx, nu = size(Bs[1]) # Get size of control/state space

    # Create optimization variables for the change in states and controls
    ΔX = cvx.Variable(nx, N)
    ΔU = cvx.Variable(nu, N - 1)

    # Setup the cost function, tracking cost on Xref and Uref
    cost = 0.0
    for k = 1:(N-1) # Loop through all timesteps but last
      cost += 0.5 * cvx.quadform(Xsim[k] + ΔX[:, k] - Xref[k], Q)
      cost += 0.5 * cvx.quadform(Ubar[k] + ΔU[:, k] - Uref[k], R)
    end
    cost += 0.5 * cvx.quadform(Xsim[N] + ΔX[:, N] - Xref[N], Qf) # Terminal cost

    # Create the optimization problem instance
    prob = cvx.minimize(cost)

    # Add constraints: initial condition and the dynamics contraints
    prob.constraints += (ΔX[:, 1] == zeros(nx)) # Initial state condition
    for k = 1:(N-1) # Loop through time steps
         # Add the linearized dynamics as constraints
        prob.constraints += (ΔX[:, k+1] == As[k] * ΔX[:, k] +
           Bs[k] * ΔU[:, k])
    end

    # Solve the optimization problem
    cvx.solve!(prob, ECOS.Optimizer; silent_solver=true)

    # Extract the optimized control input updates from the optimization variables
    ΔU = vec_from_mat(ΔU.value)
    return ΔU # return the vector of control input changes

end `,
    codeLang: "julia",
    subtitle: "Julia implementation for cost calculations and ILC updates.",
},
          
          {
            type: "image",
            content: "/media/images/ilc_trajectory.png",
            altContent: "Trajectory of the car with ILC",
            subtitle: "The car's trajectory generated by the ILC algorithm.",
          },
           {
             type: "video",
             content: "/media/videos/ilc_demo.mp4",
             altContent: "Control inputs of the car with ILC",
             subtitle: "The car's trajectory generated by the ILC algorithm.",
           },
      ],
    },
        {
        title: "Hybrid Trajectory Optimization for Bipedal Walking",
        navName: "Hybrid Trajectory Optimization for Bipedal Walking",
        navRef: "hybrid-trajectory-optimization",
        content: [
          {
            type: "text",
            content:
              "This section focuses on solving a trajectory optimization problem for a simple biped model using hybrid dynamics. We implemented a direct method to optimize a predefined gait sequence, using the IPOPT solver to manage the optimization problem. This model combines continuous-time dynamics with discrete event maps to capture the hybrid nature of bipedal walking. The system was modeled as three point masses: one for the body and one for each foot.",
          },
          {
             type: "code",
             content: `/**
 * Calculates the continuous-time dynamics when foot 1 is in contact with the ground.
 *
 * This function computes the continuous dynamics of the biped when foot 1 is on the ground.
 *
 * @param model NamedTuple containing model parameters including body mass (mb), 
 *        foot mass (mf), and gravity (g).
 * @param x Current state vector.
 * @param u Control input vector [F1, F2, τ].
 * @return The derivative of the state (xdot).
 */
function stance1_dynamics(model::NamedTuple, x::Vector, u::Vector)
    # dynamics when foot 1 is in contact with the ground
    mb, mf = model.mb, model.mf
    g = model.g

    M = Diagonal([mb mb mf mf mf mf])
    rb = x[1:2] # position of the body
    rf1 = x[3:4] # position of foot 1
    rf2 = x[5:6] # position of foot 2
    v = x[7:12] # velocities

    l1x = (rb[1] - rf1[1]) / norm(rb - rf1)
    l1y = (rb[2] - rf1[2]) / norm(rb - rf1)
    l2x = (rb[1] - rf2[1]) / norm(rb - rf2)
    l2y = (rb[2] - rf2[2]) / norm(rb - rf2)

    B = [l1x l2x l1y-l2y;
        l1y l2y l2x-l1x;
        0 0 0;
        0 0 0;
        0 -l2x l2y;
        0 -l2y -l2x]

    v = [0; -g; 0; 0; 0; -g] + M \\ (B * u)
    xdot = [v; v]
    return xdot
end
              `,
            codeLang: "julia",
            subtitle: "Julia implementation of the Stance 1 Dynamics Function.",
          },
            {
            type: "code",
            content: `/**
* Calculates the continuous-time dynamics when foot 2 is in contact with the ground.
*
* This function computes the continuous dynamics of the biped when foot 2 is on the ground.
*
* @param model NamedTuple containing model parameters including body mass (mb), foot mass (mf), and gravity (g).
* @param x Current state vector.
* @param u Control input vector [F1, F2, τ].
* @return The derivative of the state (xdot).
*/
function stance2_dynamics(model::NamedTuple, x::Vector, u::Vector)
    # dynamics when foot 2 is in contact with the ground
  mb, mf = model.mb, model.mf
  g = model.g
  M = Diagonal([mb mb mf mf mf mf])

    rb = x[1:2]
    # position of the body
  rf1 = x[3:4]
    # position of foot 1
  rf2 = x[5:6]
    # position of foot 2
  v = x[7:12]
    # velocities
  l1x = (rb[1] - rf1[1]) / norm(rb - rf1)
  l1y = (rb[2] - rf1[2]) / norm(rb - rf1)
  l2x = (rb[1] - rf2[1]) / norm(rb - rf2)
  l2y = (rb[2] - rf2[2]) / norm(rb - rf2)

  B = [l1x l2x l1y-l2y;
      l1y l2y l2x-l1x;
      -l1x 0 -l1y;
      -l1y 0 l1x;
      0 0 0;
      0 0 0]

  v = [0; -g; 0; -g; 0; 0] + M \\ (B * u)
  xdot = [v; v]
  return xdot
end
            `,
             codeLang: "julia",
             subtitle: "Julia implementation of the Stance 2 Dynamics Function.",
          },
            {
            type: "code",
             content: `/**
  * Simulates an inelastic collision for foot 1.
  *
  * This function simulates an inelastic collision at foot 1 by setting its vertical velocity to zero.
  *
  * @param x Current state of the biped.
  * @return Updated state after collision.
  */
  function jump1_map(x)
  # foot 1 experiences inelastic collision
      xn = [x[1:8]; 0.0; 0.0; x[11:12]]
      return xn
  end
             `,
             codeLang: "julia",
            subtitle: "Julia implementation of Jump 1 Map Function.",
          },
           {
            type: "code",
            content: `/**
  * Simulates an inelastic collision for foot 2.
  *
  * This function simulates an inelastic collision at foot 2 by setting its vertical velocity to zero.
  *
  * @param x Current state of the biped.
  * @return Updated state after collision.
  */
  function jump2_map(x)
      # foot 2 experiences inelastic collision
      xn = [x[1:10]; 0.0; 0.0]
      return xn
  end
            `,
             codeLang: "julia",
             subtitle: "Julia implementation of Jump 2 Map Function.",
          },
          {
            type: "code",
            content: `/**
 * Calculates the cost for the bipedal walker.
 *
 * This function calculates the total cost of a trajectory using
 * LQR cost terms.
 *
 * @param params System parameters including indices, sizes, cost matrices,
 *               reference state, and control vectors.
 * @param Z Combined state and control trajectory vector.
 * @return Total cost of the trajectory as a float.
 */
function walker_cost(params::NamedTuple, Z::Vector)::Real
    # Extract parameters for easier access
    idx, N, xg = params.idx, params.N, params.xg
    Q, R, Qf = params.Q, params.R, params.Qf
    Xref, Uref = params.Xref, params.Uref

    # Initialize total cost
    J = 0

    # Loop through each time step except for the last
    for k = 1:N-1
        # Get state and control for the current time step
        xi = Z[idx.x[k]]
        ui = Z[idx.u[k]]

        # Accumulate stage cost from tracking
        J += 0.5 * (xi - Xref[k])' * Q * (xi - Xref[k])
        J += 0.5 * (ui - Uref[k])' * R * (ui - Uref[k])
    end

    # Add the terminal cost
    J += 0.5 * (Z[idx.x[N]] - Xref[N])' * Qf * (Z[idx.x[N]] - Xref[N])

    return J # Return the total cost
end

/**
 * Computes the dynamic constraints for the bipedal walker.
 *
 * This function computes the violation of the system dynamics
 * constraints at each time step. Uses the hermite simpson rule
 * for numerical integration and applies the appropriate stance dynamics.
 *
 * @param params System parameters including time step and system indices.
 * @param Z Combined state and control trajectory vector.
 * @return A vector of constraint violations.
 */
function walker_dynamics_constraints(params::NamedTuple,
    Z::Vector)::Vector
    # Extract parameters
    idx, N, dt = params.idx, params.N, params.dt
    M1, M2 = params.M1, params.M2
    J1, J2 = params.J1, params.J2
    model = params.model

    # Initialize the constraints
    c = zeros(eltype(Z), idx.nc)

    # Loop through all time steps
    for k = 1:N-1
        # get current state, next state and control
        xi = Z[idx.x[k]]
        xip1 = Z[idx.x[k+1]]
        ui = Z[idx.u[k]]
      # Use Hermite-Simpson with appropriate dynamic function based on the phase
        if (k in M1) && !(k in J1) # Foot 1 in contact
            c[idx.c[k]] = xip1 - rk4(model, stance1_dynamics, xi, ui, dt)
        elseif (k in M2) && !(k in J2) # Foot 2 in contact
            c[idx.c[k]] = xip1 - rk4(model, stance2_dynamics, xi, ui, dt)
        elseif (k in J1) # foot 1 in transition
            c[idx.c[k]] = xip1 - jump2_map(rk4(model, stance1_dynamics, xi, ui, dt))
        elseif (k in J2)  # foot 2 in transition
            c[idx.c[k]] = xip1 - jump1_map(rk4(model, stance2_dynamics, xi, ui, dt))
        end
    end
    return c # return vector of constraint violations
end

/**
 * Defines the stance constraints for the bipedal walker.
 *
 * This function defines the equality constraints for vertical velocity
 * of the swing foot at each time step based on the stance phase.
 *
 * @param params System parameters including time step and system indices.
 * @param Z Combined state and control trajectory vector.
 * @return A vector of stance constraints
 */
function walker_stance_constraint(params::NamedTuple, Z::Vector)::Vector
    # Extract Parameters
    idx, N, dt = params.idx, params.N, params.dt
    M1, M2 = params.M1, params.M2
    J1, J2 = params.J1, params.J2
    model = params.model

    # Initialize the constraints
    c = zeros(eltype(Z), N)

    # Loop through all the time steps
    for k = 1:N
        xi = Z[idx.x[k]]# get current state
          # Add constraint based on the current stance phase
        if (k in M1) # foot 1 is the stance foot
            c[k] = xi[4] # The y velocity of foot 1 must be zero
          elseif (k in M2) # foot 2 is the stance foot
            c[k] = xi[6] # The y velocity of foot 2 must be zero
        end
    end

    return c # return all of the constraints
end

/**
 * Defines the equality constraints for the bipedal walker problem.
 *
 * This function combines the initial, terminal, dynamics,
 * and stance constraints into a single vector of equality constraints.
 *
 * @param params System parameters including initial and final state.
 * @param Z Combined state and control trajectory vector.
 * @return Vector of all equality constraints
 */
function walker_equality_constraint(params::NamedTuple, Z::Vector)::Vector
    # Get all parameters
    N, idx, xic, xg = params.N, params.idx, params.xic, params.xg

      # Create the various constraints
    initial_constaint = Z[idx.x[1]] - xic # Initial condition constraint
    terminal_constraint = Z[idx.x[N]] - xg # Terminal condition constraint
    dynamics_constraint = walker_dynamics_constraints(params, Z) # Dynamics
    stance_constraint = walker_stance_constraint(params, Z) # Stance phase constraints

    # Stack the equality constraints
    return [initial_constaint; terminal_constraint; dynamics_constraint;
        stance_constraint] # return all constraints

end

/**
 * Defines the inequality constraints for the bipedal walker problem.
 *
 * This function defines the constraints for the link lengths of the bipedal walker.
 *
 * @param params System parameters, including indexes and timestep.
 * @param Z Combined state and control trajectory vector.
 * @return A vector of inequality constraint violations
 */
function walker_inequality_constraint(params::NamedTuple, Z::Vector)::Vector
    # Unpack parameters
    idx, N, dt = params.idx, params.N, params.dt
    M1, M2 = params.M1, params.M2

    # Initialize the inequality constraints
    c = zeros(eltype(Z), 2 * N)

    # Loop through all time steps
    for k = 1:N
        # Get state variables
        xi = Z[idx.x[k]]
        rb = xi[1:2]
        r1 = xi[3:4]
        r2 = xi[5:6]

        # Compute and Add the length constraints
        c[(k-1)*2+1] = norm(rb - r1)^2  # Constraint between body and foot 1
        c[(k-1)*2+2] = norm(rb - r2)^2  # Constraint between body and foot 2
    end

    return c # Return the constraints
end
                  `,
          codeLang: "julia",
            subtitle: "Julia implementation of cost, dynamics, and constraint functions for bipedal walker."
        },
          {
            type: "video",
            content: "/media/videos/biped_walking.mp4",
             altContent: "Video of the bipedal walker",
            subtitle: "Resulting bipedal walking.",
          },
           {
            type: "image",
             content: "/media/images/biped_walking_positions.png",
             altContent: "Body Positions of the bipedal walker",
             subtitle: "Resulting Body Positions of the bipedal walking system.",
           },
        ],
      },
      {
         title: "Conclusion and Insights",
         navName: "Conclusion",
        navRef: "conclusion",
        content: [
          {
            type: "text",
            content:
              "This project provided valuable experience with advanced motion planning and control algorithms. The ILC algorithm demonstrated an effective method for iterative control, while the hybrid trajectory optimization allowed the system to develop walking gaits for a bipedal system. These methods are useful in creating robust controllers for a variety of complex systems that are used in the real world.",
          },
        ],
      },
  ],
};