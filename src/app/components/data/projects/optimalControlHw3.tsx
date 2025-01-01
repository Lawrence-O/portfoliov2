import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW3: Project = {
  title: "Trajectory Optimization with DIRCOL, iLQR, and TVLQR",
  subtitle: "Spring 2024",
  media: "/media/images/optimalControlHW.png",
  tags: ["Optimization", "Control Theory", "iLQR", "DIRCOL", "TVLQR", "Julia"],
  section: [
    {
      title: "Project Introduction",
      navName: "Introduction",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content:
            "This project explores various optimal motion planning and control algorithms, focusing on Direct Collocation (DIRCOL) and iterative Linear Quadratic Regulator (iLQR) techniques. These algorithms are applied to solve trajectory optimization problems for systems such as a cart-pole and a quadrotor. The project also includes implementations of Time-Varying LQR (TVLQR) for trajectory tracking in the presence of model mismatch. All implementations were done using the Julia programming language.",
        },
        {
           type: "image",
           content: "/media/images/optimalControlIntro.png",
           altContent: "Diagram illustrating the optimal control concept",
           subtitle: "General Optimal Control Scheme",
        }
      ],
    },
    {
      title: "Direct Collocation for Cart-Pole Trajectory Optimization",
      navName: "Direct Collocation (DIRCOL) for Cart-Pole",
      navRef: "dircol-cartpole",
      content: [
        {
          type: "text",
          content:
            "This section utilizes the Direct Collocation (DIRCOL) method to solve trajectory optimization problems for a cart-pole system. We started by solving a simple Linear Program (LP) using the IPOPT solver to showcase its effectiveness for general optimization problems. Then, DIRCOL was used to compute an open-loop trajectory, which is subsequently tracked using TVLQR.",
        },
        {
          type: "code",
          content: `
          /**
           * Solves a linear program using IPOPT.
           *
           * This function takes a cost function, equality and inequality constraint functions,
           * lower and upper bounds for the variable and the constraints, an initial guess,
           * problem-specific parameters, the differentiation type, and verbosity as inputs.
           * It minimizes the cost function subject to the given constraints, using the IPOPT solver.
           *
           * @param cost Function that calculates the cost. It takes params and x as arguments and returns a real value.
           * @param equality_constraint Function that calculates the equality constraints. Takes params and x and returns a vector.
           * @param inequality_constraint Function that calculates the inequality constraints. Takes params and x and returns a vector.
           * @param x_1 Lower bound of x. It is a vector.
           * @param x_u Upper bound of x. It is a vector.
           * @param c_l Lower bound for inequality constraint. It is a vector.
           * @param c_u Upper bound for inequality constraint. It is a vector.
           * @param x0 Initial guess for x. It is a vector.
           * @param params Problem parameters. This is a NamedTuple.
           * @param diff_type Symbol that specifies differentiation type, can be :auto or :finite.
           * @param verbose Boolean for verbosity of IPOPT. Default is true.
           * @param tol Optional argument for optimality tolerance. Default is 1e-6.
           * @param c_tol Optional argument for constraint violation tolerance. Default is 1e-6.
           * @param max_iters Optional argument for maximum number of iterations. Default is 10_000.
           *
           * @return solution x as a Vector
          */
          function fmincon(
              cost,
              equality_constraint,
              inequality_constraint,
              x_1,
              x_u,
              c_l,
              c_u,
              x0,
              params,
              diff_type;
              tol = 1e-6,
              c_tol = 1e-6,
              max_iters = 10_000,
              verbose = true
          )
              # checking dimensions of everything
              @assert length(x_1) == length(x_u)
              @assert length(x0) == length(x_1)
              @assert length(c_l) == length(c_u)

              # setting up autodiff
              if diff_type == :auto
                  diff_backend = ForwardDiff
              elseif diff_type == :finite
                  diff_backend = FiniteDiff
              else
                  error("must be either :auto or :finite")
              end

              # define functions that IPOPT needs, using autodiff
              function obj(x)
                  return cost(params, x)
              end

              function obj_grad(x, grad)
                  grad[:] = diff_backend.gradient(obj, x)
              end

              function constraints(x, con)
                  con[1:length(equality_constraint(params, x))] = equality_constraint(params, x)
                  con[(length(equality_constraint(params, x))+1):end] = inequality_constraint(params, x)
              end

              function constraints_jac(x, jac)
                  jac[:, 1:length(equality_constraint(params, x))] = diff_backend.jacobian(x -> equality_constraint(params,x), x)
                  jac[:, (length(equality_constraint(params, x))+1):end] = diff_backend.jacobian(x -> inequality_constraint(params,x), x)
              end

              # get dimension of things
              num_eq_con = length(equality_constraint(params, x0))
              num_ineq_con = length(inequality_constraint(params, x0))
              num_con = num_eq_con + num_ineq_con
              num_vars = length(x0)

              # creating the ipopt problem
              prob = MOI.Problem(;
                  sense = MOI.MIN_SENSE,
                  objective = MOI.ObjectiveFunction(obj),
                  gradient = MOI.ObjectiveGradient(obj_grad),
                  constraints = MOI.Constraints(constraints, num_con),
                  jacobian = MOI.ConstraintsJacobian(constraints_jac),
                  variable_bounds = MOI.VariableBounds(x_1, x_u),
                  constraint_bounds = MOI.ConstraintBounds([zeros(num_eq_con); c_l], [zeros(num_eq_con); c_u]),
                  initial_guess = MOI.InitialGuess(x0),
              )

              # solving
              @info "---all dimensions good---"
              @info "--diff type set to :$(diff_type) ($(typeof(diff_backend).name))----"
               @info "-testing objective gradient--"
              if diff_type == :auto
                  @test_nowarn diff_backend.gradient(obj, x0)
              end
              @info "--testing constraint Jacobian---"
               if diff_type == :auto
                  @test_nowarn  diff_backend.jacobian(x->constraints(x,zeros(num_con)), x0)
               end
              @info "-successfully compiled both derivatives-----"
              optimizer = Ipopt.Optimizer(
                  ;
                  print_level = verbose ? 5 : 0,
                  tol = tol,
                  acceptable_tol = tol,
                  max_iter = max_iters,
                  acceptable_iter = max_iters,
                  constr_viol_tol = c_tol,
                  acceptable_constr_viol_tol = c_tol
              )

              MOI.optimize!(optimizer, prob)

              @info "-IPOPT beginning solve----"
              return MOI.get(optimizer, MOI.VariablePrimal(), prob.variables[1:num_vars])
          end
          `,
          codeLang: "julia",
          subtitle: "Julia implementation of the fmincon Function for IPOPT",
        },
         {
           type: "image",
           content: "/media/images/cartpole.png",
           altContent: "Diagram of a cart-pole system",
           subtitle: "Visual representation of the cart-pole system.",
        },
      ],
    },
    {
      title: "Cart-Pole Swing-Up with Direct Collocation",
      navName: "Cart-Pole Swingup with DIRCOL",
      navRef: "cartpole-swingup",
      content: [
        {
          type: "text",
          content:
            "This section focuses on applying DIRCOL to solve the challenging cart-pole swing-up problem. The aim is to move the pole from a downward position to an upright position by applying a horizontal force on the cart. This implementation used Hermite-Simpson integration to handle the dynamics constraints, and the control inputs were held constant for each time step. Subsequently, the generated open-loop trajectory was tracked using TVLQR to provide feedforward and feedback control.",
        },
         {
          type: "code",
          content: `
          /**
            * Calculates the dynamics of a cart-pole system.
            *
            * This function takes the system parameters, the current state (x),
            * and the control input (u) as arguments and returns the derivative of the state.
            *
            * @param params Parameters for the cart-pole model. It is a NamedTuple containing cart mass (mc),
            *              pole mass (mp), and pole length (l).
            * @param x Current state of the cart-pole, it is a vector of [position, angle, position derivative, angle derivative].
            * @param u Control input, it is a vector of [horizontal force].
            * @return The derivative of the state (xdot) as a Vector.
          */
          function dynamics(params::NamedTuple, x::Vector, u::Vector)
            # unpack parameters
            mc, mp, l = params.mc, params.mp, params.l
            g = 9.81

            # unpack state
            q = x[1:2]
            qd = x[3:4]

            s = sin(q[2])
            c = cos(q[2])

            # Calculate the matrices
            H = [mc+mp mp*l*c; mp*l*c mp*l^2]
            C = [0 -mp*qd[2]*l*s; 0 0]
            G = [0, mp * g * l * s]
            B = [1, 0]
            # solve for qdd
            qdd = -H \\ (C * qd + G - B * u[1])
            # return xdot
            xdot = [qd; qdd]
            return xdot
          end
          `,
           codeLang: "julia",
           subtitle: "Julia implementation of the cart-pole dynamics function.",
        },
        {
          type: "code",
          content: `
           /**
             * Calculates the implicit integrator residual for the Hermite-Simpson method.
             *
             * This function takes the system parameters, the state at the beginning of the time step (x1),
             * the state at the end of the time step (x2), the control input (u),
             * and the time step (dt) as arguments. It returns the implicit integrator residual,
             * which is a vector.
             *
             * @param params Parameters for the system. It is a NamedTuple containing model parameters.
             * @param x1 State at the beginning of the time step. It is a Vector.
             * @param x2 State at the end of the time step. It is a Vector.
             * @param u Control input, It is a vector.
             * @param dt Time step. It is a real number.
             * @return The implicit integrator residual.
            */
            function hermite_simpson(
                params::NamedTuple,
                x1::Vector,
                x2::Vector,
                u,
                dt::Real
            )::Vector
                # calculate mid point
                Xk+1_mid = 0.5*(x1 + x2) + (dt / 8) * (dynamics (params, x1, u) - dynamics (params, x2, u))
                # calculate residual
                return x1 + (dt / 6) * (dynamics (params, x1, u) + 4 * dynamics (params, Xk+1_mid, u) + dynamics (params, x2, u)) - x2
            end
           `,
          codeLang: "julia",
          subtitle: "Julia implementation of Hermite Simpson Integration",
        },
         {
           type: "image",
           content: "/media/images/cartpole_swingup_traj.png",
           altContent: "Trajectory of the cartpole swingup",
           subtitle: "Resulting cartpole trajectory.",
        },
        {
         type: "image",
         content: "/media/images/cartpole_swingup_controls.png",
         altContent: "Control input of the cartpole swingup",
         subtitle: "Control input required to swing the cartpole.",
         },
      ],
    },
    {
      title: "Tracking DIRCOL Solution with TVLQR",
      navName: "Tracking DIRCOL Solution with TVLQR",
      navRef: "tracking-dircol-tvlqr",
      content: [
        {
          type: "text",
          content:
            "Building on the previous section, this part focuses on tracking the DIRCOL solution using TVLQR. The process involves generating K gains using the TVLQR algorithm and then simulating the closed-loop system with both feedforward and feedback control. This demonstrates how to correct a system when there is model mismatch. RK4 integration is used for the system dynamics here.",
        },
        {
          type: "code",
           content: `
            /**
              * Performs the Runge-Kutta 4th order integration.
              *
              * This function takes the system parameters, the current state (x),
              * the control input (u), and the time step (dt) as arguments.
              * It returns the next state of the system.
              *
              * @param params Parameters for the system.  It is a NamedTuple containing all model parameters.
              * @param x Current state. It is a Vector.
              * @param u Control input. It is a Vector.
              * @param dt Time step. It is a Float64.
              * @return Next state.
             */
            function rk4(params::NamedTuple, x::Vector, u, dt::Float64)
              # vanilla RK4 implementation
              k1 = dt * dynamics (params, x, u)
              k2 = dt * dynamics (params, x + k1 / 2, u)
              k3 = dt * dynamics (params, x + k2 / 2, u)
              k4 = dt * dynamics (params, x + k3, u)
              # return next state
              return x + (1/6) * (k1 + 2 * k2 + 2 * k3 + k4)
            end
           `,
           codeLang: "julia",
           subtitle: "Julia implementation of Runge-Kutta 4th order Integration.",
        },
        {
          type: "image",
           content: "/media/images/cartpole_tvlqr_traj.png",
           altContent: "Trajectory of the cartpole tracking controller",
           subtitle: "Cartpole trajectory while using a TVLQR controller.",
        },
        {
           type: "image",
           content: "/media/images/cartpole_tvlqr_controls.png",
           altContent: "Control inputs of cartpole tracking controller",
           subtitle: "Control inputs for the cartpole while tracking with TVLQR",
        },
      ],
    },
    {
      title: "iLQR for Quadrotor Trajectory Optimization",
      navName: "iLQR for Quadrotor Trajectory Optimization",
      navRef: "ilqr-quadrotor",
       content: [
        {
          type: "text",
          content:
            "This section details the implementation of the iLQR algorithm for trajectory optimization of a 6 degree of freedom (DOF) quadrotor. The algorithm was designed to make the quadrotor follow a predefined aerobatic maneuver by minimizing a defined cost function. The continuous-time dynamics of the quadrotor were discretized using RK4 integration.",
        },
         {
          type: "code",
          content: `
           /**
             * Calculates the discrete dynamics of the quadrotor using RK4.
             *
             * This function takes the system parameters, the current state (x), the control input (u),
             * and the index of the trajectory (k) as arguments. It returns the next state of the quadrotor
             * by using the Runge-Kutta 4th order integration method.
             *
             * @param params Parameters for the quadrotor model. It is a NamedTuple containing all parameters.
             * @param x Current state. It is a Vector.
             * @param u Control input. It is a Vector.
             * @param k Index of the trajectory. It is an integer.
             * @return The next state of the quadrotor.
             */
            function discrete_dynamics(
                params::NamedTuple,
                x::Vector,
                u,
                k
            )
                # get next state using RK4
                return rk4 (params.model, quadrotor_dynamics, x, u, params.model.dt)
            end
           `,
          codeLang: "julia",
          subtitle: "Julia implementation of the Quadrotor Discrete Dynamics",
        },
         {
          type: "code",
          content: `
            /**
              * Calculates the stage cost expansion for the iLQR algorithm.
              *
              * This function takes the system parameters, the current state (x),
              * the control input (u), and the time step (k) as arguments. It returns
              * the quadratic and linear approximations to the stage cost
              *
              * @param p Parameters for the system. It is a NamedTuple containing all system parameters.
              * @param x Current state. It is a Vector.
              * @param u Control input. It is a Vector.
              * @param k Time step. It is an integer.
              * @return Quadratic and linear cost terms.
              */
            function stage_cost_expansion(
                p::NamedTuple,
                x::Vector,
                u::Vector,
                k::Int
            )
                # if the stage cost is J(x,u), this function returns
                # Vx2J, VJ, Vu2J, VuJ
                Vx2J = FD.hessian(_dx -> stage_cost(p, _dx, u, k), x)
                VxJ = FD.gradient(_dx -> stage_cost(p, _dx, u, k), x)
                Vu2J = FD.hessian(_du -> stage_cost(p, x, _du, k), u)
                VuJ = FD.gradient(_du -> stage_cost(p, x, _du, k), u)
                return Vx2J, VxJ, Vu2J, VuJ
            end
          `,
          codeLang: "julia",
          subtitle: "Julia implementation of the Stage Cost Expansion for iLQR",
        },
         {
           type: "image",
           content: "/media/images/quadrotor_traj.png",
           altContent: "Trajectory of quadrotor from ilqr",
           subtitle: "Resulting Quadrotor Trajectory using iLQR.",
         },
        {
           type: "image",
           content: "/media/images/quadrotor_controls.png",
           altContent: "Control input of quadrotor from iLQR",
           subtitle: "Control Inputs for the quadrotor using iLQR.",
         },
       ],
    },
    {
      title: "Tracking iLQR Solution with Time-Varying LQR",
      navName: "Tracking iLQR Solution with TVLQR",
      navRef: "tracking-ilqr-tvlqr",
      content: [
        {
          type: "text",
          content:
            "This section demonstrates the use of TVLQR to track the iLQR trajectory from the previous section.  The closed-loop system accounts for model mismatch by using both feedforward and feedback control. The controllers were able to track the generated trajectory despite small model mismatches.",
        },
        {
           type: "image",
           content: "/media/images/quadrotor_tvlqr_traj.png",
           altContent: "Trajectory of the quadrotor with TVLQR",
           subtitle: "The quadrotor trajectory while tracking with TVLQR.",
         },
         {
            type: "image",
           content: "/media/images/quadrotor_tvlqr_attitude.png",
           altContent: "Attitude of the quadrotor with TVLQR",
           subtitle: "The attitude of the quadrotor while tracking with TVLQR.",
         },
      ],
    },
     {
      title: "Quadrotor Reorientation with Collision Avoidance",
      navName: "Quadrotor Reorientation with Collision Avoidance",
      navRef: "quadrotor-reorientation",
       content: [
        {
          type: "text",
          content:
            "This final section implements trajectory optimization for three planar quadrotors, with an emphasis on collision avoidance. The objective is to reorient the quadrotors while maintaining a specified distance away from each other. This complex problem incorporates a collision constraint along with the dynamic constraints, which was solved using DIRCOL.",
        },
         {
          type: "code",
          content: `
            /**
              * Calculates the dynamics for three planar quadrotors.
              *
              * This function takes system parameters, the current state (x),
              * and control input (u) as arguments. It returns the combined
              * dynamics of all three quadrotors.
              *
              * @param params Parameters for the system. It is a NamedTuple containing all model parameters.
              * @param x Current state of the system. It is a Vector.
              * @param u Control input for the system. It is a Vector.
              * @return Combined dynamics for all 3 quadrotors.
              */
            function combined_dynamics(params, x, u)
              # unpack state into their respective quadrotors
              x1 = x[1:6]
              u1 = u[1:2]
              x2 = x[(1:6).+6]
              u2 = u[(1:2).+2]
              x3 = x[(1:6).+12]
              u3 = u[(1:2).+4]

              # calculate the dynamics for each quadrotor
              xdot1 = single_quad_dynamics(params, x1, u1)
              xdot2 = single_quad_dynamics(params, x2, u2)
              xdot3 = single_quad_dynamics(params, x3, u3)

              # return stacked dynamics
              return [xdot1; xdot2; xdot3]
            end
           `,
           codeLang: "julia",
           subtitle: "Julia implementation of combined quadrotor dynamics.",
        },
         {
          type: "code",
          content: `
          /**
            * Calculates the implicit integrator residual for Hermite-Simpson method
            * for the 3 quadrotor system.
            *
            * This function takes system parameters, the state at the beginning of the time step (x1),
            * the state at the end of the time step (x2), the control input (u),
            * and the time step (dt) as arguments. It returns the implicit integrator residual,
            * which is a vector.
            *
            * @param params Parameters for the system. It is a NamedTuple containing model parameters.
            * @param x1 State at the beginning of the time step. It is a Vector.
            * @param x2 State at the end of the time step. It is a Vector.
            * @param u Control input. It is a vector.
            * @param dt Time step. It is a real number.
            * @return The implicit integrator residual
           */
           function hermite_simpson(
              params::NamedTuple,
              x1::Vector,
              x2::Vector,
              u,
              dt::Real
           )::Vector
              # calculate the mid point
               Xk+1_mid = 0.5*(x1 + x2) + (dt / 8) * (combined_dynamics(params, x1, u) - combined_dynamics(params, x2, u))
               # calculate the residual
               return x1 + (dt / 6) * (combined_dynamics(params, x1, u) + 4 * combined_dynamics(params, Xk+1_mid, u) + combined_dynamics(params, x2, u)) - x2
           end
          `,
           codeLang: "julia",
          subtitle: "Julia implementation of Hermite Simpson Integration for 3 quadrotors.",
         },
          {
          type: "code",
            content: `
           /**
             * Calculates a constraint based on distance between quadrotors.
             *
             * This function calculates the distance between each pair of quadrotors at each
             * time step and outputs a constraint on that distance.
             *
             * @param params Parameters for the system. It is a NamedTuple containing model parameters.
             * @param Z Optimized variable vector which contains states and controls.
             * @return Proximity constraints for the 3 quadrotors.
             */
            function quad_proximity_constraint(
                params::NamedTuple,
                Z::Vector
            )::Vector
              # unpack useful parameters
              idx, N, dt = params.idx, params.N, params.dt
              distance_threshold = params.d_threshold
              # initialize vector
              proximity = zeros(eltype(Z), 3 * (N - 1))
              # iterate though timesteps
              for i = 1:N-1
                 # get the state at this time step
                  xi = Z[idx.x[i]]
                  # unpack the positions
                  px1, px2, px3 = xi[1:2], xi[7:8], xi[13:14]
                  # set the constraints as squared norm to avoid sqrt
                  proximity[(i-1)*3+1] = sum((px1 - px2).^2) - distance_threshold^2
                  proximity[(i-1)*3+2] = sum((px2 - px3).^2) - distance_threshold^2
                  proximity[(i-1)*3+3] = sum((px1 - px3).^2) - distance_threshold^2
              end
                return proximity
            end
           `,
           codeLang: "julia",
           subtitle: "Julia implementation of the quadrotor proximity constraint.",
          },
         {
           type: "image",
           content: "/media/images/quadrotor_reorient_distance.png",
           altContent: "Distance between the 3 quadrotors",
           subtitle: "The distances between the quadrotors are shown.",
         },
          {
          type: "image",
          content: "/media/images/quadrotor_reorient_traj.png",
          altContent: "Trajectories of the three quadrotors",
           subtitle: "Trajectories of the three quadrotors.",
         },
        {
           type: "image",
           content: "/media/images/quadrotor_reorient_orientations.png",
           altContent: "Orientations of the three quadrotors",
           subtitle: "The quadrotor's orientations during reorientation.",
         },
       ],
    },
       {
      title: "Project Conclusion",
       navName: "Conclusion",
      navRef: "conclusion",
      content: [
        {
          type: "text",
          content:
            "This project provided hands-on experience with several important optimal control algorithms, starting with simple optimization using IPOPT, and then solving trajectory optimization problems for complex systems such as the cart-pole and the quadrotor. This project successfully demonstrated how to leverage these algorithms with constraint programming and model predictive control, providing a good basis for understanding how to solve more complex real-world problems in the future.",
        },
      ],
    },
  ],
};