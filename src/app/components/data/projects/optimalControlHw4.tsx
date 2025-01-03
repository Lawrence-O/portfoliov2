import { Project } from "@/app/components/project/interfaces";

export const optimalControlHW4: Project = {
  title: "Trajectory Generation with Iterative Learning and Hybrid Control",
  subtitle: "Spring 2024",
  media: "/media/images/iterativeControl.png",
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
         {
            type: "image",
            content: "/media/images/iterativeControlIntro.png",
            altContent: "Diagram illustrating the ILC and hybrid trajectory optimization concepts",
            subtitle: "General Overview of Project Concepts",
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
             /**
              * Performs the Runge-Kutta 4th order integration.
              *
              * This function takes the system model, the ODE function, the current state (x),
              * the control input (u), and the time step (dt) as arguments.
              * It returns the next state of the system.
              *
              * @param model Parameters for the system, it is a NamedTuple.
              * @param ode The ode function to perform integration on.
              * @param x Current state vector.
              * @param u Control input vector.
              * @param dt Time step.
              * @return Next state after RK4.
              */
            function rk4(model::NamedTuple, ode::Function, x::Vector, u::Vector, dt::Real)::Vector
                k1 = dt * ode(model, x, u)
                k2 = dt * ode(model, x + k1 / 2, u)
                k3 = dt * ode(model, x + k2 / 2, u)
                k4 = dt * ode(model, x + k3, u)
                return x + (1 / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            end
            `,
            codeLang: "julia",
            subtitle: "Julia implementation of the Runge-Kutta 4th Order Integration Function",
          },
          {
            type: "image",
            content: "/media/images/ilc_trajectory.png",
            altContent: "Trajectory of the car with ILC",
            subtitle: "The car's trajectory generated by the ILC algorithm.",
          },
           {
             type: "image",
             content: "/media/images/ilc_controls.png",
             altContent: "Control inputs of the car with ILC",
             subtitle: "The control inputs of the ILC algorithm.",
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
             content: `
            /**
             * Calculates the continuous-time dynamics when foot 1 is in contact with the ground.
             *
             * This function computes the continuous dynamics of the biped when foot 1 is on the ground.
             *
             * @param model NamedTuple containing model parameters including body mass (mb), foot mass (mf), and gravity (g).
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
            content: `
             /**
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
             content: `
              /**
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
            content: `
             /**
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
            type: "image",
            content: "/media/images/biped_walking_traj.png",
             altContent: "Trajectory of the bipedal walker",
            subtitle: "Resulting bipedal walking trajectory.",
          },
           {
            type: "image",
             content: "/media/images/biped_walking_controls.png",
             altContent: "Control inputs of the bipedal walker",
             subtitle: "Resulting Control inputs of the bipedal walking system.",
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