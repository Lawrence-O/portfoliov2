import { Project } from "@/app/components/project/interfaces";

export const roboticArmControl: Project = {
    title: "Robotic Arm Control and Trajectory Generation for Jenga", // Clarified title
    date: "Fall 2023",
    media: "/media/videos/rkd_jenga.mp4",
    githubLink: "https://github.com/your-username/jenga-robot",
    tags: ["Robotics", "Forward Kinematics", "Inverse Kinematics", "Trajectory Planning", "MATLAB", "5-DOF Arm", "Jenga"], // Added more specific tags
    section: [
        {
            title: "Project Introduction: Robotic Jenga Player", // Enhanced title
            navName: "Introduction",
            navRef: "introduction",
            content: [
              {
                type: "text",
                content: [
                  "This capstone project focused on the development of a control system for a 5-DOF articulated robotic arm, specifically applied to the task of playing Jenga. The project involved a comprehensive exploration of key robotics concepts including Denavit-Hartenberg (DH) parameter extraction, forward kinematics, inverse kinematics (analytical derivation), and trajectory generation using splines. The primary objective was to achieve a deep understanding of the complexities of robot arm control and motion planning, with all implementations performed in MATLAB."
                ]
              },
              {
                type: "video",
                content: "/media/videos/rkd_jenga.mp4",
                altContent: "Robotic arm playing Jenga",
                subtitle: "Demonstration of the 5-DOF robotic arm performing Jenga tasks."
              },
              {
                type: "text",
                displayAs: "subtitle",
                content: "Conceptual DH Frame Assignment"
              },
              {
                type: "text",
                content: "A sketch of the robotic arm showing coordinate frames (z-axis along joint axis, x-axis normal to current and previous z-axes) assigned to each joint according to the Denavit-Hartenberg convention. This visual representation complements the DH parameter table presented below."
              },
              {
                 type: "image",
                 content: "/media/images/roboticArmIntro.png", // This seems like a general arm, could be the DH frame assignment image
                 altContent: "Illustration of a robotic arm with DH frames",
                 subtitle: "Robotic arm with Denavit-Hartenberg frame assignments."
              }
            ],
          },
        {
            title: "Denavit-Hartenberg (DH) Parameter Extraction",
            navName: "DH Parameters", // Shortened navName
            navRef: "dh-parameter-extraction",
            content: [
                {
                    type: "text",
                    content: [
                         "The process of extracting Denavit-Hartenberg (DH) parameters began with precise measurements of the link lengths between the robotic arm's joints and the offsets between each assigned coordinate frame. DH parameters provide a standardized and systematic method to represent the geometry of robotic links and joints, forming the essential mathematical foundation for calculating the robot's configuration and end-effector pose.",
                         "These measurements were then systematically compared with established conventions to derive the corresponding DH parameters for our specific robotic system. To ensure consistency in our model, we assumed that all joint axes were oriented out of the page (along the Z-axis) in their reference configuration, adhering to standard conventions used in DH parameterization."
                    ]
                },
                {
                    type: "text",
                    content: "The following table shows the extracted DH parameters ('a_i' for link length, '\\alpha_i' for link twist, 'd_i' for link offset, and '\\theta_i' for joint angle) for the reference configuration of the arm:"
                },
                {
                    type: "code", // Using code block to preserve table formatting
                    codeLang: "plaintext", // Using plaintext as it's a table, not code
                    content: `
Link (i) | Link Length (a_i) | Link Twist (α_i) | Link Offset (d_i) | Joint Angle (θ_i)
---------|-------------------|------------------|-------------------|-------------------
1        | 0                 | π/2              | 56.05             | 0 (variable)
2        | 400               | 0                | 94                | 0 (variable)
3        | 334               | -π/2             | 3                 | 0 (variable)
4        | 0                 | π/2              | 39                | 0 (variable)
5        | 0                 | 0                | 108.05            | 0 (variable)
                  `,
                    subtitle: "Extracted DH Parameters for the reference configuration.",
                },

            ],
        },
        {
          title: "Forward Kinematics Implementation",
          navName: "Forward Kinematics",
          navRef: "forward-kinematics",
          content: [
            {
              type: "text",
              content: [
                "To implement the forward kinematics of our robotic arm, we constructed homogeneous transformation matrices for each link, representing the spatial relationship between consecutive joint frames. Homogeneous transformation matrices are a cornerstone of 3D spatial kinematics, as they allow us to represent both the rotation and translation of a coordinate frame relative to another in a single 4x4 matrix.",
                "By multiplying these individual transformation matrices sequentially from the base frame to the end-effector frame, we can compute the robot's overall pose (position and orientation of the end-effector) for any given set of joint angles. This process involved applying the extracted DH parameters using the standard transformation convention."
              ]
            },
            {
              type: "text",
                content: "The transformation matrix `H_i^{i-1}` from frame `i` to frame `i-1` was derived using the convention: `H_i^{i-1} = Rot_z(\\theta_i) \\cdot Trans_z(d_i) \\cdot Trans_x(a_i) \\cdot Rot_x(\\alpha_i)`. The general form of this matrix is:",
            },
            {
                type: "text", // This will be rendered by MathJax/KaTeX
                content: "`H_i^{i-1} = \\begin{bmatrix} \\cos\\theta_i & -\\sin\\theta_i\\cos\\alpha_i & \\sin\\theta_i\\sin\\alpha_i & a_i\\cos\\theta_i \\\\ \\sin\\theta_i & \\cos\\theta_i\\cos\\alpha_i & -\\cos\\theta_i\\sin\\alpha_i & a_i\\sin\\theta_i \\\\ 0 & \\sin\\alpha_i & \\cos\\alpha_i & d_i \\\\ 0 & 0 & 0 & 1 \\end{bmatrix}`"
            },
            {
              type: "code",
              content: `
% Calculates the homogeneous transformation matrix H_i^{i-1} using DH parameters
function H = Homog(theta, a, alpha, d)
    H = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

% Calculates the forward kinematics (all frame transformations up to end-effector)
function T_0_N = calculate_forward_kinematics(robot_dh_params, joint_angles)
    num_joints = size(robot_dh_params, 1);
    T_0_i = eye(4); % Initialize transformation from base to current joint

    % H_matrices stores T_0_1, T_0_2, ..., T_0_N
    H_matrices = zeros(4, 4, num_joints); 

    for i = 1:num_joints
        theta_i = joint_angles(i) + robot_dh_params(i, 4); % Add theta offset if any
        a_i = robot_dh_params(i, 1);
        alpha_i = robot_dh_params(i, 2);
        d_i = robot_dh_params(i, 3);
        
        H_i_minus_1_to_i = Homog(theta_i, a_i, alpha_i, d_i);
        T_0_i = T_0_i * H_i_minus_1_to_i;
        H_matrices(:,:,i) = T_0_i; % Store T_0^i
    end
    T_0_N = T_0_i; % Final transformation to end-effector
    % Optionally return all H_matrices for intermediate joint positions
end
              `,
                codeLang: "matlab",
                subtitle: "MATLAB functions for homogeneous transformation and forward kinematics.",
              },
          ],
      },
      {
          title: "Analytical Inverse Kinematics (IK) Derivation",
          navName: "Inverse Kinematics", // Shortened
          navRef: "analytical-inverse-kinematics",
          content: [
              {
                  type: "text",
                  content: [
                     "This section outlines the analytical approach taken for deriving the inverse kinematics (IK) solution for the 5-DOF robotic arm, utilizing the DH parameters established previously. The IK problem involves finding the set of joint angles that will place the robot's end-effector at a desired position and orientation in the workspace.",
                     "Our derivation strategy involved treating a significant portion of the arm (specifically joints 2, 3, and 4) as an RRR (Revolute-Revolute-Revolute) planar manipulator within a coordinate system defined by the first joint's rotation. Although the analytical IK was not fully implemented and integrated into the final demonstration due to time constraints, the derivation process and key equations are included here for completeness and transparency."
                  ]
              },
              {
                  type: "text",
                  displayAs: "subtitle",
                  content: "Key Equations for Joint Angle Calculation"
              },
                {
                    type: "text",
                    content: "For a desired end-effector pose (position `x_e, y_e, z_e` and orientation components), the joint angles `\\theta_1, \\theta_2, \\theta_3, \\theta_4, \\theta_5` are calculated. The derivation involved geometric and algebraic manipulation. Variables such as `D, v, l_2, l_3` represent arm geometry and intermediate values derived from the target pose and DH parameters. The following are some of the core equations derived:"
                },
                { // Each equation or group as a separate text block for MathJax rendering
                    type: "text",
                    content: "`\\theta_1 = \\text{atan2}(y_w, x_w)` where `(x_w, y_w, z_w)` is the wrist center position."
                },
                {
                    type: "text",
                    content: "The position of the wrist center `P_wc` is found by transforming the target end-effector pose `P_e` back by the fifth link's transformation: `P_{wc} = P_e \\cdot (H_5^4)^{-1}`."
                },
                {
                    type: "text",
                    content: "For the RRR (joints 2, 3, 4) portion, considering effective link lengths `l_2` (a_2) and `l_3` (related to a_3 and d_4):"
                },
                {
                    type: "text",
                    content: "`\\cos\\theta_3 = \\frac{x_{wc}^2 + y_{wc}^2 + (z_{wc}-d_1)^2 - l_2^2 - l_3^2}{2 l_2 l_3}` (using Law of Cosines in 3D for the RRR plane)."
                },
                {
                    type: "text",
                    content: "`\\theta_3` can then be found using `atan2` for the correct quadrant: `\\theta_3 = \\text{atan2}(\\pm\\sqrt{1-\\cos^2\\theta_3}, \\cos\\theta_3)` (multiple solutions)."
                },
                {
                    type: "text",
                    content: "`\\theta_2 = \\text{atan2}(z_{wc}-d_1, \\sqrt{x_{wc}^2+y_{wc}^2}) - \\text{atan2}(l_3 \\sin\\theta_3, l_2 + l_3 \\cos\\theta_3)`."
                },
                {
                    type: "text",
                    content: "`\\theta_4` and `\\theta_5` are then determined based on the desired end-effector orientation relative to the orientation of frame 3 (or 4, depending on derivation details)."
                },
                 {
                  type:"text",
                  content: "Note: Although the analytical inverse kinematics were derived, full implementation and testing were not completed for the final demonstration due to project time constraints. The team believes the derived equations provide a correct foundation for such an implementation."
                }
          ],
        },
        {
            title: "Trajectory-Based Robot Control for Jenga",
            navName: "Trajectory Control", // Shortened
            navRef: "robot-control-trajectories",
            content: [
                {
                    type: "text",
                    content: [
                        "The robotic arm was controlled using spline-based trajectories to achieve smoother, more predictable movements, which is crucial for delicate tasks like playing Jenga. Spline trajectories (e.g., cubic or quintic splines) were employed to generate smooth paths in joint space between specified waypoints. This method allows for precise control over position, velocity, and acceleration (and jerk for quintic splines), thereby minimizing vibrations and enabling more accurate end-effector movements compared to simple linear interpolation between joint states.",
                        "A conceptual cubic spline for a single joint angle `q(t)` can be represented as `q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3`, where the coefficients `a_k` are determined from boundary conditions (such as position and velocity at the start and end of each trajectory segment). For the Jenga task, we introduced intermediate waypoints and specific approach angles to ensure that movements for picking and placing blocks were not simple straight lines, but rather carefully planned maneuvers. An offset in `\\theta_2` was specifically added to create an appropriate approach angle for the pick and place actions. The duration of each trajectory segment was fine-tuned empirically to balance operational speed with movement accuracy, ensuring both smooth and effective motion for interacting with the Jenga tower."
                    ]
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Conceptual Steps for Spline Trajectory Generation"
                },
                {
                    type: "code",
                    codeLang: "plaintext", // Algorithm description
                    content:
`Algorithm: Joint Space Spline Trajectory Generation (Conceptual)

1. Define Waypoints: Specify a sequence of M joint angle configurations (P_1, P_2, ..., P_M) that the robot arm must pass through. These represent key poses in the task (e.g., approach, grasp, lift, move, place).

2. Define Segment Durations: Assign a time duration (T_1, T_2, ..., T_{M-1}) for travel between each pair of consecutive waypoints (P_i to P_{i+1}).

3. For each segment (from P_i to P_{i+1} over duration T_i):
    a. For each joint j (from 1 to N_joints):
        i. Determine Boundary Conditions for joint j:
           - q_start = P_i[j] (start position of joint j)
           - q_end = P_{i+1}[j] (end position of joint j)
           - v_start, v_end (start/end velocities, often zero at overall start/end, or matched for continuity between segments)
           - (Optional for quintic: a_start, a_end - start/end accelerations, also often zero or matched)
        ii. Calculate Spline Coefficients: Solve for the coefficients (e.g., a_0, a_1, a_2, a_3 for cubic) of the polynomial q_j(t) that satisfy these boundary conditions over the segment duration T_i.
    b. Discretize Segment Trajectory:
        i. For time t from 0 to T_i (with a small time step dt):
           - For each joint j, evaluate its spline polynomial q_j(t) to get the joint angle at time t.
           - Store the set of all N_joints joint angles at this time step t.

4. Concatenate Trajectories: Combine the discretized trajectories from all segments to form the complete joint space trajectory over the total task duration.

Output: A time-sequenced list of joint angle sets (q_1(t), ..., q_N_joints(t)) representing the smooth path for the robot arm to follow.`
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Overall Control System Flow"
                },
                {
                    type: "text",
                    content: "The control system operates as follows: Target pick and place poses in the workspace are first defined for the Jenga blocks. Ideally, these Cartesian poses would be fed into a fully implemented Inverse Kinematics (IK) solver to determine the required joint angles for each pose. Since the analytical IK was not fully integrated, joint space waypoints corresponding to these key poses were manually defined or fine-tuned. These joint waypoints are then used by the Trajectory Generator (which implements the spline algorithm described above) to produce a time-sequenced series of desired joint states (positions, and implicitly velocities and accelerations). These desired states are then passed to the low-level joint controllers of the robot, which calculate and send appropriate torque/current commands to the robot arm's actuators. The actual motion of the arm results in new joint angles, which can be fed into a Forward Kinematics (FK) module to determine the actual end-effector pose for visualization, logging, or potentially for a feedback control loop (though closed-loop feedback control based on Cartesian pose was not the primary focus of this trajectory generation project)."
                }
            ],
        },
        {
          title: "Project Challenges and Lessons Learned",
          navName: "Challenges & Lessons", // Shortened
          navRef: "challenges-and-lessons", // Standardized
          content: [
            {
              type: "text",
              content: [
                 "The project encountered several significant challenges, primarily stemming from hardware inconsistencies and limitations of the available robotic arm setups. Asymmetric robot configurations across different lab stations limited code interoperability and resulted in project downtime when switching setups. Intermittent issues with commanding torques reliably also prevented the effective implementation of dynamic compensation techniques like gravity compensation.",
                 "Furthermore, physical damage to the Jenga block dispenser mechanism led to inconsistent pick waypoints, requiring frequent recalibration. Despite dedicating significant effort (approximately 15 hours of focused work), the team was unable to fully implement and debug the analytical inverse kinematics solution to a production-ready state within the project timeline."
              ]
            },
             {
                type: "text",
                displayAs: "subtitle",
                content: "Recommendations for Future Iterations"
              },
              {
                type: "text",
                content: [
                    "To improve future iterations of this project, establishing a more robust and standardized lab environment with symmetrical robot setups is highly recommended. Utilizing dedicated lab machines connected via ethernet, rather than personal laptops with varying software versions, could also mitigate software compatibility issues.",
                    "Regarding technical approaches, implementing gravity compensation using the Lagrangian dynamics formulation, rather than relying solely on Jacobian-based methods for torque calculation, may offer a more robust solution. Prioritizing the full implementation and rigorous testing of fundamental components, such as inverse kinematics, before focusing extensively on higher-level trajectory planning refinements, would also be a beneficial strategic shift."
                ]
              }
          ],
        },
         {
           title: "Configuration and Workspace Analysis Plots",
           navName: "Analysis Plots", // Shortened
           navRef: "configuration-workspace-plots",
           content: [
               {
                   type: "text",
                   content: "The following plots illustrate the robot's performance in configuration space (joint positions, velocities, and torques) and workspace (end-effector position and velocity) during a sample run of the Jenga playing task."
               },
               {
                type: "text",
                displayAs: "subtitle",
                content: "Configuration Space Performance"
               },
              {
                type: "image",
                content: "/media/images/configurationSpace.png",
                altContent: "Joint position, velocity and torque plots",
                subtitle: "Configuration space: Joint positions, velocities, and torques.",
              },
                {
                    type: "text",
                    content: [
                        "The joint velocity curves exhibit some noise, which is common in real-world robotic systems, but the plots generally show an approximately periodic pattern corresponding to the repetitive pick-and-place motions. The torque curves are somewhat peaky, displaying noticeable changes, for example, when the arm rotates 90 degrees to place the third layer of a Jenga block.",
                        "The joint tracking performance reveals some deviations between commanded and actual joint angles, indicating areas where control could be improved, possibly through better tuning or more advanced control strategies."
                    ]
                },
               {
                  type: "text",
                  displayAs: "subtitle",
                  content: "Workspace Performance"
                },
               {
                type: "image",
                 content: "/media/images/workspacePositions.png",
                 altContent: "Workspace position plots",
                subtitle: "Workspace: Actual vs. commanded end-effector positions.",
               },
               {
                   type: "text",
                    content: [
                      "These plots reveal a notable challenge, likely related to uncompensated dynamics such as gravity. There are visible deviations between the actual and commanded end-effector trajectories, particularly in the X and Y axes. It is hypothesized that these deviations are, in part, caused by the lack of effective gravity compensation, especially its effect on the Z-axis which then couples into X and Y motion due to the arm's kinematics.",
                      "A possible solution would be to implement robust gravity compensation in the controller or, as a simpler workaround, introduce a sufficient preemptive offset in the Z-axis commands, ideally in conjunction with a fully working inverse kinematics solver."
                    ]
               },
                {
                type: "image",
                 content: "/media/images/workspaceVelocity.png",
                 altContent: "Workspace velocity plots",
                subtitle: "Workspace: Actual vs. commanded end-effector velocities.",
               },
               {
                   type: "text",
                    content: "The workspace velocity profiles appear somewhat peaky, which can be partly attributed to the fact that velocities were not directly specified as hard constraints in the spline trajectory generation but were rather a result of the position profiles. However, the general tracking of velocity trends is reasonably accurate, following the commanded changes in speed."
                },
           ],
        },
    ],
};
