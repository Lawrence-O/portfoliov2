import { Project } from "@/app/components/project/interfaces";

export const roboticArmControl: Project = {
  title: "Robotic Arm Control and Trajectory Generation for Jenga",
  subtitle:
    "Implementing forward/inverse kinematics and spline trajectories for a 5-DOF arm",
  date: "Fall 2023",
  media: "/media/videos/rkd_jenga.mp4",
  tags: [
    "Robotics",
    "Forward Kinematics",
    "Inverse Kinematics",
    "Trajectory Planning",
    "MATLAB",
    "5-DOF Arm",
  ],
  section: [
    {
      title: "Project Introduction",
      navName: "Introduction",
      navRef: "introduction",
      content: [
        {
          type: "text",
          content:
            "How do you teach a robot arm to play Jenga? It needs to know where its hand is (forward kinematics), how to reach a target position (inverse kinematics), and how to move smoothly without knocking over the tower (trajectory planning). This capstone project tackled all three challenges for a **5-DOF articulated arm**, implementing the math from scratch in MATLAB.",
        },
        {
          type: "video",
          content: "/media/videos/rkd_jenga.mp4",
          altContent: "Robotic arm playing Jenga",
          subtitle: "The 5-DOF robotic arm performing Jenga tasks.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Conceptual DH Frame Assignment",
        },
        {
          type: "text",
          content:
            "Coordinate frames were assigned to each joint following the **Denavit-Hartenberg convention**: z-axis along the joint axis, x-axis normal to current and previous z-axes.",
        },
        {
          type: "image",
          content: "/media/images/roboticArmIntro.png",
          altContent: "Illustration of a robotic arm with DH frames",
          subtitle: "Robotic arm with Denavit-Hartenberg frame assignments.",
        },
      ],
    },
    {
      title: "DH Parameter Extraction",
      navName: "DH Parameters",
      navRef: "dh-parameter-extraction",
      content: [
        {
          type: "text",
          content:
            "Before computing anything, we need a consistent way to describe the robot's geometry. **Denavit-Hartenberg (DH) parameters** are a standard convention: four numbers per joint that fully specify how each link connects to the next. We measured the physical arm and extracted these parameters—think of it as creating a mathematical blueprint of the robot.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Parameter Definitions",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "`a_i` — link length (distance along x-axis)",
            "`α_i` — link twist (rotation about x-axis)",
            "`d_i` — link offset (distance along z-axis)",
            "`θ_i` — joint angle (rotation about z-axis)",
          ],
        },
        {
          type: "code",
          codeLang: "plaintext",
          content: `Link | a_i   | α_i   | d_i    | θ_i
-----|-------|-------|--------|----------
1    | 0     | π/2   | 56.05  | variable
2    | 400   | 0     | 94     | variable
3    | 334   | -π/2  | 3      | variable
4    | 0     | π/2   | 39     | variable
5    | 0     | 0     | 108.05 | variable`,
          subtitle: "Extracted DH parameters for the reference configuration.",
        },
      ],
    },
    {
      title: "Forward Kinematics",
      navName: "Forward Kinematics",
      navRef: "forward-kinematics",
      content: [
        {
          type: "text",
          content:
            "Given the joint angles, where is the gripper? **Forward kinematics** answers this by chaining together transformations from base to tip. Each joint adds a rotation and translation; multiply them all together and you get the end-effector's position and orientation. It's like following a series of 'turn left, go forward, rotate' instructions.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Transformation Convention",
        },
        {
          type: "text",
          content:
            "The transformation from frame `i` to frame `i-1` follows the standard DH convention:",
        },
        {
          type: "math",
          content:
            "H_i^{i-1} = Rot_z(\\theta_i) \\cdot Trans_z(d_i) \\cdot Trans_x(a_i) \\cdot Rot_x(\\alpha_i)",
        },
        {
          type: "text",
          content: "Expanding this yields the general transformation matrix:",
        },
        {
          type: "math",
          content:
            "H_i^{i-1} = \\begin{bmatrix} c\\theta_i & -s\\theta_i c\\alpha_i & s\\theta_i s\\alpha_i & a_i c\\theta_i \\\\ s\\theta_i & c\\theta_i c\\alpha_i & -c\\theta_i s\\alpha_i & a_i s\\theta_i \\\\ 0 & s\\alpha_i & c\\alpha_i & d_i \\\\ 0 & 0 & 0 & 1 \\end{bmatrix}",
        },
        {
          type: "code",
          codeLang: "matlab",
          content: `function H = Homog(theta, a, alpha, d)
% Homogeneous transformation using DH params
  H = [
    cos(theta), -sin(theta)*cos(alpha), ...
      sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), ...
     -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha), cos(alpha), d;
    0,           0,          0,          1
  ];
end

function T = forward_kinematics(dh, q)
% Compute end-effector transformation
% dh: [a, alpha, d, theta_offset] per row
% q:  joint angles vector
  T = eye(4);
  for i = 1:size(dh, 1)
    theta_i = q(i) + dh(i, 4);
    H_i = Homog(theta_i, dh(i,1), ...
                dh(i,2), dh(i,3));
    T = T * H_i;
  end
end`,
          subtitle: "MATLAB functions for forward kinematics.",
        },
      ],
    },
    {
      title: "Analytical Inverse Kinematics",
      navName: "Inverse Kinematics",
      navRef: "analytical-inverse-kinematics",
      content: [
        {
          type: "text",
          content:
            "The harder problem: given a target position for the gripper, what joint angles get us there? **Inverse kinematics** is like solving the forward problem backwards—and it's much trickier. There might be multiple solutions (elbow up vs. elbow down), no solution (target out of reach), or infinitely many solutions. We used geometry and trigonometry to derive closed-form equations.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Joint Angle Derivation",
        },
        {
          type: "text",
          content:
            "First, compute the **wrist center** by transforming back from the end-effector:",
        },
        {
          type: "math",
          content: "P_{wc} = P_e \\cdot (H_5^4)^{-1}",
        },
        {
          type: "text",
          content:
            "The base rotation is determined from the wrist center position:",
        },
        {
          type: "math",
          content: "\\theta_1 = \\text{atan2}(y_{wc}, x_{wc})",
        },
        {
          type: "text",
          content:
            "For the RRR portion (joints 2-4), apply the **law of cosines**:",
        },
        {
          type: "math",
          content:
            "\\cos\\theta_3 = \\frac{r^2 + (z_{wc}-d_1)^2 - l_2^2 - l_3^2}{2 l_2 l_3}",
        },
        {
          type: "text",
          content:
            "where $r = \\sqrt{x_{wc}^2 + y_{wc}^2}$. Then solve for $\\theta_3$ using `atan2` for the correct quadrant (elbow-up/down configurations).",
        },
        {
          type: "math",
          content:
            "\\theta_2 = \\text{atan2}(z_{wc}-d_1, r) - \\text{atan2}(l_3 s_3, l_2 + l_3 c_3)",
        },
        {
          type: "text",
          content:
            "Finally, $\\theta_4$ and $\\theta_5$ are determined from the desired end-effector orientation relative to frame 3.",
        },
        {
          type: "text",
          content:
            "*Note: The analytical IK was derived but not fully implemented due to time constraints. The equations provide a foundation for future work.*",
        },
      ],
    },
    {
      title: "Trajectory-Based Control",
      navName: "Trajectory Control",
      navRef: "robot-control-trajectories",
      content: [
        {
          type: "text",
          content:
            "Knowing *where* to go isn't enough—*how* you get there matters too. Jerky movements would knock over the Jenga tower! **Cubic splines** create smooth paths through waypoints, with controlled velocity at each point. The math ensures the arm accelerates and decelerates gracefully, critical for the delicate task of sliding out blocks.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Cubic Spline Formulation",
        },
        {
          type: "text",
          content: "A cubic spline for joint angle $q(t)$ over segment duration $T$:",
        },
        {
          type: "math",
          content: "q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3",
        },
        {
          type: "text",
          content:
            "Coefficients $a_k$ are solved from boundary conditions: positions and velocities at segment start/end. We introduced **intermediate waypoints** and specific approach angles (via $\\theta_2$ offset) for precise block manipulation.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Trajectory Generation Algorithm",
        },
        {
          type: "code",
          codeLang: "plaintext",
          content: `Joint Space Spline Trajectory Generation:

1. Define waypoints P_1...P_M (key poses)
2. Assign segment durations T_1...T_{M-1}

3. For each segment (P_i → P_{i+1}):
   For each joint j:
     a. Set boundary conditions:
        - q_start, q_end (positions)
        - v_start, v_end (velocities)
     b. Solve spline coefficients
     c. Discretize: evaluate q_j(t) at dt steps

4. Concatenate all segments

Output: Time-sequenced joint angles`,
          subtitle: "Conceptual algorithm for spline trajectory generation.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Control System Flow",
        },
        {
          type: "text",
          content:
            "Target Cartesian poses are defined for pick/place operations. Since analytical IK wasn't fully integrated, joint waypoints were **manually defined**. The trajectory generator produces time-sequenced joint commands, which are sent to the low-level controllers. Forward kinematics provides end-effector pose for visualization and logging.",
        },
      ],
    },
    {
      title: "Challenges and Lessons Learned",
      navName: "Challenges",
      navRef: "challenges-and-lessons",
      content: [
        {
          type: "text",
          content:
            "Real robotics is messy. The theoretical math is elegant, but hardware has quirks, lab setups vary between stations, and time runs out before everything is perfect. Here's what we learned the hard way:",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Challenges",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Asymmetric robot configurations** — different lab stations had varying setups, limiting code portability",
            "**Unreliable torque commands** — prevented effective gravity compensation implementation",
            "**Damaged block dispenser** — caused inconsistent pick positions requiring frequent recalibration",
            "**Time constraints** — ~15 hours of effort couldn't fully debug the analytical IK solution",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Recommendations",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Standardize lab environment** — use symmetric robot setups across stations",
            "**Use dedicated machines** — Ethernet-connected lab computers avoid software compatibility issues",
            "**Lagrangian gravity compensation** — more robust than Jacobian-based methods",
            "**Prioritize fundamentals** — fully implement IK before refining trajectory planning",
          ],
        },
      ],
    },
    {
      title: "Performance Analysis",
      navName: "Analysis",
      navRef: "configuration-workspace-plots",
      content: [
        {
          type: "text",
          content:
            "How well did the arm actually perform? We can look at two views: **configuration space** (what the joints did) and **workspace** (where the gripper went). Comparing commanded vs. actual values reveals where the control system struggled.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Configuration Space",
        },
        {
          type: "image",
          content: "/media/images/configurationSpace.png",
          altContent: "Joint position, velocity and torque plots",
          subtitle: "Joint positions, velocities, and torques over time.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Joint velocities** show noise but exhibit periodic patterns matching pick-and-place cycles",
            "**Torques** display peaks during 90° rotations for block placement",
            "**Tracking errors** indicate room for improved control tuning",
          ],
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Workspace Performance",
        },
        {
          type: "image",
          content: "/media/images/workspacePositions.png",
          altContent: "Workspace position plots",
          subtitle: "Actual vs. commanded end-effector positions.",
        },
        {
          type: "text",
          content:
            "Visible deviations between actual and commanded trajectories—particularly in X and Y axes—suggest **uncompensated gravity effects**. The Z-axis error couples into X/Y motion through the arm's kinematics. A robust gravity compensation scheme or preemptive Z-offset could address this.",
        },
        {
          type: "image",
          content: "/media/images/workspaceVelocity.png",
          altContent: "Workspace velocity plots",
          subtitle: "Actual vs. commanded end-effector velocities.",
        },
        {
          type: "text",
          content:
            "Velocity profiles appear **peaky** since velocities weren't directly constrained in trajectory generation—they result from position profiles. However, overall velocity tracking follows commanded trends reasonably well.",
        },
      ],
    },
  ],
};
