import { Project } from "@/app/components/project/interfaces";

export const roboticArmControl: Project = {
    title: "Robotic Arm Control and Trajectory Generation",
    subtitle: "Capstone Project",
    media: "/media/videos/rkd_jenga.mp4",
    tags: ["Robotics", "Forward Kinematics", "Inverse Kinematics", "Trajectory Planning", "MATLAB"],
    section: [
        {
            title: "Project Introduction",
            navName: "Introduction",
            navRef: "introduction",
            content: [
              {
                type: "text",
                content:
                  "This capstone project focused on the development of a robotic arm control system through trajectory generation. The project involved a comprehensive exploration of key robotics concepts including Denavit-Hartenberg (DH) parameter extraction, forward kinematics, and trajectory planning. The objective was to achieve a deep understanding of the complexities of robot arm control and motion planning, with all implementations performed in MATLAB.",
              },
              {
                 type: "image",
                 content: "/media/images/roboticArmIntro.png",
                 altContent: "Illustration of a robotic arm",
                 subtitle: "Overview of the robotic arm system."
              }
            ],
          },
        {
            title: "Denavit-Hartenberg Parameter Extraction",
            navName: "DH Parameter Extraction",
            navRef: "dh-parameter-extraction",
            content: [
                {
                    type: "text",
                    content:
                         "The process of extracting Denavit-Hartenberg (DH) parameters began by taking precise measurements of the link lengths between the robotic arm's joints, as well as the offsets between each frame. These measurements were then systematically compared with established conventions to derive the corresponding DH parameters for our robotic system. To ensure consistency, we assumed that all joints were positioned out of the page, adhering to the standard conventions used in DH parameterization." ,
                },
                {
                    type: "text",
                    content: "The following table shows the extracted DH parameters for the reference configuration:"
                },
                {
                    type: "code",
                    content: `
    | Link Lengths | Link Twist | Link Offset | Joint Angle |
    |--------------|------------|-------------|-------------|
    | 0            |    π/2     |    56.05    |     0       |
    | 400          |     0      |     94      |     0       |
    | 334          |   -π/2     |     3       |     0       |
    | 0            |     π/2    |      39     |     0       |
    | 0            |      0     |    108.05   |     0       |
                  `,
                    codeLang: "markdown",
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
              content:
                "To implement the forward kinematics of our robotic arm, we constructed homogeneous transformation matrices between each frame. This process involved applying the extracted DH parameters using the standard transformation convention. The homogeneous transformation matrix was derived from the four DH parameters."
            },
            {
              type: "text",
                content:
                    "The transformation matrix was derived with the convention H-1 = Rotz,θi Transz,di Transx,ai Rotx,αi.",
              },
            {
              type: "code",
              content: `
/**
 * Calculates the homogeneous transformation matrix using DH parameters.
 *
 * This function takes the joint angle (theta), link length (a), link twist (alpha),
 * and link offset (d) as inputs, and returns the corresponding 4x4 homogeneous transformation matrix.
 *
 * @param theta Joint angle.
 * @param a Link length.
 * @param alpha Link twist.
 * @param d Link offset.
 * @return The homogeneous transformation matrix.
 */
function H=Homog(theta,a,alpha,d)
    H=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
    0   sin(alpha)     cos(alpha)       d
    0       0              0             1];
end
              `,
                codeLang: "matlab",
                subtitle: "MATLAB function for calculating the homogeneous transformation matrix.",
              },
             {
              type: "code",
              content: `
/**
 * Calculates the forward kinematics of the robot.
 *
 * This function calculates the transformation matrices for each link of the robot and
 * stores the results in a 4x4xN array, where N is the number of degrees of freedom (DOF).
 * It uses the DH parameters and the joint angles to calculate the positions of each joint
 * frame relative to the base frame.
 *
 * @param robot A struct containing robot properties, including DH parameters and DOF.
 * @param thetas A vector of joint angles for the robot.
 * @return A 4x4xN array of transformation matrices for the robot.
*/
function frames = forward_kinematics(robot, thetas)
    frames = zeros(4,4,robot.dof);
    n = robot.dof;
    frames(:,:,1)=Homog(thetas(1),robot.dh_parameters(1,1), 
      robot.dh_parameters(1,2), 
      robot.dh_parameters(1,3));
    for i=2:n
        frames(:,:,i)=frames(:,:,i-1)*Homog(thetas(i)+robot.dh_parameters(i,4),
          robot.dh_parameters(i, 1), 
          robot.dh_parameters(i, 2), 
          robot.dh_parameters(i,3));
    end
end
              `,
              codeLang: "matlab",
              subtitle: "MATLAB function for calculating the forward kinematics.",
             },
          ],
      },
      {
          title: "Analytical Inverse Kinematics Derivation",
          navName: "Analytical Inverse Kinematics",
          navRef: "analytical-inverse-kinematics",
          content: [
              {
                  type: "text",
                  content:
                     "This section outlines the analytical inverse kinematics (IK) for the robotic arm using the derived DH parameters from the previous section. The IK derivation involved treating a portion of the arm as an RRR (Revolute-Revolute-Revolute) manipulator. Although the analytical IK was not fully implemented due to time constraints, the derivation process is included here for completeness and transparency.",
              },
              {
                  type: "text",
                  content: "The following equations were derived for calculating the joint angles of the robotic arm:",
              },
                {
                    type: "text",
                    content:
                        "θ4 = θ + θ3 - θ2, where θ is a variable angle."
                },
               {
                   type: "text",
                    content:
                       "The position of the arm in the xy plane are defined as:" +
                       "xp = xe + DS1  \n"+
                       "yp = ye – DC1  \n"+
                       "where D is a variable, and C1 and S1 represent cosine and sine of theta1 respectively. "+
                       "The position of the arm in the z plane are defined as: \n" +
                       "xc = xp - d5C1  \n"+
                       "yc = yp - d5S1  \n"+
                       "zc = ze - d5S0"
               },
              {
                  type: "text",
                   content: "The projection onto the xy plane was calculated using: dist = sqrt(xc^2 + yc^2)."
                },
              {
                  type: "text",
                   content: "The cosine rule was used to derive θ3 = (v² + dist² – l3² – l2²)/(2l3l2), where v is a variable, and l2 and l3 represent link lengths. "
                },
                 {
                  type: "text",
                   content: "θ2 = atan2(v, dist) – atan2(l3S3/(l2 + l3C3)), where S3 and C3 are the sine and cosine of theta3."
                 },
                {
                   type: "text",
                    content: "θ1 = θ1 - γ + γ, where gamma is an arbitrary angle."
                 },
                {
                   type: "text",
                   content: "θ1 = atan2(ye, xe) + arcsin (D/√(x² + y²)), which determines the base angle of the robot."
                 },
                 {
                  type:"text",
                  content:"*Note:* Although the analytical inverse kinematics were not fully implemented for the demo, the team believes them to be reasonably correct, based on the derivation process."
                }
          ],
        },
        {
            title: "Trajectory-Based Robot Control",
            navName: "Robot Control using Trajectories",
            navRef: "robot-control-trajectories",
            content: [
                {
                    type: "text",
                    content:
                        "The robotic arm was controlled using spline trajectories to minimize jerk and achieve smoother movements for repetitive tasks. We introduced midpoints and approach angles to ensure that the movement between key positions was not a simple straight line. An offset in theta_2 was added to create an approach angle for the pick and place action. The duration of each trajectory was fine-tuned to balance speed and accuracy, ensuring both smooth and effective motion.",
                },
            ],
        },
        {
          title: "Project Challenges and Lessons Learned",
          navName: "Challenges and Approach",
          navRef: "challenges-and-approach",
          content: [
            {
              type: "text",
              content:
                 "The project faced several challenges, primarily due to hardware inconsistencies in the robotic arm. Asymmetric robot setups limited code interoperability and resulted in project downtime. Issues with commanding torques prevented the implementation of gravity compensation. Damage to the block dispenser led to inconsistent pick waypoints. Despite significant effort (approximately 15 hours), the team was unable to fully implement the inverse kinematics.",
            },
             {
                type: "text",
                content:
                    "To improve future iterations of this project, a more robust lab environment with symmetrical robots is recommended. Using dedicated lab machines over ethernet to isolate software version issues could also be beneficial. Furthermore, implementing gravity compensation using the Lagrangian method, rather than the Jacobian, may be a better approach. Focusing on implementation of fundamental parts of the project such as inverse kinematics first, rather than focusing on implementation of trajectory planning, would also be beneficial."
              }
          ],
        },
         {
           title: "Configuration and Workspace Analysis",
           navName: "Configuration and Workspace Plots",
           navRef: "configuration-workspace-plots",
           content: [
               {
                   type: "text",
                   content: "The following are plots of the configuration space, showing the joint positions, velocities, and torques during a sample run of the robotic arm."
               },
              {
                type: "image",
                content: "/media/images/configurationSpace.png",
                altContent: "Joint position, velocity and torque",
                subtitle: "Configuration space plots illustrating joint positions, velocities, and torques.",
              },
                {
                    type: "text",
                    content:
                        "The velocity curves exhibit some noise, but the plots generally show an approximately periodic pattern. The torque curves are peaky, and display changes when the arm rotates 90 degrees to place the third layer of the block. The joint tracking reveals deviations and some inconsistencies in the tracking performance."
                },
               {
                  type: "text",
                    content: "The following plots show the end effector position, velocity, and orientation during a sample run of the robotic arm."
                },
               {
                type: "image",
                 content: "/media/images/workspacePositions.png",
                 altContent: "Workspace position",
                subtitle: "Workspace position plots showing actual and commanded trajectories.",
               },
               {
                   type: "text",
                    content:
                      "These plots reveal a challenge with gravity compensation, as there are noticeable deviations between the actual and commanded trajectories. The hypothesis is that the deviations in the x and y axes are caused by the lack of gravity compensation in the z axis. A possible solution would be to implement gravity compensation or introduce a sufficient offset in the z axis combined with the inverse kinematics."
               },
                {
                type: "image",
                 content: "/media/images/workspaceVelocity.png",
                 altContent: "Workspace velocity",
                subtitle: "Workspace velocities plots illustrating actual and commanded trajectories.",
               },
               {
                   type: "text",
                    content:
                        "The workspace velocities are peaky, partly due to not being directly specified. However, the velocity tracking is generally accurate."
                },
           ],
        },
    ],
};