import { resumeBlurb } from "@/app/components/resume/ResumeBlurb";
import { resumeNavProps } from "@/app/components/resume/ResumeNav";



export const currCoursework: string[] = [
  "Robot Kinematics & Dynamics",
  "Computer Vision",
  "Optimal Control & Reinforcement Learning",
  "Robotic Planning",
  "Principles of Imperative Computation",
  "Feedback Control Systems",
  "Robotic Systems Engineering",
  "Electromechanical Systems Design",
  "Engineering Design",
];

const relevantCourses = (
  <div className="flex flex-col space-y-1">
    <h3 className="text-xm group-hover:text-textHover">Relevant Coursework</h3>
    <div className="flex flex-col ">
      {currCoursework.map((course, index) => (
        <p key={index} className="text-xs">
          {course}
        </p>
      ))}
    </div>
  </div>
);

export const currEducation: resumeBlurb = {
  title: "Carnegie Mellon University",
  time: "2020 - 2024",
  subtitle:
    "Bachelor of Science in Mechanical Engineering and Robotics Engineering",
  location: "Pittsburgh, PA",
  showSubtitle: true,
  description: ["GPA: 3.8 / 4.00  Honors: CIT Dean List, CIT Honors Degree"],
  additionalContent: relevantCourses,
};

export const currResumeBody: resumeBlurb[] = [
  {
    title: "Microsoft (Azure IoT Operations)",
    time: "Jul. 2024 - Present",
    subtitle: "Software Engineer (Full Stack)",
    location: "Redmond, WA",
    showSubtitle: true,
    description: [
      "Spearheading offline testing for Azure IoT Operations, collaborating with component teams to create and implement an offline testing plan. Leveraging Azure, Kubernetes, and C# to ensure rigorous metric validation and alignment with performance goals, contributing to the successful launch of the platform.",
      "Designing and developing an in-house web application for debugging Azure IoT Operations with React, TypeScript, Next.js, and .NET Framework. Leading feature proposals and implementation, reducing debugging time by up to 80% by centralizing and automating information display, eliminating manual data retrieval, and streamlining the troubleshooting process.",
    ],
    technologies: [
      "Azure",
      "Kubernetes",
      "C#",
      "React",
      "TypeScript",
      "Next.js",
      ".NET Framework",
    ],
  },
  {
    title: "Atlassian",
    time: "May – Aug. 2023",
    subtitle: "Software Engineering Intern (Backend)",
    location: "New York, NY",
    showSubtitle: true,
    description: [
      "Consolidated and deployed two critical tier microservices utilizing Spring, Kotlin, Coroutines streamlining sequence number generation and eliminating redundant code across multiple services. By eliminating duplicated logic, this unified approach enhanced system efficiency and maintainability while maintaining similar response times.",
      "Designed customized dashboards using Splunk and Terraform, empowering individual service teams with dedicated monitoring tools for service performance.",
      "Utilized Docker for containerization, DynamoDB for database management, and AWS Kubernetes for orchestrating microservices, ensuring efficient deployment, scaling, and management of the system.",
    ],
    technologies: [
      "Spring",
      "Kotlin",
      "Coroutines",
      "Splunk",
      "Terraform",
      "Docker",
      "DynamoDB",
      "AWS Kubernetes",
    ],
  },
  {
    title: "Google (Cloud TI Platforms)",
    time: "May – Aug. 2022",
    subtitle: "Software Engineering Intern (SWE)",
    location: "Sunnyvale, CA",
    showSubtitle: true,
    description: [
      "Developed a diagnostic tool capable of remotely accessing each SSD via SSH, automatically extracting core logs and conducting detailed analysis. Analysis was performed at the individual core level within the SSDs, contributing to the identification and isolation of malfunctioning cores.",
      "Achieved a noteworthy 30% reduction in debugging time for SSD developers, streamlining issue resolution processes and notably enhancing the overall debugging experience.",
    ],
    technologies: ["Python", "Bash"],
  },
  {
    title: "Microsystems & Mechanobiology Lab (MMBL)",
    time: "Jun. – Sep. 2021",
    subtitle: "Undergraduate Researcher",
    location: "Pittsburgh, PA",
    showSubtitle: true,
    description: [
      "Mapped the analytical design space for DNA origami-based forceps sensors that use Forster Resonance Energy Transfer (FRET) to measure distances beyond the Forster distance",
      "Generated over 1,000,000+ valid data for use in designing custom DNA Origami-based forceps sensors.",
      "Created a program that would take user specifications for custom forceps sensors and then output a corresponding forceps sensor and a visualization based off the generated data points.",
    ],
    technologies: ["Python", "SQL", "MATLAB"],
  },
];

export const resumeSections: resumeNavProps[] = [
  { id: "education", label: "Education" },
  { id: "experience", label: "Experience" },
  { id: "skills", label: "Skills" },
];
