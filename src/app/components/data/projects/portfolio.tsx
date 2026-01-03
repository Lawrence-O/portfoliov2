import { Project } from "@/app/components/project/interfaces";

export const portfolioWebsite: Project = {
    title: "Personal Portfolio Website",
    date: "2024-2026",
    media: "/media/videos/portfolio_demo.mp4", // Replace with actual demo video or use a screenshot
    githubLink: "https://github.com/your-username/portfolio-website",
    tags: ["Next.js", "React", "TypeScript", "Tailwind CSS", "Web Development", "UI/UX"],
    section: [
        {
            title: "Project Overview",
            navName: "Overview",
            navRef: "overview",
            content: [
                {
                    type: "text",
                    content: [
                        "This portfolio website was built from scratch to showcase my engineering projects, technical skills, and professional experience. The site features a modern, responsive design with smooth animations and an intuitive navigation system.",
                        "The primary goal was to create a platform that effectively communicates the depth and breadth of my work in robotics, control systems, and software development—while also demonstrating my web development capabilities."
                    ]
                },
                {
                    type: "text",
                    content: "Key features include dynamic project pages generated from structured data, a filterable project gallery, responsive design for all screen sizes, and optimized media loading for fast performance."
                },
            ]
        },
        {
            title: "Technology Stack",
            navName: "Tech Stack",
            navRef: "tech-stack",
            content: [
                {
                    type: "text",
                    content: "The website leverages modern web technologies to achieve optimal performance and developer experience:"
                },
                {
                    type: "text",
                    content: [
                        "Next.js 14: React framework with App Router for server-side rendering and static generation",
                        "TypeScript: Type-safe development with enhanced IDE support and error catching",
                        "Tailwind CSS: Utility-first CSS framework for rapid, consistent styling",
                        "React Components: Modular, reusable UI components for maintainable code"
                    ],
                    displayAs: "list"
                },
                {
                    type: "code",
                    codeLang: "typescript",
                    content: `
// Example: Type-safe project interface
export interface Project {
    title: string;
    date?: string;
    media: string;
    githubLink?: string;
    section: Section[];
    tags?: string[];
}

export interface Section {
    title: string;
    content: ContentBlock[];
    navName?: string;
    navRef: string;
}
                    `,
                    subtitle: "TypeScript interfaces ensure consistent data structure across all projects."
                },
            ]
        },
        {
            title: "Architecture and Design Decisions",
            navName: "Architecture",
            navRef: "architecture",
            content: [
                {
                    type: "text",
                    content: [
                        "The architecture follows a data-driven approach where each project is defined as a structured TypeScript object. This separation of content from presentation allows for easy updates and ensures consistency across all project pages.",
                        "The file structure organizes components by feature: shared UI elements, project-specific components, resume sections, and data files. This modular approach simplifies maintenance and enables code reuse."
                    ]
                },
                {
                    type: "code",
                    codeLang: "text",
                    content: `
src/
├── app/
│   ├── components/
│   │   ├── data/           # Project definitions
│   │   │   └── projects/   # Individual project files
│   │   ├── project/        # Project page components
│   │   ├── resume/         # Resume section components
│   │   ├── shared/         # Reusable UI components
│   │   └── utils/          # Helper functions
│   ├── projects/
│   │   ├── page.tsx        # Project gallery
│   │   └── [projectTitle]/ # Dynamic project pages
│   └── resume/
│       └── page.tsx
└── public/
    └── media/              # Images, videos, and files
                    `,
                    subtitle: "Project structure following Next.js App Router conventions."
                },
                {
                    type: "text",
                    content: "Dynamic routing with Next.js allows each project to have its own URL (e.g., /projects/multi-robot-frontier-exploration) while sharing a common page template. The project data is fetched based on the URL slug and rendered using reusable section components."
                },
            ]
        },
        {
            title: "Content Management System",
            navName: "Content System",
            navRef: "content-system",
            content: [
                {
                    type: "text",
                    content: "Rather than using a traditional CMS or database, the site employs a 'content-as-code' approach. Each project is a TypeScript file exporting a structured object that defines all content blocks: text, images, videos, and code snippets."
                },
                {
                    type: "code",
                    codeLang: "typescript",
                    content: `
// Example project definition
export const multiRobotExploration: Project = {
    title: "Multi-Robot Frontier Exploration",
    date: "Fall 2023",
    media: "/media/videos/frontier_grid.mp4",
    tags: ["Robotics", "Multi-Agent Systems", "A*"],
    section: [
        {
            title: "Project Overview",
            navName: "Overview",
            navRef: "overview",
            content: [
                {
                    type: "text",
                    content: "Description of the project..."
                },
                {
                    type: "image",
                    content: "/media/images/frontier_map.png",
                    subtitle: "Exploration visualization"
                },
            ]
        },
        // Additional sections...
    ]
};
                    `,
                    subtitle: "Type-safe content definition with full IDE autocomplete support."
                },
                {
                    type: "text",
                    content: [
                        "This approach offers several advantages:",
                        "Version control: All content changes are tracked in Git",
                        "Type safety: TypeScript catches errors before deployment",
                        "No external dependencies: No database or CMS to maintain",
                        "Fast builds: Content is compiled at build time for optimal performance"
                    ],
                    displayAs: "list"
                },
            ]
        },
        {
            title: "Responsive Design and UI Components",
            navName: "UI Design",
            navRef: "ui-design",
            content: [
                {
                    type: "text",
                    content: "The UI was designed with a mobile-first approach using Tailwind CSS. Custom components handle responsive layouts, ensuring optimal viewing on devices from smartphones to large desktop monitors."
                },
                {
                    type: "code",
                    codeLang: "tsx",
                    content: `
// Responsive project card grid
<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
    {projects.map((project) => (
        <ProjectCard
            key={project.title}
            title={project.title}
            description={project.description}
            media={project.media}
            tags={project.tags}
        />
    ))}
</div>
                    `,
                    subtitle: "Tailwind's responsive utilities enable adaptive layouts."
                },
                {
                    type: "text",
                    content: [
                        "Key UI features include:",
                        "Sticky navigation sidebar for project pages with scroll-aware highlighting",
                        "Smooth scroll behavior with intersection observer for section tracking",
                        "Lazy-loaded images and videos for improved initial page load",
                        "Dark theme optimized for readability and reduced eye strain",
                        "Animated gradient dividers and hover effects for visual polish"
                    ],
                    displayAs: "list"
                },
            ]
        },
        {
            title: "Project Navigation System",
            navName: "Navigation",
            navRef: "navigation",
            content: [
                {
                    type: "text",
                    content: "Each project page features a dynamic sidebar navigation that highlights the current section as the user scrolls. This is implemented using the Intersection Observer API to track which sections are visible in the viewport."
                },
                {
                    type: "code",
                    codeLang: "typescript",
                    content: `
// Scroll-aware navigation hook
useEffect(() => {
    const observer = new IntersectionObserver(
        (entries) => {
            entries.forEach((entry) => {
                if (entry.isIntersecting) {
                    setActiveSection(entry.target.id);
                }
            });
        },
        { rootMargin: "-20% 0px -70% 0px" }
    );

    sections.forEach((section) => {
        const element = document.getElementById(section.navRef);
        if (element) observer.observe(element);
    });

    return () => observer.disconnect();
}, [sections]);
                    `,
                    subtitle: "Intersection Observer tracks visible sections for navigation highlighting."
                },
            ]
        },
        {
            title: "Performance Optimization",
            navName: "Performance",
            navRef: "performance",
            content: [
                {
                    type: "text",
                    content: "Performance was a key consideration throughout development. The site leverages Next.js features for optimal loading:"
                },
                {
                    type: "text",
                    content: [
                        "Static Generation: Project pages are pre-rendered at build time",
                        "Image Optimization: Next.js Image component with automatic resizing and WebP conversion",
                        "Code Splitting: Each page loads only the JavaScript it needs",
                        "Font Optimization: Self-hosted fonts with display swap for fast text rendering"
                    ],
                    displayAs: "list"
                },
                {
                    type: "text",
                    content: "Media files are organized in the public directory and served directly by the CDN, with videos using native lazy loading to prevent unnecessary bandwidth usage on initial page load."
                },
            ]
        },
        {
            title: "Challenges and Solutions",
            navName: "Challenges",
            navRef: "challenges",
            content: [
                {
                    type: "text",
                    content: "Building this portfolio presented several interesting challenges:"
                },
                {
                    type: "text",
                    content: [
                        "Dynamic Routing with Static Content: Solved by creating a project directory that maps URL slugs to project data files, enabling type-safe lookups while maintaining static generation.",
                        "Consistent Styling Across Content Types: Developed a unified component system where each content block type (text, image, video, code) has consistent spacing and responsive behavior.",
                        "Managing Large Media Files: Implemented a structured media organization system with separate folders for images, videos, and downloadable files, with consistent naming conventions.",
                        "Type-Safe Content Authoring: Created comprehensive TypeScript interfaces that provide IDE autocomplete and catch errors when adding new project content."
                    ],
                    displayAs: "list"
                },
            ]
        },
        {
            title: "Future Improvements",
            navName: "Future Work",
            navRef: "future-work",
            content: [
                {
                    type: "text",
                    content: "Planned enhancements for the portfolio include:"
                },
                {
                    type: "text",
                    content: [
                        "Blog Section: Add a technical blog for in-depth articles on robotics and software topics",
                        "Search Functionality: Implement full-text search across all projects",
                        "Interactive Demos: Embed interactive visualizations for select projects",
                        "Internationalization: Support for multiple languages"
                    ],
                    displayAs: "list"
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
                    content: [
                        "This portfolio website demonstrates not only my engineering projects but also my ability to build modern, performant web applications. The data-driven architecture ensures the site remains maintainable as new projects are added.",
                        "The combination of Next.js, TypeScript, and Tailwind CSS provides an excellent foundation for a professional portfolio—balancing developer experience with end-user performance. Every project page you've viewed on this site is powered by the same component system described here."
                    ]
                },
            ]
        },
    ]
};