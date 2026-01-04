import { Project } from "@/app/components/project/interfaces";

export const portfolioWebsite: Project = {
    title: "Portfolio Website",
    date: "2024-2026",
    media: "/media/videos/portfolio_demo.mp4",
    githubLink: "https://github.com/your-username/portfolio-website",
    tags: ["Next.js", "TypeScript", "Tailwind CSS", "React"],
    section: [
        {
            title: "Project Overview",
            navName: "Overview",
            navRef: "overview",
            content: [
                {
                    type: "text",
                    content:
                        "This portfolio website was built to document engineering projects and provide a central place to share technical work. It uses a data-driven architecture where each project is defined in a structured TypeScript file, making it easy to add new content.",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Data-driven content** — projects defined as structured TypeScript objects",
                        "**Responsive design** — works on phones, tablets, and desktops",
                        "**Dynamic routing** — each project gets its own URL automatically",
                    ],
                },
            ],
        },
        {
            title: "Tech Stack",
            navName: "Tech Stack",
            navRef: "tech-stack",
            content: [
                {
                    type: "text",
                    content:
                        "The site uses a modern web stack with Next.js for routing and static generation, TypeScript for type safety, and Tailwind CSS for styling.",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Next.js 14** — React framework with file-based routing and static generation",
                        "**TypeScript** — type-safe development with compile-time error checking",
                        "**Tailwind CSS** — utility-first CSS framework",
                        "**React** — component-based UI architecture",
                    ],
                },
                {
                    type: "code",
                    codeLang: "typescript",
                    content: `// Each project is a typed object
export interface Project {
    title: string;
    subtitle?: string;
    media: string;
    tags?: string[];
    section: Section[];
}`,
                    subtitle: "TypeScript interfaces keep project data consistent",
                },
            ],
        },
        {
            title: "Architecture",
            navName: "Architecture",
            navRef: "architecture",
            content: [
                {
                    type: "text",
                    content:
                        "The architecture follows a data-driven approach where each project is defined as a structured TypeScript object. This separation of content from presentation allows for easy updates and ensures consistency across all project pages.",
                },
                {
                    type: "code",
                    codeLang: "text",
                    content: `src/app/
├── components/
│   ├── data/projects/   # Project content files
│   ├── project/         # Page components
│   ├── resume/          # Resume components
│   └── shared/          # Reusable UI components
├── projects/
│   ├── page.tsx         # Project gallery
│   └── [projectTitle]/  # Dynamic routes
└── resume/`,
                    subtitle: "Next.js App Router file structure",
                },
                {
                    type: "text",
                    content:
                        "Dynamic routing with Next.js allows each project to have its own URL (e.g., `/projects/multi-robot-frontier-exploration`) while sharing a common page template. The project data is fetched based on the URL slug and rendered using reusable section components.",
                },
            ],
        },
        {
            title: "Content System",
            navName: "Content System",
            navRef: "content-system",
            content: [
                {
                    type: "text",
                    content:
                        "Rather than using a traditional CMS or database, the site employs a content-as-code approach. Each project is a TypeScript file exporting a structured object that defines all content blocks: text, images, videos, code snippets, and math equations.",
                },
                {
                    type: "code",
                    codeLang: "typescript",
                    content: `// Adding a new project = creating a new file
export const myProject: Project = {
    title: "Project Name",
    media: "/media/videos/demo.mp4",
    section: [
        {
            title: "Overview",
            navRef: "overview",
            content: [
                { type: "text", content: "Description..." },
                { type: "image", content: "/media/images/fig.png" },
            ],
        },
    ],
};`,
                    subtitle: "Project definition with typed content blocks",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Benefits",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Version controlled** — all content changes tracked in Git",
                        "**Type safe** — TypeScript catches errors at compile time",
                        "**No external dependencies** — no database or CMS to maintain",
                        "**IDE support** — autocomplete and error highlighting",
                    ],
                },
            ],
        },
        {
            title: "UI Components",
            navName: "UI Design",
            navRef: "ui-design",
            content: [
                {
                    type: "text",
                    content:
                        "The UI uses a dark theme with responsive layouts built using Tailwind's breakpoint utilities.",
                },
                {
                    type: "code",
                    codeLang: "tsx",
                    content: `// Responsive grid: 1 column on mobile, 3 on desktop
<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
    {projects.map((project) => (
        <ProjectCard key={project.title} {...project} />
    ))}
</div>`,
                    subtitle: "Tailwind handles responsive breakpoints",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Features",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Sticky sidebar** — navigation follows scroll position",
                        "**Section highlighting** — sidebar indicates current section",
                        "**Lazy-loaded media** — videos load on scroll",
                        "**Dark theme** — dark color scheme throughout",
                    ],
                },
            ],
        },
        {
            title: "Scroll-Aware Navigation",
            navName: "Navigation",
            navRef: "navigation",
            content: [
                {
                    type: "text",
                    content:
                        "The sidebar highlights the current section using the Intersection Observer API to track which sections are visible in the viewport.",
                },
                {
                    type: "code",
                    codeLang: "typescript",
                    content: `// Track which section is visible
const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
        if (entry.isIntersecting) {
            setActiveSection(entry.target.id);
        }
    });
}, { rootMargin: "-20% 0px -70% 0px" });`,
                    subtitle: "Intersection Observer updates the active section",
                },
            ],
        },
        {
            title: "Performance",
            navName: "Performance",
            navRef: "performance",
            content: [
                {
                    type: "text",
                    content:
                        "The site leverages Next.js features for performance: static generation, automatic image optimization, and code splitting.",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Static generation** — pages built once, served fast",
                        "**Image optimization** — Next.js auto-converts to WebP",
                        "**Code splitting** — each page loads its own bundle",
                        "**Lazy video loading** — videos wait until you scroll to them",
                    ],
                },
            ],
        },
        {
            title: "Challenges",
            navName: "Challenges",
            navRef: "challenges",
            content: [
                {
                    type: "text",
                    content:
                        "Key challenges addressed during development:",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Dynamic routing with static content** — mapping URL slugs to project data files while maintaining type safety",
                        "**Consistent styling** — unified component system for different content types (text, images, video, code)",
                        "**Media organization** — structured folder system for images, videos, and files",
                    ],
                },
            ],
        },
        {
            title: "Future Work",
            navName: "Future",
            navRef: "future-work",
            content: [
                {
                    type: "text",
                    content:
                        "Potential future additions:",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Blog section** — longer-form technical writing",
                        "**Search** — full-text search across projects",
                        "**Interactive demos** — embedded simulations for select projects",
                    ],
                },
            ],
        },
        {
            title: "Conclusion",
            navName: "Conclusion",
            navRef: "conclusion",
            content: [
                {
                    type: "text",
                    content:
                        "The data-driven architecture makes it straightforward to add new projects—each one is just a new TypeScript file following the same structure. All project pages use the same component system shown here.",
                },
            ],
        },
    ],
};