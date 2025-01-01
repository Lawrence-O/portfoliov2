import Layout from "@/app/components/layout";
import { Project } from "@/app/components/project/interfaces";
import { ProjectBody } from "@/app/components/project/ProjectBody";
import { notFound } from "next/navigation";

interface ProjectPageProps {
    params: {
        projectTitle: string;
    };
}

async function fetchProjectData(projectSlug: string): Promise<Project>{
    const projects = await import("@/app/components/data/projectPageDirectory");
    const projectPageMapping = projects.projectPagesDict;
    return projectPageMapping[projectSlug] || null;
}

export async function generateStaticParams(){
    const projects = await import("@/app/components/data/projectPageDirectory");
    const projectPageMapping = projects.projectPagesDict;
    return Object.keys(projectPageMapping).map((projectSlug) => {
        return {
            params: {
                projectTitle: projectSlug,
            },
        };
    });
}

export default async function ProjectPage({ params }: ProjectPageProps) {
    const { projectTitle } = await params;
    const project = await fetchProjectData(projectTitle) ?? notFound();
    return (
        <Layout>
            <ProjectBody project={project} />
        </Layout>
    );
}

