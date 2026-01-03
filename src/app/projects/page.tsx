"use client";
import { useState, useCallback } from 'react';
import { projects as allProjectsData } from "@/app/components/data/projectBlurbData";
import { ProjectCard, ProjectBlurb } from "@/app/components/project/ProjectCard"; // Ensure ProjectBlurb is exported and imported
import { ProjectControls } from "@/app/components/project/ProjectControls";
import { NavBar } from "@/app/components/shared/NavBar";
import { Footer } from "@/app/components/shared/Footer";

export default function Projects() {
    const [displayedProjects, setDisplayedProjects] = useState<ProjectBlurb[]>(allProjectsData);

    const handleFilterChange = useCallback((filteredProjects: ProjectBlurb[]) => {
        setDisplayedProjects(filteredProjects);
    }, []);

    return (
      <div className="flex flex-col flex-grow min-h-screen">
        <NavBar />
        <main className="flex-grow container mx-auto px-4">
          <ProjectControls allProjects={allProjectsData} onFilterChange={handleFilterChange} />
          {displayedProjects.length > 0 ? (
            <ProjectCard projects={displayedProjects} />
          ) : (
            <div className="text-center py-10">
              <p className="text-xl text-textMuted">No projects match your current filters.</p>
            </div>
          )}
        </main>
        <Footer />
      </div>
    );
}
