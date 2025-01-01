import { NavBar } from "@/app/components/shared/NavBar"
import {projects} from "@/app/components/data/projectBlurbData"
import { ProjectCard } from "@/app/components/project/ProjectCard"




export default function Projects() {
    return (
      <div className="flex flex-col flex-grow h-screen">
        <NavBar />
        <ProjectCard projects={projects} />
      </div>
    );
}
