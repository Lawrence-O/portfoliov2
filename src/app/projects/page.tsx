import { NavBar } from "@/app/components/shared/NavBar"
import { Footer } from "@/app/components/shared/Footer"
import {testProjects} from "@/app/components/data/projectBlurbData"
import { ProjectCard } from "@/app/components/project/ProjectCard"




export default function Projects() {
    return (
      <div className="flex flex-col flex-grow h-screen">
        <NavBar />
        <ProjectCard projects={testProjects} />
        {/* <Footer /> */}
      </div>
    );
}
