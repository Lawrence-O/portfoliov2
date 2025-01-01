"use client";

import { NavBar } from "@/app/components/shared/NavBar";
import { resumeBlurb } from "@/app/components/resume/ResumeBlurb";
import { skill, ResumeSkills } from "@/app/components/resume/ResumeSkills";
import { GradientDivider } from "@/app/components/shared/GradientDivider";
import { ResumeText } from "@/app/components/resume/ResumeText";
import { ResumeNav } from "@/app/components/resume/ResumeNav";
import {
  currEducation,
  currResumeBody,
  resumeSections,
} from "@/app/components/data/info";

import { currResumeSkills } from "@/app/components/data/resumeSkillsData";
import React, { useEffect, useRef } from "react";
import { getVisibleSection } from "@/app/components/utils/scrollUtils";

interface ResumeContainerProps {
  education: resumeBlurb;
  body: resumeBlurb[];
  skills: skill[];
}

function ResumeContainer(props: ResumeContainerProps) {
  const [currentSection, setCurrentSection] = React.useState("education");
  const sectionsRef = useRef<(HTMLElement | null)[]>([]);
  useEffect(() => {
    const handleActiveSection = () => {
      const section = getVisibleSection(sectionsRef.current);
      if (section) {
        setCurrentSection(section.id);
      }
    };
    window.addEventListener("scroll", handleActiveSection);
    window.addEventListener("resize", handleActiveSection);
    return () => {
      window.removeEventListener("scroll", handleActiveSection);
      window.removeEventListener("resize", handleActiveSection);
    };
  }, []);

  return (
    <div className="flex flex-row w-full h-full align-text-top mt-5">
      <div className="flex w-1/2 justify-end pr-10 relative">
        <ResumeNav
          activeSection={currentSection}
          resumeSections={resumeSections}
        />
      </div>
      <div className="flex flex-col w-[29%] justify-start pl-5 overflow-visible space-y-2">
        <section
          id="education"
          ref={(el) => {
            sectionsRef.current[0] = el;
          }}
        >
          <ResumeText resumeBody={props.education} />
        </section>
        <GradientDivider />
        <section
          id="experience"
          ref={(el) => {
            sectionsRef.current[1] = el;
          }}
        >
          {props.body.map((blurb, index) => (
            <ResumeText key={index} resumeBody={blurb} />
          ))}
        </section>
        <GradientDivider />
        <section
          id="skills"
          ref={(el) => {
            sectionsRef.current[2] = el;
          }}
        >
          <ResumeSkills skills={props.skills} />
        </section>
        <GradientDivider />
      </div>
    </div>
  );
}

export default function Page() {
  return (
    <div className="flex flex-col h-screen">
      <NavBar />
      <ResumeContainer
        education={currEducation}
        body={currResumeBody}
        skills={currResumeSkills}
      />
    </div>
  );
}
