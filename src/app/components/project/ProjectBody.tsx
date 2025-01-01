"use client";

import { Project } from "@/app/components/project/interfaces";
import Image from "next/image";
import { ProjectSection } from "@/app/components/project/ProjectSection";
import { ProjectNav } from "@/app/components/project/ProjectNav";
import { useEffect, useRef, useState } from "react";
import { getVisibleSection } from "@/app/components/utils/scrollUtils";
import { GradientDivider } from "../shared/GradientDivider";

function ProjectTitle(props: {
  title: string;
  subtitle?: string;
  media: string;
}) {
  return (
    <>
      <div>
        <h1 className="text-5xl">{props.title}</h1>
        <h3>{props.subtitle}</h3>
      </div>
      <div className="rounded-3xl overflow-hidden"> {/* Rounded corners and overflow hidden */}
    {props.media.endsWith('.mp4') ? (
      <video
        src={props.media}
        className="aspect-video w-full rounded-3xl border-2 border-secondary" // Border styles
        controls
        autoPlay
        muted
      />
    ) : (
      <Image
        src={props.media}
        className="aspect-video w-full object-cover border-2 border-secondary" // Border and object-cover
        alt=""
        width={500}
        height={500}
      />
    )}
  </div>
    </>
  );
}

export function ProjectBody(props: { project: Project }) {
  const navMap = props.project.section.reduce((acc, section) => {
    acc[section.navName ?? section.navRef] = section.navRef;
    return acc;
  }, {} as Record<string, string>);

  const sectionsRef = useRef<(HTMLElement | null)[]>([]);
  const [activeSection, setActiveSection] = useState(
    props.project.section[0].navRef
  );

  useEffect(() => {
    const handleActiveSection = () => {
      const section = getVisibleSection(sectionsRef.current);
      if (section) {
        setActiveSection(section.id);
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
    <div className="h-full w-full flex justify-center items-start mt-5 space-x-2">
      <div className="w-[40%] space-y-10">
        <ProjectTitle
          title={props.project.title}
          subtitle={props.project.subtitle}
          media={props.project.media}
        />
        <div className="rounded-xl border-2 border-secondary ">
          {props.project.section.map((section, index) => (
            <div key={`divider-${section.navRef}`}>
              <ProjectSection
                section={section}
                index={index}
                sectionsRef={sectionsRef.current}
              />
              <GradientDivider key={`divider-${index}`} color={true} />
            </div>
          ))}
        </div>
      </div>
      <div>
        <ProjectNav navItems={navMap} currentSection={activeSection} />
      </div>
    </div>
  );
}
