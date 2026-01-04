"use client";

import Image from "next/image";
import { useEffect, useRef, useState } from "react";

import { Project } from "@/app/components/project/interfaces";
import { ProjectNav } from "@/app/components/project/ProjectNav";
import { ProjectSection } from "@/app/components/project/ProjectSection";
import { getVisibleSection } from "@/app/components/utils/scrollUtils";

import { GradientDivider } from "../shared/GradientDivider";

function ProjectTitle(props: {
  title: string;
  subtitle?: string;
  media: string;
  date?: string;
  tags?: string[];
  githubLink?: string;
}) {
  return (
    <>
      <div className="space-y-2 sm:space-y-3">
        <h1 className="text-2xl sm:text-3xl md:text-4xl lg:text-5xl font-bold bg-gradient-to-r from-white to-gray-400 bg-clip-text text-transparent leading-tight">
          {props.title}
        </h1>
        {props.subtitle && (
          <h3 className="text-sm sm:text-base md:text-lg text-gray-400">{props.subtitle}</h3>
        )}
        <div className="flex flex-wrap items-center gap-2 sm:gap-3">
          {props.date && (
            <span className="text-xs sm:text-sm text-gray-400 flex items-center gap-1">
              <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 7V3m8 4V3m-9 8h10M5 21h14a2 2 0 002-2V7a2 2 0 00-2-2H5a2 2 0 00-2 2v12a2 2 0 002 2z" />
              </svg>
              {props.date}
            </span>
          )}
          {props.githubLink && (
            <a
              href={props.githubLink}
              target="_blank"
              rel="noopener noreferrer"
              className="text-xs sm:text-sm text-gray-400 hover:text-[#4FA3E8] transition-colors flex items-center gap-1"
            >
              <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4" fill="currentColor" viewBox="0 0 24 24">
                <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
              </svg>
              <span className="hidden xs:inline">View Code</span>
              <span className="xs:hidden">Code</span>
            </a>
          )}
        </div>
        {props.tags && props.tags.length > 0 && (
          <div className="flex flex-wrap gap-1.5 sm:gap-2 pt-1 sm:pt-2">
            {props.tags.map((tag, index) => (
              <span
                key={index}
                className="px-2 sm:px-3 py-0.5 sm:py-1 text-[10px] sm:text-xs font-medium rounded-full bg-[#4FA3E8]/10 text-[#4FA3E8] border border-[#4FA3E8]/20"
              >
                {tag}
              </span>
            ))}
          </div>
        )}
      </div>
      <div className="rounded-xl sm:rounded-2xl md:rounded-3xl overflow-hidden">
        {props.media.endsWith('.mp4') ? (
          <video
            src={props.media}
            className="aspect-video w-full rounded-xl sm:rounded-2xl md:rounded-3xl border border-secondary sm:border-2"
            controls
            autoPlay
            muted
            playsInline
          />
        ) : (
          <Image
            src={props.media}
            className="aspect-video w-full object-cover border border-secondary sm:border-2"
            alt=""
            width={1200}
            height={675}
            sizes="(max-width: 640px) 100vw, (max-width: 1024px) 90vw, 896px"
            priority
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

  const [mobileNavOpen, setMobileNavOpen] = useState(false);

  const handleMobileNavClick = (navRef: string) => {
    const element = document.getElementById(navRef);
    if (element) {
      element.scrollIntoView({ behavior: "smooth" });
    }
    setMobileNavOpen(false);
  };

  return (
    <div className="h-full w-full flex justify-center items-start mt-3 sm:mt-5 px-3 sm:px-4 lg:px-8">
      <div className="w-full max-w-4xl space-y-6 sm:space-y-8 md:space-y-10">
        <ProjectTitle
          title={props.project.title}
          subtitle={props.project.subtitle}
          media={props.project.media}
          date={props.project.date}
          tags={props.project.tags}
          githubLink={props.project.githubLink}
        />
        <div className="rounded-lg sm:rounded-xl border border-secondary/50 hover:border-[#4FA3E8]/30 transition-colors duration-300">
          {props.project.section.map((section, index) => (
            <div key={`divider-${section.navRef}`}>
              <ProjectSection
                section={section}
                index={index}
                sectionsRef={sectionsRef.current}
              />
              {index < props.project.section.length - 1 && (
                <GradientDivider key={`divider-${index}`} color={true} />
              )}
            </div>
          ))}
        </div>
      </div>

      {/* Desktop Navigation */}
      <div className="hidden xl:block">
        <ProjectNav navItems={navMap} currentSection={activeSection} />
      </div>

      {/* Mobile/Tablet Floating Navigation Button */}
      <div className="xl:hidden fixed bottom-4 right-4 sm:bottom-6 sm:right-6 z-50">
        <button
          onClick={() => setMobileNavOpen(!mobileNavOpen)}
          className="w-12 h-12 sm:w-14 sm:h-14 rounded-full bg-[#4FA3E8] text-white shadow-lg shadow-[#4FA3E8]/30 flex items-center justify-center hover:bg-[#3d8ed4] transition-all duration-300 hover:scale-105 active:scale-95"
          aria-label="Toggle navigation menu"
        >
          {mobileNavOpen ? (
            <svg className="w-5 h-5 sm:w-6 sm:h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          ) : (
            <svg className="w-5 h-5 sm:w-6 sm:h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
            </svg>
          )}
        </button>

        {/* Mobile Navigation Dropdown */}
        {mobileNavOpen && (
          <div className="absolute bottom-16 sm:bottom-20 right-0 w-48 sm:w-56 bg-gray-900/95 backdrop-blur-sm border border-gray-700 rounded-xl shadow-xl overflow-hidden animate-slide-in-up">
            <nav className="py-2">
              {Object.entries(navMap).map(([navName, navRef], index) => (
                <button
                  key={index}
                  onClick={() => handleMobileNavClick(navRef)}
                  className={`w-full text-left px-4 py-2.5 sm:py-3 text-sm sm:text-base transition-colors ${
                    activeSection === navRef
                      ? "text-[#4FA3E8] bg-[#4FA3E8]/10 font-medium"
                      : "text-gray-300 hover:text-white hover:bg-white/5"
                  }`}
                >
                  {navName}
                </button>
              ))}
            </nav>
          </div>
        )}
      </div>
    </div>
  );
}
