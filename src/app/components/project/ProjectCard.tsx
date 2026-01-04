"use client";
import Image from "next/image";
import Link from "next/link";

import { createSlug } from "@/app/components/utils/scrollUtils";

export interface ProjectBlurb {
  title: string;
  description: string;
  media: string;
  href?: string;
  tags: string[];
  status?: string;
  date?: string; // YYYY-MM-DD
}

function ProjectContainer(props: ProjectBlurb & { index?: number }) {
  const handleClick = () => {
    window.location.href = props.href ?? `/projects/${createSlug(props.title)}`;
  };

  return (
    <div
      role="button"
      tabIndex={0}
      className="flex flex-col h-full bg-background p-4 rounded-3xl cursor-pointer transition-all duration-300 ease-in-out transform hover:scale-[1.02] border border-transparent hover:border-[#4FA3E8]/40 hover:shadow-lg hover:shadow-[#4FA3E8]/15 animate-fade-in"
      style={{ animationDelay: `${(props.index || 0) * 100}ms`, animationFillMode: 'backwards' }}
      onClick={handleClick}
      onKeyDown={(e) => { if (e.key === 'Enter' || e.key === ' ') handleClick(); }}
    >
      {props.media.endsWith(".mp4") || props.media.endsWith(".gif") ? (
        props.media.endsWith(".gif") ? (
          <Image
            src={props.media}
            className="aspect-video w-full object-cover rounded-3xl border-2 border-secondary"
            alt=""
            width={500}
            height={500}
            unoptimized
          />
        ) : (
          <video
            src={props.media}
            className="aspect-video w-full rounded-3xl border-2 border-secondary object-cover"
            autoPlay
            loop
            muted
            playsInline
            preload="metadata"
          />
        )
      ) : (
        <Image
          src={props.media}
          className="aspect-video w-full object-cover rounded-3xl border-2 border-secondary"
          alt=""
          width={500}
          height={500}
        />
      )}
      <div className="flex flex-col flex-grow">
        <h1 className="pl-4 pt-5 text-2xl font-bold text-left w-full">
          {props.title}
        </h1>
        <p className="pl-4 whitespace-pre-wrap text-left text-sm w-full line-clamp-3">
          {props.description}
        </p>
        <div className="pl-4 mt-2 w-full flex flex-wrap gap-2">
          {props.tags.map((tag, index) => (
            <span key={index} className="bg-secondary text-xs font-semibold px-2.5 py-0.5 rounded-full text-text">
              {tag}
            </span>
          ))}
        </div>
        {props.status === "Under Construction" && (
          <div className="pl-4 mt-3 w-full text-left">
            <span className="bg-yellow-500 text-black text-xs font-bold px-2.5 py-0.5 rounded-full">
              Under Construction
            </span>
          </div>
        )}
        <div className="pl-4 mt-auto pt-5 w-full text-left">
          <Link
            className="inline-block font-bold text-lg transition-all duration-300 ease-in-out transform hover:text-[#4FA3E8] hover:translate-x-1"
            href={props.href ?? `/projects/${createSlug(props.title)}`}
            onClick={handleClick}
          >
            Continue Reading →
          </Link>
        </div>
      </div>
    </div>
  );
}

export function ProjectCard(props: { projects: ProjectBlurb[] }) {
  return (
    <div className="flex justify-center items-start w-full mt-5 px-4">
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 w-full max-w-7xl">
        {props.projects.map((project, index) => (
          <ProjectContainer key={index} index={index} {...project} />
        ))}
      </div>
    </div>
  );
}