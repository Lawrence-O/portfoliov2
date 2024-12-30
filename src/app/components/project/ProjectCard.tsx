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
}

function ProjectContainer(props: ProjectBlurb) {
  const handleClick = () => {
    window.location.href = props.href ?? `/projects/${createSlug(props.title)}`;
  };

  return (
    <div
      className="flex flex-col bg-background p-4 rounded-3xl items-center cursor-pointer transition-transform duration-300 ease-in-out transform hover:scale-105"
      onClick={handleClick}
    >
      {props.media.endsWith(".mp4") ? (
        <video
          src={props.media}
          className="aspect-video w-full rounded-3xl border-2 border-secondary transition-transform duration-300 ease-in-out transform hover:scale-105" // Border styles
          autoPlay
          loop
          muted
        />
      ) : (
        <Image
          src={props.media}
          className="aspect-video w-full object-cover rounded-3xl border-2 border-secondary transition-transform duration-300 ease-in-out transform hover:scale-105" // Border and object-cover
          alt=""
          width={500}
          height={500}
        />
      )}
      <h1 className="pl-8 pt-5 text-2xl font-bold text-left w-full">
        {props.title}
      </h1>
      <p className="pl-8 whitespace-pre-wrap text-left text-xm w-full">
        {props.description}
      </p>
      <div className="pl-8 mt-5 w-full text-left">
        <Link
          className="inline-block font-bold text-lg transition duration-300 ease-in-out transform hover:text-textHover hover:scale-110"
          href={props.href ?? `/projects/${createSlug(props.title)}`}
          onClick={(e) => e.stopPropagation()}
        >
          Continue Reading
        </Link>
      </div>
    </div>
  );
}

export function ProjectCard(props: { projects: ProjectBlurb[] }) {
  return (
    <div className="flex justify-center items-center w-full mt-5">
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6 w-[50%]">
        {props.projects.map((project, index) => (
          <ProjectContainer key={index} {...project} />
        ))}
      </div>
    </div>
  );
}
