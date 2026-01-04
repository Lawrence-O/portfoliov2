"use client"

import React from "react";

export interface resumeBlurb {
  title: string;
  time: string;
  subtitle?: string;
  location?: string;
  showSubtitle?: boolean;
  description: string[];
  technologies?: string[];
  additionalContent?: React.ReactNode;
}

export function ResumeBlurb(props: resumeBlurb) {
  const [isClicked, setIsClicked] = React.useState(false);

  const handleClick = () => {
    setIsClicked(!isClicked);
  }

  return (
    <div className="flex flex-row items-start space-x-10">
      <div className="flex flex-col flex-shrink-0 w-32"> 
        <h2 className="text-sm italic text-gray-400 transition duration-300 ease-in-out group-hover:text-[#4FA3E8]">
          {props.time}
        </h2>
        {props.showSubtitle && (
          <h3 className="text-sm italic text-gray-500">{props.location}</h3>
        )}
      </div>

      <div className="flex flex-col space-y-1"> 
        <h1 className="text-xl font-bold text-white transition duration-300 ease-in-out group-hover:text-[#4FA3E8]">
          {props.title}
        </h1>
        {props.showSubtitle && (
          <h3 className="text-sm italic text-[#4FA3E8]">{props.subtitle}</h3>
        )}
        <ul className="list-disc pl-5 space-y-1 text-sm">
          {props.description.map((desc, index) => (
            <li key={index}>{desc}</li>
          ))}
        </ul>
        <div>
          {props.technologies && (
            <ul className="flex flex-row flex-wrap mt-2"> {/* Added mt-2 for spacing */}
              {props.technologies.map((tech, index) => (
                <li key={index} className="text-xs bg-secondary p-1 m-1 rounded-md transition duration-300 ease-in-out group-hover:bg-contrast">
                  {tech}
                </li>
              ))}
            </ul>
          )}
        </div>
        {props.additionalContent && (
          <div className="flex flex-row justify-between">
            <div>
              {isClicked && props.additionalContent}
            </div>
            <button onClick={handleClick} className="text-xs italic underline transition duration-300 ease-in-out group-hover:text-textHover">
              {isClicked ? "Hide" : "Show More"}
            </button>
            
          </div>
        )}
      </div>
    </div>
  );
}
