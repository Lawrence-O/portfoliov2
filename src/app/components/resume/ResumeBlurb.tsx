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
        <h2 className="text-sm italic font-bold transition duration-300 ease-in-out group-hover:text-textHover">
          {props.time}
        </h2>
        {props.showSubtitle && (
          <h3 className="text-sm italic">{props.location}</h3>
        )}
      </div>

      <div className="flex flex-col space-y-1"> 
        <h1 className="text-xl font-bold transition duration-300 ease-in-out group-hover:text-textHover">
          {props.title}
        </h1>
        {props.showSubtitle && (
          <h3 className="text-sm italic">{props.subtitle}</h3>
        )}
        {props.description.map((desc, index) => (
          <p key={index} className="text-sm">{desc}</p>
        ))}
        <div>
          {props.technologies && (
            <ul className="flex flex-row flex-wrap">
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

