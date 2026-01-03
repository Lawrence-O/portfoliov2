import Image from "next/image";

export interface skill {
  skill: string;
  tag: string;
  href?: string;
}


function ResumeSkillCard(props: { skill: skill } & { index: number }) {
  return (
    <div
      key={props.index}
      className="group/navitem w-full bg-background p-2 rounded-lg text-center transform transition-transform duration-300 hover:scale-110 flex flex-col items-center"
    >
      {/* Container for the image to control its size and ensure alignment */}
      <div className="h-10 w-10 sm:h-12 sm:w-12 mb-2 flex items-center justify-center">
        <Image
          className="object-contain dark:invert" // Changed to object-contain
          src={props.skill.href ?? ""}
          alt={`${props.skill.skill} icon`} // Added descriptive alt text
          width={48} // Adjusted for sm:h-12 sm:w-12 (12 * 4px = 48px)
          height={48} // Adjusted for sm:h-12 sm:w-12
        />
      </div>
      <p className="text-xs font-bold group-hover/navitem:text-textHover mt-auto">
        {props.skill.skill}
      </p>
    </div>
  );
}

export function ResumeSkills(props: { skills: skill[] }) {
  return (
    <div className="w-full mt-5 mb-5 py-5">
      {Object.entries(
        props.skills.reduce((acc, skill) => {
          if (!acc[skill.tag]) {
            acc[skill.tag] = [];
          }
          acc[skill.tag].push(skill);
          return acc;
        }, {} as Record<string, skill[]>)
      ).map(([tag, skills]) => (
        <div key={tag} className="group/card mb-8"> {/* Added mb-8 for spacing between tag groups */}
          <h3 className="text-lg font-bold mb-4 text-center transition-transform duration-300 group-hover/card:text-textHover group-hover/card:scale-110">
            {tag}
          </h3>
          {/* This grid defines columns for skills under the current tag */}
          <div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-4 lg:grid-cols-5 gap-4">
            {skills.map((skill, index) => (
              <ResumeSkillCard key={index} skill={skill} index={index} />
            ))}
          </div>
        </div>
      ))}
    </div>
  );
}
