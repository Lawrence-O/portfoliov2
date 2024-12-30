import Image from "next/image";

export interface skill {
  skill: string;
  tag: string;
  href?: string;
}


function ResumeSkillCard(props: { skill: skill}  & {index: number} ) {
  return(
    <div key={props.index} className="group/navitem w-[60%] bg-background p-2 rounded-lg text-center mx-auto transform transition-transform duration-300 hover:scale-110 flex flex-col items-center">
          <Image
            className="object-fill mx-auto dark:invert"
            src={props.skill.href ?? ""}
            alt=""
            width={1000}
            height={1000}
          />
          <p className="text-xm font-bold group-hover/navitem:text-textHover mt-2">{props.skill.skill}</p>
        </div>
  );
}

export function ResumeSkills(props: { skills: skill[]}) {
  return (
    <div className="w-full grid grid-cols-2 sm:grid-cols-3 md:grid-cols-4 lg:grid-cols-5 gap-4 mt-5 mb-5 py-5 justify-center space-y-10">
      {Object.entries(
      props.skills.reduce((acc, skill) => {
        if (!acc[skill.tag]) {
        acc[skill.tag] = [];
        }
        acc[skill.tag].push(skill);
        return acc;
      }, {} as Record<string, skill[]>)
      ).map(([tag, skills]) => (
      <div key={tag} className="group/card col-span-full">
        <h3 className="text-lg font-bold mb-2 text-center transition-transform duration-300 group-hover/card:text-textHover group-hover:scale-110">{tag}</h3>
        <div className="grid grid-cols-2 sm:grid-cols-3 md:grid-cols-4 lg:grid-cols-5 gap-4 align-top">
        {skills.map((skill, index) => (
          <ResumeSkillCard key={index} skill={skill} index={index} />
        ))}
        </div>
      </div>
      ))}
    </div>
  );
}
