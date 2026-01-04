import { useRouter } from 'next/navigation';

import { Socials } from "@/app/components/shared/Socials";

export interface resumeNavProps {
  id: string;
  label: string;
}

function ResumeNavItem(props: resumeNavProps & { isActive?: boolean }) {
  return (
    <div className="flex flex-row items-center space-x-4 group">
      <span
        className={`inline-block resume-nav ${props.isActive ? "clicked" : ""}`}
      ></span>
      <a
        key={props.id}
        href={`#${props.id}`}
        className={`resume-nav-link text-sm transition duration-150 ease-in-out transform hover:scale-110 ${
          props.isActive ? "clicked" : ""
        }`}
      >
        {props.label}
      </a>
    </div>
  );
}

function ResumeNavBlurb() {
  const router = useRouter();
    const redirectResume = () => {
        router.push("/media/files/Lawrence_Onyango_Resume.pdf");
    }
  return (
    <div className="flex flex-col space-y-3 animate-fade-in">
      <h1 className="text-4xl font-bold animate-gradient-wave bg-gradient-to-l from-[#4FA3E8] via-white to-white bg-[length:200%_100%] bg-clip-text text-transparent">Lawrence Onyango</h1>
      <h2 className="text-xl font-bold text-gray-200">
        Software Engineer & Future Roboticist
      </h2>
      <p className="text-sm text-gray-400">
        I design intuitive, software-driven solutions that bring engineering
        principles to life in the digital world.
      </p>
      <button 
        onClick={redirectResume} 
        className="self-end mt-2 px-4 py-2 text-sm text-gray-300 border border-gray-600 rounded-full transition-all duration-300 ease-in-out hover:border-[#4FA3E8] hover:text-[#4FA3E8] hover:shadow-md hover:shadow-[#4FA3E8]/20"
      >
        View Full Resume →
      </button>
    </div>
  );
}

export function ResumeNav(props: {
  resumeSections: resumeNavProps[];
  activeSection: string;
}) {
  return (
    <div className="flex flex-col w-[20%] mt-10 fixed h-[80%] justify-between">
      <div className="flex flex-col space-y-6">
        <ResumeNavBlurb />
        {props.resumeSections.map((section, index) => (
          <div key={index} className="animate-fade-in" style={{ animationDelay: `${(index + 1) * 150}ms`, animationFillMode: 'backwards' }}>
            <ResumeNavItem
              isActive={props.activeSection === section.id}
              {...section}
            />
          </div>
        ))}
      </div>
      <div className="animate-fade-in" style={{ animationDelay: '500ms', animationFillMode: 'backwards' }}>
        <Socials />
      </div>
    </div>
  );
}
