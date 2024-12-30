import { Socials } from "@/app/components/shared/Socials";
import { useRouter } from 'next/navigation';

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
        className={`resume-nav-link text-xm transition duration-150 ease-in-out transform hover:scale-110 ${
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
    <div className="flex flex-col space-y-2">
      <h1 className="text-4xl font-bold">Lawrence Onyango</h1>
      <h2 className="text-xl font-bold">
        Software Engineer & Future Roboticist
      </h2>
      <p className="text-xm">
        I design intuitive, software-driven solutions that bring engineering
        principles to life in the digital world.
      </p>
      <div className="flex flex-row justify-between">
        <div></div>
        <button onClick={redirectResume} className="text-xm italic underline transition duration-300 ease-in-out hover:text-textHover">
          View Full Resume
        </button>
      </div>
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
          <ResumeNavItem
            isActive={props.activeSection === section.id}
            key={index}
            {...section}
          />
        ))}
      </div>
      <Socials />
    </div>
  );
}
