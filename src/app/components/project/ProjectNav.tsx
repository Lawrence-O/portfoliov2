import { ProjectNavScroll } from "@/app/components/project/ProjectNavScroll";

function ProjectNavitem(props: {
  navName: string;
  navRef: string;
  isActive: boolean;
}) {
  const handleClick = (e: React.MouseEvent<HTMLAnchorElement, MouseEvent>) => {
    e.preventDefault();
    const element = document.getElementById(props.navRef);
    if (element) {
      element.scrollIntoView({ behavior: "smooth" });
    }
  };

  return (
    <a
      key={props.navName}
      href={`#${props.navRef}`}
      id={`nav-${props.navRef}`}
      className={`text-sm transition-all duration-300 hover:text-[#4FA3E8] ${
        props.isActive 
          ? "text-[#4FA3E8] font-medium" 
          : "text-gray-400"
      }`}
      onClick={handleClick}
    >
      {props.navName}
    </a>
  );
}

export function ProjectNav(props: {
  navItems: Record<string, string>;
  currentSection: string;
}) {
  return (
    <div id="nav-container" className="ml-6 2xl:ml-10 flex flex-row space-x-3 2xl:space-x-4 fixed top-1/4 max-w-[200px] 2xl:max-w-none">
      <ProjectNavScroll navItems={props.navItems} currentSelection={props.currentSection} />
      <nav className="flex flex-col space-y-1.5 2xl:space-y-2">
        {Object.entries(props.navItems).map(([navName, navRef], index) => {
          return (
            <ProjectNavitem
              key={index}
              navName={navName}
              navRef={navRef}
              isActive={props.currentSection === navRef}
            />
          );
        })}
      </nav>
    </div>
  );
}
