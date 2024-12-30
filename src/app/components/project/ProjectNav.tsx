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
      className={`text-sm justify-center hover:text-textHover ${props.isActive ? "text-textHover" : ""}`}
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
    <div id="nav-container" className="ml-10 flex flex-row space-x-4 fixed">
      <ProjectNavScroll navItems={props.navItems} currentSelection={props.currentSection} />
      <nav className="flex flex-col space-y-1">
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
