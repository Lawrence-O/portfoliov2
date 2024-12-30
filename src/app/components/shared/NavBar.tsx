"use client";

import Link from "next/link";
import React from "react";
import { usePathname } from "next/navigation";

interface navItemProps {
  text: string;
  href: string;
  currentPath: string;
}

export function NavItem(props: navItemProps) {
  const isActive = props.href === props.currentPath;
  return (
    <Link
      href={props.href}
      className={`inline-block text-white px-3 py-2 rounded-md text-sm font-medium transition duration-300 ease-in-out transform hover:text-textHover hover:bg-secondary hover:scale-110 ${
        isActive ? "bg-secondary" : ""
      }`}
    >
      {props.text}
    </Link>
  );
}

export function NavBar() {
  const pathname = usePathname();

  return (
    <div className="flex justify-center w-full p-4 mt-5 mb-5 rounded-lg">
      <div className="bg-gray-900 p-2 rounded-full border-2 border-contrast space-x-4 pl-5 pr-5">
        <NavItem currentPath={pathname} text="HOME" href="/" />
        <NavItem currentPath={pathname} text="ABOUT" href="/resume" />
        <NavItem currentPath={pathname} text="PROJECTS" href="/projects" />
      </div>
    </div>
  );
}
