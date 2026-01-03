"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import React from "react";

/**
 * Props for the NavItem component.
 */
interface NavItemProps {
  /** The text to display for the navigation item. */
  text: string;
  /** The URL the navigation item links to. */
  href: string;
  /** The current active path of the application. */
  currentPath: string;
}

/**
 * Represents a single item in the navigation bar.
 * Highlights itself if its href matches the currentPath.
 * @param props - The properties for the NavItem.
 * @returns A Next.js Link component styled as a navigation item.
 */
export function NavItem(props: NavItemProps) {
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

/**
 * The main navigation bar component for the portfolio.
 * It displays navigation links to different sections of the site like Home, About (Resume), and Projects.
 * @returns A div element containing the navigation structure.
 */
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
