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
      className={`inline-block text-white px-4 py-2 rounded-full text-sm font-medium transition-all duration-300 ease-in-out hover:text-[#4FA3E8] hover:bg-[#4FA3E8]/10 hover:shadow-[0_0_12px_rgba(79,163,232,0.3)] ${
        isActive ? "bg-[#4FA3E8]/20 text-[#4FA3E8] shadow-[0_0_12px_rgba(79,163,232,0.2)]" : ""
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
      <div className="backdrop-blur-md bg-gray-900/70 p-2 rounded-full border border-[#4FA3E8]/50 shadow-lg shadow-[#4FA3E8]/10 space-x-2 pl-5 pr-5">
        <NavItem currentPath={pathname} text="HOME" href="/" />
        <NavItem currentPath={pathname} text="ABOUT" href="/resume" />
        <NavItem currentPath={pathname} text="PROJECTS" href="/projects" />
      </div>
    </div>
  );
}
