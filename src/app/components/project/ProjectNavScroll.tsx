"use client";
import { useEffect, useState } from "react";

export function ProjectNavScroll(props: {
  navItems: Record<string, string>;
  currentSelection: string;
}) {
  const [scrollbarTop, setScrollbarTop] = useState(0);
  const scrollbarHeight = 18;

  const CalculateScrollbarTop = () => {
    const navElement = document.getElementById(`nav-${props.currentSelection}`);
    const container = document.getElementById("nav-container");
    if (navElement && container) {
      const containerBounds = container.getBoundingClientRect();
      const elementBounds = navElement.getBoundingClientRect();
      const elementCenter =
        elementBounds.top + elementBounds.height / 2 - containerBounds.top;
      const scrollRatio =
        elementCenter / (containerBounds.bottom - containerBounds.top);
      return (
        scrollRatio * (containerBounds.bottom - containerBounds.top) -
        scrollbarHeight / 2
      );
    }
    return 0;
  };

  useEffect(() => {
    const handleScroll = () => {
      setScrollbarTop(CalculateScrollbarTop());
    };
    setScrollbarTop(CalculateScrollbarTop());
    window.addEventListener("scroll", handleScroll);
    window.addEventListener("resize", handleScroll);
    return () => {
      window.removeEventListener("scroll", handleScroll);
      window.removeEventListener("resize", handleScroll);
    };
  }, [props.currentSelection, props.navItems]);

  return (
    <div className="absolute left-0 w-1 h-full bg-secondary rounded-3xl overflow-clip">
      <div
        className="bg-contrast w-1 cursor-pointer rounded-3xl transition-transform duration-300 ease-in-out"
        style={{
          height: `${scrollbarHeight}px`,
          transform: `translateY(${scrollbarTop}px)`,
        }}
      ></div>
    </div>
  );
}
