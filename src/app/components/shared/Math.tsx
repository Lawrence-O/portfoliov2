"use client";

import "katex/dist/katex.min.css";
import { InlineMath, BlockMath } from "react-katex";

interface MathProps {
  children: string;
  block?: boolean;
  subtitle?: string;
}

export function Math({ children, block = false, subtitle }: MathProps) {
  const renderMath = () => {
    try {
      if (block) {
        return <BlockMath math={children} />;
      }
      return <InlineMath math={children} />;
    } catch (error) {
      console.error("KaTeX rendering error:", error);
      return (
        <code className="text-red-400 bg-red-900/20 px-2 py-1 rounded text-sm">
          Error rendering: {children.slice(0, 50)}...
        </code>
      );
    }
  };

  if (block) {
    return (
      <div className="my-4 sm:my-6 overflow-x-auto -mx-2 sm:mx-0 px-2 sm:px-0">
        <div className="flex justify-center min-w-0">
          <div className="text-sm sm:text-base md:text-lg [&_.katex]:text-gray-100">
            {renderMath()}
          </div>
        </div>
        {subtitle && (
          <p className="mt-2 sm:mt-3 text-[10px] sm:text-xs text-center text-gray-400">{subtitle}</p>
        )}
      </div>
    );
  }
  
  return (
    <span className="[&_.katex]:text-gray-100 text-sm sm:text-base">
      {renderMath()}
    </span>
  );
}
