"use client";

import React from "react";

interface InlineMarkdownProps {
  children: string;
  className?: string;
}

/**
 * Lightweight inline markdown parser
 * Supports: **bold**, *italic*, `code`, [links](url)
 */
export function InlineMarkdown({ children, className }: InlineMarkdownProps) {
  const parseInline = (text: string): React.ReactNode[] => {
    // Tokenize the text with markers for different inline styles
    let result = text;
    
    // Bold: **text**
    result = result.replace(/\*\*(.+?)\*\*/g, '{{BOLD:$1}}');
    // Italic: *text* (but not inside bold markers)
    result = result.replace(/(?<!\{)\*([^*]+?)\*(?!\})/g, '{{ITALIC:$1}}');
    // Code: `text`
    result = result.replace(/`([^`]+?)`/g, '{{CODE:$1}}');
    // Links: [text](url)
    result = result.replace(/\[([^\]]+?)\]\(([^)]+?)\)/g, '{{LINK:$1||$2}}');

    // Split by tokens and render
    const tokens = result.split(/(\{\{(?:BOLD|ITALIC|CODE|LINK):.+?\}\})/g);
    
    return tokens.map((token, i) => {
      if (token.startsWith('{{BOLD:')) {
        const content = token.slice(7, -2);
        return (
          <strong key={i} className="text-gray-100 font-semibold">
            {content}
          </strong>
        );
      }
      if (token.startsWith('{{ITALIC:')) {
        const content = token.slice(9, -2);
        return (
          <em key={i} className="italic text-gray-200">
            {content}
          </em>
        );
      }
      if (token.startsWith('{{CODE:')) {
        const content = token.slice(7, -2);
        return (
          <code
            key={i}
            className="text-[#4FA3E8] bg-[#4FA3E8]/10 px-1.5 py-0.5 rounded text-[0.9em] font-mono"
          >
            {content}
          </code>
        );
      }
      if (token.startsWith('{{LINK:')) {
        const content = token.slice(7, -2);
        const [linkText, url] = content.split('||');
        return (
          <a
            key={i}
            href={url}
            target="_blank"
            rel="noopener noreferrer"
            className="text-[#4FA3E8] hover:text-[#6bb5f0] underline underline-offset-2 decoration-[#4FA3E8]/50 hover:decoration-[#4FA3E8] transition-colors"
          >
            {linkText}
          </a>
        );
      }
      return token;
    });
  };

  return <span className={className}>{parseInline(children)}</span>;
}

/**
 * Render text that may contain inline markdown
 * Use this as a wrapper for paragraph content
 */
export function MarkdownText({
  children,
  as: Component = "p",
  className,
}: {
  children: string;
  as?: "p" | "span" | "li" | "div";
  className?: string;
}) {
  const parseInline = (text: string): React.ReactNode[] => {
    let result = text;
    
    // Bold: **text**
    result = result.replace(/\*\*(.+?)\*\*/g, '{{BOLD:$1}}');
    // Italic: *text* (but not inside bold markers)
    result = result.replace(/(?<!\{)\*([^*]+?)\*(?!\})/g, '{{ITALIC:$1}}');
    // Code: `text`
    result = result.replace(/`([^`]+?)`/g, '{{CODE:$1}}');
    // Links: [text](url)
    result = result.replace(/\[([^\]]+?)\]\(([^)]+?)\)/g, '{{LINK:$1||$2}}');

    const tokens = result.split(/(\{\{(?:BOLD|ITALIC|CODE|LINK):.+?\}\})/g);
    
    return tokens.map((token, i) => {
      if (token.startsWith('{{BOLD:')) {
        const content = token.slice(7, -2);
        return (
          <strong key={i} className="text-gray-100 font-semibold">
            {content}
          </strong>
        );
      }
      if (token.startsWith('{{ITALIC:')) {
        const content = token.slice(9, -2);
        return (
          <em key={i} className="italic text-gray-200">
            {content}
          </em>
        );
      }
      if (token.startsWith('{{CODE:')) {
        const content = token.slice(7, -2);
        return (
          <code
            key={i}
            className="text-[#4FA3E8] bg-[#4FA3E8]/10 px-1.5 py-0.5 rounded text-[0.9em] font-mono"
          >
            {content}
          </code>
        );
      }
      if (token.startsWith('{{LINK:')) {
        const content = token.slice(7, -2);
        const [linkText, url] = content.split('||');
        return (
          <a
            key={i}
            href={url}
            target="_blank"
            rel="noopener noreferrer"
            className="text-[#4FA3E8] hover:text-[#6bb5f0] underline underline-offset-2 decoration-[#4FA3E8]/50 hover:decoration-[#4FA3E8] transition-colors"
          >
            {linkText}
          </a>
        );
      }
      return token;
    });
  };

  return <Component className={className}>{parseInline(children)}</Component>;
}
