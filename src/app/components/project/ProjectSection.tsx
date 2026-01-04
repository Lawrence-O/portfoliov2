"use client";

import { useState } from "react";
import Image from "next/image";
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { dracula } from 'react-syntax-highlighter/dist/cjs/styles/prism';

import { TextContentBlock, CodeContentBlock, ContentBlock, ImageContentBlock, Section, VideoContentBlock, MathContentBlock } from "@/app/components/project/interfaces";
import { MarkdownText } from "@/app/components/shared/InlineMarkdown";
import { Math } from "@/app/components/shared/Math";

// Language display names
const languageNames: Record<string, string> = {
  javascript: "JavaScript",
  typescript: "TypeScript",
  python: "Python",
  julia: "Julia",
  rust: "Rust",
  go: "Go",
  cpp: "C++",
  c: "C",
  java: "Java",
  bash: "Bash",
  shell: "Shell",
  plaintext: "Text",
  json: "JSON",
  yaml: "YAML",
  html: "HTML",
  css: "CSS",
  sql: "SQL",
  markdown: "Markdown",
};

function CodeBlockHeader({ language, onCopy, copied }: { language?: string; onCopy: () => void; copied: boolean }) {
  const displayLang = language ? (languageNames[language.toLowerCase()] || language) : "Code";
  
  return (
    <div className="flex items-center justify-between bg-gray-800 px-3 py-1.5 sm:px-4 sm:py-2 border-b border-gray-700">
      <span className="text-xs sm:text-sm text-gray-400 font-mono">{displayLang}</span>
      <button
        onClick={onCopy}
        className="flex items-center gap-1.5 text-xs text-gray-400 hover:text-white transition-colors px-2 py-1 rounded hover:bg-gray-700"
        title="Copy code"
      >
        {copied ? (
          <>
            <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
            </svg>
            <span className="hidden sm:inline">Copied!</span>
          </>
        ) : (
          <>
            <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 16H6a2 2 0 01-2-2V6a2 2 0 012-2h8a2 2 0 012 2v2m-6 12h8a2 2 0 002-2v-8a2 2 0 00-2-2h-8a2 2 0 00-2 2v8a2 2 0 002 2z" />
            </svg>
            <span className="hidden sm:inline">Copy</span>
          </>
        ) }
      </button>
    </div>
  );
}

function CodeBlock({ contentBlock }: { contentBlock: CodeContentBlock }) {
  const [copied, setCopied] = useState(false);
  const { content: codeContent, codeLang, subtitle: codeSubtitle } = contentBlock;

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(codeContent);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error("Failed to copy:", err);
    }
  };

  return (
    <div className="rounded-md sm:rounded-lg bg-gray-900 border border-gray-700 overflow-hidden shadow-lg my-3 sm:my-4 -mx-2 sm:mx-0">
      {codeSubtitle ? (
        <div className="flex items-center justify-between bg-gray-800 px-3 py-1.5 sm:px-4 sm:py-2 border-b border-gray-700">
          <span className="text-white text-xs sm:text-sm font-bold">{codeSubtitle}</span>
          <div className="flex items-center gap-3">
            <span className="text-xs text-gray-500 font-mono hidden sm:inline">
              {codeLang ? (languageNames[codeLang.toLowerCase()] || codeLang) : ""}
            </span>
            <button
              onClick={handleCopy}
              className="flex items-center gap-1.5 text-xs text-gray-400 hover:text-white transition-colors px-2 py-1 rounded hover:bg-gray-700"
              title="Copy code"
            >
              {copied ? (
                <>
                  <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                  </svg>
                  <span className="hidden sm:inline">Copied!</span>
                </>
              ) : (
                <>
                  <svg className="w-3.5 h-3.5 sm:w-4 sm:h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 16H6a2 2 0 01-2-2V6a2 2 0 012-2h8a2 2 0 012 2v2m-6 12h8a2 2 0 002-2v-8a2 2 0 00-2-2h-8a2 2 0 00-2 2v8a2 2 0 002 2z" />
                  </svg>
                  <span className="hidden sm:inline">Copy</span>
                </>
              )}
            </button>
          </div>
        </div>
      ) : (
        <CodeBlockHeader language={codeLang} onCopy={handleCopy} copied={copied} />
      )}
      <SyntaxHighlighter
        language={codeLang}
        style={dracula}
        className="!text-xs sm:!text-sm md:!text-base !p-2 sm:!p-3 md:!p-4 !whitespace-pre-wrap break-words overflow-x-auto"
        wrapLines={true}
        showLineNumbers={true}
        lineNumberStyle={{ paddingRight: '0.5rem', opacity: 0.5, color: '#6272a4', minWidth: '2em' }}
      >
        {codeContent}
      </SyntaxHighlighter>
    </div>
  );
}

function ProjectContentCard(props: { contentBlock: ContentBlock }) {
  const { type } = props.contentBlock;
  switch (type) {
    case "text": {
      const { content, displayAs, orderedList } = props.contentBlock as TextContentBlock;
      if (displayAs === "list" && Array.isArray(content)) {
        const ListTag = orderedList ? 'ol' : 'ul';
        return (
          <ListTag className={`text-sm sm:text-base md:text-lg lg:text-xl leading-6 sm:leading-7 md:leading-8 text-gray-300 ${orderedList ? 'list-decimal' : 'list-disc'} list-inside space-y-1.5 sm:space-y-2`}>
            {content.map((item, index) => (
              <MarkdownText key={index} as="li">{item}</MarkdownText>
            ))}
          </ListTag>
        );
      } else if (displayAs === "subtitle") {
        return <h3 className="text-base sm:text-lg font-semibold mt-4 sm:mt-6 mb-2 sm:mb-3 text-center">{content as string}</h3>;
      } else if (Array.isArray(content)) {
        return (
          <>
            {content.map((paragraph, index) => (
              <MarkdownText key={index} as="p" className="text-sm sm:text-base md:text-lg lg:text-xl leading-6 sm:leading-7 md:leading-8 text-gray-300 mb-3 sm:mb-4">{paragraph}</MarkdownText>
            ))}
          </>
        );
      }
      return <MarkdownText as="p" className="text-sm sm:text-base md:text-lg lg:text-xl leading-6 sm:leading-7 md:leading-8 text-gray-300">{content as string}</MarkdownText>;
    }
    case "image": {
      const { content: imageContent, altContent, subtitle: imageSubtitle } = props.contentBlock as ImageContentBlock;
      return (
        <div className="flex justify-center">
          <div className="flex flex-col w-full max-w-lg sm:max-w-xl md:max-w-2xl">
            <Image 
              className="mt-3 sm:mt-5 rounded-lg sm:rounded-xl w-full h-auto" 
              src={imageContent} 
              alt={altContent ?? ""} 
              width={800} 
              height={600}
              sizes="(max-width: 640px) 100vw, (max-width: 768px) 80vw, 672px"
            />
            {imageSubtitle && (
              <p className="mt-2 sm:mt-3 mb-0 text-[10px] sm:text-xs text-center text-gray-400">{imageSubtitle}</p>
            )}
          </div>
        </div>
      );
    }
    case "video": {
      const { content: videoContent, subtitle: videoSubtitle } = props.contentBlock as VideoContentBlock;
      return (
        <div className="flex justify-center">
          <div className="flex flex-col w-full max-w-lg sm:max-w-xl md:max-w-2xl">
            <video 
              className="mt-3 sm:mt-5 rounded-lg sm:rounded-xl w-full" 
              src={videoContent} 
              controls 
              playsInline
              preload="none"
            >
              <track kind="captions" />
            </video>
            {videoSubtitle && (
              <p className="mt-2 sm:mt-3 mb-0 text-[10px] sm:text-xs text-center text-gray-400">{videoSubtitle}</p>
            )}
          </div>
        </div>
      );
    }
    case "code":
      return <CodeBlock contentBlock={props.contentBlock as CodeContentBlock} />;
    case "math": {
      const { content: mathContent, block: mathBlock, subtitle: mathSubtitle } = props.contentBlock as MathContentBlock;
      return (
        <Math block={mathBlock ?? true} subtitle={mathSubtitle}>
          {mathContent}
        </Math>
      );
    }
    default:
      return null;
  }
}
// MAKE TEXT GREY
export function ProjectSection(props: {
  section: Section;
  index: number;
  sectionsRef: (HTMLElement | null)[];
}) {
  return (
    <section
      id={props.section.navRef}
      ref={(el) => {
        props.sectionsRef[props.index] = el;
      }}
      className="w-full h-full p-4 sm:p-6 md:p-8 lg:p-10 space-y-3 sm:space-y-4 transition-all duration-300 hover:bg-white/[0.02]"
    >
      <h2 className="mb-3 sm:mb-4 md:mb-6 text-xl sm:text-2xl md:text-3xl lg:text-4xl font-bold bg-gradient-to-r from-white to-gray-400 bg-clip-text text-transparent">
        {props.section.title}
      </h2>
      {props.section.content.map((content, index) => {
        return <ProjectContentCard key={index} contentBlock={content} />;
      })}
    </section>
  );
}
