import Image from "next/image";
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { dracula } from 'react-syntax-highlighter/dist/esm/styles/prism';

import { TextContentBlock, CodeContentBlock, ContentBlock, ImageContentBlock, Section, VideoContentBlock } from "@/app/components/project/interfaces";

function ProjectContentCard(props: { contentBlock: ContentBlock }) {
  const { type } = props.contentBlock;
  switch (type) {
    case "text":
      const { content, displayAs, orderedList } = props.contentBlock as TextContentBlock;
      if (displayAs === "list" && Array.isArray(content)) {
        const ListTag = orderedList ? 'ol' : 'ul';
        return (
          <ListTag className={`text-xl leading-8 ${orderedList ? 'list-decimal' : 'list-disc'} list-inside space-y-2`}>
            {content.map((item, index) => (
              <li key={index}>{item}</li>
            ))}
          </ListTag>
        );
      } else if (displayAs === "subtitle") {
        return <h3 className="text-lg font-semibold mt-6 mb-3 text-center">{content}</h3>; // Adjusted size, added text-center and margins
      } else if (Array.isArray(content)) {
        return (
          <>
            {content.map((paragraph, index) => (
              <p key={index} className="text-xl leading-8 mb-4">{paragraph}</p>
            ))}
          </>
        );
      }
      return <p className="text-xl leading-8">{content}</p>; // Fallback for single string content
    case "image":
      const { content: imageContent, altContent, subtitle: imageSubtitle } = props.contentBlock as ImageContentBlock;
      return (
        <div className="flex justify-center">
          <div className="flex flex-col">
            <Image className="mt-5" src={imageContent} alt={altContent ?? ""} width={500} height={500} />
            <p className="mt-3 mb-0 text-xs text-center">{imageSubtitle ?? ""}</p>
          </div>
        </div>
      );
    case "video":
      const { content: videoContent, subtitle: videoSubtitle } = props.contentBlock as VideoContentBlock;
      return (
        <div className="flex justify-center">
          <div className="flex flex-col">
        <video className="mt-5" src={videoContent} controls />
        <p className="mt-3 mb-0 text-xs text-center">{videoSubtitle ?? ""}</p>
          </div>
        </div>
      );
      case "code":
        const { content: codeContent, codeLang, subtitle: codeSubtitle } = props.contentBlock as CodeContentBlock;
        return (
          <div className="rounded-md bg-gray-900 border border-gray-700 overflow-hidden shadow-lg my-4">
            {codeSubtitle && (
              <div className="bg-gray-800 p-2 border-b border-gray-700 text-white font-bold">
                {codeSubtitle}
              </div>
            )}
            <SyntaxHighlighter
              language={codeLang}
              style={dracula}
              className="text-sm md:text-base p-4 !whitespace-pre-wrap break-words overflow-hidden" // Ensure !whitespace-pre-wrap
              wrapLines={true}
              showLineNumbers={true}
              lineNumberStyle={{ paddingRight: '1rem', opacity: 0.5, color: '#6272a4' }}
            >
              {codeContent}
            </SyntaxHighlighter>
          </div>
        );
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
      className="w-full h-full p-10 space-y-4"
    >
      <h2 className="mb-5 text-5xl">{props.section.title}</h2>
      {props.section.content.map((content, index) => {
        return <ProjectContentCard key={index} contentBlock={content} />;
      })}
    </section>
  );
}
