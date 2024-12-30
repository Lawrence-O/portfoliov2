import Image from "next/image";
import { CodeContentBlock, ContentBlock, ImageContentBlock, Section } from "@/app/components/project/interfaces";
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { dracula } from 'react-syntax-highlighter/dist/esm/styles/prism';

function ProjectContentCard(props: { contentBlock: ContentBlock }) {
  const { type, content } = props.contentBlock;
  switch (type) {
    case "text":
      return <p className="text-xl leading-8">{content}</p>;
    case "image":
      const { altContent, subtitle: imageSubtitle } = props.contentBlock as ImageContentBlock;
      return (
        <div className="flex justify-center">
          <div className="flex flex-col">
            <Image className="mt-5" src={content} alt={altContent ?? ""} width={500} height={500} />
            <p className="mt-3 mb-0 text-xs text-center">{imageSubtitle ?? "Test Subtitle"}</p>
          </div>
        </div>
      );
    case "video":
      return <video src={content} controls />;
      case "code":
        const { codeLang, subtitle: codeSubtitle } = props.contentBlock as CodeContentBlock;
        return (
          <div className="rounded-md bg-gray-900 border border-gray-700 overflow-hidden shadow-lg">
            {codeSubtitle && (
              <div className="bg-gray-800 p-2 border-b border-gray-700 text-white font-bold">
                {codeSubtitle}
              </div>
            )}
            <SyntaxHighlighter
              language={codeLang}
              style={dracula}
              className="text-sm md:text-base p-4 text-wrap break-words overflow-hidden"
              wrapLines={true}
              showLineNumbers={true}
              lineNumberStyle={{ paddingRight: '1rem', opacity: 0.5, color: '#6272a4' }}
            >
              {content}
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
