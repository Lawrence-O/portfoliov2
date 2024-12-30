export interface Project {
    title: string;
    subtitle?: string;
    media: string;
    section: Section[];
    tags?: string[];
}

export interface TextContentBlock {
    type: "text";
    content: string; // text content
}

export interface ImageContentBlock {
    type: "image";
    content: string; // url for image
    altContent?: string; // alt text for image
    subtitle?: string; // subtitle for image
}

export interface VideoContentBlock {
    type: "video";
    content: string; // url for video
    altContent?: string; // alt text for video
    subtitle?: string; // subtitle for video
}

export interface CodeContentBlock {
    type: "code";
    content: string; // code content
    codeLang?: string; // language for code
    subtitle?: string; // subtitle for code
}

export type ContentBlock = TextContentBlock | ImageContentBlock | VideoContentBlock | CodeContentBlock;
  
export interface Section {
    title: string;
    content: ContentBlock[];
    navName?: string;
    navRef: string;
}