export interface Project {
    title: string;
    subtitle?: string;
    date?: string; 
    media: string;
    githubLink?: string; 
    section: Section[];
    tags?: string[];
}

export interface TextContentBlock {
    type: "text";
    content: string | string[]; 
    displayAs?: "text" | "list" | "subtitle"; 
    orderedList?: boolean; 
}

export interface ImageContentBlock {
    type: "image";
    content: string;
    altContent?: string;
    subtitle?: string; 
}

export interface VideoContentBlock {
    type: "video";
    content: string; 
    altContent?: string; 
    subtitle?: string; 
}

export interface CodeContentBlock {
    type: "code";
    content: string; 
    codeLang?: string; 
    subtitle?: string; 
}

export interface MathContentBlock {
    type: "math";
    content: string;
    block?: boolean; // true for block display, false for inline
    subtitle?: string;
}

export type ContentBlock = TextContentBlock | ImageContentBlock | VideoContentBlock | CodeContentBlock | MathContentBlock;
  
export interface Section {
    title: string;
    content: ContentBlock[];
    navName?: string;
    navRef: string;
}
