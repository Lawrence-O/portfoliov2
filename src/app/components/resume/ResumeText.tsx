import { ResumeBlurb, resumeBlurb } from "@/app/components/resume/ResumeBlurb";

// Timeline wrapper - remove this component and use ResumeBlurb directly to disable timeline
export function ResumeText(props: { resumeBody: resumeBlurb; showTimeline?: boolean }) {
    return (
      <div className="relative group">
        {/* Timeline line - set showTimeline={false} or remove this div to disable */}
        {props.showTimeline !== false && (
          <div className="absolute left-0 top-0 bottom-0 w-px bg-gradient-to-b from-transparent via-[#4FA3E8]/30 to-transparent" />
        )}
        <div className="ml-4 mt-5 mb-5 p-5 border-2 border-transparent rounded-xl transition-all duration-300 ease-in-out hover:border-[#4FA3E8]/50 hover:bg-secondary/50 hover:shadow-lg hover:shadow-[#4FA3E8]/10 animate-fade-in">
          {/* Timeline dot */}
          {props.showTimeline !== false && (
            <div className="absolute -left-[5px] top-10 w-[11px] h-[11px] rounded-full bg-background border-2 border-[#4FA3E8]/50 group-hover:border-[#4FA3E8] group-hover:bg-[#4FA3E8]/20 transition-all duration-300" />
          )}
          <ResumeBlurb {...props.resumeBody} />
        </div>
      </div>
    );
  }