import { ResumeBlurb, resumeBlurb } from "@/app/components/resume/ResumeBlurb";

export function ResumeText(props: { resumeBody: resumeBlurb }) {
    return (
      <div className="group mt-5 mb-5 p-5 border-2 border-transparent rounded-md transition duration-300 ease-in-out hover:border-contrast hover:bg-secondary ">
        <ResumeBlurb {...props.resumeBody} />
      </div>
    );
  }