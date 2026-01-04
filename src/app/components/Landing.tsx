import Image from "next/image";

function LandingImage() {
  return (
    <div className="relative flex flex-col justify-center items-center animate-fade-in">
      <Image
        className="rounded-full object-contain w-48 h-48 sm:w-64 sm:h-64 md:w-80 md:h-80 lg:w-[500px] lg:h-[500px]"
        src="/media/images/lawrence-onyango-thumbnail.jpg"
        alt="Picture of the author"
        width={500}
        height={500}
      />
    </div>
  );
}

function LandingBlurb() {
  return (
    <div className="flex flex-col justify-start w-full max-w-2xl px-4 text-center md:text-left space-y-4">
      <h1 className="text-lg sm:text-xl md:text-2xl font-bold text-[#4FA3E8] animate-fade-in">
        HELLO
      </h1>
      <h2 className="text-2xl sm:text-3xl md:text-4xl lg:text-5xl font-bold animate-gradient-wave bg-gradient-to-l from-textHover via-white to-white bg-[length:200%_100%] bg-clip-text text-transparent">
        I&#39;m Lawrence Onyango
      </h2>
      <p className="text-lg sm:text-xl md:text-2xl lg:text-3xl text-gray-400 animate-fade-in">
        An engineer with a passion for robotics and software, constantly
        exploring creative ways to merge these fields and solve real-world
        problems.
      </p>
      
      {/* Add CTA Buttons */}
      <div className="flex flex-col sm:flex-row gap-4 mt-6 justify-center md:justify-start animate-fade-in">
        <a
          href="/projects"
          className="px-6 py-3 bg-contrast text-white font-semibold rounded-full 
                     hover:bg-contrast/80 transition-all duration-300 hover:scale-105"
        >
          View Projects
        </a>
        <a
          href="/resume"
          className="px-6 py-3 border-2 border-contrast text-white font-semibold rounded-full 
                     hover:bg-contrast/20 transition-all duration-300 hover:scale-105"
        >
          About Me
        </a>
      </div>
    </div>
  );
}

export function Landing() {
  return (
    <div className="relative flex flex-col-reverse md:flex-row justify-center flex-grow items-center gap-8 px-4 py-8 overflow-hidden">
      <LandingBlurb />
      <LandingImage />
    </div>
  );
}