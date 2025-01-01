import Image from "next/image";

function LandingImage() {
  return (
    <div className="flex flex-col justify-center items-center">
      <Image
        className="rounded-full object-contain"
        src="/media/images/landing-page-author.jpg"
        alt="Picture of the author"
        width={500}
        height={500}
      />
    </div>
  );
}

function LandingBlurb() {
  return (
    <div className="flex flex-col justify-start w-[40rem]">
      <h1 className="text-2xl font-bold">HELLO</h1>
      <h2 className="text-5xl font-bold">I&#39;m Lawrence Onyango</h2>
      <p className="text-4xl">
        An engineer with a passion for robotics and software, constantly
        exploring creative ways to merge these fields and solve real-world
        problems.
      </p>
    </div>
  );
}

export function Landing() {
    return (
      <div className="flex flex-row justify-center flex-grow items-center">
        <LandingBlurb/>
        <LandingImage />
      </div>
    );
  }
