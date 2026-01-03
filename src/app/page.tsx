import { Landing } from "@/app/components/Landing"
import { Footer } from "@/app/components/shared/Footer"
import { NavBar } from "@/app/components/shared/NavBar"



export default function Home() {
  return (
    <div className="flex flex-col flex-grow h-screen">
      <NavBar />
      <Landing />
      <Footer />
    </div>
  );
}
