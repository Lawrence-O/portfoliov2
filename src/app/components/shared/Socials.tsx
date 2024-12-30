import Image from "next/image";
import { useRouter } from 'next/navigation';

interface SocialCardProps {
    href: string;
    redirect: string;
}

function SocialCard(props: SocialCardProps) {
    const router = useRouter();
    const redirectPage = () => {
        router.push(props.redirect);
    }
    return (
        <div className="cursor-pointer transform transition-transform duration-300 hover:scale-110" onClick={redirectPage}>
            <Image
            className="object-fill mx-auto dark:invert"
            src={props.href}
            alt=""
            width={50}
            height={50}
            />
        </div>
    );
}

export function Socials() {
  return (
    <div className="flex flex-row align-top w-full justify-start space-x-8">
        <SocialCard href={"/media/images/linkedin.svg"} redirect="https://www.linkedin.com/in/lawrence-onyango/" />
        <SocialCard href={"/media/images/github.svg"} redirect="https://github.com/Lawrence-O" />
        <SocialCard href={"/media/images/mail.svg"} redirect="mailto:lawrence.onyango@outlook.com" />
    </div>
  );
}
