import Image from "next/image";
import { useRouter } from 'next/navigation';

/**
 * Props for the SocialCard component.
 */
interface SocialCardProps {
  /** The path to the social icon image. */
  iconSrc: string;
  /** The URL to redirect to when the card is clicked. */
  redirectUrl: string;
  /** Alt text for the social icon image. */
  altText: string;
}

/**
 * A card component that displays a social media icon and redirects to a given URL on click.
 * @param props - The properties for the SocialCard.
 * @returns A div element representing the social card, which acts as a clickable link.
 */
function SocialCard(props: SocialCardProps) {
  const router = useRouter();
  const handleRedirect = () => {
    // For external links, window.open is more appropriate than router.push
    if (props.redirectUrl.startsWith("http") || props.redirectUrl.startsWith("mailto:")) {
      window.open(props.redirectUrl, "_blank", "noopener noreferrer");
    } else {
      router.push(props.redirectUrl);
    }
  };

  return (
    // Consider replacing div with <a> for better accessibility if appropriate,
    // or add role="link" and keyboard event handlers (onKeyDown).
    <div
      className="cursor-pointer transform transition-all duration-300 hover:scale-110 hover:drop-shadow-[0_0_8px_rgba(79,163,232,0.5)]"
      onClick={handleRedirect}
      onKeyDown={(e) => { if (e.key === 'Enter' || e.key === ' ') handleRedirect(); }}
      role="link"
      tabIndex={0}
    >
      <Image
        className="object-fill mx-auto dark:invert"
        src={props.iconSrc}
        alt={props.altText}
        width={50}
        height={50}
      />
    </div>
  );
}

/**
 * A component that displays a row of social media icons/links.
 * @returns A div element containing multiple SocialCard components.
 */
export function Socials() {
  return (
    <div className="flex flex-row align-top w-full justify-start space-x-8">
      <SocialCard iconSrc={"/media/images/linkedin.svg"} redirectUrl="https://www.linkedin.com/in/lawrence-onyango/" altText="LinkedIn Profile" />
      <SocialCard iconSrc={"/media/images/github.svg"} redirectUrl="https://github.com/Lawrence-O" altText="GitHub Profile" />
      <SocialCard iconSrc={"/media/images/mail.svg"} redirectUrl="mailto:lawrence.onyango@outlook.com" altText="Send Email" />
    </div>
  );
}
