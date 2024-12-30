import type { Metadata } from "next";
import localFont from "next/font/local";
import "./globals.css";

const geistSpace = localFont({
  src: "./fonts/SpaceGrotesk.ttf",
  variable: "--font-space-grotesk",
  weight: "300 700",
});

export const metadata: Metadata = {
  title: "Lawrence Onyango - Software Engineer & Future Roboticist",
  description: "Generated by create next app",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en"> 
      <body
        className={`${geistSpace.variable} antialiased`}
      >
        {children}
      </body>
    </html>
  );
}
