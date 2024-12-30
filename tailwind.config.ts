import type { Config } from "tailwindcss";

export default {
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      fontFamily: {
          "main": "var(--font-space-grotesk)",
      },
      colors: {
        background: "var(--background)",
        foreground: "var(--foreground)",
        secondary: "var(--secondary)",
        contrast: "var(--contrast)",
        textHover: "var(--secondary-text)",
      },
      animation: {
        'fade-in': 'fadeIn 1s ease-out',
        'slide-in-left': 'slideInLeft 1s ease-out',
      },
      keyframes: {
        fadeIn: {
          '0%': { opacity: "0" },
          '100%': { opacity: "1" },
        },
        slideInLeft: {
          '0%': { transform: 'translateX(-100%)' },
          '100%': { transform: 'translateX(0)' },
        },
      },
    },

  },
  plugins: [],
} satisfies Config;


// transition duration-300 ease-in-out transform hover:bg-gray-700 hover:text-gray-300 hover:scale-105"