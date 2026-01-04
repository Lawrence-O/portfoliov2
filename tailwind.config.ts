import type { Config } from "tailwindcss";

export default {
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    screens: {
      'xs': '480px',
      'sm': '640px',
      'md': '768px',
      'lg': '1024px',
      'xl': '1280px',
      '2xl': '1536px',
    },
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
        'fade-in-70': 'fadeIn70 1s ease-out forwards',
        'slide-in-left': 'slideInLeft 1s ease-out',
        'gradient-wave': 'gradientWave 4s ease-in-out infinite',
        'slide-in-up': 'slideInUp 0.2s ease-out',
      },
      keyframes: {
        fadeIn: {
          '0%': { opacity: "0" },
          '100%': { opacity: "1" },
        },
        fadeIn70: {
          '0%': { opacity: "0" },
          '100%': { opacity: "0.7" },
        },
        slideInLeft: {
          '0%': { transform: 'translateX(-100%)' },
          '100%': { transform: 'translateX(0)' },
        },
        gradientWave: {
          '0%': { backgroundPosition: '100% 50%' },
          '50%': { backgroundPosition: '0% 50%' },
          '100%': { backgroundPosition: '100% 50%' },
        },
        slideInUp: {
          '0%': { opacity: '0', transform: 'translateY(10px)' },
          '100%': { opacity: '1', transform: 'translateY(0)' },
        },
      },
    },
  },
  plugins: [],
} satisfies Config;


// transition duration-300 ease-in-out transform hover:bg-gray-700 hover:text-gray-300 hover:scale-105"