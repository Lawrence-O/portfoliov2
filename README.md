# Lawrence Onyango - Personal Portfolio

Welcome to the repository for my personal portfolio website! This site showcases my projects, skills, and experience in robotics and software engineering.

**Live Demo:** [Your Portfolio URL Here] (Please replace with your actual deployed URL)

[![Portfolio Screenshot](public/media/images/landing-page-author.jpg)](public/media/images/landing-page-author.jpg)
*(Suggestion: Replace `public/media/images/landing-page-author.jpg` with an actual screenshot of your portfolio's landing page. You can name it `portfolio-screenshot.png` and place it in `public/` or `public/media/images/`)*

## Description

This portfolio is built with Next.js, React, TypeScript, and Tailwind CSS. It features a clean, modern design with a focus on presenting my work in a clear and accessible manner. Key sections include an interactive resume, detailed project pages, and a showcase of my technical skills. The site is fully responsive and includes a dark mode theme.

## Tech Stack

*   **Framework:** [Next.js](https://nextjs.org/) (v13+ with App Router)
*   **Language:** [TypeScript](https://www.typescriptlang.org/)
*   **UI Library:** [React](https://reactjs.org/)
*   **Styling:** [Tailwind CSS](https://tailwindcss.com/)
*   **Linting:** [ESLint](https://eslint.org/)
*   **Formatting:** [Prettier](https://prettier.io/) (integrated with Tailwind CSS)

## Features

*   **Responsive Design:** Adapts to various screen sizes (desktop, tablet, mobile).
*   **Dark Mode:** User-selectable light and dark themes.
*   **Project Showcase:** Detailed pages for individual projects, including descriptions, images, and videos.
*   **Interactive Resume:** A dynamic resume page highlighting education, experience, and skills.
*   **Skills Display:** Visually engaging presentation of technical skills with icons.
*   **Optimized Performance:** Leverages Next.js features for fast loading and rendering.

## Project Structure

The project is organized using the Next.js App Router structure:

```
/home/law/Workspace/repos/portfoliov2
├── .eslintrc.json        # ESLint configuration
├── .gitignore            # Git ignore file
├── next.config.mjs       # Next.js configuration
├── package.json          # Project dependencies and scripts
├── postcss.config.mjs    # PostCSS configuration (for Tailwind CSS)
├── README.md             # This file
├── tailwind.config.ts    # Tailwind CSS configuration
├── tsconfig.json         # TypeScript configuration
├── public/               # Static assets (images, videos, fonts, etc.)
│   ├── media/
│   └── ...
└── src/                  # Source code
    ├── app/              # Next.js App Router directory
    │   ├── components/   # Reusable React components
    │   │   ├── data/     # Data files for projects, resume, etc.
    │   │   ├── project/  # Components specific to project pages
    │   │   ├── resume/   # Components specific to the resume page
    │   │   ├── shared/   # Shared components (NavBar, Footer, etc.)
    │   │   └── utils/    # Utility functions
    │   ├── fonts/        # Font files
    │   ├── projects/     # Project listing and individual project pages
    │   ├── resume/       # Resume page
    │   ├── globals.css   # Global styles
    │   ├── layout.tsx    # Root layout
    │   └── page.tsx      # Landing page
    └── ...
```

## Getting Started

To run this project locally:

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/your-repo-name.git # Replace with your repo URL
    cd your-repo-name
    ```

2.  **Install dependencies:**
    ```bash
    npm install
    # or
    # yarn install
    # or
    # pnpm install
    ```

3.  **Run the development server:**
    ```bash
    npm run dev
    # or
    # yarn dev
    # or
    # pnpm dev
    ```

4.  Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

The page auto-updates as you edit files in the `src/app/` directory.

## Deployment

This project can be easily deployed using [Vercel](https://vercel.com/), the creators of Next.js.

Check out the [Next.js deployment documentation](https://nextjs.org/docs/app/building-your-application/deploying) for more details.

## Contributing

Contributions, issues, and feature requests are welcome! Feel free to check the [issues page](https://github.com/your-username/your-repo-name/issues). (Replace with your repo URL)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details (You will need to create this file).
