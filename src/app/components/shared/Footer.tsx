/**
 * The footer component for the portfolio.
 * Displays copyright information and a note about the website being under construction.
 * @returns A div element representing the footer.
 */
export function Footer() {
    return (
      <div className="bottom-0 flex justify-center w-full p-4">
        <p className="text-xs text-gray-500 dark:text-gray-400">
          | Website Is Still Under Construction; There May Be Missing Content | © 2024 Lawrence Onyango |
        </p>
      </div>
    );
  }
