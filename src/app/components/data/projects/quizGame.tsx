import { Project } from "@/app/components/project/interfaces";

export const ultimateQuiz: Project = {
  title: "The Ultimate Quiz",
  date: "Spring 2021",
  media: "/media/videos/quizGame.mp4",
  githubLink: "https://github.com/your-username/ultimate-quiz-game",
  tags: ["Web Scraping", "Game Development", "Algorithms", "Python"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "project-overview",
      content: [
        {
          type: "text",
          content:
            "The Ultimate Quiz is a trivia game about historical rulers that generates questions by scraping Wikipedia. Instead of using a fixed question bank, the game pulls real data—birth dates, death dates, reign periods, successors—and turns it into multiple-choice questions on the fly.",
        },
        {
          type: "text",
          content:
            "The game also tracks how well you're doing and adapts accordingly. Questions you get wrong come back more often, while ones you've mastered appear less frequently. This is based on spaced repetition techniques used in flashcard apps.",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "Key Features",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Dynamic question generation** — questions created from scraped Wikipedia data",
            "**Adaptive difficulty** — tracks performance and adjusts question frequency",
            "**Spaced repetition** — reinforces learning by revisiting missed questions",
            "**Visual learning** — displays ruler portraits fetched from Wikipedia",
          ],
        },
      ],
    },
    {
      title: "Web Scraping for Ruler Data",
      navName: "Ruler Data",
      navRef: "game-structure",
      content: [
        {
          type: "text",
          content:
            "The game uses BeautifulSoup (a Python library) to scrape Wikipedia pages for historical rulers. It extracts structured information like birth/death dates, reign periods, and successor names, then stores this in a `Ruler` class.",
        },
        {
          type: "text",
          content:
            "This object-oriented approach keeps the scraping logic separate from the game mechanics. Adding a new ruler just means pointing the scraper at a new Wikipedia page—the rest of the system handles it automatically.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**BeautifulSoup** — parses HTML to extract structured data",
            "**Ruler class** — encapsulates name, dates, reign info, and image URL",
            "**Separation of concerns** — scraping logic isolated from game logic",
          ],
        },
      ],
    },
    {
      title: "Adaptive Mastery System",
      navName: "Mastery System",
      navRef: "mastery-system",
      content: [
        {
          type: "text",
          content:
            "The mastery system is inspired by spaced repetition algorithms like the Leitner System and SuperMemo. The basic idea: questions you answer correctly move to higher 'levels' and appear less often, while questions you miss drop back down and come up more frequently.",
        },
        {
          type: "text",
          content:
            "The system also tracks how long you take to answer. Fast correct answers indicate strong recall, while slow answers (even if correct) suggest the material isn't fully learned yet. A simple linear regression predicts future performance when recycling old questions.",
        },
        {
          type: "code",
          codeLang: "python",
          content: `class MasterySystem:
    def __init__(self, numLevels, listQuestions):
        self.levels = [[] for _ in range(numLevels)]  # Mastery boxes
        self.stats = {}   # Tracks correct/incorrect per question type
        
    def updateLevel(self, question, correct):
        # Move question up a level if correct, down if incorrect
        currentLevel = self.findLevel(question)
        if correct:
            newLevel = min(currentLevel + 1, self.numLevels - 1)
        else:
            newLevel = max(currentLevel - 1, 0)
        self.moveToLevel(question, newLevel)`,
          subtitle: "Core mastery system logic",
        },
        {
          type: "text",
          displayAs: "subtitle",
          content: "How It Works",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Level-based boxes** — questions organized by mastery level (like Leitner flashcards)",
            "**Performance tracking** — records correct/incorrect and response time",
            "**Difficulty adjustment** — question difficulty updates based on history",
            "**Linear regression** — predicts performance when recycling old questions",
          ],
        },
      ],
    },
    {
      title: "Question Generation",
      navName: "Question System",
      navRef: "question-generation",
      content: [
        {
          type: "text",
          content:
            "Questions are generated dynamically from the scraped ruler data. Each question pulls information from one ruler (e.g., 'When did Queen Victoria die?') and generates wrong answers using data from other rulers, making it harder to guess based on familiarity.",
        },
        {
          type: "text",
          content:
            "The system tracks response time along with correctness. A fast correct answer lowers the question's difficulty, while a slow answer (even if correct) keeps it at a higher difficulty level. This helps distinguish between genuine knowledge and lucky guesses.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Multiple choice format** — one correct answer from ruler data, three distractors from other rulers",
            "**Randomized answer position** — correct answer placed randomly among choices",
            "**Response time tracking** — faster responses indicate stronger recall",
          ],
        },
      ],
    },
    {
      title: "User Interface",
      navName: "UI Features",
      navRef: "user-interface",
      content: [
        {
          type: "text",
          content:
            "The game includes several features to help players track their progress and understand the mechanics.",
        },
        {
          type: "text",
          displayAs: "list",
          content: [
            "**Stats screen** — shows performance breakdown by question type",
            "**Help screen** — explains game mechanics and scoring",
            "**Ruler portraits** — images fetched from Wikipedia for visual learning",
            "**Error handling** — graceful fallbacks when scraping fails",
          ],
        },
      ],
    },
    {
      title: "Conclusion",
      navName: "Conclusion",
      navRef: "conclusion",
      content: [
        {
          type: "text",
          content:
            "This project combined web scraping, data parsing, and adaptive learning algorithms into an interactive quiz game. The spaced repetition system provides a practical application of learning science principles, while the dynamic question generation ensures varied content without manual question authoring.",
        },
      ],
    },
  ],
};
