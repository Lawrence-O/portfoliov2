import { Project } from "@/app/components/project/interfaces";

export const ultimateQuiz: Project = {
  title: "The Ultimate Quiz",
  date: "Spring 2021",
  media: "/media/videos/quizGame.mp4",
  githubLink: "https://github.com/your-username/ultimate-quiz-game",
  tags: ["Web Scraping", "Game Development", "Algorithms", "Python", "Object-Oriented Design"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "project-overview",
      content: [
        {
          type: "text",
          content: [
            "The Ultimate Quiz is a dynamic and educational quiz game that was created through an exploration of web scraping and game design principles. This project uniquely merges real-time data acquisition, specifically from historical information about rulers, with engaging gameplay.",
            "Rather than relying on static question banks, the game dynamically generates its content through web scraping techniques, thereby offering a unique and personalized learning experience for each user. A key element is the integration of a personalized mastery system, which adapts to the player's learning curve and ensures a consistently challenging and effective learning process through spaced repetition."
          ]
        },
      ],
    },
    {
      title: "Dynamic Game Structure and Data Handling",
      navName: "Ruler Data",
      navRef: "game-structure",
      content: [
        {
          type: "text",
          content: [
            "At the heart of The Ultimate Quiz lies a sophisticated data structure centered around historical rulers. The game utilizes web scraping techniques with BeautifulSoup to extract specific details from Wikipedia pages, including crucial information such as birth and death dates, reign details, and successors. This scraped data is then used to power dynamic question generation, ensuring each game is both unique and educational.",
            "The 'Ruler' class plays a pivotal role in organizing this data. By encapsulating all details of a ruler—including their name, associated Wikipedia URL, and scraped information—the game becomes scalable and extensible, as adding new rulers is seamless. Additionally, the class is responsible for extracting the ruler's image, which greatly enhances visual engagement. This object-oriented approach isolates the web scraping logic from the core game mechanics, providing a clear separation of concerns that greatly improves the maintainability and readability of the code."
          ]
        }
      ],
    },
     {
      title: "Personalized and Adaptive Mastery System", // Merged and rephrased title
      navName: "Mastery System",
      navRef: "mastery-system",
      content: [
        {
          type: "text",
          content: [
           "To ensure that The Ultimate Quiz is both educational and enjoyable, a personalized mastery system, inspired by the Leitner System and SuperMemo algorithms, was implemented. This system is fundamental to managing the player's learning progression. It actively tracks player performance on each question, adjusting the frequency of questions accordingly: correctly answered questions appear less often, while incorrect answers are presented more frequently to reinforce learning.",
           "The 'MasterySystem' class is the core of this adaptive learning experience. It organizes questions into different 'levels' representing stages of mastery. Key methods like 'updateStats' and 'updateLevel' dynamically adjust this progression. The 'calculateDifficulty' method further refines the challenge based on performance history, and a custom linear regression algorithm can predict future performance on recycled questions. These features combine to create a unique learning path for each player, promoting continuous learning and engagement."
          ]
        },
        {
          type: "code",
          codeLang: "python",
          content: `
class MasterySystem(object):
    def __init__(self, numLevels, listQuestions):
        self.numLevels = numLevels
        self.listQuestions = listQuestions # All available MultipleChoiceQuestion objects
        # self.stats: Tracks correct/incorrect counts per question type
        # self.appearedQTypes: Tracks appearance count per question type
        # self.levels: A 2D list representing mastery boxes/levels, initially populated
        # self.oldQuestions: Logs past questions and correctness
        # self.recycleQ: Flag indicating if questions are being recycled
        # ... (initialization of these attributes) ...

    def calculateDifficulty(self, mcQuestion, correct, currRetrivalTime):
        # Adjusts mcQuestion.difficulty
        # If not self.recycleQ:
        #   difficulty decreases if correct, increases if incorrect.
        # Else (recycling questions, linear regression is used):
        #   anticipatedTime = self.linearRegression(mcQuestion.getRetrivalTime())
        #   anticipatedDifficulty = self.linearRegression(mcQuestion.getPastDifficulties())
        #   Difficulty adjusted based on comparison of currRetrivalTime to anticipatedTime,
        #   and weighted by anticipatedDifficulty.
        # ...
        pass

    def updateStats(self, mcQuestion, pickedAnswer, correct, retrivalTime):
        # Update self.stats for the mcQuestion's type
        # mcQuestion.setDifficulty(self.calculateDifficulty(...))
        # mcQuestion.addRetrivalTime(retrivalTime), mcQuestion.addPastDifficulties(...)
        # Log to self.oldQuestions
        # self.updateLevel(mcQuestion, correct, retrivalTime)
        # ...
        pass

    def updateLevel(self, mcQuestion, correct, currRetrivalTime):
        # Finds current level of mcQuestion.getQuestion().getQuestionType()
        # If not self.recycleQ:
        #   If correct, move type to next higher level (or stay in highest).
        #   If incorrect, move type to next lower level (or stay in lowest).
        # Else (recycling, uses linear regression from past difficulties):
        #   dRow (change in level) determined by regression slope.
        #   Move type by dRow (up if correct, down if incorrect).
        # ...
        pass

    def linearRegression(self, listYPoints):
        # Performs linear regression on listYPoints (e.g., past retrieval times or difficulties)
        # Returns: slope (m), anticipated_next_y_value
        # ... (implementation as shown previously) ...
        pass

    def getNextQuestion(self):
        # If self.listQuestions is empty, trigger recycling logic (self.recycleSelfQuestions)
        # If not self.recycleQ:
        #   nextQType = self.findNextQType() (selects based on appearance count, difficulty)
        #   Find and return a question of nextQType from self.listQuestions.
        # Else (recycling):
        #   nextQType = self.findQTypeInLevels() (selects from levels)
        #   nextQuestion = self.findRecycledQuestion(nextQType) (uses linear regression to pick)
        #   Return nextQuestion.
        # ...
        pass
`,
          subtitle: "Core logic of the MasterySystem class (simplified)",
        },
      ],
    },
    {
      title: "Dynamic Question Generation",
      navName: "Question System",
      navRef: "question-generation",
       content: [
        {
          type: "text",
          content: [
            "The game’s question system was designed to be highly dynamic, with each question generated from ruler data and presented in an engaging multiple-choice format. This approach prevents predictability and ensures accessibility. The 'MultipleChoiceQuestion' class is vital, creating varied answer choices using data from different rulers to prevent guessing based on familiarity with a single name.",
            "The system also tracks retrieval time, dynamically adjusting question difficulty: correct and rapid responses lower difficulty, while incorrect or slow answers increase it, personalizing the challenge. Each question, managed by the 'Question' class (holding text, answer, and ruler data), uses a randomization algorithm in 'createMultipleChoice' for answer generation and 'createResultAnswers' to vary the correct answer's position, ensuring an unpredictable yet rewarding experience."
          ]
        }
      ],
    },
    {
      title: "User Interface and Key Features",
      navName: "UI Features",
      navRef: "user-interface",
      content: [
        {
          type: "text",
          content: [
            "To enhance user experience, several key features were implemented. A Stats Screen provides a comprehensive overview of player progress, highlighting strengths and areas for improvement. The Help Screen offers guidance on game mechanics, aiding new users.",
            "Furthermore, the game dynamically fetches and displays images of each ruler from Wikipedia, enriching gameplay with a visual learning dimension. Robust error handling, crucial for web scraping, ensures stability. A Linear Regression model dynamically adjusts difficulty levels based on performance, keeping the game challenging yet attainable. These features combine to create a reliable and user-friendly learning tool."
          ],
          displayAs: "list" // Changed to list for better readability of features
        }
      ],
    },
    {
      title: "Conclusion",
      navName: "Conclusion",
      navRef: "conclusion",
      content: [
        {
          type: "text",
          content: [
            "Developing The Ultimate Quiz provided valuable experience in web scraping, data parsing, and game development. The project resulted in an interactive tool that effectively combines educational content with an engaging gaming framework.",
            "This endeavor demonstrates the power of complex algorithms in creating personalized learning experiences and illustrates how techniques from diverse fields can merge to produce effective and engaging educational applications."
          ]
        },
      ],
    },
  ],
};
