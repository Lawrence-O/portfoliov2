import { Project } from "@/app/components/project/interfaces";

export const ultimateQuiz: Project = {
  title: "The Ultimate Quiz",
  subtitle: "Spring 2021",
  media: "/media/videos/quizGame.mp4",
  tags: ["Web Scraping", "Game Development", "Algorithms", "Python", "Object-Oriented Design"],
  section: [
    {
      title: "Project Overview",
      navName: "Overview",
      navRef: "project-overview",
      content: [
        {
          type: "text",
          content:
            "The Ultimate Quiz is a dynamic quiz game born from my exploration of web scraping and game design. This project uniquely combines the real-time acquisition of data, specifically historical information about various rulers, with engaging gameplay mechanics. Instead of relying on static question banks, the game dynamically generates content using web scraping techniques and enhances the player experience with a personalized mastery system that adapts to their learning curve. This system is built upon the principles of spaced repetition, ensuring effective knowledge retention and a consistently challenging experience.",
        },
      ],
    },
    {
      title: "Game Structure and Ruler Data",
      navName: "Ruler Data",
      navRef: "game-structure",
      content: [
        {
          type: "text",
          content:
            "The game’s core is its sophisticated data structure, where each session is centered around historical rulers. The game scrapes specific details from their Wikipedia pages using BeautifulSoup. This includes crucial information such as birth and death dates, reign details, and successors. This scraped data isn't simply displayed; it's the engine behind the dynamic question generation, ensuring each game is unique and educational. The Ruler class is key to organizing this data. It encapsulates all details of a ruler, including their name, associated Wikipedia URL, and the scraped information. This approach makes the game scalable and extensible by making it easy to add new rulers without overhauling the underlying system. The class is also responsible for extracting the ruler's image, significantly enhancing visual engagement.",
        },
        {
          type: "text",
          content:
            "The Ruler class effectively serves as the foundation for the game's question-generating process. By encapsulating each ruler’s data into an object, it allows for an efficient, maintainable, and easily expandable system. This object-oriented approach isolates the web scraping logic from the rest of the game. This clear separation of concerns makes the code much easier to understand and maintain.",
        },
      ],
    },
    {
      title: "Mastery System and Player Progression",
      navName: "Mastery System",
      navRef: "mastery-system",
      content: [
        {
          type: "text",
          content:
            "To make The Ultimate Quiz both educational and fun, I've implemented a mastery system inspired by the Leitner System and SuperMemo algorithms. This system is central to managing player learning and progression. It actively monitors player performance on each question and adjusts the question frequency accordingly. Questions that players answer correctly with consistency appear less often, while incorrect ones are presented more frequently. This approach reinforces learning and knowledge retention. Furthermore, the system dynamically adjusts question difficulty, ensuring that the game becomes more challenging as the player improves, creating a dynamic and personalized learning environment.",
        },
        {
          type: "text",
          content:
            "The MasterySystem class manages the entire mastery system. This class keeps track of a player’s learning and organizes questions into different 'levels,' representing the stages of mastery. The system is not static and is updated with methods like 'updateStats' and 'updateLevel.' The 'calculateDifficulty' method adjusts the question difficulty using the player's historical performance data. A custom linear regression algorithm is also used to predict the player's future performance on a question. These combined features offer a unique experience to each player. It maintains player engagement and encourages continuous learning throughout the game.",
        },
      ],
    },
    {
      title: "Core Mastery System Implementation",
      navName: "Core System",
      navRef: "core-mastery-system",
      content: [
        {
          type: "text",
          content:
            "The adaptive learning within the game is primarily driven by the `MasterySystem` class. This class manages tracking player performance, adjusting question difficulty, and managing question frequency using the principles of the Leitner System and SuperMemo. This creates a personalized learning experience. The following code block showcases the core logic of this class:",
        },
        {
          type: "code",
          content: `
/**
 *  Mastery System Class: Manages question difficulty and frequency.
 */
class MasterySystem {
    /**
     * @param {number} numLevels - The number of levels in the mastery system.
     * @param {Array} listQuestions - The list of all questions.
     */
    constructor(numLevels, listQuestions) {
      this.numLevels = numLevels;
      this.listQuestions = listQuestions;
      this.stats = {
        Successor: [0, 0],
        BirthDate: [0, 0],
        BirthPlace: [0, 0],
        DeathDate: [0, 0],
        DeathPlace: [0, 0],
        AgeOfDeath: [0, 0],
        Spouse: [0, 0],
        House: [0, 0],
        Total: 0,
      };
      this.appearedQTypes = {
        Successor: 0,
        BirthDate: 0,
        BirthPlace: 0,
        DeathDate: 0,
        DeathPlace: 0,
        AgeOfDeath: 0,
        Spouse: 0,
        House: 0,
      };
      this.levels = this.initializeStartingBox(numLevels);
      this.oldQuestions = [];
      this.numCycles = 0;
      this.recycleQ = false;
    }

    /**
     * Initializes the starting boxes for the mastery system.
     * @param {number} numLevels The number of levels for the mastery system.
     * @return {Array} The initialized 2D array representing the levels.
     */
    initializeStartingBox(numLevels){
      //Implementation of the mastery level initialization
        return twoD;
    }
    /**
     * Calculates the difficulty of a question based on player performance.
     * @param {object} mcQuestion - The multiple-choice question object.
     * @param {boolean} correct - Whether the answer was correct.
     * @param {number} currRetrivalTime - The time taken to answer the question.
     * @return {number} The new difficulty of the question.
     */
    calculateDifficulty(mcQuestion, correct, currRetrivalTime) {
      //Implementation to calculate question difficulty
      return currDiff;
    }
    /**
     * Updates the game stats and the player's level.
     * @param {object} mcQuestion - The multiple-choice question object.
     * @param {string} pickedAnswer - The answer the player picked.
     * @param {boolean} correct - Whether the answer was correct.
     * @param {number} retrivalTime - The time taken to answer the question.
     */
    updateStats(mcQuestion, pickedAnswer, correct, retrivalTime) {
       // Implementation to update game stats and question level
    }
   /**
     * Updates the level of a question based on the player's performance.
     * @param {object} mcQuestion The multiple-choice question object.
     * @param {boolean} correct Whether the answer was correct.
     * @param {number} currRetrivalTime The time taken to answer the question.
    */
    updateLevel(mcQuestion, correct, currRetrivalTime){
      //Implementation of updating question level based on correctness and time
    }
     /**
     * Finds the next question type to be asked.
     * @return {string} The question type to be asked.
     */
    findNextQType() {
      // Implementation to find the next question type
      return qType;
    }
    /**
     * Performs linear regression on a list of y-points.
     * @param {Array<number>} listYPoints The list of y-points for regression.
     * @return {Array<number>} An array containing the slope and the anticipated y-value.
     */
    linearRegression(listYPoints) {
      // Implementation for linear regression
      return [m, newTime];
    }

     /**
     * Finds a recycled question based on question type.
     * @param {string} qType The type of question to look for.
     * @return {object} The recycled multiple-choice question object.
     */
    findRecycledQuestion(qType){
      // Implementation to find the recycled question
      return mcQuestion;
    }
    /**
     * Finds a question type in the level box.
     * @return {string} The question type to be asked.
     */
    findQTypeInLevels(){
        //Implementation to find question type within level box
        return qType;
    }
     /**
     * Recycles self questions, shuffles them and sets the recycle flag to true.
     */
    recycleSelfQuestions(){
        //Implementation to recycle all previous questions
    }
     /**
     * Gets the next question from the list of questions.
     * @return {object} The next multiple-choice question object.
     */
    getNextQuestion(){
        //Implementation to get next question using find next question type or recycled question
        return mcQuestion;
    }
}
                    `,
          codeLang: "javascript",
          subtitle:
            "A snippet of the Mastery System implementation, showing key methods for question management.",
        },
      ],
    },
    {
      title: "Question Generation and Multiple Choice System",
      navName: "Question System",
      navRef: "question-generation",
      content: [
        {
          type: "text",
          content:
            "The game's question system is built to be highly dynamic. Each question is generated from ruler data and presented in an engaging multiple-choice format, making them more accessible and less predictable. The MultipleChoiceQuestion class is crucial for this. It creates answer choices using data from different rulers, preventing players from simply guessing based on one familiar name. Also, by tracking retrieval time—how long it takes a player to answer—the system can dynamically adjust the difficulty of the questions. Correct answers given quickly result in a lower difficulty, whereas incorrect or slow answers increase the difficulty, creating a tailored and adaptive experience.",
        },
        {
          type: "text",
          content:
            "Each question is first created and managed by a Question class, which holds the question text, the correct answer and the associated ruler. From this Question class the multiple-choice list is then generated using a randomizing algorithm within the 'createMultipleChoice' method to ensure that each question is both challenging and varied. The ‘createResultAnswers’ method is responsible for ensuring that the correct answer is not always presented in the same position, preventing predictability. This careful design has created a game that is simple to understand yet complex enough to offer a very rewarding and engaging experience.",
        },
      ],
    },
    {
      title: "User Interface and Additional Features",
      navName: "UI Features",
      navRef: "user-interface",
      content: [
        {
          type: "text",
          content:
            "To enhance the overall user experience, I implemented several features that add to the game's usability and functionality. A Stats Screen provides a detailed overview of the player’s progress, highlighting both strengths and weaknesses. The Help Screen is always available to guide new players through the game mechanics. Additionally, the game leverages image scraping to fetch and display images of each ruler directly from Wikipedia. This significantly enriches the gameplay and adds a visual dimension to the learning experience. These elements ensure that the game is both easy to use and enjoyable for all players.",
        },
        {
          type: "text",
          content:
            "The design emphasizes both robustness and stability. The system is equipped with comprehensive error-handling capabilities, which is important for a web scraping-based application because source websites are always susceptible to unexpected changes. The game also uses a Linear Regression model to dynamically adjust difficulty levels based on the player's performance history. This ensures that the game remains consistently challenging yet attainable. These features create a game that is not only an engaging learning tool but is also reliable and user-friendly.",
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
            "Through the development of 'The Ultimate Quiz,' I was able to explore the intricacies of web scraping, data parsing, and game development. The end result is an interactive tool that skillfully combines educational content with an engaging gaming structure. It effectively demonstrates the use of complex algorithms for providing personalized learning. This project clearly shows how different techniques and principles can be combined from various fields to create highly effective and interactive learning tools.",
        },
      ],
    },
  ],
};