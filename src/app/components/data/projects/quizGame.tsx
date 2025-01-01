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
            "The Ultimate Quiz is a dynamic and educational quiz game that was created through an exploration of web scraping and game design principles. This project uniquely merges real-time data acquisition, specifically from historical information about rulers, with engaging gameplay. Rather than relying on static question banks, the game dynamically generates its content through web scraping techniques, thereby offering a unique and personalized learning experience for each user. A key element is the integration of a personalized mastery system, which adapts to the player's learning curve and ensures a consistently challenging and effective learning process through spaced repetition.",
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
          content:
            "At the heart of The Ultimate Quiz lies a sophisticated data structure centered around historical rulers. The game utilizes web scraping techniques with BeautifulSoup to extract specific details from Wikipedia pages, including crucial information such as birth and death dates, reign details, and successors. This scraped data is then used to power dynamic question generation, ensuring each game is both unique and educational. The 'Ruler' class plays a pivotal role in organizing this data. By encapsulating all details of a ruler—including their name, associated Wikipedia URL, and scraped information—the game becomes scalable and extensible, as adding new rulers is seamless. Additionally, the class is responsible for extracting the ruler's image, which greatly enhances visual engagement.",
        },
        {
          type: "text",
          content:
            "The 'Ruler' class serves as the foundational component for the game’s question-generating process. By encapsulating each ruler's data into an object, the game maintains an efficient, organized, and easily expandable system. This object-oriented approach isolates the web scraping logic from the core game mechanics, providing a clear separation of concerns that greatly improves the maintainability and readability of the code.",
        }
      ],
    },
     {
      title: "Personalized Mastery System",
      navName: "Mastery System",
      navRef: "mastery-system",
      content: [
        {
          type: "text",
          content:
           "To ensure that The Ultimate Quiz is both educational and enjoyable, I implemented a mastery system inspired by the Leitner System and SuperMemo algorithms. This system is fundamental to managing the player's learning progression. It actively tracks player performance on each question, adjusting the frequency of questions accordingly. Questions answered correctly consistently appear less often, while incorrect answers are presented more frequently, strengthening knowledge retention. Furthermore, the system dynamically adjusts the question difficulty, providing a personalized learning environment that evolves with the player's skills.",
        },
        {
          type: "text",
          content:
            "The 'MasterySystem' class is the core of the adaptive learning experience. It tracks each player's learning progress, organizing questions into different 'levels' that represent stages of mastery. The system is dynamic and updated with methods such as 'updateStats' and 'updateLevel.' The 'calculateDifficulty' method adjusts question difficulty based on the player’s performance history. In addition, a custom linear regression algorithm predicts the player's future performance on a question. These combined features create a unique learning experience for each player, promoting continuous learning and engagement throughout the game.",
        }
      ],
    },
    {
      title: "Core Mastery System Logic",
      navName: "Core System",
      navRef: "core-mastery-system",
      content: [
        {
          type: "text",
          content:
            "The game's adaptive learning is driven by the 'MasterySystem' class, which is responsible for tracking player performance, adjusting question difficulty, and managing question frequency. It employs the principles of the Leitner System and SuperMemo to create a personalized learning experience. The code snippet below demonstrates the core logic of the 'MasterySystem' class:",
        },
        {
          type: "code",
          content: `"""
MasterySystem Class: Manages question difficulty and frequency using a spaced
repetition approach.
"""
class masterySystem(object):
    def __init__(self, numLevels, listQuestions):
        """
        Initializes the mastery system.

        Args:
            numLevels (int): The number of levels in the mastery system.
            listQuestions (list): A list of all multiple-choice questions.
        """
        self.numLevels = numLevels
        self.listQuestions = listQuestions
        # stats dictionary tracks correct/incorrect answers
        self.stats = {'Successor': (0, 0), 'BirthDate': (0, 0),
                      'BirthPlace': (0, 0), 'DeathDate': (0, 0),
                      'DeathPlace': (0, 0), 'AgeOfDeath': (0, 0),
                      'Spouse': (0, 0), 'House': (0, 0), 'Total': 0}
        # appearedQTypes tracks how many times each question type appeared
        self.appearedQTypes = {'Successor': 0, 'BirthDate': 0, 'BirthPlace': 0,
                               'DeathDate': 0, 'DeathPlace': 0, 'AgeOfDeath': 0,
                               'Spouse': 0, 'House': 0}
        # initializes the levels/boxes
        self.levels = self.initializeStartingBox(numLevels)
        # oldQuestions list tracks previous questions
        self.oldQuestions = []
        # tracks how many times the recycle self question happened
        self.numCycles = 0
        # flags whether the questions were recycled
        self.recycleQ = False
    def initializeStartingBox(self, numLevels):
        """
        Initializes the levels of the mastery system.

        Args:
            numLevels (int): The number of levels to initialize.

        Returns:
            list: A 2D list representing the levels of the mastery system.
        """
        twoD = [[] for row in range(numLevels)]
        for qType in self.appearedQTypes.keys():
            twoD[0].append(qType) #Add all the Question Types
        return twoD
    
    def calculateDifficulty(self, mcQuestion, correct, currRetrivalTime):
        """
        Calculates the difficulty of a question based on player performance.

        Args:
            mcQuestion (multipleChoiceQuestion): The question being evaluated.
            correct (bool): Whether the player answered correctly.
            currRetrivalTime (float): Time taken to answer the question.

        Returns:
            int: The new difficulty of the question.
        """
        currDiff = mcQuestion.getDifficulty()
        if not self.recycleQ:
            if correct:
                return currDiff - 1 #Decrease Difficulty if correct
            else:
                return currDiff + 1 #Increase Difficulty if incorrect
        else: #Linear Regression implementation
            slope_time, anticipatedTime = self.linearRegression(
                mcQuestion.getRetrivalTime())
            if anticipatedTime < 2.3:
                anticipatedTime = 2.3
            slope_difficulty, anticipatedDifficulty = self.linearRegression(
                mcQuestion.getPastDifficulties())
            timeWeight = 0.2
            if correct:
                if anticipatedTime > currRetrivalTime:
                     timeWeight = 0.25
                return currDiff - timeWeight * anticipatedDifficulty
            else:
                if anticipatedTime < currRetrivalTime:
                     timeWeight = 0.35
                return currDiff + anticipatedDifficulty * timeWeight
    
    def updateStats(self, mcQuestion, pickedAnswer, correct, retrivalTime):
        """
        Updates the statistics of the game based on player input.

        Args:
            mcQuestion (multipleChoiceQuestion): The question answered.
            pickedAnswer (str): The answer the player selected.
            correct (bool): Whether the player was correct.
            retrivalTime (float): Time taken to answer the question.
        """
        numRight, numWrong = self.stats[mcQuestion.getQuestion().
                                       getQuestionType()]
        currDiff = mcQuestion.getDifficulty()
        mcQuestion.setDifficulty(self.calculateDifficulty(mcQuestion,
                                                        correct,
                                                        retrivalTime))
        mcQuestion.addRetrivalTime(retrivalTime)
        mcQuestion.addPastDifficulties(currDiff)
        if correct:
            numRight += 1
        else:
            numWrong += 1
            mcQuestion.addWrongAns(pickedAnswer)
        self.stats[mcQuestion.getQuestion().getQuestionType()] = (numRight,
                                                                 numWrong)
        self.appearedQTypes[mcQuestion.getQuestion().getQuestionType()] += 1
        self.stats['Total'] += 1
        self.oldQuestions.append((mcQuestion, correct))
        self.updateLevel(mcQuestion, correct, retrivalTime)
    
    def updateLevel(self, mcQuestion, correct, currRetrivalTime):
        """
        Updates the level of a question in the mastery system based on
        player performance.

        Args:
            mcQuestion (multipleChoiceQuestion): The question answered.
            correct (bool): Whether the player was correct.
            currRetrivalTime (float): Time taken to answer.
        """
        currentPos = (None, None)
        # find the current pos of the question
        for i in range(len(self.levels)):
            for j in range(len(self.levels[i])):
                if (mcQuestion.getQuestion().getQuestionType() ==
                        self.levels[i][j]):
                    currentPos = (i, j)
        currRow, currCol = currentPos
        if not self.recycleQ:
            if correct:
                if currRow + 1 >= len(self.levels):
                   self.levels[currRow].append(mcQuestion.getQuestion().
                                               getQuestionType())
                else:
                    self.levels[currRow + 1].append(mcQuestion.getQuestion().
                                                    getQuestionType())
            else:
                if currRow - 1 < 0:
                    self.levels[currRow].append(mcQuestion.getQuestion().
                                                getQuestionType())
                else:
                    self.levels[currRow - 1].append(mcQuestion.getQuestion().
                                                    getQuestionType())
        else:
            slope_difficulty, anticipatedDifficulty = self.linearRegression(
                mcQuestion.getPastDifficulties())
            dRow = 1
            if slope_difficulty < 0 and abs(slope_difficulty) > 1.25:
                dRow = 2
            if correct:
                if currRow + dRow >= len(self.levels):
                    self.levels[currRow].append(mcQuestion.getQuestion().
                                                getQuestionType())
                else:
                    self.levels[currRow + dRow].append(mcQuestion.getQuestion().
                                                     getQuestionType())
            else:
                if currRow - dRow < 0:
                    self.levels[currRow].append(mcQuestion.getQuestion().
                                                getQuestionType())
                else:
                     self.levels[currRow - dRow].append(mcQuestion.getQuestion().
                                                        getQuestionType())

    def findNextQType(self):
        """
        Finds the next question type to be asked based on past performance.

        Returns:
            str: The question type to be asked.
        """
        minQAppearance = set()
        minNum = None
        for qType in self.appearedQTypes:
            if minNum is None or self.appearedQTypes[qType] < minNum:
                minNum = self.appearedQTypes[qType]
                minQAppearance = {qType}
            elif self.appearedQTypes[qType] == minNum:
                minQAppearance.add(qType)
        if len(minQAppearance) == 0:
            print("Error")
        if len(minQAppearance) == 1:
            return list(minQAppearance)[0]
        elif len(minQAppearance) > 1 and minNum > 2:
            mostNumDiff = None
            mostDiffQuestion = None
            for mcQuestion, correctness in self.oldQuestions:
                if (mostNumDiff is None or (correctness is False and
                        mostNumDiff > mcQuestion.getDifficulty() and
                        mcQuestion.getQuestion().getQuestionType() in
                        minQAppearance)):
                    mostNumDiff = mcQuestion.getDifficulty()
                    mostDiffQuestion = mcQuestion
            if mostDiffQuestion is not None and mostNumDiff is not None:
                return mostDiffQuestion.getQuestion().getQuestionType()
            else:
                index = random.randint(0, len(minQAppearance)-1)
                return list(minQAppearance)[index]
        else:
            return random.choice(list(minQAppearance))
    
    def linearRegression(self, listYPoints):
        """
        Performs linear regression on a list of y-points.
        https://www.statisticshowto.com/probability-and-statistics/
        regression-analysis/find-a-linear-regression-equation/

        Args:
            listYPoints (list): A list of y-values.

        Returns:
            tuple: A tuple containing the slope (m) and the anticipated
                   y-value.
        """
        coordinates = []
        if len(listYPoints) == 0:
            print("Linear Regression Error")
            return None
        for i in range(len(listYPoints)):
            coordinates.append((i, listYPoints[i]))
        # b = (sum(y) * sum(x^2) - sum(x) * sum(xy)) / (n*sum(x^2) - sum(x)^2)
        b = ((sum(listYPoints) * sum([x**2 for x, y in coordinates])) -
             (sum([x for x, y in coordinates]) *
              sum([x * y for x, y in coordinates]))) / 
            (len(listYPoints) * sum([x**2 for x, y in coordinates]) -
             sum([x for x, y in coordinates])**2)
        # m = (n*sum(xy) - sum(x)*sum(y)) / (n*sum(x^2) - sum(x)^2)
        m = (len(listYPoints) * sum([x * y for x, y in coordinates]) -
             (sum([x for x, y in coordinates]) *
              sum([y for x, y in coordinates]))) / 
            (len(listYPoints) * sum([x**2 for x, y in coordinates]) -
             sum([x for x, y in coordinates])**2)
        newTime = m * (len(listYPoints) + 1) + b
        return m, newTime
    
    def findRecycledQuestion(self, qType):
        """
        Finds a recycled question based on question type.

        Args:
            qType (str): The type of question to look for.

        Returns:
            multipleChoiceQuestion: The recycled multiple-choice question
                                    object.
        """
        largestDecreaseInTime_nSlope = None
        largestDecreaseInTimeQuestion_nSlope = None
        largestDecreaseInTime_pSlope = None
        largestDecreaseInTimeQuestion_pSlope = None
        for mcQuestion in self.listQuestions:
            if mcQuestion.getQuestion().getQuestionType() == qType:
                retrivalTime = mcQuestion.getRetrivalTime()
                slope, newTime = self.linearRegression(retrivalTime)
                #Positive Slope --> Got worse; #Negative Slope --> Got Better
                if slope < 0:
                    if (largestDecreaseInTime_nSlope is None or
                            newTime < largestDecreaseInTime_nSlope):
                        largestDecreaseInTime_nSlope = newTime
                        largestDecreaseInTimeQuestion_nSlope = mcQuestion
                    elif math.isclose(newTime, largestDecreaseInTime_nSlope,
                                     rel_tol=1e-2):
                        if (mcQuestion.getDifficulty() >
                                largestDecreaseInTimeQuestion_nSlope.
                                getDifficulty()):
                            largestDecreaseInTime_nSlope = newTime
                            largestDecreaseInTimeQuestion_nSlope = mcQuestion
                elif slope > 0:
                    if (largestDecreaseInTime_pSlope is None or
                            newTime > largestDecreaseInTime_pSlope):
                         largestDecreaseInTime_pSlope = newTime
                         largestDecreaseInTimeQuestion_pSlope = mcQuestion
                    elif math.isclose(newTime, largestDecreaseInTime_pSlope,
                                     rel_tol=1e-1):
                        if (mcQuestion.getDifficulty() >
                                largestDecreaseInTimeQuestion_pSlope.
                                getDifficulty()):
                            largestDecreaseInTime_pSlope = newTime
                            largestDecreaseInTimeQuestion_pSlope = mcQuestion
        if largestDecreaseInTime_pSlope is not None:
            index = self.listQuestions.index(largestDecreaseInTimeQuestion_pSlope)
            self.listQuestions.pop(index)
            return largestDecreaseInTimeQuestion_pSlope
        elif largestDecreaseInTime_nSlope is not None:
            index = self.listQuestions.index(largestDecreaseInTimeQuestion_nSlope)
            self.listQuestions.pop(index)
            return largestDecreaseInTimeQuestion_nSlope
        else:
            return self.listQuestions.pop(0)
    def getNextQuestion(self):
        """
        Gets the next question from the list of questions.

        Returns:
             multipleChoiceQuestion: The next multiple-choice question object.
        """
        if len(self.listQuestions) == 0:
            self.recycleSelfQuestions()
        if not self.recycleQ:
            nextQType = self.findNextQType()
            for i in range(len(self.listQuestions)):
                if (self.listQuestions[i].getQuestion().getQuestionType() ==
                        nextQType):
                    result = self.listQuestions.pop(i)
                    random.shuffle(self.listQuestions)
                    self.developer_forceRecycle()
                    return result
        else:
            nextQType = self.findQTypeInLevels()
            nextQuestion = self.findRecycledQuestion(nextQType)
            random.shuffle(self.listQuestions)
            return nextQuestion`,
          codeLang: "python",
          subtitle:
            "A snippet of the Mastery System implementation, showing key methods for question management.",
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
          content:
            "The game’s question system was designed to be highly dynamic, with each question generated from ruler data and presented in an engaging multiple-choice format. This approach prevents predictability and ensures accessibility for players. The 'MultipleChoiceQuestion' class is a vital part of this system, responsible for creating varied answer choices using data from different rulers, thus preventing players from simply guessing based on one familiar name. The system also tracks retrieval time, dynamically adjusting question difficulty based on how quickly a player answers. Correct and rapid responses result in a lower difficulty, while incorrect or slow answers increase the difficulty, creating a highly personalized and adaptive experience.",
        },
        {
          type: "text",
          content:
            "Each question is first created and managed by the 'Question' class, which holds the question text, the correct answer, and the associated ruler. The multiple-choice list is then generated using a randomization algorithm within the 'createMultipleChoice' method. Additionally, the 'createResultAnswers' method ensures that the correct answer is not always presented in the same position, enhancing unpredictability. This careful design has resulted in a game that is easy to understand yet offers a very rewarding and challenging experience.",
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
          content:
            "To enhance user experience, several key features were implemented to ensure the game is both usable and engaging. A dedicated Stats Screen provides a comprehensive overview of the player’s progress, highlighting both areas of strength and areas needing improvement. The Help Screen offers continuous guidance through the game mechanics, ensuring new users can quickly grasp the rules. Furthermore, the game dynamically fetches and displays images of each ruler directly from Wikipedia, enriching gameplay with a visual learning dimension. These features help ensure the game is both easy to use and enjoyable for all players.",
        },
        {
          type: "text",
          content:
            "The game's design places a high emphasis on both robustness and stability. The system has comprehensive error handling, a crucial aspect for web scraping applications, as source websites are always subject to unexpected changes. Moreover, a Linear Regression model is employed to dynamically adjust difficulty levels according to the player's performance, ensuring that the game remains both challenging and attainable. These features combine to create a reliable and user-friendly learning tool.",
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
            "Through the development of The Ultimate Quiz, I gained valuable experience in web scraping, data parsing, and game development. The resulting project is an interactive tool that combines educational content with an engaging gaming framework. This project effectively demonstrates the use of complex algorithms for providing a personalized learning experience. This project illustrates how different techniques and principles from various fields can be combined to create effective and engaging learning applications.",
        },
      ],
    },
  ],
};