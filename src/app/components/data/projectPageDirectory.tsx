import { Project } from "@/app/components/project/interfaces";
import { movingTargetPlanner } from "@/app/components/data/projects/movingTargetPlanner";
import { motionPlanner } from "@/app/components/data/projects/motionPlanners";
import { symbolicPlanner } from "@/app/components/data/projects/symbolicPlanner";
import { multiRobotFrontierExploration } from "@/app/components/data/projects/multiagentFrontier";
import { safeguardAgainstPests } from "@/app/components/data/projects/sap";
import { ultimateQuiz } from "@/app/components/data/projects/quizGame";

export const projectPagesDict: { [slug: string]: Project } = {
  "real-time-target-interception-with-multi-goal-a": movingTargetPlanner,
  "motion-planner": motionPlanner,
  "symbolic-planner": symbolicPlanner,
  "multi-robot-frontier-exploration": multiRobotFrontierExploration,
  "robotic-sentry-safe-guard-against-pests": safeguardAgainstPests,
  "quiz-game": ultimateQuiz,
};
