import { autonomousGardenRobot } from "@/app/components/data/projects/autonomousGardenRobot";
import { cableSuspendedLoads } from "@/app/components/data/projects/cableSuspendLoads";
import { gripAssistiveGlove } from "@/app/components/data/projects/gripAssistiveGlove";
import { habitatAirlockLinkage } from "@/app/components/data/projects/habitatAirlock";
import { roboticArmControl } from "@/app/components/data/projects/jengaRobot";
import { motionPlanner } from "@/app/components/data/projects/motionPlanners";
import { movingTargetPlanner } from "@/app/components/data/projects/movingTargetPlanner";
import { multiRobotFrontierExploration } from "@/app/components/data/projects/multiagentFrontier";
import {optimalControlHW2} from "@/app/components/data/projects/optimalControlHw2";
import { optimalControlHW3 } from "@/app/components/data/projects/optimalControlHw3";
import { optimalControlHW4 } from "@/app/components/data/projects/optimalControlHw4";
import { ultimateQuiz } from "@/app/components/data/projects/quizGame";
import { safeguardAgainstPests } from "@/app/components/data/projects/sap";
import { symbolicPlanner } from "@/app/components/data/projects/symbolicPlanner";
import { Project } from "@/app/components/project/interfaces";

export const projectPagesDict: { [slug: string]: Project } = {
  "real-time-target-interception-with-multi-goal-a": movingTargetPlanner,
  "sampling-based-motion-planners": motionPlanner,
  "symbolic-planner": symbolicPlanner,
  "multi-robot-frontier-exploration": multiRobotFrontierExploration,
  "robotic-sentry-safe-guard-against-pests": safeguardAgainstPests,
  "the-ultimate-quiz-ruler-edition": ultimateQuiz,
  "grip-assistive-glove": gripAssistiveGlove,
  "habitat-airlock-linkage-system": habitatAirlockLinkage,
  "optimal-control-of-linear-systems-lqr-tvlqr-and-mpc":optimalControlHW2,
  "trajectory-optimization-with-dircol-ilqr-and-tvlqr":optimalControlHW3,
  "jenga-building-robotic-arm": roboticArmControl,
  "trajectory-generation-with-iterative-learning-and-hybrid-control": optimalControlHW4,
  "distributed-quadrotors-transporting-a-load":cableSuspendedLoads,
  "autonomous-garden-maintenance-robot":autonomousGardenRobot
};
