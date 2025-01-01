import { Project } from "@/app/components/project/interfaces";
import { movingTargetPlanner } from "@/app/components/data/projects/movingTargetPlanner";
import { motionPlanner } from "@/app/components/data/projects/motionPlanners";
import { symbolicPlanner } from "@/app/components/data/projects/symbolicPlanner";
import { multiRobotFrontierExploration } from "@/app/components/data/projects/multiagentFrontier";
import { safeguardAgainstPests } from "@/app/components/data/projects/sap";
import { ultimateQuiz } from "@/app/components/data/projects/quizGame";
import { gripAssistiveGlove } from "@/app/components/data/projects/gripAssistiveGlove";
import { habitatAirlockLinkage } from "@/app/components/data/projects/habitatAirlock";
import {optimalControlHW2} from "@/app/components/data/projects/optimalControlHw2";
import { optimalControlHW3 } from "@/app/components/data/projects/optimalControlHw3";
import { roboticArmControl } from "@/app/components/data/projects/jengaRobot";
import { optimalControlHW4 } from "@/app/components/data/projects/optimalControlHw4";
import { cableSuspendedLoads } from "@/app/components/data/projects/cableSuspendLoads";
import { autonomousGardenRobot } from "@/app/components/data/projects/autonomousGardenRobot";

export const projectPagesDict: { [slug: string]: Project } = {
  "real-time-target-interception-with-multi-goal-a": movingTargetPlanner,
  "motion-planner": motionPlanner,
  "symbolic-planner": symbolicPlanner,
  "multi-robot-frontier-exploration": multiRobotFrontierExploration,
  "robotic-sentry-safe-guard-against-pests": safeguardAgainstPests,
  "quiz-game": ultimateQuiz,
  "grip-assitive-glove": gripAssistiveGlove,
  "habitat-airlock-linkage-system": habitatAirlockLinkage,
  "optimal-control-of-linear-systems-lqr-tvlqr-and-mpc":optimalControlHW2,
  "trajectory-optimization-with-dircol-ilqr-and-tvlqr":optimalControlHW3,
  "jenga-building-robotic-arm": roboticArmControl,
  "trajectory-generation-with-iterative-learning-and-hybrid-control": optimalControlHW4,
  "distributed-quadrotors-transporting-a-load":cableSuspendedLoads,
  "autonomous-garden-maintenance-robot":autonomousGardenRobot
};
