import { Project} from "@/app/components/project/interfaces";

export const symbolicPlanner: Project = {
    title: "Symbolic Planner",
    date: "Spring 2024",
    media: "/media/images/symbolicPlanner.png",
    tags: ["AI", "Planning", "A-Star", "C++"],
    section: [
        {
            title: "Introduction",
            navName: "Introduction",
            navRef: "introduction",
            content: [
                {
                    type: "text",
                    content:
                        "A symbolic planner is an AI system that figures out how to achieve a goal by reasoning about actions and their effects. Given a starting state, a goal, and a set of possible actions, the planner finds a sequence of actions that transforms the start into the goal.",
                },
                {
                    type: "text",
                    content:
                        "For example, in the classic 'blocks world' problem, you might have blocks stacked in one configuration and want them in another. The planner determines which blocks to move and in what order.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Key Components",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**Initial state** — what's true at the start (e.g., Block A is on Block B)",
                        "**Goal state** — what should be true at the end (e.g., Block A is on the table)",
                        "**Actions** — operations that change the state (e.g., Move block X from Y to Z)",
                        "**Plan** — the output sequence of actions that achieves the goal",
                    ],
                },
                {
                    type: "image",
                    content: "/media/images/symbolicPlanner.png",
                    altContent: "Symbolic planner concept",
                    subtitle: "Symbolic planning process.",
                },
            ],
        },
        {
            title: "A-Star Search Algorithm",
            navName: "A-Star Search",
            navRef: "planner-design",
            content: [
                {
                    type: "text",
                    content:
                        "The planner uses A-Star search to find optimal plans. A-Star balances two factors: the cost already spent to reach a state (g-score) and an estimate of the remaining cost to the goal (h-score, the heuristic). States are explored in order of f = g + h, prioritizing paths that seem most promising.",
                },
                {
                    type: "text",
                    content:
                        "The heuristic counts how many goal conditions aren't yet satisfied. This is admissible (never overestimates) since each unsatisfied condition requires at least one action to fix. An admissible heuristic guarantees A-Star finds optimal solutions.",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**g-score** — number of actions taken so far",
                        "**h-score** — number of unsatisfied goal conditions (heuristic)",
                        "**f-score** — g + h, used to prioritize which state to explore next",
                    ],
                },
            ],
        },
        {
            title: "Implementation Details",
            navName: "Implementation",
            navRef: "a-star-implementation",
            content: [
                {
                    type: "text",
                    content:
                        "Two key functions drive the search: `is_goal()` checks if the current state satisfies all goal conditions, and `heuristic()` estimates how far we are from the goal by counting unsatisfied conditions.",
                },
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `bool is_goal(const Node &currNode) {
    // Check if all goal conditions exist in current state
    for (auto &goalCond : goal.grounded_conditions) {
        if (currNode.state.find(goalCond) == currNode.state.end())
            return false;
    }
    return true;
}

int heuristic(const State &state) {
    // Count unsatisfied goal conditions
    int unmet = 0;
    for (auto &goalCond : goal.grounded_conditions) {
        if (state.find(goalCond) == state.end()) unmet++;
    }
    return unmet;
}`,
                    subtitle: "Goal check and heuristic functions",
                },
            ],
        },
        {
            title: "State Expansion",
            navName: "State Expansion",
            navRef: "state-expansion",
            content: [
                {
                    type: "text",
                    content:
                        "State expansion generates all possible next states from the current state. For each action, the planner tries all valid argument combinations (grounding), checks if preconditions are met, and if so, applies the action's effects to create a new state.",
                },
                {
                    type: "text",
                    content:
                        "For example, a `Move(block, from, to)` action might be grounded as `Move(A, B, Table)`. If preconditions hold (A is on B, A is clear, Table is clear), the action creates a new state where A is now on Table.",
                },
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `vector<State> expand_state(State &currState) {
    vector<State> successors;
    for (const Action &action : actions) {
        // Try all argument permutations
        for (auto &args : get_action_arg_permutations(action)) {
            auto preconditions = ground_preconditions(action, args);
            if (check_conditions(preconditions, currState)) {
                auto effects = ground_effects(action, args);
                successors.push_back(apply_action(effects, currState));
            }
        }
    }
    return successors;
}`,
                    subtitle: "State expansion generates successor states",
                },
            ],
        },
        {
            title: "Action Grounding",
            navName: "Action Grounding",
            navRef: "action-permutations",
            content: [
                {
                    type: "text",
                    content:
                        "Actions are defined with symbolic parameters like `Move(b, x, y)`. Grounding replaces these with concrete objects: if we have blocks A, B, C and Table, then `Move(b, x, y)` grounds to `Move(A, B, Table)`, `Move(A, C, Table)`, etc.",
                },
                {
                    type: "text",
                    content:
                        "The planner generates all permutations of available symbols for each action's argument count, then filters by checking preconditions. Only valid groundings produce successor states.",
                },
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `bool check_conditions(vector<Condition> &preconditions, State &state) {
    for (auto &cond : preconditions) {
        bool found = state.conditions.count(cond) > 0;
        if (cond.is_positive() != found) return false;
    }
    return true;
}`,
                    subtitle: "Precondition checking filters invalid actions",
                },
            ],
        },
        {
            title: "Example: Blocks World",
            navName: "Blocks World",
            navRef: "solved-scenarios",
            content: [
                {
                    type: "text",
                    content:
                        "The classic blocks world problem demonstrates symbolic planning. Given blocks A, B, C initially stacked, the goal is to rearrange them into a target configuration.",
                },
                {
                    type: "code",
                    codeLang: "text",
                    content: `Initial: On(A,B) On(B,Table) On(C,Table) Clear(A) Clear(C)
Goal:    On(A,Table) On(C,A) On(B,C)

Actions:
  Move(b,x,y):     On(b,x) Clear(b) Clear(y) → On(b,y) Clear(x)
  MoveToTable(b,x): On(b,x) Clear(b) → On(b,Table) Clear(x)

Plan found:
  1. MoveToTable(A,B)  — A from B to Table
  2. Move(C,Table,A)   — C from Table to A
  3. Move(B,Table,C)   — B from Table to C`,
                    subtitle: "Blocks world: initial state, goal, and generated plan",
                },
                {
                    type: "text",
                    content:
                        "The planner recognizes that A must be moved first (it's blocking B), then builds the target stack from the bottom up.",
                },
            ],
        },
        {
            title: "Example: Fire Extinguisher",
            navName: "Fire Scenario",
            navRef: "fire-scenario",
            content: [
                {
                    type: "text",
                    content:
                        "A more complex scenario: a quadcopter must extinguish a fire by making multiple trips to fill water and recharge batteries. This demonstrates planning with resource management.",
                },
                {
                    type: "code",
                    codeLang: "text",
                    content: `Initial: Quad(Q) At(Q,B) At(R,A) Fire(F) InAir(Q) EmptyTank(Q) HighCharge(Q)
Goal:    ExtThree(F)  — fire fully extinguished after 3 pours

Key Actions:
  PourOnce/Twice/Thrice(x) — requires FullTank, HighCharge, empties tank
  FillWater(Q)             — fill tank at water location W
  Charge(Q)                — recharge battery while on robot
  MoveTogether(x,y)        — robot carries quadcopter between locations

Plan found (21 actions):
  MoveToLoc(A,B) → LandOnRob(B) → MoveTogether(B,W) → FillWater(Q)
  → MoveTogether(W,F) → TakeOffFromRob(F) → PourOnce(F)
  → LandOnRob(F) → Charge(Q) → MoveTogether(F,W) → FillWater(Q)
  → MoveTogether(W,F) → TakeOffFromRob(F) → PourTwice(F)
  → LandOnRob(F) → Charge(Q) → MoveTogether(F,W) → FillWater(Q)
  → MoveTogether(W,F) → TakeOffFromRob(F) → PourThrice(F)`,
                    subtitle: "Fire extinguisher scenario requires resource management",
                },
                {
                    type: "text",
                    content:
                        "The planner handles the constraint that the quadcopter can only pour once per tank and needs to recharge between pours, automatically generating a multi-trip plan.",
                },
            ],
        },
        {
            title: "Performance",
            navName: "Performance",
            navRef: "performance-evaluation",
            content: [
                {
                    type: "text",
                    content:
                        "The planner was tested with and without the heuristic to measure its impact on search efficiency.",
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Evaluation Metrics",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**States expanded** — how many states were explored before finding a solution",
                        "**Plan length** — number of actions in the solution",
                        "**Wall clock time** — total time to find the plan",
                    ],
                },
                {
                    type: "text",
                    content:
                        "With the heuristic enabled, the planner expanded significantly fewer states and found solutions faster. The heuristic's guidance is especially important in larger state spaces where uninformed search becomes intractable.",
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
                        "The symbolic planner successfully solves a variety of planning problems using A-Star search. The heuristic function proved critical for efficiency—without it, the planner struggles with larger state spaces.",
                },
                {
                    type: "text",
                    displayAs: "list",
                    content: [
                        "**A-Star with admissible heuristic** — guarantees optimal plans",
                        "**State expansion** — tries all valid action groundings from current state",
                        "**Scalability challenge** — large domains require better heuristics or alternative approaches",
                    ],
                },
            ],
        },
    ],
};
