import { Project} from "@/app/components/project/interfaces";

export const symbolicPlanner: Project = {
    title: "Symbolic Planner",
    date: "Spring 2024", // Removed trailing space
    media: "/media/images/symbolicPlanner.png",
    githubLink: "https://github.com/your-username/symbolic-planner",
    tags: ["AI", "Planning", "A*", "C++", "Algorithm"],
    section: [
        {
            title: "Introduction to Symbolic Planning",
            navName: "Introduction",
            navRef: "introduction",
            content: [
                {
                    type: "text",
                    content: [
                        "This project involved the development of a symbolic planner, a general-purpose Artificial Intelligence tool designed to solve problems using abstract symbolic representations. Unlike traditional planners that might operate in a grid-based or continuous physical space, a symbolic planner deals with symbols, their properties, and the logical relationships between them.",
                        "The planner takes a detailed environment description as input, which specifies the initial state of the world, the desired goal state (what the system should achieve), and a set of available actions that can alter the state. The planner's objective is to generate a sequence of these actions, known as a plan, that, when executed, will transition the system from its initial state to the specified goal state. This type of planner is a core component in many AI systems, enabling them to reason about tasks and achieve complex objectives through deliberate action sequences."
                    ]
                },
                {
                    type: "image",
                    content: "/media/images/symbolicPlanner.png",
                    altContent: "Symbolic Planner Concept",
                    subtitle: "Conceptual overview of a symbolic planning process."
                }
            ],
        },
        {
            title: "Planner Design and Core Algorithm",
            navName: "Planner Design",
            navRef: "planner-design",
              content: [
                  {
                  type: "text",
                  content: [
                    "The planner's design is centered around the A* search algorithm, a widely used and effective pathfinding technique known for its efficiency and optimality guarantees under certain conditions (i.e., with an admissible heuristic). A* is particularly suitable for symbolic planning because it intelligently combines two key factors when exploring the vast space of possible solutions.",
                    "First, it considers the actual cost incurred to reach a given state from the initial state (often denoted as 'gScore' or g(n)). Second, it employs a heuristic function to estimate the remaining cost to reach the goal state from the current state (often 'hScore' or h(n)). This combination, f(n) = g(n) + h(n), allows the algorithm to prioritize the exploration of paths that appear most promising, making A* significantly more efficient than uninformed search methods like breadth-first or depth-first search. The planner efficiently manages and explores the state space by maintaining an open set (priority queue) of states to visit, ordered by their f-scores."
                  ]
                },
                 ],
        },
        {
            title: "A* Search Implementation Details",
            navName: "A* Implementation",
            navRef: "a-star-implementation",
            content: [
                {
                    type: "text",
                    content: "The A* search algorithm is implemented using two critical functions: 'is_goal()' and 'heuristic()'. The 'is_goal()' function determines if the current state satisfies all specified goal conditions. It achieves this by comparing the set of 'GroundedCondition' objects (propositions that are true in the current state) with those defined in the goal state. The 'heuristic()' function estimates the 'distance' or cost to reach the goal from a given state. In our implementation, this is typically done by counting how many goal conditions are not yet met or have a different truth value in the current state compared to the goal state. A lower heuristic value indicates that a state is heuristically closer to achieving the goal."
                },
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `// Checks if all goal conditions are present in currNode's state.
bool is_goal(const Node &currNode) {
    // Compares currNode.current_state.grounded_conditions 
    // with goal.grounded_conditions.
    // Returns true if all goal conditions are met.
    // ...
}

// Estimates cost to reach goal from 'state'.
int heuristic(const State &state) {
    // Compares state.grounded_conditions with goal.grounded_conditions.
    // Counts mismatches or unmet conditions.
    // Returns (total_goal_conditions - num_matched_true_conditions).
    // ...
}`,
                    subtitle: "Core A* functions: goal check and heuristic (simplified)",
                },
            ],
        },
       {
            title: "State Expansion Mechanism",
            navName: "State Expansion",
            navRef: "state-expansion",
            content: [
                {
                    type: "text",
                    content: [
                        "The 'expand_state()' function is responsible for generating all valid successor states from a given 'currState'. It iterates through each defined 'Action' available in the planning domain. For each action, it first generates all possible argument permutations using the 'get_action_arg_permutations()' helper function, which typically leverages 'std::next_permutation' over the set of available symbols in the environment.",
                        "For each resulting permutation (a grounded action with concrete arguments), the planner generates its specific 'grounded_preconditions' and 'effects' using dedicated helper functions. If the 'check_action_conditions()' function confirms that all preconditions of the grounded action are met in the 'currState', then 'apply_action()' is called. The 'apply_action()' function creates a new state by taking a copy of 'currState's conditions and updating them based on the action's effects. This involves adding positive effects (conditions that become true) and removing conditions that are negated by negative effects."
                    ]
                },
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `// Generates all successor states from currState.
vector<State> expand_state(State &currState) {
    vector<State> expanded_states;
    for (const Action &action : actions) {
        // Get or generate permutations for action arguments
        vector<vector<string>> perm_action_args = 
            get_action_arg_permutations(action);

        for (const vector<string> perm_action_arg : perm_action_args) {
            // Map permutation to action's symbolic arguments
            unordered_map<string, string> action_args_map = /* ... */;
            GroundedAction generated_action(action.get_name(), /* ... */);

            vector<GroundedCondition> preconditions = 
                generate_pre_conditions(action_args_map, action);

            if (check_action_conditions(preconditions, currState)) {
                vector<GroundedCondition> effects = 
                    generate_effects(action_args_map, action);
                State new_state = apply_action(effects, generated_action, currState);
                expanded_states.push_back(new_state);
            }
        }
    }
    return expanded_states;
}

// Applies action effects to produce a new state.
State apply_action(const vector<GroundedCondition> &effects, /*...*/) {
    // new_conditions = currState.grounded_conditions;
    // For each effect:
    //  if !effect.get_truth(): remove opposite from new_conditions
    //  else: insert effect into new_conditions
    // return State(new_conditions, action);
    // ...
}

// Helper: bool check_action_conditions(preconditions, currState) - checks if all preconditions hold.
// Helper: vector<GroundedCondition> generate_pre_conditions(map, action) - grounds preconditions.
// Helper: vector<GroundedCondition> generate_effects(map, action) - grounds effects.
`,
                    subtitle: "State expansion and action application (simplified)",
                },
            ],
        },
        {
            title: "Action Grounding and Precondition Checking",
            navName: "Action Permutations",
            navRef: "action-permutations",
            content: [
                 {
                    type: "text",
                    content: "To apply actions effectively, the planner must first determine all valid ways an action's symbolic arguments can be instantiated with concrete symbols from the environment. This process is known as grounding. The 'get_action_arg_permutations()' function handles this by generating all unique combinations of symbols that match the arity (number of arguments) of the action, typically using 'std::next_permutation' for systematic generation. Before a grounded action (an action with all its symbolic arguments replaced by concrete symbols) is applied, the 'check_action_conditions()' function verifies that all of its preconditions are satisfied in the current state. This crucial step ensures that only valid actions are considered during the state expansion process, preventing the exploration of illegal or irrelevant paths in the search space."
                },
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `// Checks if all pre_conditions are met in currState.
bool check_action_conditions(vector<GroundedCondition> &pre_conditions, State currState) {
    // For each pre_condition:
    //  Check if it exists in currState.grounded_conditions with the same truth value.
    //  If not, return false.
    // Return true if all are met.
    // ...
}

// Generates all unique symbol permutations for an action's arguments.
vector<vector<string>> get_action_arg_permutations(const Action &action) {
    // int perm_size = action.get_args().size();
    // vector<string> symbols_vec(symbols.begin(), symbols.end());
    // Use std::next_permutation to generate permutations of 'perm_size' from symbols_vec.
    // Store unique permutations in a set, then convert to vector.
    // ...
}`,
                    subtitle: "Action applicability and argument permutation (simplified)",
                },
            ],
        },
         {
            title: "Demonstration: Solved Scenarios",
            navName: "Solved Scenarios",
            navRef: "solved-scenarios",
            content: [
               {
                    type: "text",
                    content: "The following scenarios demonstrate the symbolic planner's capability to solve different types of planning problems. Each scenario is defined by an environment file specifying the available symbols, initial conditions, goal conditions, and a set of permissible actions. The planner successfully generates a sequence of actions (a 'plan') that transforms the initial conditions to meet the goal conditions for each case."
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Environment: Blocks.txt"
                },
                {
                    type: "code",
                    codeLang: "text",
                    content: `
***** Environment *****

Symbols: Table,B,C,A,
Initial conditions: Clear(C) Clear(A) Block(C) Block(B) Block(A) On(C,Table) On(B,Table) On(A,B)
Goal conditions: On(A,Table) On(C,A) On(B,C)
Actions:
Move(b,x,y)
Precondition: On(b,x) Clear(b) Clear(y) Block(b) Block(y)
Effect: On(b,y) Clear(x) !On(b,x) !Clear(y)

MoveToTable(b,x)
Precondition: On(b,x) Clear(b) Block(b) Block(x)
Effect: On(b,Table) Clear(x) !On(b,x)

***** Environment Created! *****

Plan:
MoveToTable(A,B)
Move(C,Table,A)
Move(B,Table,C)
                    `,
                     subtitle: "Blocks World Scenario Output",
                },
                {
                  type: "text",
                  content: "The 'Blocks.txt' scenario is a classic problem in AI planning, involving the rearrangement of blocks on a table to achieve a target configuration. The planner correctly identifies that block A must first be moved to the table to clear block B. Subsequently, it moves block C from the table onto A, and finally block B from the table onto C, successfully satisfying all goal conditions."
                },
               {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Environment: BlocksTriangle.txt"
                },
                 {
                    type: "code",
                    codeLang: "text",
                   content: `
***** Environment *****

Symbols: Table,T0,B4,B3,B2,B1,T1,B0,
Initial conditions: NotTable(B2) NotTable(B0) Clear(T1) Triangle(T1) On(B1,B4) 
NotTable(T0) Block(B1) On(B2,Table) Block(B3) On(B3,B2) On(T0,B0) NotTable(B4) 
On(B4,Table) NotTable(T1) On(T1,B3) Clear(T0) Block(B4) NotTable(B3) NotTable(B1) 
Block(B0) Block(B2) On(B0,B1) Triangle(T0)

Goal conditions: On(T1,B0) On(B1,B3) On(B0,B1)
Actions:
Move(x,y,z)
Precondition: On(x,y) Clear(z) NotTable(x) Block(z) Clear(x) NotTable(z)
Effect: Clear(y) On(x,z) !Clear(z) !On(x,y)

MoveToTable(x,y)
Precondition: On(x,y) NotTable(x) Clear(x) NotTable(y) Block(y)
Effect: On(x,Table) Clear(y) !On(x,y)

***** Environment Created! *****

Plan:
MoveToTable(T1,B3)
MoveToTable(T0,B0)
MoveToTable(B0,B1)
Move(B1,B4,B3)
Move(B0,Table,B1)
Move(T1,Table,B0)
                    `,
                     subtitle: "Blocks Triangle Scenario Output",
                },
                 {
                  type: "text",
                  content: "The 'BlocksTriangle.txt' scenario presents a more complex variation of the blocks world problem, where blocks (B0-B4) and triangles (T0-T1) must be arranged into a specific stacked formation. The planner demonstrates its ability to deconstruct existing stacks and reconstruct new ones, such as moving T1 and T0 to the table before correctly positioning B1 on B3, B0 on B1, and finally T1 on B0."
                },
                {
                    type: "text",
                    displayAs: "subtitle",
                    content: "Environment: FireExtinguisher.txt"
                },
                {
                    type: "code",
                    codeLang: "text",
                    content: `
***** Environment *****

Symbols: R,Q,F,W,E,B,D,C,A,
Initial conditions: Quad(Q) At(Q,B) Rob(R) At(R,A) Loc(F) HighCharge(Q) Loc(E) Fire(F) 
InAir(Q) EmptyTank(Q) Loc(C) Loc(B) Loc(D) Loc(A) Loc(W)
Goal conditions: ExtThree(F)
Actions:
PourThrice(x)
Precondition: Fire(x) ExtTwo(x) At(Q,x) InAir(Q) FullTank(Q) HighCharge(Q)
Effect: !ExtTwo(x) ExtThree(x) EmptyTank(Q) !HighCharge(Q) !FullTank(Q) LowCharge(Q)

PourTwice(x)
Precondition: Fire(x) At(Q,x) InAir(Q) ExtOne(x) FullTank(Q) HighCharge(Q)
Effect: !ExtOne(x) ExtTwo(x) EmptyTank(Q) !HighCharge(Q) !FullTank(Q) LowCharge(Q)

PourOnce(x)
Precondition: Fire(x) At(Q,x) InAir(Q) FullTank(Q) HighCharge(Q)
Effect: ExtOne(x) EmptyTank(Q) !HighCharge(Q) !FullTank(Q) LowCharge(Q)

FillWater(x)
Precondition: Quad(x) OnRob(x) EmptyTank(x) At(R,W) At(Q,W)
Effect: !EmptyTank(x) FullTank(Q)

LandOnRob(z)
Precondition: Loc(z) At(R,z) At(Q,z) InAir(Q)
Effect: !InAir(Q) OnRob(Q)

Charge(x)
Precondition: Quad(x) LowCharge(x) OnRob(x)
Effect: !LowCharge(x) HighCharge(x)

TakeOffFromRob(z)
Precondition: Loc(z) At(R,z) At(Q,z) HighCharge(Q) OnRob(Q)
Effect: InAir(Q) !OnRob(Q)

MoveTogether(x,y)
Precondition: Loc(x) Loc(y) At(R,x) At(Q,x) OnRob(Q)
Effect: !At(R,x) !At(Q,x) At(R,y) At(Q,y)

MoveToLoc(x,y)
Precondition: Loc(x) Loc(y) At(R,x) InAir(Q)
Effect: At(R,y) !At(R,x)

***** Environment Created! *****

Plan:
MoveToLoc(A,B)
LandOnRob(B)
MoveTogether(B,W)
FillWater(Q)
MoveTogether(W,F)
TakeOffFromRob(F)
PourOnce(F)
LandOnRob(F)
Charge(Q)
MoveTogether(F,W)
FillWater(Q)
MoveTogether(W,F)
TakeOffFromRob(F)
PourTwice(F)
LandOnRob(F)
Charge(Q)
MoveTogether(F,W)
FillWater(Q)
MoveTogether(W,F)
TakeOffFromRob(F)
PourThrice(F)
                    `,
                     subtitle: "Fire Extinguisher Scenario Output",
                },
                {
                  type: "text",
                  content: "The 'FireExtinguisher.txt' scenario models a more intricate environment requiring an agent (a quadcopter, Q, carried by a robot, R) to perform a sequence of tasks to extinguish a fire at location F. This involves moving to different locations (A, B, W, F), landing/taking off from the robot, filling its water tank at W, charging its battery, and pouring water on the fire in three stages (PourOnce, PourTwice, PourThrice). The planner successfully generates a lengthy plan involving multiple trips for refilling water and recharging, demonstrating its ability to handle complex sequences and resource management (charge, water tank level)."
                }
            ],
        },
        {
            title: "Performance Evaluation and Analysis",
            navName: "Performance",
            navRef: "performance-evaluation",
            content: [
                {
                    type: "text",
                    content: "The performance of the symbolic planner was rigorously evaluated across a variety of simulated environments and problem complexities. We conducted tests both with and without the heuristic function enabled to directly assess its impact on search efficiency. The primary evaluation metrics included:"
                },
                {
                    type: "text",
                    displayAs: "list",
                    orderedList: true,
                    content: [
                        "The number of states expanded during the search (a measure of computational cost and search space explored).",
                        "The length of the final plan generated (the number of actions in the solution, indicating plan conciseness).",
                        "The total wall clock time required to find a solution (or determine that none exists within reasonable limits)."
                    ]
                },
                {
                    type: "text",
                    content: "These metrics allowed for a quantitative comparison of the planner's performance under different configurations and highlighted the crucial role of the heuristic in guiding the search and significantly reducing the number of expanded states and overall planning time. The results consistently demonstrated that a well-designed heuristic dramatically improves search efficiency."
                }
            ],
        },
        {
            title: "Conclusion and Future Work",
            navName: "Conclusion",
            navRef: "conclusion", // Standardized navRef (was "Conclusion" before)
            content: [
                {
                    type: "text",
                    content: [
                        "In conclusion, the symbolic planner, implemented using the A* search algorithm, effectively addresses a range of planning problems within symbolic environments. The project successfully demonstrated that a well-chosen heuristic function plays an extremely important role in enhancing search efficiency by intelligently guiding the algorithm towards the goal state, thereby reducing computational overhead.",
                        "However, performance in highly complex environments with large state spaces made it apparent that the careful design or selection of heuristics is crucial for scalability. This project serves as a practical implementation of fundamental AI planning algorithms and underscores the challenges involved. It also emphasizes the continuing need for research into more sophisticated heuristic design, advanced search techniques, or alternative planning paradigms (e.g., HTN planning, planning graphs) for addressing increasingly complex state spaces and real-world applications. Future explorations could involve developing domain-specific heuristics automatically or integrating machine learning techniques to learn better heuristic functions."
                    ]
                },
            ],
        },
    ],
};
