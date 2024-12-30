import { Project} from "@/app/components/project/interfaces";

export const symbolicPlanner: Project = {
    title: "Symbolic Planner",
    subtitle: "Spring 2024 ",
    media: "/media/images/symbolicPlanner.png", 
    tags: ["AI", "Planning", "A*", "C++", "Algorithm"],
    section: [
        {
            title: "Introduction",
            navName: "Introduction",
            navRef: "Introduction",
            content: [
                {
                    type: "text",
                    content:
                        "This project involved the development of a symbolic planner, a general-purpose AI tool designed to solve problems using symbolic representations. Unlike traditional planners that might operate in a grid or continuous space, a symbolic planner deals with abstract symbols and relationships between them. The planner takes an environment description as input, detailing the initial state, the goal state (what the system should achieve), and the available actions. The planner then generates a sequence of these actions, called a plan, that, when executed, will transition the system from the initial state to the goal state. This type of planner is a core component in many AI systems, enabling them to reason about and achieve complex goals.",
                },
            ],
        },
        {
            title: "Planner Design",
            navName: "Planner Design",
            navRef: "Planner Design",
              content: [
                  {
                  type: "text",
                  content:
                    "The planner's design is centered around the A* search algorithm, a widely used pathfinding technique known for its efficiency and completeness. A* is particularly suitable for symbolic planning because it combines two key factors when exploring possible solutions. First, it considers the actual cost of reaching a given state (the `gScore`). Second, it uses a heuristic function to estimate the cost of reaching the goal from that state (the `hScore`). This combination allows the algorithm to prioritize exploration of the most promising paths, making A* far more efficient than uninformed search methods like breadth-first or depth-first search. The planner efficiently manages and explores the state space using A* logic.",
                },
                 ],
        },
        {
            title: "A* Search Implementation",
            navName: "A* Implementation",
            navRef: "A* Implementation",
            content: [

                {
                    type: "code",
                    codeLang: "cpp",
                    content: `/**
     * @brief Checks if the state of the given node satisfies the goal conditions.
     *
     * This function iterates through the goal conditions and checks if each
     * goal condition is present in the current node's state. If any goal
     * condition is not found in the current state, the function returns
     * false. Otherwise, if all goal conditions are present, the function
     * returns true.
     *
     * @param currNode The node whose state needs to be checked.
     * @return True if the state satisfies all goal conditions, false otherwise.
     */
    bool is_goal(const Node &currNode)
    {
        // Get the set of goal conditions.
        unordered_set<GroundedCondition, GroundedConditionHasher,
        GroundedConditionComparator> goal_conditions = goal.grounded_conditions;
        // Get the set of current conditions from the node's state.
        unordered_set<GroundedCondition, GroundedConditionHasher,
        GroundedConditionComparator> current_conditions =
        currNode.current_state.grounded_conditions;
        // Iterate through each goal condition.
        for (const GroundedCondition &goal_condition : goal_conditions)
        {
            // If a goal condition is not found in the current state,
            // the goal is not met.
            if (current_conditions.find(goal_condition) ==
            current_conditions.end())
            {
                return false;
            }
        }
        // All goal conditions are satisfied, return true.
        return true;
    }
    /**
     * @brief Calculates the heuristic value of the given state based on the goal
     * conditions.
     *
     * This function computes the heuristic value by counting the number of
     * grounded conditions in the current state that match the goal conditions
     * with the same truth value. The heuristic is then calculated as the
     * difference between the total number of goal conditions and the
     * number of matching true conditionals. This value estimates the
     * distance to the goal, with a lower value indicating a state closer
     * to the goal.
     *
     * @param state The state for which to calculate the heuristic.
     * @return An integer representing the heuristic value.
     */
    int heuristic(const State &state)
    {
        // Get the set of goal conditions.
        unordered_set<GroundedCondition, GroundedConditionHasher,
        GroundedConditionComparator> goal_conditions = goal.grounded_conditions;
        // Get the set of current state conditions.
        unordered_set<GroundedCondition, GroundedConditionHasher,
        GroundedConditionComparator> state_conditions = state.grounded_conditions;
        int true_conditionals = 0;
         // Iterate through each condition in the state.
        for (const GroundedCondition &state_condition : state_conditions)
        {
            // Find the equivalent goal condition in the goal set.
            auto goal_equivalent_condition = goal_conditions.find(state_condition);
            // If an equivalent goal condition is found.
            if (goal_equivalent_condition != goal_conditions.end())
            {
                // If the truth values of the state condition and the goal
                // condition are the same, increment the true conditionals count.
                if (state_condition.get_truth() ==
                goal_equivalent_condition->get_truth())
                {
                    true_conditionals += 1;
                }
            }
        }
        // Return the heuristic value based on the number of unmatched
        // goal conditions.
        return (goal_conditions.size() - true_conditionals);
    }`,
                    subtitle: "Core A* functions: goal check and heuristic",
                },
                {
                    type: "text",
                    content:
                        "The A* search is implemented using two key functions: `is_goal()` and `heuristic()`.  `is_goal()` checks if the current state satisfies all goal conditions by comparing the grounded conditions of the current state with those of the goal state. The `heuristic()` function is used to estimate the distance from the current state to the goal. It uses the number of matching true conditions between the two states, providing a measure of how close the planner is to reaching its goal.",
                },
            ],
        },
       {
            title: "State Expansion",
            navName: "State Expansion",
            navRef: "State Expansion",
            content: [
                 {
                    type: "code",
                    codeLang: "cpp",
                    content: `/**
    /**
     * @brief Applies an action to the current state and returns the new state.
     *
     * This function takes the effects of an action and applies them to the
     * current state to produce a new state.  Positive effects are added to
     * the state and negative effects remove any opposite conditions.
     *
     * @param effects A vector of grounded effects of the action.
     * @param action The grounded action that is being applied.
     * @param currState The current state to apply the action on.
     * @return A new State object reflecting the result of the action.
     */
    State apply_action(const vector<GroundedCondition> &effects,
    GroundedAction &action, State &currState)
    {
        // Create a new set of conditions based on the current state.
        unordered_set<GroundedCondition, GroundedConditionHasher,
        GroundedConditionComparator> new_conditions(currState.grounded_conditions);
        // Iterate through each effect of the action.
        for (const GroundedCondition &effect : effects)
        {
            // If the effect is negative (i.e., removes a condition).
            if (!effect.get_truth())
            {
                 // Create the opposite condition (with the opposite truth value).
                GroundedCondition opposite = GroundedCondition(
                    effect.get_predicate(), effect.get_arg_values(),
                    !effect.get_truth());
                // Check if the opposite condition exists in the new state.
                auto exisiting_effect = new_conditions.find(opposite);
                if (exisiting_effect != new_conditions.end())
                {
                    // Remove the existing (opposite) condition.
                    new_conditions.erase(exisiting_effect);
                }
            }
            else
            {
                // If the effect is positive, add it to the new conditions.
                new_conditions.insert(effect);
            }
        }
        // Return the new state with the updated conditions.
        return State(new_conditions, action);
    }
    /**
     * @brief Generates the grounded preconditions for a specific action
     * using an argument map.
     *
     * This function takes a map of action arguments to symbols and an action.
     * It iterates through the preconditions of the action and creates a
     * grounded precondition by substituting the symbolic arguments with
     * concrete symbols from the provided map.
     *
     * @param action_args_map A map of action argument names to symbol values.
     * @param action The action for which to generate preconditions.
     * @return A vector of grounded preconditions.
     */
    vector<GroundedCondition> generate_pre_conditions(
        unordered_map<string, string> &action_args_map, Action action)
    {
        vector<GroundedCondition> grounded_preconditions;
        // Iterate through the preconditions of the action.
        for (const Condition &pre_condition : action.get_preconditions())
        {
            list<string> grounded_args;
            // Iterate through the arguments of the precondition.
            for (const string pre_condition_arg : pre_condition.get_args())
            {
                // Check if the argument has a concrete value in the map
                if (action_args_map.find(pre_condition_arg) !=
                action_args_map.end())
                {
                    // Add the concrete symbol to the grounded argument list.
                    grounded_args.push_back(action_args_map[pre_condition_arg]);
                }
                else
                {
                    // If no mapping exists, use the original argument.
                    grounded_args.push_back(pre_condition_arg);
                }
            }
            // Create a grounded precondition object.
            GroundedCondition grounded_precondition(pre_condition.get_predicate(),
            grounded_args, pre_condition.get_truth());
            // Add the grounded precondition to the result vector.
            grounded_preconditions.push_back(grounded_precondition);
        }
        return grounded_preconditions;
    }
    /**
     * @brief Generates the grounded effects for a specific action using an
     * argument map.
     *
     * This function takes a map of action arguments to symbols and an action.
     * It iterates through the effects of the action and creates grounded
     * effects by substituting the symbolic arguments with concrete
     * symbols from the provided map.
     *
     * @param action_args_map A map of action argument names to symbol values.
     * @param action The action for which to generate effects.
     * @return A vector of grounded effects.
     */
    vector<GroundedCondition> generate_effects(
        unordered_map<string, string> &action_args_map, Action action)
    {
        vector<GroundedCondition> effects;
        // Iterate through the ungrounded effects of the action.
        for (const Condition &ungrounded_effect : action.get_effects())
        {
            list<string> grounded_args;
            // Iterate through the arguments of the effect.
            for (const string pre_condition_arg : ungrounded_effect.get_args())
            {
                // Check if the argument has a concrete value in the map.
                if (action_args_map.find(pre_condition_arg) !=
                action_args_map.end())
                {
                    // Add the concrete symbol to the grounded argument list.
                    grounded_args.push_back(action_args_map[pre_condition_arg]);
                }
                else
                {
                    // If no mapping exists, use the original argument.
                    grounded_args.push_back(pre_condition_arg);
                }
            }
            // Create a grounded effect object.
            GroundedCondition grounded_effect(ungrounded_effect.get_predicate(),
            grounded_args, ungrounded_effect.get_truth());
            // Add the grounded effect to the result vector.
            effects.push_back(grounded_effect);
        }
        return effects;
    }
    /**
     * @brief Expands the given state using a set of available actions.
     *
     * This function generates all possible successor states by applying
     * applicable actions to the given current state. It iterates through
     * all actions, generates all their possible permutations with symbols,
     * checks their applicability, and applies the applicable actions to
     * generate the expanded states.
     *
     * @param currState The current state to be expanded.
     * @return A vector of all the expanded states.
     */
    vector<State> expand_state(State &currState)
    {
        vector<State> expanded_states;
        // Iterate through each available action.
        for (const Action &action : actions)
        {
            // Check if permutations for this action have been generated
            // before.
            if (action_permutations_map.find(action.get_name()) ==
            action_permutations_map.end())
            {
                // If not, generate the permutations.
                action_permutations_map[action.get_name()] =
                    get_action_arg_permutations(action);
            }
            // Get the permutations of the action's arguments.
            vector<vector<string>> perm_action_args =
            action_permutations_map[action.get_name()];
            // Iterate through each permutation.
            for (const vector<string> perm_action_arg : perm_action_args)
            {
                // Create a map of action arguments to their corresponding
                // symbol values.
                unordered_map<string, string> action_args_map;
                list<string> env_action_args = action.get_args();
                auto perm_action_it = perm_action_arg.begin();
                for (const string env_action_arg : env_action_args)
                {
                     // Map the action argument to a symbol for the permutation.
                    action_args_map[env_action_arg] = *perm_action_it;
                    perm_action_it++;
                }
                // Create a grounded action.
                list<string> action_arg_list(perm_action_arg.begin(),
                perm_action_arg.end());
                GroundedAction generated_action(action.get_name(),
                action_arg_list);
                // Generate the preconditions for the action.
                vector<GroundedCondition> grounded_preconditions =
                    generate_pre_conditions(action_args_map, action);
                // Check if the action is applicable in the current state.
                if (check_action_conditions(grounded_preconditions, currState))
                {
                    // Generate the effects for the action.
                    vector<GroundedCondition> effects =
                    generate_effects(action_args_map, action);
                    // Apply the action to get the new state.
                    State new_state = apply_action(effects, generated_action,
                    currState);
                    // Add the new state to the expanded states.
                    expanded_states.push_back(new_state);
                }
            }
        }
        // Return the vector of expanded states.
        return expanded_states;
    }`,
                    subtitle: "Action application and state expansion",
                },
                {
                    type: "text",
                     content:
                        "The `expand_state()` function is responsible for exploring the state space, taking the current state, and producing all possible next states. This is done by iterating through all available actions. The function checks for applicability using the `check_action_conditions()` and then applies valid actions using the `apply_action()` function, thereby generating new successor states.  This iterative process forms the core of how the planner explores the problem.",
                },
            ],
        },
        {
            title: "Action Permutations",
            navName: "Action Permutations",
            navRef: "Action Permutations",
            content: [
                {
                    type: "code",
                    codeLang: "cpp",
                    content: `
    /**
    * @brief Checks if an action is applicable in the current state.
     *
     * This function verifies if all preconditions of an action are
     * satisfied in the given current state. It iterates through each
     * precondition and checks if it exists in the current state with the
     * same truth value. If any precondition is not found or has a
     * different truth value, the action is not applicable.
     *
     * @param pre_conditions A vector of grounded preconditions for the action.
     * @param currState The current state to check against.
     * @return True if all preconditions are satisfied, false otherwise.
     */
    bool check_action_conditions(vector<GroundedCondition> &pre_conditions,
    State currState)
    {
        // Get the set of grounded conditions from the current state.
        unordered_set<GroundedCondition, GroundedConditionHasher,
        GroundedConditionComparator> grounded_conditions =
        currState.grounded_conditions;
        // Iterate through each precondition.
        for (const GroundedCondition &pre_condition : pre_conditions)
        {
            // Try to find the precondition in the current state's
            // conditions.
            auto existing_condition = grounded_conditions.find(pre_condition);
            // If the precondition is not found or has a different
            // truth value, the action is not applicable.
            if (existing_condition == grounded_conditions.end() ||
            (existing_condition->get_truth() != pre_condition.get_truth()))
            {
                return false;
            }
        }
        // All preconditions are satisfied.
        return true;
    }
/**
     * @brief Generates all possible symbol permutations for an action's
     * arguments.
     *
     * This function generates all possible combinations of symbols
     * from a global set of symbols, with the combination size matching the
     * number of arguments for the given action. The function ensures that
     * each combination is unique.
     *
     * @param action The action for which to generate argument permutations.
     * @return A vector of vectors, where each inner vector represents a
     *         unique permutation of symbols for the action's arguments.
     */
    vector<vector<string>> get_action_arg_permutations(const Action &action)
    {
         // Get the number of arguments for the action.
        int perm_size = action.get_args().size();
        // Create a vector of symbols for the available set of symbols.
        vector<string> symbols_vec(symbols.begin(), symbols.end());
        // Sort the symbols vector
        sort(symbols_vec.begin(), symbols_vec.end());
        // Use a set to store unique permutations.
        set<vector<string>> permutations_set;
        // Generate all permutations of symbols.
        do
        {
            // Create a permutation by taking the first 'perm_size'
            // symbols from the vector.
            vector<string> perm(symbols_vec.begin(),
            next(symbols_vec.begin(), perm_size));
            // Insert the permutation into the set (ensuring uniqueness).
            permutations_set.insert(perm);
        } while (next_permutation(symbols_vec.begin(), symbols_vec.end()));

        // Convert the set of permutations to a vector for output.
        vector<vector<string>> permutations(permutations_set.begin(),
        permutations_set.end());
        return permutations;
    }
`,
                    subtitle: "Checking Action Applicability and Generating Argument Permutations",
                },
                {
                    type: "text",
                     content:
                        "The `get_action_arg_permutations()` function is responsible for generating all possible ways to apply an action given the available symbols, creating 'grounded' actions (i.e. where the symbolic variables are replaced by concrete values). This function uses C++'s `next_permutation` algorithm to generate all unique combinations of symbols. Before actions are applied, the `check_action_conditions()` function validates if the preconditions of a grounded action are met in the current state. It ensures that only valid actions are considered, preventing the exploration of invalid paths.",
                },
            ],
        },
         {
            title: "Solved Scenarios",
            navName: "Solved Scenarios",
            navRef: "Solved Scenarios",
            content: [
               {
                    type: "text",
                    content:
                    "The following scenarios demonstrate the planner's ability to solve different types of planning problems. Each scenario is defined by an environment, initial conditions, goal conditions, and a set of available actions. The planner is able to generate a sequence of actions (a 'plan') that transforms the initial conditions to meet the goal.",
                },
                {
                    type: "text",
                    content:
                        "**Environment: Blocks.txt**",
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
                     subtitle: "Blocks World Scenario",
                },
                {
                  type: "text",
                  content:
                   "The 'Blocks.txt' scenario is a classic problem involving rearranging blocks on a table. The planner correctly identifies that block A must be moved to the table to start, and then restructures the rest of the blocks on top of eachother to satisfy the goal."
                },
               {
                    type: "text",
                    content:
                        "**Environment: BlocksTriangle.txt**",
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
                     subtitle: "Blocks Triangle Scenario",
                },
                 {
                  type: "text",
                  content:
                   "The 'BlocksTriangle.txt' scenario is a more complex blocks world scenario where blocks are not simply on the table but can be stacked on each other. The planner demonstrates an ability to reorganize complex structures to match the goal, taking into consideration each block."
                },
                {
                    type: "text",
                    content:
                        "**Environment: FireExtinguisher.txt**",
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
                     subtitle: "Fire Extinguisher Scenario",
                },
                {
                  type: "text",
                  content:
                    "The 'FireExtinguisher.txt' scenario models a more complex environment that requires an agent (a quadcopter) to move to different locations to charge and fill a water tank to extinguish a fire. This scenario shows that the planner is able to handle more complex sequences of actions, including multiple refuels and charge steps before completing the task."
                }
            ],
        },
        {
            title: "Performance Evaluation",
            navName: "Performance Evaluation",
            navRef: "Performance Evaluation",
            content: [
                {
                    type: "text",
                    content:
                        "The performance of the symbolic planner was rigorously evaluated in a variety of simulated environments. We conducted tests both with and without the heuristic function to directly assess its impact on search efficiency. The primary evaluation metrics included the number of states expanded during the search (a measure of computation cost), the length of the final plan (the number of actions in the solution), and the total wall clock time required to find the solution (or determine that none exists). These metrics help us compare the performance of different configurations and determine the benefits and drawbacks of different design choices. The results from these tests highlighted the crucial role of the heuristic and its impact on the efficiency of search.",
                },
            ],
        },
        {
            title: "Conclusion",
            navName: "Conclusion",
            navRef: "Conclusion",
            content: [
                {
                    type: "text",
                    content:
                        "In conclusion, the symbolic planner, implemented using the A* algorithm, effectively addresses planning problems within symbolic environments. The project shows that a heuristic plays an extremely important role in enhancing the search efficiency, by guiding the algorithm towards the goal state. However, performance in complex environments made it apparent that careful design or selection of heuristics is crucial. This project successfully demonstrated the practical implementation and challenges of AI planning algorithms, and emphasized the continuing need for research into more sophisticated heuristic design or alternative search techniques for addressing complex state spaces. It serves as a basis for future explorations of more advanced AI planning methodologies.",
                },
            ],
        },
    ],
};
