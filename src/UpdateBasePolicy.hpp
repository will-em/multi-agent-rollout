#pragma once
#include "Environment.hpp"
#include <vector>
/**
    Updates the base policy of an agent 

    @param env Reference to an environment 
    @param targets Reference to a vector of agent positions
    @param hasUpdatedTarget Reference to a boolean vector that, for each agent, tells you whether or not the agent has updated its target
    @param basePolicies Reference to the current set of base policies

*/
void updateBasePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<bool> &hasUpdatedTarget, std::vector<std::vector<int>> &basePolicies, int iteration);