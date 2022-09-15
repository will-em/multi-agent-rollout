#pragma once
#include "Environment.hpp"
#include <vector>

/**
    Updates the target of agents that have picked up or dropped of a box 

    @param env Reference to an environment 
    @param targets Reference to a vector of agent positions
    @param beforeValues Reference to a vector of matrix elements before executing the set of given controls
    @param afterValues Reference to a vector of matrix elements after executing the set of given controls 
    @returns A boolean vector that, for each agent, tells you whether or not the agent has updated its target

*/
std::vector<bool> updateTargets(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<int> &beforeValues, std::vector<std::pair<int, int>> dropOffPoints);