#pragma once
#include <vector>
#include "Environment.hpp"
#include <unordered_map>

/**
    Calculates the heuristic policy for a given agent

    @param env Reference to an environment 
    @param targets Reference to a vector of agent positions
    @param agentIdx Index of the agent
    @return A vector containing the resulting sequence of controls 
*/

std::vector<int> basePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, int agentIdx, char* paths, std::unordered_map<int,int> &posToTargetIdx);