#pragma once
#include <vector>
#include "Environment.hpp"
/**
    Converts a flattened position index into to a 2D-coordinate

    @param costs Reference to a vector containing costs
    @param agentIdx Index of the agents
    @param targets Reference to a vector of agent positions
    @param env Reference to an environment 
    @return The resulting control
*/
int costsToControl(std::vector<double> &costs, int agentIdx, std::vector<std::pair<int,int>> &targets, Environment &env);