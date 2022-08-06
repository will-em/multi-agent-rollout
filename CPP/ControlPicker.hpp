#pragma once
#include "Environment.hpp"
#include <vector>
/**
    Approximates the optimal set of controls for a given environment

    @param env Reference to an environment 
    @param targets Reference to a vector of agent positions
    @param dropOffPoints A vector of pairs of integers representing drop off points
    @param agentOrder A reference to a vector of agent indices
    @param basePolicies Reference to the current set of base policies
    @return A set of controls

*/
std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> dropOffPoints, std::vector<int> &agentOrder, bool freeze = false);
