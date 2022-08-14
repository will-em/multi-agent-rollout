#pragma once
#include "Environment.hpp"
#include <vector>
#include "IndexToPair.hpp"

/**
    Set target of an agent to a box. If the target vector is empty all of the agents are assigned a box.  

    @param env Reference to an environment 
    @param targets Reference to a vector of agent positions
    @param agentIdx Index of the agents

*/
void boxPicker(Environment &env, std::vector< std::pair<int,int> > &targets, int agentIdx);