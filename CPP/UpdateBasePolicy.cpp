#include "UpdateBasePolicy.hpp"
#include "BasePolicy.hpp"

void updateBasePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<bool> &hasUpdatedTarget, std::vector<std::vector<int>> &basePolicies){
    int numOfAgents = env.getNumOfAgents();

    for(size_t i = 0; i < numOfAgents; ++i){
        if(hasUpdatedTarget[i])
            basePolicies[i] = basePolicy(env, targets, i);
    }    
}