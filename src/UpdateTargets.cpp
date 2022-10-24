#include "UpdateTargets.hpp"
#include "BoxPicker.hpp"
#include "coop-astar.hpp"

std::vector<bool> updateTargets(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<int> &beforeValues, std::vector<std::pair<int, int>> dropOffPoints){
    std::vector<int> afterValues = env.getAgentValues();
    int numOfAgents = env.getNumOfAgents();

    std::vector<bool> hasUpdatedTarget(numOfAgents, false);

    // Update targets if necessary 
    for(size_t i = 0; i < numOfAgents; ++i){
        srand(i + 1); // AgentIdx + 1
        if(afterValues[i] > beforeValues[i]){ // Agent i picks up box
            std::pair<int,int> position = indexToPair(env.getMatrixIndex(i), env.getWidth());

            Node node_pos(position.first, position.second);
            //remove_from_obstacles(node_pos); 

            targets[i] = dropOffPoints[rand() % dropOffPoints.size()];
            hasUpdatedTarget[i] = true;
        }
        else if(afterValues[i] < beforeValues[i]){ // Agent i drops off box
            boxPicker(env, targets, i);
            hasUpdatedTarget[i] = true;
        }
    }

    return hasUpdatedTarget; // C++11 move
}