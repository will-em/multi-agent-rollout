#include "BoxPicker.hpp"
#include <random>

void boxPicker(Environment &env, std::vector< std::pair<int,int> > &targets, int agentIdx){
    int height = env.getHeight();
    int width = env.getWidth();
    int* matrix = env.getMatPtr();

    std::vector<int> boxPositions;

    for(size_t i = 0; i < height * width; ++i){
        if(matrix[i] == 2){
            boxPositions.push_back(i);
        }
    }

    if(targets.empty()){ // Initialization of targets
        // Shuffle the targets
        auto rd = std::random_device {}; 
        auto rng = std::default_random_engine {rd()};
        std::shuffle(std::begin(boxPositions), std::end(boxPositions), rng);

        for(size_t i = 0; i < env.getNumOfAgents(); i++){
            auto boxPos = indexToPair(boxPositions[i], width);
            targets.push_back(boxPos);
        }
    }
    else{ // Find the first box that is not the target of an other agent

        std::vector<std::pair<int, int>> availableBoxPositions;
        for(auto boxIndex : boxPositions){
            auto boxPos = indexToPair(boxIndex, width);

            // Check that this box is not the target of another agent
            bool anotherAgentsTarget = false;
            for(auto target : targets){
                if(boxPos == target){
                    anotherAgentsTarget = true;
                    break;
                }
            }

            if(anotherAgentsTarget)
                continue;

            availableBoxPositions.push_back(boxPos);
        }



        if(availableBoxPositions.empty()){ // No boxes left to pick up
			env.forceFinish();
        }else{
            targets[agentIdx] = availableBoxPositions[rand() % availableBoxPositions.size()];
        }
    }
}
