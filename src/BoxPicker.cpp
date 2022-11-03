#include "BoxPicker.hpp"
#include <random>

void boxPicker(Environment &env, std::vector< std::pair<int,int> > &targets, int agentIdx){
    int height = env.getHeight();
    int width = env.getWidth();
    int* matrix = env.getMatPtr();



    std::vector<std::pair<int,int>> &availableBoxes = env.getAvailableBoxes();
    if(targets.empty()){ // Initialization of targets
        // Shuffle the targets

        std::vector<std::pair<int,int>> boxes = availableBoxes;
        auto rd = std::random_device {}; 
        auto rng = std::default_random_engine {rd()};
        std::shuffle(std::begin(boxes), std::end(boxes), rng);

        for(size_t i = 0; i < env.getNumOfAgents(); i++){
            auto boxPos = boxes[i];
            targets.push_back(boxPos);
            availableBoxes.erase(std::remove(availableBoxes.begin(), availableBoxes.end(), boxPos), availableBoxes.end());
        }
    }
    else{ // Find the first box that is not the target of an other agent

        if(availableBoxes.empty()){ // No boxes left to pick up
			env.forceFinish();
        }else{
            auto boxPos = availableBoxes[rand() % availableBoxes.size()];
            targets[agentIdx] = boxPos;
            availableBoxes.erase(std::remove(availableBoxes.begin(), availableBoxes.end(), boxPos), availableBoxes.end());
        }
    }
}
