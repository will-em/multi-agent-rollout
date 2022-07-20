#include "CostsToControl.hpp"
#include "BasePolicy.hpp"
#include "IndexToPair.hpp"

int costsToControl(std::vector<double> &costs, int agentIdx, std::vector<std::pair<int,int>> &targets, Environment &env){
        // Pick the control with the lowest cost
        int lowestCostIdx = 0;
        for(size_t i = 0; i < 5; ++i){
            if(costs[i] < costs[lowestCostIdx])
                lowestCostIdx = i;
        }

        auto heuristic = basePolicy(env, targets, agentIdx);

        // Find if there are several controls with the same lowest cost
        std::vector<int> lowestCostIndices;
        for(size_t i = 0; i < 5; ++i){
            if(costs[i] == costs[lowestCostIdx]){
                if(i == heuristic[heuristic.size() - 1])
                    return i;
                lowestCostIndices.push_back(i);
            }
        }

        int agentMatrixIndex = env.getMatrixIndex(agentIdx);
        auto agentPos = indexToPair(agentMatrixIndex, env.getWidth());

        std::vector<int> distances;

        // Find out which one of these controls moves the agent closest to its target
        for(int control : lowestCostIndices){
            auto newPos = agentPos; 
            switch(control){
                case 1: // Move up
                    newPos.first = agentPos.first - 1;
                    break;
                case 2: // Move down
                    newPos.first = agentPos.first + 1;
                    break;
                case 3: // Move left
                    newPos.second = agentPos.second - 1;
                    break;
                case 4: // Move right
                    newPos.second = agentPos.second + 1;
                    break;
            }

            int newPosEl = env.envMat(newPos.first, newPos.second);

            // Calculate Manhattan distance
            int distance;

            // If control is standing still or the new position is a wall or an undesired box
            int dx, dy;
            if(control == 0 || newPosEl == 1 || (newPosEl == 2 && newPos != targets[agentIdx])){
                dx = agentPos.first - targets[agentIdx].first;
                dy = agentPos.second - targets[agentIdx].second;
            }
            else{
                dx = newPos.first - targets[agentIdx].first;
                dy = newPos.second - targets[agentIdx].second;
            }

            distance = abs(dx) + abs(dy);

            distances.push_back(distance);
        }


        // Pick the control with the lowest distance 
        int lowestDistanceIdx = 0;
        for(size_t i = 0; i < distances.size(); ++i){

            if(distances[i] < distances[lowestDistanceIdx])
                lowestDistanceIdx = i;
        }

        // Find if there are several controls with the same lowest distance 
        std::vector<int> lowestDistanceIndices;
        for(size_t i = 0; i < distances.size(); ++i){
            if(distances[i] == distances[lowestDistanceIdx]){
                if(lowestCostIndices[i] == 0) // Standing still
                    return 0;
                lowestDistanceIndices.push_back(lowestCostIndices[i]);
            }
        }


        // If there is a tie, i.e lowestCostIndices.size() > 1, just take a random one out of them
        int control = lowestDistanceIndices[rand() % lowestDistanceIndices.size()];

        return control;
}