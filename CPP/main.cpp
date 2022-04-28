#include <iostream>
#include "Environment.hpp"
#include <math.h>   
#include <stdlib.h> 
#include "astar.cpp"
#include <thread>
#include <chrono>
#include <algorithm>
#include <random>


std::pair<int, int> indexToPair(int i, const int dim){
    // Convert flattened index to 2d coordinate
    int x = (int) floor(i / dim);
    int y = i % dim; 

    std::pair target = {x, y};
    return target;
}

std::vector<int> basePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, int agentIdx);

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
        auto agentPos = indexToPair(agentMatrixIndex, env.getDim());

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
            if(control == 0 || newPosEl == 1 || (newPosEl == 2 && newPos != targets[agentIdx])){
                distance = abs(agentPos.first - targets[agentIdx].first) + abs(agentPos.second - targets[agentIdx].second);
            }
            else{
                distance = abs(newPos.first - targets[agentIdx].first) + abs(newPos.second - targets[agentIdx].second);
            }

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

void boxPicker(Environment &env, std::vector< std::pair<int,int> > &targets, int agentIdx){
    const int dim = env.getDim();
    int* matrix = env.getMatPtr();

    std::vector<int> boxPositions;

    for(size_t i = 0; i < dim*dim; ++i){
        if(matrix[i] == 2){
            boxPositions.push_back(i);
        }
    }

    //Shuffle the box positions
    auto rd = std::random_device {}; 
    auto rng = std::default_random_engine {rd()};
    std::shuffle(std::begin(boxPositions), std::end(boxPositions), rng);
    

    if(targets.empty()){ // Initialization of targets
        for(size_t i = 0; i < env.getNumOfAgents(); i++){
            auto boxPos = indexToPair(boxPositions[i], dim);
            targets.push_back(boxPos);
        }
    }
    else{ // Find the first box that is not the target of an other agent
        bool updatedTarget = false;

        bool anotherAgentsTarget = true;


        for(auto boxIndex : boxPositions){
            auto boxPos = indexToPair(boxIndex, dim);

            // Check that this box is not the target of another agent
            anotherAgentsTarget = false;
            for(auto target : targets){
                if(boxPos == target){
                    anotherAgentsTarget = true;
                    break;
                }
            }

            if(anotherAgentsTarget)
                continue;

            targets[agentIdx] = boxPos;
            updatedTarget = true;
            break;
        }

        if(!updatedTarget){ // No boxes left to pick up
            targets[agentIdx] = std::pair<int, int>(1, 1 + agentIdx);
        }
    }
}


std::vector<int> basePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, int agentIdx){ 
    int dim = env.getDim();
    int* matrix = env.getMatPtr();

    float* obstacles = new float[dim * dim](); // Maybe remove outer wall
    int* path = new int[dim * dim]();    

    for(size_t i = 0; i < dim * dim; ++i){
        if(matrix[i] == 1 || matrix[i] == 2)
            obstacles[i] = std::numeric_limits<float>::max();
        else{
            obstacles[i] = 1.0f;
        }
    }

    obstacles[targets[agentIdx].second * dim + targets[agentIdx].first] = 1.0f;

    int startIdx = env.getMatrixIndex(agentIdx);

    auto goal = targets[agentIdx];
    int goalIdx = dim * goal.first + goal.second;

    bool foundPath = astar(obstacles, dim, dim, startIdx, goalIdx, false, path); 
    assert(foundPath);

    std::vector<int> controls;

    int index = goalIdx;

    if(index == startIdx){
        controls = std::vector<int>(1, 0);
    }else{
        while(index != startIdx){
            int prevIndex = path[index];
            
            // Calculate control
            auto curr = indexToPair(index, dim);
            auto prev = indexToPair(prevIndex, dim);

            int di = curr.first - prev.first;
            int dj = curr.second - prev.second;

            int control;
            if(di > 0){      // Move down
                control = 2;
            }
            else if(di < 0){ // Move up
                control = 1;
            }
            else if(dj > 0){ // Move right
                control = 4;
            }
            else{            // Move left
                control = 3;
            }

            controls.push_back(control);

            index = prevIndex;
        }
    }


    delete[] obstacles;
    delete[] path;
    return controls;
}

std::vector<bool> updateTargets(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<int> &beforeValues, std::vector<std::pair<int, int>> dropOffPoints){
    std::vector<int> afterValues = env.getAgentValues();
    int numOfAgents = env.getNumOfAgents();

    std::vector<bool> hasUpdatedTarget(numOfAgents, false);

    // Update targets if necessary 
    for(size_t i = 0; i < numOfAgents; ++i){
        if(afterValues[i] > beforeValues[i]){ // Agent i picks up box
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
void updateBasePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<bool> &hasUpdatedTarget, std::vector<std::vector<int>> &basePolicies){
    int numOfAgents = env.getNumOfAgents();

    for(size_t i = 0; i < numOfAgents; ++i){
        if(hasUpdatedTarget[i])
            basePolicies[i] = basePolicy(env, targets, i);
    }    
}
std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> dropOffPoints, std::vector<int> &agentOrder){

    int numOfAgents = env.getNumOfAgents(); 

    std::vector< std::vector<int> > preComputedBasePolicies(numOfAgents);
    // Get base policies for numOfAgents 
    for(size_t i = 1; i < numOfAgents; ++i){
        int agentIdx = agentOrder[i];
        auto basePolicyControls = basePolicy(env, targets, agentIdx);
        preComputedBasePolicies[agentIdx] = basePolicyControls;
    } 

    std::vector<int> optimizedControls;
    
    for(int agentIdx : agentOrder){
        std::vector<double> costs(5);
        for(size_t control = 0; control < 5; ++control){

            std::vector<std::pair<int, int>> simTargets(targets);
            std::vector<int> controls(numOfAgents);

            std::vector<std::vector<int>> basePolicies(preComputedBasePolicies);


            for(size_t i = 0; i < optimizedControls.size(); ++i){
                controls[i] = optimizedControls[i];
                basePolicies[i].clear();
            }

            controls[agentIdx] = control;

            Environment simEnv = env; // Copy environment
            
            double cost = 0.0;
            int iteration = 0; 

            std::vector<int> agentIdxBasePolicy;

            while(!simEnv.isDone() && iteration < 1000){

                for(size_t i = 0; i < numOfAgents; ++i){
                    int n = basePolicies[i].size();

                    if(n > 0){
                        controls[i] = basePolicies[i][n - 1];
                        basePolicies[i].pop_back();
                    }
                }

                auto beforeValues = simEnv.getAgentValues();

                cost += simEnv.step(controls, simTargets); 

                if(cost > 1000.0){
                    break;
                }

                // Update targets
                auto hasUpdatedTarget = updateTargets(simEnv, simTargets, beforeValues, dropOffPoints);
                updateBasePolicy(simEnv, simTargets, hasUpdatedTarget, basePolicies);

                for(size_t i = 0; i < numOfAgents; ++i){
                    if(basePolicies[i].empty())
                        basePolicies[i] = basePolicy(simEnv, targets, i); 
                }
                
                iteration++;
            }

            costs[control] = cost;
        }

        optimizedControls.push_back(costsToControl(costs, agentIdx, targets, env));
    }

    // Reorder optimizedControls
    std::vector<int> reorderedOptimizedControls(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        reorderedOptimizedControls[agentOrder[i]] = optimizedControls[i];
    return reorderedOptimizedControls;
}

bool simulate(int numOfAgents){ 
    int wallOffset = 5;
    int boxOffset = 4;
    int n = (int)ceil(sqrt((double)numOfAgents) / 2.0);

    Environment env(wallOffset, boxOffset, n, numOfAgents);

    std::pair<int, int> dropOff1 = {env.getDim() - 4, 3};
    std::pair<int, int> dropOff2 = {env.getDim() - 4, env.getDim() - 4};
    std::vector<std::pair<int, int>> dropOffPoints = {dropOff1};

    std::vector< std::pair<int, int> > targets;
    boxPicker(env, targets, -1); // Initialize targets

    double cost = 0.0;
    auto rd = std::random_device {}; 
    auto rng = std::default_random_engine {rd()};

    // Initialize agent order
    std::vector<int> agentOrder(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        agentOrder[i] = i;

    while(!env.isDone()){
        auto controls = controlPicker(env, targets, dropOffPoints, agentOrder);

        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;

        cost = env.step(controls, targets); 

        if(cost > 10000.0){
            double shuffleCost = std::numeric_limits<float>::max();
            std::vector<int> shuffledAgentOrder = agentOrder;

            Environment shuffleEnv = beforeEnv;
            std::vector<int> shuffleControls;

            int shuffleCount = 0;

            while(shuffleCost > 10000.0){
                shuffleEnv = beforeEnv;

                std::shuffle(std::begin(shuffledAgentOrder), std::end(shuffledAgentOrder), rng);
                shuffleControls = controlPicker(shuffleEnv, targets, dropOffPoints, shuffledAgentOrder);

                shuffleCost = shuffleEnv.step(shuffleControls, targets); 
                shuffleCount++; 

                if(shuffleCount > 100000){
                    beforeEnv.printMatrix();
                    return false;
                }
            }
            //std::cout << "Number of shuffles " << shuffleCount << std::endl;
            env = shuffleEnv;
            cost = shuffleCost;
            controls = shuffleControls;
        }


        //env.printMatrix();

        updateTargets(env, targets, beforeValues, dropOffPoints);
        /* 
        std::cout << '\n';
        for(auto el : controls)
            std::cout << el << " ";
        std::cout << '\n';
        */
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return true;
}

bool simulateBasePolicy(int numOfAgents){
    int wallOffset = 5;
    int boxOffset = 4;
    int n = (int)ceil(sqrt((double)numOfAgents) / 2.0);

    Environment env(wallOffset, boxOffset, n, numOfAgents);

    std::pair<int, int> dropOff1 = {env.getDim() - 4, 3};
    std::pair<int, int> dropOff2 = {env.getDim() - 4, env.getDim() - 4};
    std::vector<std::pair<int, int>> dropOffPoints = {dropOff1};

    std::vector< std::pair<int, int> > targets;
    boxPicker(env, targets, -1); // Initialize targets

    double cost = 0.0;
    auto rd = std::random_device {}; 
    auto rng = std::default_random_engine {rd()};

    std::vector< std::vector<int> > basePolicies(numOfAgents);

    for(size_t i = 0; i < numOfAgents; ++i)
        basePolicies[i] = basePolicy(env, targets, i);

    std::vector<int> controls(numOfAgents);

    while(!env.isDone()){

        for(size_t i = 0; i < numOfAgents; ++i){
            controls[i] = basePolicies[i][basePolicies[i].size() - 1];
            basePolicies[i].pop_back();
        }
        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;

        cost = env.step(controls, targets); 
        if(cost > 1000)
            return false;

        env.printMatrix();

        auto hasUpdatedTarget = updateTargets(env, targets, beforeValues, dropOffPoints);
        updateBasePolicy(env, targets, hasUpdatedTarget, basePolicies);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return true;
}

int main(){
    int simulationCount = 0;
    int numOfSuccess = 0;
    while(true){
        bool success = simulateBasePolicy(2);

        if(success)
            numOfSuccess++;

        simulationCount++;

        std::cout << "Number of simulations: " << simulationCount << ", number of successes: " << numOfSuccess << std::endl;
    }
    return 0;
}