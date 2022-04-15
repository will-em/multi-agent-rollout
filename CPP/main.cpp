#include <iostream>
#include "Environment.hpp"
#include <math.h>   
#include <stdlib.h> 
#include "astar.cpp"
#include <thread>
#include <chrono>

std::pair<int, int> indexToPair(int i, const int dim){
    // Convert flattened index to 2d coordinate
    int x = (int) floor(i / dim);
    int y = i % dim; 

    std::pair target = {x, y};
    return target;
}


int costsToControl(std::vector<double> &costs, int agentIdx, std::vector<std::pair<int,int>> &targets, Environment &env){
        /*
        for(auto cost : costs)
            std::cout << "Cost " << cost << std::endl;
        std::cout << '\n';
        */
        // Pick the control with the lowest cost
        int lowestCostIdx = 0;
        for(size_t i = 0; i < 5; ++i){
            if(costs[i] < costs[lowestCostIdx])
                lowestCostIdx = i;
        }

        // Find if there are several controls with the same lowest cost
        std::vector<int> lowestCostIndices;
        for(size_t i = 0; i < 5; ++i){
            if(costs[i] == costs[lowestCostIdx])
                lowestCostIndices.push_back(i);
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

            //std::cout << "Control: " << lowestCostIndices[i]<< "  Distance " << distances[i] << std::endl;
            if(distances[i] < distances[lowestDistanceIdx])
                lowestDistanceIdx = i;
        }

        //std::cout << '\n';

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
        //srand(42);
        /*
        std::cout << "Size " << lowestDistanceIndices.size() << std::endl;
        for(auto el : lowestDistanceIndices)
            std::cout << lowestCostIndices[el] << std::endl;
        */
        int control = lowestDistanceIndices[rand() % lowestDistanceIndices.size()];
        //std::cout << "CHOICE " << control << std::endl;
        return control;
}

void boxPicker(Environment &env, std::vector< std::pair<int,int> > &targets, int agentIdx){
    const int dim = env.getDim();
    int* matrix = env.getMatPtr();

    if(targets.empty()){ // Initialization of targets
        int targetsAdded = 0;
        for(size_t i = 0; i < (dim * dim); ++i){
            if(matrix[i] == 2){ // If it is a box
                targets.push_back(indexToPair(i, dim));
                targetsAdded++; 
                if(targetsAdded == env.getNumOfAgents())
                    break;
            }
        }
    }
    else{ // Find the first box that is not the target of an other agent
        bool updatedTarget = false;
        for(size_t i = 0; i < (dim * dim); ++i){
            if(matrix[i] == 2){

                auto boxPos = indexToPair(i, dim);

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

                targets[agentIdx] = boxPos;
                updatedTarget = true;
                break;
            }
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

    int startIdx = env.getMatrixIndex(agentIdx);

    auto goal = targets[agentIdx];
    int goalIdx = dim * goal.first + goal.second;

    bool foundPath = astar(obstacles, dim, dim, startIdx, goalIdx, false, path); 
    assert(foundPath);

    std::vector<int> controls;

    int index = goalIdx;
    while(index != startIdx){
        int prevIndex = path[index];
        
        // Calculate control
        auto curr = indexToPair(index, dim);
        auto prev = indexToPair(prevIndex, dim);

        int di = curr.first - prev.first;
        int dj = curr.second - prev.second;

        int control;
        if(di > 0){      // Move up
            control = 2;
        }
        else if(di < 0){ // Move down
            control = 1;
        }
        else if(dj > 0){ // Move left
            control = 4;
        }
        else{            // Move right
            control = 3;
        }

        controls.push_back(control);

        index = prevIndex;
    }

    delete[] obstacles;
    delete[] path;
    return controls;
}

std::vector<bool> updateTargets(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<int> &beforeValues, std::pair<int, int> dropOff){
    std::vector<int> afterValues = env.getAgentValues();
    int numOfAgents = env.getNumOfAgents();

    std::vector<bool> hasUpdatedTarget(numOfAgents, false);

    // Update targets if necessary 
    for(size_t i = 0; i < numOfAgents; ++i){
        if(afterValues[i] > beforeValues[i]){ // Agent i picks up box
            targets[i] = dropOff; 
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
std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::pair<int, int> dropOff, std::vector<int> &agentOrder){

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

            while(!simEnv.isDone() && iteration < 100){

                for(size_t i = 0; i < numOfAgents; ++i){
                    int n = basePolicies[i].size();

                    if(n > 0){
                        controls[i] = basePolicies[i][n - 1];
                        basePolicies[i].pop_back();
                    }
                }

                auto beforeValues = simEnv.getAgentValues();

                cost += simEnv.step(controls, simTargets); 
                /*
                if(targets[0] == dropOff && targets[1] == dropOff){
                    if(iteration == 0){
                        std::cout << "Agent id " << agentIdx << " control " << control << std::endl;
                    }
                    simEnv.printMatrix();
                    std::cout << "Cost " << cost << " stepCount: " << simEnv.getStepCount() << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                */
                if(cost > 1000.0){
                    break;
                }

                // Update targets
                auto hasUpdatedTarget = updateTargets(simEnv, simTargets, beforeValues, dropOff);
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
        reorderedOptimizedControls[i] = optimizedControls[agentOrder[i]];
    return reorderedOptimizedControls;
}

void simulate(int numOfAgents){ 
    int wallOffset = 5;
    int boxOffset = 1;
    int n = (int)ceil(sqrt((double)numOfAgents) / 2.0);

    Environment env(wallOffset, boxOffset, n, numOfAgents);

    std::pair<int, int> dropOff = {env.getDim() - 3, 2};

    std::vector< std::pair<int, int> > targets;
    boxPicker(env, targets, -1); // Initialize targets

    double cost = 0.0;

    std::vector<int> agentOrder(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        agentOrder[i] = i;

    while(!env.isDone()){
        auto controls = controlPicker(env, targets, dropOff, agentOrder);

        auto beforeValues = env.getAgentValues();

        cost += env.step(controls, targets); 
        env.printMatrix();

        if(cost > 1000.0){
            break;
        }

        updateTargets(env, targets, beforeValues, dropOff);

        for(auto el : targets)
            std::cout << el.first << " " << el.second << std::endl;
        std::cout << '\n';

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

int main(){
    /*
    Environment env(3, 3, 3, 2);
    env.printMatrix();

    std::vector<uint8_t> actions = {4, 4};

    std::pair<unsigned int, unsigned int> target1(0, 0);
    std::pair<unsigned int, unsigned int> target2(2, 2);
    std::vector<std::pair<unsigned int, unsigned int>> targets = {target1, target2};
    std::cout << env.step(actions, targets) << std::endl;
    actions[1] = 3;

    env.printMatrix();
    targets[1] = {2, 3};
    std::cout << env.step(actions, targets) << std::endl;
    env.printMatrix();
    std::pair<unsigned int, unsigned int> target(150, 150);
    int N = 100;
    std::vector<std::pair<unsigned int, unsigned int>> targets(N, target);
    std::vector<uint8_t> actions(N, 2);

    for(int i=0; i<N*5; i++){
        Environment env(N/2, 1, 1, N);
        for(int j=0; j<100; j++){
            env.step(actions, targets);
        }
    }
    */
   /*
    Environment env(5, 1, 1, 2);
    env.printMatrix();

    std::vector<int> actions = {1, 4};

    std::pair<int, int> target1(0, 0);
    std::pair<int, int> target2(2, 2);
    std::vector<std::pair<int, int>> targets = {target1, target2};
    std::cout << env.step(actions, targets) << std::endl;
    env.printMatrix(); 

    std::cout << env.step(actions, targets) << std::endl;
    env.printMatrix(); 
    */


    simulate(3);

    return 0;
}