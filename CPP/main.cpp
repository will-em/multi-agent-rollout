#include <iostream>
#include "Environment.hpp"
#include <math.h>   
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
        for(size_t i = 0; i < (dim * dim); ++i){
            if(matrix[i] == 2){
                // Check that this box is not the target of another agent
                auto boxPos = indexToPair(i, dim);
                for(auto target : targets){
                    if(boxPos == target)
                        continue;
                }
                targets[agentIdx] = boxPos;
                break;
            }
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
            obstacles[i] = (float) matrix[i];
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
        else if(dj < 0){ // Move left
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

std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::pair<int, int> dropOff){

    int numOfAgents = env.getNumOfAgents(); 

    // Get base policies for numOfAgents - 1 agents 
    std::vector< std::vector<int> > basePolicies(numOfAgents - 1);
    for(size_t i = 1; i < numOfAgents; ++i){
        auto controls = basePolicy(env, targets, i);
        basePolicies[i - 1] = controls;
    } 

    std::vector<int> optimizedControls;

    for(size_t agentIdx = 0; agentIdx < numOfAgents; ++agentIdx){
        std::vector<double> costs(5);
        for(size_t control = 0; control < 5; ++control){
            std::vector<int> controls(optimizedControls);
            controls.push_back(control);

            //controls.insert(controls.end(), basePolicies[agentIdx].begin() + control, basePolicies[agentIdx].end()); // BUG?

            for(size_t i = optimizedControls.size() + 1; i < numOfAgents; ++i){
                int n = basePolicies[i - 1].size();
                controls.push_back(basePolicies[i - 1][n - 1]);
                basePolicies[i - 1].pop_back();
            }

            Environment simEnv = env; // Copy environment
            int* matrix = simEnv.getMatPtr();
            
            double cost = 0.0;
            
            while(!simEnv.isDone()){
                std::vector<int> agentValuesBefore = simEnv.getAgentValues();

                cost += simEnv.step(controls, targets); 

                if(cost < -1000.0){
                    simEnv.printMatrix();
                }

                if(cost > 10000.0)
                    break;

                std::vector<int> agentValuesAfter = simEnv.getAgentValues();

                //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                // Update targets if necessary !!! ALSO UPDATE BASE-POLICY 
                for(size_t i = 0; i < numOfAgents; ++i){
                    if(agentValuesAfter[i] > agentValuesBefore[i]){ // Agent i picks up box
                        targets[i] = dropOff; 
                        basePolicies[i] = basePolicy(simEnv, targets, i);
                    }
                    else if(agentValuesAfter[i] < agentValuesBefore[i]){ // Agent drops off box
                        boxPicker(env, targets, i);
                        basePolicies[i] = basePolicy(simEnv, targets, i);
                    }
                }
            }

            costs[control] = cost;
        }

        // Pick the control with the lowest cost
        int lowestCostIdx = 0;
        for(size_t i = 1; i < 5; ++i){
            if(costs[i] < costs[lowestCostIdx])
                lowestCostIdx = i;
        }

        optimizedControls.push_back(lowestCostIdx);
    }

    return optimizedControls;
}

void simulate(int numOfAgents){ 
    int wallOffset = 5;
    int boxOffset = 1;
    int n = (int)ceil(sqrt((double)numOfAgents) / 2.0);

    Environment env(wallOffset, boxOffset, n, numOfAgents);

    std::pair<int, int> dropOff = {env.getDim() - 2, 1};

    std::vector< std::pair<int, int> > targets;
    boxPicker(env, targets, -1); // Initialize targets

    while(!env.isDone()){
        auto controls = controlPicker(env, targets, dropOff);
        env.step(controls, targets);
        //env.printMatrix();
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
    simulate(10);
    return 0;
}