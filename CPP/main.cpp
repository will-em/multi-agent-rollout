#include <iostream>
#include "Environment.hpp"
#include <math.h>   

std::pair indexToPair(int i, dim){
    // Convert flattened index to 2d coordinate
    int x = (int) floor(i / dim);
    int y = i % dim; 

    std::pair target = {x, y};
    return target;
}

void targetPicker(Environment &env, std::vector< std::pair<int,int> > &targets, int agentIdx){
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
    else{
        for(size_t i = 0; i < (dim * dim); ++i){
            if(matrix[i] == 2){
                // Check that this box is not the target of another agent
                auto boxPos = indexToPair(i, dim);
                for(auto target : targets){
                    if(boxPos == target)
                        continue;
                }
                targets.push_back(boxPos);
            }
        }        
    }
}

std::vector<int> basePolicy(Environment &env, int agentIdx){
    
}

std::vector<int> actionPicker(Environment &env){

    int numOfAgents = env.getNumOfAgents(); 


    // Get base policies for numOfAgents - 1 agents 
    std::vector< std::vector<int> > basePolicies(numOfAgents - 1);
    for(size_t i = 1; i < numOfAgents; ++i){
        basePolicies.push_back(basePolicy(env, i));
    } 
}

void simulate(int numOfAgents){ 
    int wallOffset = 1;
    int boxOffset = 1;
    int n = (int)ceil(sqrt(numOfAgents));
    Environment env(wallOffset, boxOffset, n, numOfAgents);

    while(!env.isDone()){

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
    */
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
    return 0;
}