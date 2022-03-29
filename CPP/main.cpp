#include <iostream>
#include "Environment.hpp"
#include <math.h>   

bool isDone(Environment &env){
    /*
        TODO:
        Check if all matrix elements at the agentsPositions are non-negative
    */
    return false;
}

void simulate(int numOfAgents){ 
    int n = (int)ceil(sqrt(numOfAgents));
    Environment env(1, 1, n, numOfAgents);

    while(!isDone(env)){
        
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