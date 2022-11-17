#include <iostream>
#include <stdexcept>
#include "Simulate.hpp"

/*

    Usage: rollout [<number_of_agents> <print_environment> <sleep_duration>]
    
        - number_of_agents  - Integer specifying the number of agents in the simulation
        - print_environment - (1|0), whether the environment is printed in the matrix at every turn
        - sleed_duration    - Integer, number of miliseconds of sleep between the simulation of succesive turns

*/
int main(int argc, char* argv[]){
    // Argument processing
    int numOfAgents;
    bool displayEnvironment;
    int msSleepDuration;

    if(argc == 1){ // Default values
        numOfAgents = 100;
        displayEnvironment = true;
        msSleepDuration = 50;    
        std::cout << "Using 100 agents with 50ms sleep." << std::endl;
    }else if(argc == 4){
        numOfAgents = atoi(argv[1]);
        displayEnvironment = atoi(argv[2]);
        msSleepDuration = atoi(argv[3]);
    }else{
        throw std::invalid_argument("Usage: rollout [<number_of_agents> <print_environment> <sleep_duration_ms>]");
    }

    int simulationCount = 0;
    int numOfSuccess = 0;
    while(true){
        bool success = simulate(numOfAgents, displayEnvironment, msSleepDuration);

        if(success)
            numOfSuccess++;

        simulationCount++;

        std::cout << "Number of simulations: " << simulationCount << ", number of successes: " << numOfSuccess << std::endl;
    }
    return 0;
}