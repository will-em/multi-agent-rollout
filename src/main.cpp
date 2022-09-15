#include <iostream>
#include "Environment.hpp"
#include "IndexToPair.hpp"
#include "BasePolicy.hpp"
#include "CostsToControl.hpp"
#include "BoxPicker.hpp"
#include "UpdateTargets.hpp"
#include "UpdateBasePolicy.hpp"
#include "ControlPicker.hpp"
#include "Simulate.hpp"

int main(){
    int simulationCount = 0;
    int numOfSuccess = 0;
    while(true){
        bool success = simulate(10);

        if(success)
            numOfSuccess++;

        simulationCount++;

        std::cout << "Number of simulations: " << simulationCount << ", number of successes: " << numOfSuccess << std::endl;
    }
    return 0;
}