#include "Simulate.hpp"
#include "Environment.hpp"
#include "BoxPicker.hpp"
#include "ControlPicker.hpp"
#include "UpdateTargets.hpp"
#include <random>
#include <math.h>   
#include <chrono>
#include <thread>

int main(){

    Environment env(1);
    int numOfAgents = 2;

    std::vector<std::pair<int, int>> dropOffPoints;

    std::vector< std::pair<int, int> > targets(2);
    targets[0] = std::pair<int, int>(7, 4);
    targets[1] = std::pair<int, int>(1, 4);

    double cost = 0.0;
    auto rd = std::random_device {}; 
    auto rng = std::default_random_engine {rd()};

    // Initialize agent order
    std::vector<int> agentOrder(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        agentOrder[i] = i;

    int iteration = 0;
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

            auto prevControls = std::vector<int>();

            while(shuffleCost > 10000.0){
                shuffleEnv = beforeEnv;
                //std::cout << "SHUFFLING" << std::endl;

                std::shuffle(std::begin(shuffledAgentOrder), std::end(shuffledAgentOrder), rng);
                shuffleControls = controlPicker(shuffleEnv, targets, dropOffPoints, shuffledAgentOrder, true);

                shuffleCost = shuffleEnv.step(shuffleControls, targets); 
                shuffleCount++; 

                if(shuffleCount > 10000){
                    beforeEnv.printMatrix(dropOffPoints, false);
                }

                prevControls = shuffleControls;
            }
            //std::cout << "Number of shuffles " << shuffleCount << std::endl;
            env = shuffleEnv;
            cost = shuffleCost;
            controls = shuffleControls;
            agentOrder = shuffledAgentOrder;

            iteration++;

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        env.printMatrix(dropOffPoints, true);

        updateTargets(env, targets, beforeValues, dropOffPoints);
    }
}