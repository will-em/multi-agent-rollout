#include "Simulate.hpp"
#include "Environment.hpp"
#include "BoxPicker.hpp"
#include "ControlPicker.hpp"
#include "UpdateTargets.hpp"
#include <random>
#include <math.h>   

#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

bool simulate(int numOfAgents){ 
    int wallOffset = 10;
    int boxOffset = 5;
    int n = (int)ceil(sqrt((double)numOfAgents) / 2.0);

    Environment env(wallOffset, boxOffset, n, numOfAgents);

    std::vector<std::pair<int, int>> dropOffPoints;
    for(int i = 3; i < env.getHeight() - 2; ++i){
        if((i - 2) % 3 != 0){
            std::pair<int, int> dropOff1 = {i, 1};
            std::pair<int, int> dropOff2 = {i, env.getWidth() - 2};
            dropOffPoints.push_back(dropOff1);
            dropOffPoints.push_back(dropOff2);
        }
    }

    std::vector< std::pair<int, int> > targets;
    boxPicker(env, targets, -1); // Initialize targets

    double cost = 0.0;
    auto rd = std::random_device {}; 
    auto rng = std::default_random_engine {rd()};

    // Initialize agent order
    std::vector<int> agentOrder(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        agentOrder[i] = i;

    int iteration = 0;
    while(!env.isDone()){
        auto t1 = high_resolution_clock::now();
        auto controls = controlPicker(env, targets, dropOffPoints, agentOrder);
        auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as an integer. */
        auto ms_int = duration_cast<milliseconds>(t2 - t1);

        //std::cout << ms_int.count() << "ms\n";

        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;
        //env.printMatrix(dropOffPoints);

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
                    return false;
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


        env.printMatrix(dropOffPoints, true);

        updateTargets(env, targets, beforeValues, dropOffPoints);
        /* 
        std::cout << '\n';
        for(auto el : controls)
            std::cout << el << " ";
        std::cout << '\n';
        */
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
}
