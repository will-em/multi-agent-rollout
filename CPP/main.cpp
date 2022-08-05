#include <iostream>
#include "Environment.hpp"
#include <math.h>   
#include <stdlib.h> 
#include <thread>
#include <chrono>
#include <algorithm>
#include "IndexToPair.hpp"
#include "BasePolicy.hpp"
#include "CostsToControl.hpp"
#include "BoxPicker.hpp"
#include "UpdateTargets.hpp"
#include <random>

void updateBasePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, std::vector<bool> &hasUpdatedTarget, std::vector<std::vector<int>> &basePolicies){
    int numOfAgents = env.getNumOfAgents();

    for(size_t i = 0; i < numOfAgents; ++i){
        if(hasUpdatedTarget[i])
            basePolicies[i] = basePolicy(env, targets, i);
    }    
}
std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> dropOffPoints, std::vector<int> &agentOrder, bool freeze = false){

    int numOfAgents = env.getNumOfAgents(); 

    std::vector< std::vector<int> > preComputedBasePolicies(numOfAgents);

    int agentNotToFreeze = rand() % numOfAgents;

    // Get base policies for numOfAgents 
    for(size_t i = 1; i < numOfAgents; ++i){
        int agentIdx = agentOrder[i];
        auto basePolicyControls = basePolicy(env, targets, agentIdx);
        preComputedBasePolicies[agentIdx] = basePolicyControls;

        if(freeze){
            for(size_t j = 0; j < (rand() % 20 + 1); j++){
                preComputedBasePolicies[agentIdx].push_back(0);
            }
        }
    } 

    std::vector<int> optimizedControls;
    
    for(int agentIdx : agentOrder){
        std::vector<double> costs(5);
        for(size_t c0 = 0; c0 < 5; ++c0){
            std::vector<double> subTreeCosts;
            //for(size_t c1 = 0; c1 < 5; ++c1){
                //for(size_t c2 = 0; c2 < 5; ++c2){
                    std::vector<std::pair<int, int>> simTargets(targets);
                    std::vector<int> controls(numOfAgents);

                    std::vector<std::vector<int>> basePolicies(preComputedBasePolicies);

                    for(size_t i = 0; i < optimizedControls.size(); ++i){
                        controls[agentOrder[i]] = optimizedControls[i];
                        basePolicies[agentOrder[i]].clear();
                    }

                    basePolicies[agentIdx].clear();
                    //basePolicies[agentIdx].push_back(c2);
                    //basePolicies[agentIdx].push_back(c1);
                    basePolicies[agentIdx].push_back(c0);

                    Environment simEnv = env; // Copy environment
                    
                    double cost = 0.0;
                    int iteration = 0; 

                    std::vector<int> agentIdxBasePolicy;

                    while(!simEnv.isDone() && iteration < 50){

                        for(size_t i = 0; i < numOfAgents; ++i){
                            int n = basePolicies[i].size();

                            if(n > 0){
                                controls[i] = basePolicies[i][n - 1];
                                basePolicies[i].pop_back();
                            }
                        }

                        auto beforeValues = simEnv.getAgentValues();

                        cost += simEnv.step(controls, simTargets); 
                        //simEnv.printMatrix();
                        //std::this_thread::sleep_for(std::chrono::milliseconds(500));

                        if(cost > 1000.0){
                            break;
                        }

                        // Update targets
                        auto hasUpdatedTarget = updateTargets(simEnv, simTargets, beforeValues, dropOffPoints, iteration);

                        if(iteration < 1)
                            hasUpdatedTarget[agentIdx] = false;

                        updateBasePolicy(simEnv, simTargets, hasUpdatedTarget, basePolicies);

                        for(size_t i = 0; i < numOfAgents; ++i){
                            if(basePolicies[i].empty())
                                basePolicies[i] = basePolicy(simEnv, simTargets, i); 
                        }
                        
                        iteration++;
                    }
                    subTreeCosts.push_back(cost);
                //}
            //}
            /*
            double lowestSubTreeCost = subTreeCosts[0];
            for(auto el : subTreeCosts){
                if(el < lowestSubTreeCost) {
                    lowestSubTreeCost = el;
                }
                std::cout << el << " ";
            }
            std::cout << '\n';
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout
            */
            costs[c0] = *std::min_element(subTreeCosts.begin(), subTreeCosts.end()) ;
        }
        /*
        for(auto el : costs)
            std::cout << el << " ";
        std::cout << '\n';
        */
        optimizedControls.push_back(costsToControl(costs, agentIdx, targets, env));
    }

    // Reorder optimizedControls
    std::vector<int> reorderedOptimizedControls(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        reorderedOptimizedControls[agentOrder[i]] = optimizedControls[i];
    return reorderedOptimizedControls;
}

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
        auto controls = controlPicker(env, targets, dropOffPoints, agentOrder);

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
                    beforeEnv.printMatrix(dropOffPoints);
                    return false;
                }

                prevControls = shuffleControls;

                /*
                if(shuffleCount % 10){
                    for(size_t i = 0; i < numOfAgents; i++)
                        prevControls[i] = rand() % 5;
                }
                */
            }
            std::cout << "Number of shuffles " << shuffleCount << std::endl;
            env = shuffleEnv;
            cost = shuffleCost;
            controls = shuffleControls;
            agentOrder = shuffledAgentOrder;

            iteration++;

        }


        env.printMatrix(dropOffPoints);

        updateTargets(env, targets, beforeValues, dropOffPoints, iteration);
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

int main(){
    int simulationCount = 0;
    int numOfSuccess = 0;
    while(true){
        bool success = simulate(50);

        if(success)
            numOfSuccess++;

        simulationCount++;

        std::cout << "Number of simulations: " << simulationCount << ", number of successes: " << numOfSuccess << std::endl;
    }
    return 0;
}