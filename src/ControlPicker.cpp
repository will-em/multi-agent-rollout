#include "ControlPicker.hpp"
#include "BasePolicy.hpp"
#include "UpdateTargets.hpp"
#include "UpdateBasePolicy.hpp"
#include "CostsToControl.hpp"
#include <iostream>

std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> dropOffPoints, std::vector<int> &agentOrder, bool freeze){

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
    int test_n = 0;
    
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
                        auto basePolicy = basePolicies[agentOrder[i]];
                        int n = basePolicy.size();
                        if(n == 0 || basePolicy[n - 1] != optimizedControls[i]){
                            controls[agentOrder[i]] = optimizedControls[i];
                            basePolicies[agentOrder[i]].clear();
                            test_n++;
                        }
                    }

                    basePolicies[agentIdx].clear();
                    //basePolicies[agentIdx].push_back(c2);
                    //basePolicies[agentIdx].push_back(c1);
                    basePolicies[agentIdx].push_back(c0);

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

                        //if(iteration == 0)
                            //hasUpdatedTarget[agentIdx] = false;

                        updateBasePolicy(simEnv, simTargets, hasUpdatedTarget, basePolicies);

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

    std::cout << test_n << std::endl;
    // Reorder optimizedControls
    std::vector<int> reorderedOptimizedControls(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        reorderedOptimizedControls[agentOrder[i]] = optimizedControls[i];
    return reorderedOptimizedControls;
}