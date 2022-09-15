#include "ControlPicker.hpp"
#include "BasePolicy.hpp"
#include "UpdateTargets.hpp"
#include "UpdateBasePolicy.hpp"
#include "CostsToControl.hpp"
#include <iostream>

std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> dropOffPoints, std::vector<int> &agentOrder, bool freeze){

    int numOfAgents = env.getNumOfAgents(); 

    std::vector< std::vector<int> > preComputedBasePolicies(numOfAgents);

    // Get base policies for numOfAgents 
    for(size_t i = 0; i < numOfAgents; ++i){
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
        std::vector<std::vector<int>> candidateBasePolicies(5);

        for(size_t u0 = 0; u0 < 5; ++u0){
            std::vector<std::pair<int, int>> simTargets(targets);
            std::vector<int> controls(numOfAgents);

            std::vector<std::vector<int>> basePolicies(preComputedBasePolicies);

            for(size_t i = 0; i < optimizedControls.size(); ++i){
                auto basePolicy = basePolicies[agentOrder[i]];

                if(basePolicy.back() != optimizedControls[i]){
                    //std::cout << "josidfjiosijodfjoisdjofjosdjfsijdof" << std::endl;
                    controls[agentOrder[i]] = optimizedControls[i];
                    basePolicies[agentOrder[i]].clear();
                    test_n++;
                }
            }

            basePolicies[agentIdx].clear();
            basePolicies[agentIdx].push_back(u0);

            Environment simEnv = env; // Copy environment
            
            double cost = 0.0;
            int iteration = 0; 

            while(!simEnv.isDone() && iteration < 100){

                for(size_t i = 0; i < numOfAgents; ++i){
                    if(basePolicies[i].size() > 0){
                        controls[i] = basePolicies[i].back();
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

                updateBasePolicy(simEnv, simTargets, hasUpdatedTarget, basePolicies);
                if(iteration == 0){
                    candidateBasePolicies[u0] = basePolicies[agentIdx];
                }

                iteration++;
            }
            costs[u0] = cost;
        }
        int optimalControl = costsToControl(costs, agentIdx, targets, env);

        optimizedControls.push_back(optimalControl);
        preComputedBasePolicies[agentIdx] = candidateBasePolicies[optimalControl];
        preComputedBasePolicies[agentIdx].push_back(optimalControl);

    }

    std::cout << test_n << std::endl;
    // Reorder optimizedControls
    std::vector<int> reorderedOptimizedControls(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        reorderedOptimizedControls[agentOrder[i]] = optimizedControls[i];
    return reorderedOptimizedControls;
}