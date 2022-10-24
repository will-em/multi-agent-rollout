#include "ControlPicker.hpp"
#include "BasePolicy.hpp"
#include "UpdateTargets.hpp"
#include "UpdateBasePolicy.hpp"
#include "CostsToControl.hpp"
#include <iostream>
#include "coop-astar.hpp"
#include "IndexToPair.hpp"
#include <chrono>
#include<thread>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

std::vector<int> controlPicker(Environment &env, std::vector<std::pair<int, int>> targets, std::vector<std::pair<int, int>> dropOffPoints, std::vector<int> &agentOrder, bool freeze){

    int numOfAgents = env.getNumOfAgents(); 

    std::vector< std::vector<int> > preComputedBasePolicies(numOfAgents);

    std::vector<Node> obstacles;

    for(int i = 0; i < env.getHeight(); i++){
        for(int j = 0; j < env.getWidth(); j++){
            int el = env.envMat(i, j);
            if(el == 1 || el == 2){
                obstacles.push_back(Node(i, j));
            }
        }
    }
    init_reservation_table(env.getHeight(), env.getWidth(), obstacles);

    //std::vector<std::vector<int>> reservedControls;

    // Get base policies for numOfAgents 
    for(size_t i = 0; i < numOfAgents; ++i){
        int agentIdx = agentOrder[i];
        auto basePolicyControls = basePolicy(env, targets, agentIdx, 0);
        preComputedBasePolicies[agentIdx] = basePolicyControls;
    } 

    std::vector<int> optimizedControls;
    
    for(int agentIdx : agentOrder){
        std::vector<double> costs(5);
        std::vector<std::vector<int>> candidateBasePolicies(5);

        auto pos = indexToPair(env.getMatrixIndex(agentIdx), env.getWidth());
        remove_agent_path(pos, preComputedBasePolicies[agentIdx], 0);

        for(size_t u0 = 0; u0 < 5; ++u0){
            std::vector<std::pair<int, int>> simTargets(targets);
            std::vector<int> controls(numOfAgents);

            std::vector<std::vector<int>> basePolicies(preComputedBasePolicies);

            for(size_t i = 0; i < optimizedControls.size(); ++i){
                auto basePolicy = basePolicies[agentOrder[i]];

                if(basePolicy.back() != optimizedControls[i]){
                    controls[agentOrder[i]] = optimizedControls[i];
                    basePolicies[agentOrder[i]].clear();
                }
            }


            basePolicies[agentIdx].clear();
            basePolicies[agentIdx].push_back(u0);

            Environment simEnv = env; // Copy environment
            
            double cost = 0.0;
            int iteration = 0; 

            std::vector<std::vector<int>> addBasePolicies(numOfAgents, std::vector<int>());
            std::vector<std::pair<int,int>> startPos(numOfAgents, std::pair<int,int>());
            std::vector<int> startTime(numOfAgents, -1); 
            std::pair<int,int> agentIdxPosition; 

            while(!simEnv.isDone() && iteration < 200){
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
                //std::cout << agentIdx << " " << u0 << " " << iteration << std::endl;
                //simEnv.printMatrix(dropOffPoints, false);
                //std::this_thread::sleep_for(milliseconds(50));
                // Update targets
                auto hasUpdatedTarget = updateTargets(simEnv, simTargets, beforeValues, dropOffPoints);

                for(int i = 0; i < numOfAgents; ++i){
                    if(basePolicies[i].size() == 0){
                        basePolicies[i] = basePolicy(simEnv, simTargets, i, iteration);

                        startPos[i] = indexToPair(simEnv.getMatrixIndex(i), simEnv.getWidth());
                        startTime[i] = iteration;

                        for(int j = basePolicies[i].size() - 1; j >= 0; j--)
                            addBasePolicies[i].push_back(basePolicies[i][j]);
                        
                        for(auto el : basePolicies[i]){
                            std::cout << el;
                        }
                        std::cout << '\n';
                        for(auto el : addBasePolicies[i]){
                            std::cout << el;
                        }
                        std::cout << '\n';
                    }
                }
                //updateBasePolicy(simEnv, simTargets, hasUpdatedTarget, basePolicies, iteration);
                if(iteration == 0){
                    candidateBasePolicies[u0] = basePolicies[agentIdx];
                }

                iteration++;
            }

            costs[u0] = cost;

            // Clean reservation table

            for(int i=0; i < numOfAgents; i++){
                if(addBasePolicies[i].size() > 0){
                    std::reverse(addBasePolicies[i].begin(), addBasePolicies[i].end());
                    remove_agent_path(startPos[i], addBasePolicies[i], startTime[i]);
                }
            }
        }
        int optimalControl = costsToControl(costs, agentIdx, targets, env);


        for(auto cost : costs)
            std::cout << cost << std::endl;

        optimizedControls.push_back(optimalControl);
        preComputedBasePolicies[agentIdx] = candidateBasePolicies[optimalControl];
        preComputedBasePolicies[agentIdx].push_back(optimalControl);
        
        add_agent_path(pos, preComputedBasePolicies[agentIdx], 0);

    }

    // Reorder optimizedControls
    std::vector<int> reorderedOptimizedControls(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        reorderedOptimizedControls[agentOrder[i]] = optimizedControls[i];
    return reorderedOptimizedControls;
}