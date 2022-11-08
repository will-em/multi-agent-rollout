#include "Simulate.hpp"
#include "Environment.hpp"
#include "BoxPicker.hpp"
#include "ControlPicker.hpp"
#include "UpdateTargets.hpp"
#include "InitAstar.hpp"
#include <random>
#include <math.h>   
#include <unordered_map>

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



    int height = env.getHeight();
    int width = env.getWidth();
    int* matrix = env.getMatPtr();

    std::unordered_map<int, int> posToTargetIdx;

    std::vector<std::pair<int, int>> boxPositions;
    for(size_t i = 0; i < height * width; ++i){
        if(matrix[i] == 2){
            boxPositions.push_back(indexToPair(i, env.getWidth()));
            posToTargetIdx[i] = boxPositions.size() - 1;
        }
    }
    std::vector< std::pair<int, int> > targets;
    boxPicker(env, targets, -1); // Initialize targets

    std::vector<std::pair<int, int>> dropOffPoints;
    for(int i = 3; i < env.getHeight() - 2; ++i){
        if((i - 2) % 3 != 0){
            std::pair<int, int> dropOff1 = {i, 1};
            std::pair<int, int> dropOff2 = {i, width - 2};
            dropOffPoints.push_back(dropOff1);
            dropOffPoints.push_back(dropOff2);
            posToTargetIdx[1 + i * width] = boxPositions.size() + dropOffPoints.size() - 2;
            posToTargetIdx[width - 2 + i * width] = boxPositions.size() + dropOffPoints.size() - 1;
        }
    }

    int pathsSize = env.getWidth() * env.getHeight() * (env.getBoxesLeft() * dropOffPoints.size());
    char* paths = new char[pathsSize];
    for(int i = 0; i < pathsSize; i++)
        paths[i] = -1;

    initAstar(paths, env, boxPositions, dropOffPoints);



    double cost = 0.0;
    auto rd = std::random_device {}; 
    auto rng = std::default_random_engine {rd()};

    // Initialize agent order
    std::vector<int> agentOrder(numOfAgents);
    for(size_t i = 0; i < numOfAgents; ++i)
        agentOrder[i] = i;

    int iteration = 0;
    while(!env.isDone()){

		if (iteration % 3 == 0 && iteration > 0 && iteration <= 60) {
			env.breakRobot(5*(iteration/3 - 1), targets);
		}

        auto t1 = high_resolution_clock::now();
        auto controls = controlPicker(env, targets, dropOffPoints, agentOrder, false, paths, posToTargetIdx);
        auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as an integer. */
        auto ms_int = duration_cast<milliseconds>(t2 - t1);

        //std::cout << ms_int.count() << "ms\n";

        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;
        //env.printMatrix(dropOffPoints);

        cost = env.step(controls, targets); 

        if(cost > 10000.0){
            /*
            std::unordered_map<int, int> posIndexToAgent;
            for(int agentIdx = 0; agentIdx < numOfAgents; agentIdx++){
                int posIndex = env.getMatrixIndex(agentIdx);
                
            }
            */
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
                shuffleControls = controlPicker(shuffleEnv, targets, dropOffPoints, shuffledAgentOrder, true, paths, posToTargetIdx);

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

    delete[] paths;
    return true;
}
