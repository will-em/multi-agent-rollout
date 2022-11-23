#include "Simulate.hpp"
#include "Environment.hpp"
#include "BoxPicker.hpp"
#include "ControlPicker.hpp"
#include "UpdateTargets.hpp"
#include "InitAstar.hpp"
#include <random>
#include <math.h>   
#include <unordered_map>
#include <thread>
#include "drawer.h"

#define MAX_NUMBER_OF_SHUFFLES 10000
#define RESHUFFLING_THRESHOLD 10000.0

#define GENERATE_TURN_IMAGES true 
#define GENERATE_SMOOTH_IMAGES false 

bool simulate(int numOfAgents, bool displayEnvironment, int msSleepDuration){ 
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
    for(int i = 4; i < env.getHeight() - 4; ++i){
        std::pair<int, int> dropOff1 = {i, 1};
        std::pair<int, int> dropOff2 = {i, width - 2};
        dropOffPoints.push_back(dropOff1);
        dropOffPoints.push_back(dropOff2);
        posToTargetIdx[dropOff1.second + dropOff2.first * width] = boxPositions.size() + dropOffPoints.size() - 2;
        posToTargetIdx[dropOff2.second + dropOff2.first * width] = boxPositions.size() + dropOffPoints.size() - 1;
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
        auto controls = controlPicker(env, targets, dropOffPoints, agentOrder, false, paths, posToTargetIdx);

        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;

        cost = env.step(controls, targets); 

        if(cost > RESHUFFLING_THRESHOLD){
            double shuffleCost = std::numeric_limits<float>::max();
            std::vector<int> shuffledAgentOrder = agentOrder;

            Environment shuffleEnv = beforeEnv;
            std::vector<int> shuffleControls;

            int shuffleCount = 0;

            auto prevControls = std::vector<int>();

            while(shuffleCost > RESHUFFLING_THRESHOLD){
                shuffleEnv = beforeEnv;

                std::shuffle(std::begin(shuffledAgentOrder), std::end(shuffledAgentOrder), rng);
                shuffleControls = controlPicker(shuffleEnv, targets, dropOffPoints, shuffledAgentOrder, true, paths, posToTargetIdx);

                shuffleCost = shuffleEnv.step(shuffleControls, targets); 
                shuffleCount++; 

                if(shuffleCount > MAX_NUMBER_OF_SHUFFLES){
                    if (displayEnvironment) {
                        beforeEnv.printMatrix(dropOffPoints, false);
                    }
                    return false;
                }

                prevControls = shuffleControls;
            }

            env = shuffleEnv;
            cost = shuffleCost;
            controls = shuffleControls;
            agentOrder = shuffledAgentOrder;


        }


		if (GENERATE_TURN_IMAGES) {
			std::pair<int,int> size(env.getWidth(), env.getHeight());
            std::cout << "Draw frame" << std::endl;
			generate_image("video/frames/", iteration, env.getMatPtr(), &size);
		}

		if (GENERATE_SMOOTH_IMAGES) {
			std::pair<int,int> size(env.getWidth(), env.getHeight());
			for (int i=0; i<12; i++) {
				float alpha = ((float) i)/12.0;
				generate_interpolated_image("video/frames/", iteration, beforeEnv.getMatPtr(), env.getMatPtr(), alpha, &size, numOfAgents);
			}
			
		}
        updateTargets(env, targets, beforeValues, dropOffPoints);

        if (displayEnvironment) {
            env.printMatrix(dropOffPoints, true);
        }
        if (msSleepDuration != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(msSleepDuration));
        }
        iteration++;
    }

    delete[] paths;
    return true;
}
