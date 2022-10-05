#include "Simulate.hpp"
#include "Environment.hpp"
#include "BoxPicker.hpp"
#include "ControlPicker.hpp"
#include "UpdateTargets.hpp"
#include <random>
#include <math.h>   
#include "coop-astar.hpp"

#include <chrono>
#include<thread>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

bool simulateCoop(int numOfAgents){ 
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

    // Calculate controls
    int height = env.getHeight();
    int width = env.getWidth();
    int* matPtr = env.getMatPtr();
    
    std::vector<Node> obstacles;
    std::vector<Node> initial_positions;
	std::vector<Node> target_positions;

    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            int el = env.envMat(i, j);
            if(el == 1 || el == 2){
                obstacles.push_back(Node(i, j));
            }
            else if(abs(el) >= 4){
                initial_positions.push_back(Node(i, j));
            }
        }
    }

    for(auto el : targets)
        target_positions.push_back(Node(el.first, el.second));

    auto controlsVec = compute_controls(env.getHeight(), env.getWidth(), obstacles, initial_positions, target_positions);

    int iteration = 0;
    while(!env.isDone()){
        std::this_thread::sleep_for(milliseconds(100));
        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;

        std::vector<int> controls;
        for(int i = 0; i < numOfAgents; i++)
            controls.push_back(controlsVec[i][iteration]);

        cost = env.step(controls, targets); 
        env.printMatrix(dropOffPoints, true);

        updateTargets(env, targets, beforeValues, dropOffPoints);
        iteration++;
    }

    return true;
}
