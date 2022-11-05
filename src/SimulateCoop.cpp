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



    init_reservation_table(height, width, obstacles);


    //auto controlsVec = compute_controls(env.getHeight(), env.getWidth(), obstacles, initial_positions, target_positions);

    std::vector<std::vector<int>> controlsVec(numOfAgents, std::vector<int>());

    int iteration = 0;
    while(!env.isDone()){
        target_positions.clear();
        for(auto el : targets)
            target_positions.push_back(Node(el.first, el.second));
        std::vector<Node> positions;
        for(int agent_idx = 0; agent_idx < numOfAgents; agent_idx++){

            if(controlsVec[agent_idx].size() == 0){
                std::pair<int,int> position = indexToPair(env.getMatrixIndex(agent_idx), width);

                Node node_pos(position.first, position.second);
                remove_from_obstacles(node_pos); 
                controlsVec[agent_idx] = compute_controls_for_single_agent(node_pos, target_positions, iteration, agent_idx);

				if (controlsVec[agent_idx].size() == 0) {
					return false;
				}
            }
        }
        std::this_thread::sleep_for(milliseconds(1));
        auto beforeValues = env.getAgentValues();

        Environment beforeEnv = env;
        std::vector<int> controls;
        for(int i = 0; i < numOfAgents; i++){
            int N = controlsVec[i].size();
            controls.push_back(controlsVec[i][N - 1]);
        }

        cost = env.step(controls, targets); 
        env.printMatrix(dropOffPoints, true);

        updateTargets(env, targets, beforeValues, dropOffPoints);

		/*
        for(int agent_idx = 0; agent_idx < numOfAgents; agent_idx++){
            
        }
		*/

        iteration++;
        

        for(int agent_idx = 0; agent_idx < numOfAgents; agent_idx++){
            controlsVec[agent_idx].pop_back();
        }
    }

    return true;
}
