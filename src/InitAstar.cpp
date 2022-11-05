#include "BasePolicy.hpp"
#include "IndexToPair.hpp"
#include "InitAstar.hpp"
#include "Astar.hpp"


void reservePath(char* paths, float* obstacles, int height, int width, int startIdx, int goalIdx, int numOfTargets, int targetIdx){

    int* path = new int[height * width]();    
    bool foundPath = astar(obstacles, height, width, startIdx, goalIdx, false, path); 

    int index = goalIdx;

    while(index != startIdx){
        int prevIndex = path[index];
        
        // Calculate control
        auto curr = indexToPair(index, width);
        auto prev = indexToPair(prevIndex, width);

        int di = curr.first - prev.first;
        int dj = curr.second - prev.second;

        int control;
        if(di > 0){      // Move down
            control = 2;
        }
        else if(di < 0){ // Move up
            control = 1;
        }
        else if(dj > 0){ // Move right
            control = 4;
        }
        else{            // Move left
            control = 3;
        }

        paths[prev.second + width * prev.first + height * width * targetIdx] = control;

        index = prevIndex;
    }

    delete[] path; // Consider overriding instead of deleting every time
}

void initAstar(char* paths, Environment &env, std::vector<std::pair<int, int>> boxes, std::vector<std::pair<int, int>> dropOffPoints){
    int height = env.getHeight();
    int width = env.getWidth();
    int* matrix = env.getMatPtr();

    float* obstacles = new float[height * width](); // Consider removing outer wall

    for(size_t i = 0; i < height * width; ++i){
        if(matrix[i] == 1 || matrix[i] == 2)
            obstacles[i] = std::numeric_limits<float>::max();
        else{
            obstacles[i] = 1.0f;
        }
    }

    int numOfTargets = boxes.size() + dropOffPoints.size();

    // Initial positions to boxes
    for(int i = 0; i < boxes.size(); i++){
        auto box = boxes[i];
        int goalIdx = width * box.first + box.second;
        obstacles[goalIdx] = 1.0f;

        for(int agentIdx = 0; agentIdx < env.getNumOfAgents(); agentIdx++){
            int startIdx = env.getMatrixIndex(agentIdx);
            reservePath(paths, obstacles, height, width, startIdx, goalIdx, numOfTargets, i);
        }

        for(int dropOffIdx = 0; dropOffIdx < dropOffPoints.size(); dropOffIdx++){
            auto dropOff = dropOffPoints[dropOffIdx];
            int startIdx = width * dropOff.first + dropOff.second;
            reservePath(paths, obstacles, height, width, startIdx, goalIdx, numOfTargets, i);
            reservePath(paths, obstacles, height, width, goalIdx, startIdx, numOfTargets, boxes.size() + dropOffIdx);
        }

        obstacles[goalIdx] = std::numeric_limits<float>::max();
    }

    delete[] obstacles;
}