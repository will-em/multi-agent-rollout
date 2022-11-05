#include "BasePolicy.hpp"
#include "IndexToPair.hpp"
#include "Astar.hpp"

std::vector<int> basePolicy(Environment &env, std::vector<std::pair<int, int>> &targets, int agentIdx, char* paths, std::unordered_map<int,int> &posToTargetIdx){ 
    int height = env.getHeight();
    int width = env.getWidth();
    int* matrix = env.getMatPtr();

    int startIdx = env.getMatrixIndex(agentIdx);

    auto startPos = indexToPair(startIdx, width);

    int targetIdx = posToTargetIdx[width * targets[agentIdx].first + targets[agentIdx].second];


    //paths[curr.second + width * curr.first + height * width * targetIdx] = control;
    int lookUpControl = paths[startPos.second + width * startPos.first + height * width * targetIdx];
    std::vector<int> lookUpControls;
    if(lookUpControl != -1){
        auto pos = startPos;
        while(pos.first != targets[agentIdx].first || pos.second != targets[agentIdx].second){
            lookUpControl = paths[pos.second + width * pos.first + height * width * targetIdx];
            lookUpControls.push_back(lookUpControl);
            //std::cout << lookUpControl << std::endl;
            switch(lookUpControl){
                case 1:
                    pos.first -= 1;
                    break;
                case 2: 
                    pos.first += 1;
                    break;
                case 3:
                    pos.second -= 1;
                    break;
                case 4:
                    pos.second += 1;
                    break;
                default:
                    assert(false);
            }
            //std::cout << pos.first << "," << pos.second << "; " << targets[agentIdx].first << "," << targets[agentIdx].second << std::endl;
        }

        std::reverse(lookUpControls.begin(), lookUpControls.end());
        return lookUpControls;
    }




    float* obstacles = new float[height * width](); // Maybe remove outer wall
    int* path = new int[height * width]();    

    for(size_t i = 0; i < height * width; ++i){
        if(matrix[i] == 1 || matrix[i] == 2)
            obstacles[i] = std::numeric_limits<float>::max();
        else{
            obstacles[i] = 1.0f;
        }
    }

    obstacles[width * targets[agentIdx].first + targets[agentIdx].second] = 1.0f;


    auto goal = targets[agentIdx];
    int goalIdx = width * goal.first + goal.second;

    bool foundPath = astar(obstacles, height, width, startIdx, goalIdx, false, path); 
    assert(foundPath);

    std::vector<int> controls;

    int index = goalIdx;

    if(index == startIdx){
        controls = std::vector<int>(1, 0);
    }else{
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

            controls.push_back(control);

            paths[prev.second + width * prev.first + height * width * targetIdx] = control;

            index = prevIndex;
        }
    }

    delete[] obstacles;
    delete[] path;
    return controls;

}
