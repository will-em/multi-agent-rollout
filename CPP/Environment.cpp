#include "Environment.h"
#include <iostream>
#include <cstdlib>
#include <cassert>

enum object {space, wall, box, dropOff};

Environment::Environment() : m_stepCount(0) {
    m_matrix = std::vector< std::vector <int> > {
        {1, 1, 1, 1, 1, 1}, 
        {1, 0, 0, 0, 0, 1},
        {1, 0, 0, box, 0, 1},
        {1, 0, 0, 0, 0, 1},
        {1, -4, 0, -5, 0, 1},
        {1, 1, 1, 1, 1, 1},
        };
    
    std::vector< std::pair< unsigned int, unsigned int > > { {4, 1}, {4, 3} };
}

void Environment::printMatrix(){
    for(auto &row : m_matrix){
        for(auto &el : row){
            if(el < 0){
                std::cout << " " << el;
            }else{
            std::cout << "  " << el; 
            }
        }
        std::cout << '\n';
    }
}

// Returns false if a collision between agents occurred
bool Environment::step(std::vector<uint8_t> actions){
    assert(actions.size() == m_agentPositions.size());

    for(size_t agent_index = 0; i < m_agentPositions.size(); i++){
        std::pair agentPos = m_agentPositions[agent_index];
        uint8_t action = actions[agent_index];

        switch(action){

            case 0: // Stand still
                break;
            case 1: // Move up
                int newPos = m_matrx[agentPos.first - 1][agentPos.second]
                if(newPos != wall && newPos != box){
                    m_agentPositions[agent_index].first -= 1;
                }
                break;
            case 2: // Move down
            
                break;
            case 3: // Move left

                break;
            case 4: // Move right

                break;
            default: // Stand still

        }

    }

    // Check for collisions

    return true;
}