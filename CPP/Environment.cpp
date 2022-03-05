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
bool Environment::step(std::vector<uint8_t> actions, std::vector<std::pair<int, int>> targets){
    assert(actions.size() == m_agentPositions.size());
    assert(targets.size() == m_agentPositions.size());

    std::vector< std::pair<int int> > newPositions;

    // Find new positions 
    for(size_t agent_index = 0; i < m_agentPositions.size(); ++agent_index){
        std::pair agentPos = m_agentPositions[agent_index];
        std::pair target = targets[agent_index];
        uint8_t action = actions[agent_index];

        std::pair newPos(agentPos.first, agentPos.second); // Stand still

        switch(action){
            case 1: // Move up
                newPos.first = agentPos.first - 1;
                break;
            case 2: // Move down
                newPos.first = agentPos.first + 1;
                break;
            case 3: // Move left
                newPos.second = agentPos.second - 1;
                break;
            case 4: // Move right
                newPos.second = agentPos.second + 1;
                break;
        }

        int newPosEl = m_matrix[newPos.first][newPos.second];
        /*
        If the agent picks up its designated box or 
        the new position is empty space, then add position to newPositions
        */
        if((newPosEl == box && newPos == targets[agent_index]) || newPosEl == space){
            newPositions.push_back(newPos);
        }
    }

    // Check for collisions
    /*
    Algorithm is O(n^2) in time where n is the number of agents
    which is fine since n is rather small (<100) and a linear or O(nlogn)
    solution would be slower due to cache locality etc.
    */
    for(size_t i = 0; < m_agentPositions.size(); ++i){
        for(size_t j = i; j < m_agentPositions.size(); ++j){
            if(newPositions[i] == newPositions[j])
                return false;
        }
    }

    // Check for swap 
        /*
        Implement unordered map from coordinate pair to coordinate pair

        BEWARE OF STANDSTILL!
        */
    return true;
}