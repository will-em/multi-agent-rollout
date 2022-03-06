#include "Environment.h"
#include <iostream>

// Objects in matrix
enum object {space, wall, box, dropOff};

// Costs
const double c_step = 1.0, c_pickUp = 0.0, c_dropOff = -1000.0, c_collision = 1e10;

Environment::Environment() : m_stepCount(0) {
    m_matrix = std::vector< std::vector <int> > {
        {1, 1, 1, 1, 1, 1}, 
        {1, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 1},
        {1, 0, 0, box, 0, 1},
        {1, -4, 0, -5, 0, 1},
        {1, 1, 1, 1, 1, 1},
        };
    
    m_agentPositions.push_back(std::pair<unsigned int, unsigned int>(4, 1));
    m_agentPositions.push_back(std::pair<unsigned int, unsigned int>(4, 3));
    m_stepCount = 0;
}

void Environment::printMatrix(){
    for(auto &row : m_matrix){
        for(auto &el : row){
            if(el < 0){
                std::cout << " " << el;
            }
            else
            {
                std::cout << "  ";
                switch(el){
                    case 0:
                        std::cout << " ";
                        break;
                    case 1:
                        std::cout << 'X';
                        break;
                    default:
                        std::cout << el;
                        break;
                }
            }
        }
        std::cout << '\n';
    }
}

// Returns the cost of a given set of actions as well as updates the environment
double Environment::step(std::vector<uint8_t> actions, std::vector<std::pair<unsigned int, unsigned int>> targets){
    assert(actions.size() == m_agentPositions.size());
    assert(targets.size() == m_agentPositions.size());

    std::vector< std::pair<unsigned int, unsigned int> > newPositions(m_agentPositions.size());

    double cost;

    // Find new positions 
    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        std::pair agentPos = m_agentPositions[agent_index];
        std::pair target = targets[agent_index];
        uint8_t action = actions[agent_index];

        std::pair<unsigned int, unsigned int> newPos(agentPos.first, agentPos.second); // Stand still

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
            newPositions[agent_index] = newPos;
        }
        else{
            newPositions[agent_index] = m_agentPositions[agent_index];
        }
    }
    // Check for collisions
    /*
    Algorithm is O(n^2) in time where n is the number of agents
    which is fine since n is rather small (<100) and a linear or O(nlogn)
    solution would be slower due to cache locality etc.
    */
    for(size_t i = 0; i < m_agentPositions.size(); ++i){
        std::vector<unsigned int> oldToNew = {m_agentPositions[i].first, m_agentPositions[i].second, newPositions[i].first, newPositions[i].second};
        for(size_t j = i + 1; j < m_agentPositions.size(); ++j){
            if(newPositions[i] == newPositions[j]){ // If two agents have the same position
                cost += c_collision;
            }
            std::vector<unsigned int> complement = {newPositions[j].first, newPositions[j].second, m_agentPositions[j].first, m_agentPositions[j].second};

            if(oldToNew == complement){ // If to agents swap positions
                cost += c_collision;
            }
        }
    }

    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        
        if(actions[agent_index] != 0)
            cost += c_step;

        // If an agent moves
        if(m_agentPositions[agent_index] == newPositions[agent_index])
            continue;
        
        // Update matrix
        unsigned int temp = m_matrix[m_agentPositions[agent_index].first][m_agentPositions[agent_index].second];
        m_matrix[m_agentPositions[agent_index].first][m_agentPositions[agent_index].second] = space;
        m_matrix[newPositions[agent_index].first][newPositions[agent_index].second] = temp;

        // If an agent reaches its target
        if(newPositions[agent_index] == targets[agent_index]){
            if(m_matrix[newPositions[agent_index].first][newPositions[agent_index].second] > 0){ // If the agent has a box
                cost += c_dropOff;
            }else{
                cost += c_pickUp;
            }
            m_matrix[newPositions[agent_index].first][newPositions[agent_index].second] = -m_matrix[newPositions[agent_index].first][newPositions[agent_index].second];
        }
    }

    m_agentPositions = newPositions;
    m_stepCount += 1;

    return cost;
}