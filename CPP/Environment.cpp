#include "Environment.hpp"
#include <iostream>

// Objects in matrix
enum object {space, wall, box, dropOff, firstAgent};

// Costs
static const double c_step = 1.0, c_pickUp = -100.0, c_dropOff = -1000.0, c_collision = 1e10;

static constexpr double discountFactor = 0.99;

template<int N>
struct Discount {
    double arr[N];
    constexpr Discount () : arr() {
        arr[0] = 1.0;
        for (int i = 1; i <  N; ++i)
            arr[i] = arr[i - 1] * discountFactor;
    }
};
static constexpr auto discountFactors = Discount<10000>();




Environment::Environment(int wallOffset, int boxOffset, int n, int agentCount) : m_stepCount(0) {
    int dim = 2 + 2 * wallOffset + (n - 1) * boxOffset + 2 * n;

    // Populate matrix with walls and boxes
    for(int i = 0; i < dim; ++i){
        int fill_value = (i == 0 || i == dim - 1) ? wall : space;
        std::vector<int> row(dim, fill_value);
        row[0] = wall;
        row[dim - 1] = wall;
        m_matrix.push_back(row);
    }

    // Populate matrix with agents
    for(int i = 0; i < agentCount; ++i){ 
        m_matrix[1][1 + i] = -firstAgent - i;
        m_agentPositions.push_back(std::pair<unsigned int, unsigned int>(1, 1 + i));
    }

    // Populate matrix with boxes
    for(int n_i = 0; n_i < n; ++n_i){
        for(int n_j = 0; n_j < n; ++n_j){
            int i = 1 + wallOffset + n_i * (2 + boxOffset);
            int j = 1 + wallOffset + n_j * (2 + boxOffset);
            m_matrix[i][j] = box; 
            m_matrix[i + 1][j] = box; 
            m_matrix[i][j + 1] = box; 
            m_matrix[i + 1][j + 1] = box; 
        }
    }

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

// Getter for environment matrix
std::vector< std::vector< int > > Environment::getMatrix(){
    return m_matrix;
}

// Returns the cost of a given set of actions as well as updates the environment
double Environment::step(std::vector<uint8_t> actions, std::vector<std::pair<unsigned int, unsigned int>> targets){
    assert(actions.size() == m_agentPositions.size());
    assert(targets.size() == m_agentPositions.size());

    std::vector< std::pair<unsigned int, unsigned int> > newPositions(m_agentPositions.size());

    double cost;
    const unsigned int dim = m_matrix.size(); 
    std::vector<std::vector<unsigned int> > coll_mat( dim, std::vector<unsigned int> (dim, 0));  

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

        // If the new position is a wall or an undesired box, stand still, otherwise move
        if(newPosEl == wall || (newPosEl == box && newPos != targets[agent_index])){
            newPositions[agent_index] = m_agentPositions[agent_index];
        }
        else{
            newPositions[agent_index] = newPos;
        }

        // Check for collisions
        if(coll_mat[newPositions[agent_index].first][newPositions[agent_index].second] != 0){
            cost += c_collision * discountFactors.arr[m_stepCount];
        }else{
            coll_mat[newPositions[agent_index].first][newPositions[agent_index].second] = 1;
        }
    }

    /*
    Check for collisions.
    Algorithm is O(n^2) in time where n is the number of agents
    which is fine since n is rather small (<100) and a linear or O(nlogn)
    solution would be slower due to cache locality etc.
    */
    // OPTIMIZE
    /*
    const unsigned int dim = m_matrix.size(); 
    std::vector<std::vector<unsigned int> > coll_mat( dim, std::vector<unsigned int> (dim, 0));  


    for(size_t i = 0; i < m_agentPositions.size(); ++i){
        if(coll_mat[newPositions[i].first][newPositions[i].second] != 0){
            cost += c_collision * discountFactors.arr[m_stepCount];
        }else{
            coll_mat[newPositions[i].first][newPositions[i].second] = 1;
        }
    }
    */
    /*
    for(size_t i = 0; i < m_agentPositions.size(); ++i){
        std::vector<unsigned int> oldToNew = {m_agentPositions[i].first, m_agentPositions[i].second, newPositions[i].first, newPositions[i].second};
        for(size_t j = i + 1; j < m_agentPositions.size(); ++j){
            std::vector<unsigned int> complement = {newPositions[j].first, newPositions[j].second, m_agentPositions[j].first, m_agentPositions[j].second};
            // If two agents have the same position or swap positions
            if(newPositions[i] == newPositions[j] || oldToNew == complement){ 
                cost += c_collision * discountFactors.arr[m_stepCount];
            }

        }
    }
    */
    std::vector<int> oldEl(m_agentPositions.size());
    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        oldEl[agent_index] = m_matrix[m_agentPositions[agent_index].first][m_agentPositions[agent_index].second];
        m_matrix[m_agentPositions[agent_index].first][m_agentPositions[agent_index].second] = space;
    }
    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        
        if(actions[agent_index] != 0)
            cost += c_step;

        // Update matrix
        m_matrix[newPositions[agent_index].first][newPositions[agent_index].second] = oldEl[agent_index];

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