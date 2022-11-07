#include "Environment.hpp"
#include <iostream>

// Objects in matrix
enum object {space, wall, box, dropOff, firstAgent};

// Costs
static const double c_step = 1.0, c_pickUp = -10000.0, c_dropOff = -10000.0, c_returnToStart = 0, c_collision = 1e20;

static constexpr double discountFactor = 0.999;

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
    m_height = 35+(4*3);
    m_width = 67+(6*8);
    m_matrix = new int[m_height * m_width]();
    m_boxesLeft = 0;

    // Populate matrix with walls 
    for(int i = 0; i < m_height; ++i){
        int fill_value = (i == 0 || i == m_height- 1) ? wall : space;
        for(int j = 0; j < m_width; ++j){
            envMat(i, j) = fill_value;
        }
        envMat(i, 0) = wall;
        envMat(i, m_width - 1) = wall;
    }

    /*
    // Populate matrix with agents
    int offset = 0;
    for(int i = 0; i < agentCount/2; i+=2){ 
        int n_i = (i < agentCount/2) ? (1) : (m_height - 3);
        int n_j = (i < agentCount/2) ? (4 + i) : (4 + i - agentCount/4);
        if(i == agentCount / 4)
            offset = 0;
        envMat(n_i, n_j + offset) = -firstAgent - i;
        m_agentPositions.push_back(std::pair<int, int>(n_i, n_j));

        envMat(n_i + 1, n_j + offset) = -firstAgent - i - 1;
        m_agentPositions.push_back(std::pair<int, int>(n_i + 1, n_j ));

        offset++;
    }
    */

    int placedAgents = 0;
    int i = 2;
    while(placedAgents < agentCount){
        envMat(1, i) = -firstAgent - placedAgents;
        m_agentPositions.push_back(std::pair<int, int>(1, i));
        placedAgents++;
        if(placedAgents == agentCount) break;

        envMat(3, i) = -firstAgent - placedAgents;
        m_agentPositions.push_back(std::pair<int, int>(3, i));
        placedAgents++;
        if(placedAgents == agentCount) break;

        envMat(m_height - 4, i) = -firstAgent - placedAgents;
        m_agentPositions.push_back(std::pair<int, int>(m_height - 4, i));
        placedAgents++;
        if(placedAgents == agentCount) break;

        envMat(m_height - 2, i) = -firstAgent - placedAgents;
        m_agentPositions.push_back(std::pair<int, int>(m_height - 2, i));
        placedAgents++;
        if(placedAgents == agentCount) break;

        i+=2;
    }

    // Populate matrix with boxes
    for(int i = 6; i < m_height - 2; ++i){
        if((i - 3) % 3 == 0){
            for(int j = 4; j < m_width - 6; ++j){
                if((j - 4) % 8 != 0){
                    envMat(i - 1, j + 1) = box; 
                    m_boxesLeft++;
                    m_availableBoxes.push_back(std::pair(i - 1, j + 1));
                }
            }
        }
    }
}

Environment::Environment(Environment &other) : m_stepCount(0), m_height(other.m_height), m_width(other.m_width), m_agentPositions(other.m_agentPositions), m_boxesLeft(other.m_boxesLeft), m_availableBoxes(other.m_availableBoxes){
    m_matrix = new int[m_height * m_width]();
    for (size_t i = 0; i < m_height* m_width; ++i)
        m_matrix[i] = other.m_matrix[i];
}
Environment& Environment::operator=(const Environment &other){
    if (this != &other) 
    {
        int *new_m_matrix = new int[other.m_height* other.m_width];
        for (size_t i = 0; i < (other.m_height* other.m_width); ++i)
            new_m_matrix[i] = other.m_matrix[i];

        m_stepCount = 0;
        m_height = other.m_height;
        m_width = other.m_width;
        m_boxesLeft = other.m_boxesLeft;
        m_agentPositions = other.m_agentPositions;
        m_availableBoxes = other.m_availableBoxes;

        delete[] m_matrix;
        m_matrix = new_m_matrix; // Point to new place in memory 
    }

    return *this;
}
Environment::~Environment(){
    if(m_matrix != nullptr)
        delete[] m_matrix;
}

int &Environment::envMat(int n, int m){
    return m_matrix[m + n * m_width];
}
void Environment::printMatrix(std::vector<std::pair<int, int>> dropOffPoints, bool redraw){
    if(redraw)
        std::cout << "\033[2J\033[H";
    for(int i = 0; i < m_height; i++){
        for(int j = 0; j < m_width; j++){
            std::cout << "  ";
            int el = envMat(i, j);
            if(std::find(dropOffPoints.begin(), dropOffPoints.end(), std::make_pair(i, j)) != dropOffPoints.end()){
                if(envMat(i, j) != space){
                    char padding = '\0';
                    if(el < 13 && el > -13)
                        padding = ' ';
                    
                    if(el < 0){
                        std::cout << "\033[0;39;100m" << padding << -el - 3 << "\033[0m";
                    }else{
                       std::cout << "\033[0;39;42m" << padding << el - 3 << "\033[0m";
                    }
                }else{
                    std::cout << "\033[1;39;100m" << "  " << "\033[0m";
                }
                continue;
            }
            switch(el){
                case space:
                    std::cout << "  ";
                    break;
                case wall:
                    std::cout << "\033[1;39;44m" << "  " << "\033[0m";
                    break;
                case box:
                    std::cout << "\033[1;39;42m" << "  " << "\033[0m";
                    break;
                default:
                    char padding = '\0';
                    if(el < 13 && el > -13)
                        padding = ' ';
                    
                    if(el < 0){
                        std::cout << padding << -el - 3;
                    }else{
                       std::cout << "\033[0;39;42m" << padding << el - 3 << "\033[0m";
                    }
                    
                    break;
            }
        }
        std::cout << "\n\n";
    }
}

int Environment::getNumOfAgents(){
    return m_agentPositions.size();
}

int Environment::getHeight(){
    return m_height;
}

int Environment::getWidth(){
    return m_width;
}

int* Environment::getMatPtr(){
    return m_matrix;
}

bool Environment::isDone(){
    return m_boxesLeft == 0 ? true : false;
}


int Environment::getMatrixIndex(int agentIdx){
    auto pos = m_agentPositions[agentIdx];
    return m_width * pos.first + pos.second;
}

std::vector<int> Environment::getAgentValues(){
    std::vector<int> output(m_agentPositions.size());

    for(size_t i = 0; i < m_agentPositions.size(); ++i)
        output[i] = m_matrix[m_width * m_agentPositions[i].first + m_agentPositions[i].second];

    return output;
}


std::vector<std::pair<int,int>> &Environment::getAvailableBoxes(){
    return m_availableBoxes;
}

int Environment::getStepCount(){
    return m_stepCount;
}

int Environment::getBoxesLeft(){
    return m_boxesLeft;
}

// Returns the cost of a given set of controls as well as updates the environment
double Environment::step(std::vector<int> &controls, std::vector<std::pair<int, int>> &targets){

    std::vector< std::pair<int, int> > newPositions(m_agentPositions.size());

    double cost = 0;
    const int tot_dim = m_height * m_width;
    int coll_mat[tot_dim];
    for(int i = 0; i < tot_dim; ++i)
        coll_mat[i] = 0;

    // Find new positions 
    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        std::pair agentPos = m_agentPositions[agent_index];
        std::pair target = targets[agent_index];
        uint8_t control = controls[agent_index];

        std::pair<int, int> newPos(agentPos.first, agentPos.second); // Stand still

        switch(control){
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
        int newPosEl = envMat(newPos.first, newPos.second);

        // If the new position is a wall or an undesired box, stand still, otherwise move
        if(newPosEl == wall || (newPosEl == box && newPos != targets[agent_index])){
            newPositions[agent_index] = m_agentPositions[agent_index];
        }
        else{
            newPositions[agent_index] = newPos;
        }
        // Check for and penalize collisions
        if(coll_mat[m_width * newPositions[agent_index].first + newPositions[agent_index].second] != 0){
            cost += c_collision * discountFactors.arr[m_stepCount];
        }else{
            coll_mat[m_width * newPositions[agent_index].first + newPositions[agent_index].second] = 1;
        }
    }


    int swapPenalized[m_agentPositions.size()];
    for(int i = 0; i < tot_dim; ++i)
        swapPenalized[i] = 0;
    // Check for and penalize swaps 
    for(size_t i = 0; i < m_agentPositions.size(); ++i){
        // Cannot swap if standing still
        if(newPositions[i] == m_agentPositions[i] || swapPenalized[i] != 0) 
            continue;

        int agent_index = envMat(newPositions[i].first, newPositions[i].second); // Which agent stood here before?
        agent_index = (agent_index>=0) ? agent_index : -agent_index; 

        // Neglect non-agents 
        if(agent_index < firstAgent)
            continue;

        // Is the agent that stood here before, now at my previous position?
        if(newPositions[agent_index - firstAgent] == m_agentPositions[i]){ 
            cost += c_collision * discountFactors.arr[m_stepCount];
            swapPenalized[agent_index - firstAgent] = 1;
        }
    }

    std::vector<int> oldEl(m_agentPositions.size());
    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        oldEl[agent_index] = envMat(m_agentPositions[agent_index].first, m_agentPositions[agent_index].second);
        envMat(m_agentPositions[agent_index].first, m_agentPositions[agent_index].second) = space;
    }
    for(size_t agent_index = 0; agent_index < m_agentPositions.size(); ++agent_index){
        
        if(controls[agent_index] != 0)
            cost += c_step;

        // Update matrix
        envMat(newPositions[agent_index].first, newPositions[agent_index].second) = oldEl[agent_index];

        // If an agent reaches its target
        if(newPositions[agent_index] == targets[agent_index] && m_agentPositions[agent_index] != newPositions[agent_index]){
            if(targets[agent_index].first != 1 || targets[agent_index].first != m_height - 2){
                if(envMat(newPositions[agent_index].first, newPositions[agent_index].second) > 0){ // If the agent has a box
                    cost += c_dropOff * discountFactors.arr[m_stepCount];
                    m_boxesLeft--;
                }else{
                    cost += c_pickUp * discountFactors.arr[m_stepCount];
                }
                envMat(newPositions[agent_index].first, newPositions[agent_index].second) = -envMat(newPositions[agent_index].first, newPositions[agent_index].second);
            }
            else{ // Return to starting position
                cost += c_returnToStart * discountFactors.arr[m_stepCount];
            }
        }

    }
    m_agentPositions = newPositions;
    m_stepCount += 1;

    return cost;
}

void Environment::forceFinish() {
	m_boxesLeft = 0;
}
