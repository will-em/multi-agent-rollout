#include <vector>
#include <cstdint>
class Environment{ 
    private:
        int m_stepCount;
        std::vector< std::vector< int > > m_matrix;
        std::vector< std::pair< unsigned int, unsigned int > > m_agentPositions; 

    public:
        Environment(int wallOffset, int boxOffset, int n, int agentCount);

        void printMatrix();

        // Getter for environment matrix
        std::vector< std::vector< int > > getMatrix();

        // Returns the cost of a given set of actions as well as updates the environment
        double step(std::vector<uint8_t> actions, std::vector< std::pair<unsigned int, unsigned int> > targets); 


};