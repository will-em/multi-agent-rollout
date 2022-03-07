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
        std::vector< std::vector< int > > getMatrix();

        // Returns false if a collision between agents occurred
        // Should also take a vector of targets
        double step(std::vector<uint8_t> actions, std::vector< std::pair<unsigned int, unsigned int> > targets); 

        // Implement getters for targets

};