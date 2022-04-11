#include <vector>
#include <cstdint>
class Environment{ 
    private:
        int m_stepCount;
        int m_dim;
        int m_boxesLeft;
        int *m_matrix;
        std::vector< std::pair<int, int > > m_agentPositions; 

    public:
        Environment(int wallOffset, int boxOffset, int n, int agentCount);
        Environment(Environment &other); // Copy constructor
        Environment &operator=(const Environment &other);
        ~Environment(); // Destructor

        int &envMat(int n, int m);
        void printMatrix();
        int getNumOfAgents();
        int getDim();
        int* getMatPtr();
        bool isDone();
        int getStepCount();

        int getMatrixIndex(int agentIdx); 
        std::vector<int> getAgentValues();

        // Getter for environment matrix
        //std::vector< std::vector< int > > getMatrix();

        // Returns the cost of a given set of actions as well as updates the environment
        double step(std::vector<int> &actions, std::vector< std::pair<int, int> > &targets); 


};