#include <vector>
class Environment{ 
    private:
        int m_stepCount;
        std::vector< std::vector< int > > m_matrix;
    public:
        Environment();

        void printMatrix();

};